//! Phase 2.8 — Multi-solver beam validation and numerical residual audit.
//!
//! Runs the same analytical beam cases through every applicable direct
//! `LinearSolver` backend (`dense`, `skyline_ldlt`, `sparse_lu`), comparing each
//! against (a) closed-form solutions and (b) each other (dense as reference).
//!
//! It also performs an independent **residual** and **energy** audit using a
//! test-side assembly of `K` and `f` from the public model/API — deliberately
//! *not* the production reaction routine — so that the audit is not
//! self-referential.
//!
//! Iterative backends: CG is applicable to these SPD condensed beam systems and
//! is validated here; ICCG's IC(0) factorisation currently fails on several beam
//! stiffness matrices (it returns an error rather than a silently wrong answer)
//! and is therefore documented, not asserted, below.

use section_properties::beam_fem::{
    BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver, FemError,
};
use section_properties::fea::solver::SolverRegistry;
use section_properties::material::Material;

// ---------------------------------------------------------------------------
// Constants / helpers
// ---------------------------------------------------------------------------

const E0: f64 = 200e9;
const A0: f64 = 0.02;

fn i0() -> f64 {
    0.1 * 0.2_f64.powi(3) / 12.0
}

/// Direct backends, ordered with the numerical reference (dense) first.
const DIRECT: [&str; 3] = ["dense", "skyline_ldlt", "sparse_lu"];

fn mat(e: f64) -> Material {
    Material::new(e, 0.3, 7850.0, "Steel")
}

fn chain(n_elem: usize, total: f64, e: f64, a: f64, i: f64) -> BeamModel {
    let mut model = BeamModel::new();
    let dx = total / n_elem as f64;
    for k in 0..=n_elem {
        model.add_node(BeamNode::new(k, k as f64 * dx, 0.0));
    }
    for k in 0..n_elem {
        model.add_element(BeamElement::new(k, k + 1, mat(e), BeamSection::new(a, i)).unwrap());
    }
    model.fix_node(0);
    model
}

fn solve_backend(model: &BeamModel, backend: &str) -> BeamSolver {
    let mut solver = BeamSolver::from_model(model).unwrap();
    let registry = SolverRegistry::default();
    let mut linear = registry
        .create(backend)
        .unwrap_or_else(|| panic!("backend '{}' not registered", backend));
    solver
        .solve(&mut *linear)
        .unwrap_or_else(|e| panic!("backend '{}' failed to solve: {:?}", backend, e));
    solver
}

fn assert_pair(a: f64, b: f64, abs: f64, rel: f64, label: &str) {
    let bound = abs + rel * a.abs().max(b.abs());
    assert!(
        (a - b).abs() <= bound,
        "{}: |{} - {}| = {:.3e} > {:.3e}",
        label,
        a,
        b,
        (a - b).abs(),
        bound
    );
}

// Analytical cantilever formulas (project sign convention).
fn an_tip_disp(p: f64, l: f64, ei: f64) -> f64 {
    p * l.powi(3) / (3.0 * ei)
}
fn an_tip_rot(p: f64, l: f64, ei: f64) -> f64 {
    p * l.powi(2) / (2.0 * ei)
}
fn an_udl_disp(q: f64, l: f64, ei: f64) -> f64 {
    q * l.powi(4) / (8.0 * ei)
}
fn an_udl_rot(q: f64, l: f64, ei: f64) -> f64 {
    q * l.powi(3) / (6.0 * ei)
}

// ---------------------------------------------------------------------------
// Test-side independent assembly of K (dense) and f, from public APIs only.
// Used for the residual and energy audits (not the production reaction routine).
// ---------------------------------------------------------------------------

fn element_dof_map(model: &BeamModel, e: &BeamElement) -> [usize; 6] {
    [
        model.dof_index(e.node_i, 0),
        model.dof_index(e.node_i, 1),
        model.dof_index(e.node_i, 2),
        model.dof_index(e.node_j, 0),
        model.dof_index(e.node_j, 1),
        model.dof_index(e.node_j, 2),
    ]
}

/// Map a local (element) vector to global and scatter it into `f`.
fn scatter_local(model: &BeamModel, e: &BeamElement, local: [f64; 6], f: &mut [f64]) {
    let ni = model.nodes[e.node_i].point();
    let nj = model.nodes[e.node_j].point();
    let t = e.transformation_matrix(ni, nj);
    let map = element_dof_map(model, e);
    for (i, &m) in map.iter().enumerate() {
        let mut g = 0.0;
        for (j, &l) in local.iter().enumerate() {
            g += t[j][i] * l; // f_global = Tᵀ f_local
        }
        f[m] += g;
    }
}

/// Assemble the full (unconstrained) global stiffness `K` and load vector `f`.
fn assemble_k_f(model: &BeamModel) -> (Vec<Vec<f64>>, Vec<f64>) {
    let n = model.n_dof();
    let mut k = vec![vec![0.0f64; n]; n];
    for e in &model.elements {
        let ni = model.nodes[e.node_i].point();
        let nj = model.nodes[e.node_j].point();
        let ke = e.global_stiffness(ni, nj);
        let map = element_dof_map(model, e);
        for a in 0..6 {
            for b in 0..6 {
                k[map[a]][map[b]] += ke[a][b];
            }
        }
    }

    let mut f = vec![0.0f64; n];
    for &(node, dof, v) in &model.nodal_forces {
        f[model.dof_index(node, dof)] += v;
    }
    for dl in &model.distributed_loads {
        let e = &model.elements[dl.element_idx];
        let ni = model.nodes[e.node_i].point();
        let nj = model.nodes[e.node_j].point();
        let local = e.consistent_nodal_load(ni, nj, dl.qx, dl.qy).unwrap();
        scatter_local(model, e, local, &mut f);
    }
    for pl in &model.point_loads {
        let e = &model.elements[pl.element_idx];
        let ni = model.nodes[e.node_i].point();
        let nj = model.nodes[e.node_j].point();
        let local = e
            .consistent_nodal_load_point(ni, nj, pl.position, pl.fx, pl.fy, pl.mz)
            .unwrap();
        scatter_local(model, e, local, &mut f);
    }
    for am in &model.applied_moments {
        f[model.dof_index(am.node_idx, 2)] += am.value;
    }
    (k, f)
}

struct Audit {
    free_res: f64,
    reaction_res: f64,
    energy_rel: f64,
    range: f64,
}

fn audit(model: &BeamModel, solver: &BeamSolver) -> Audit {
    let n = model.n_dof();
    let (k, f) = assemble_k_f(model);
    let u = solver.displacements();
    let reactions = solver.reactions();

    let mut fixed = vec![false; n];
    for &(node, dof, _) in &model.fixed_dofs {
        fixed[model.dof_index(node, dof)] = true;
    }

    // r = K u - f  (independent dense mat-vec).
    let mut r = vec![0.0f64; n];
    for i in 0..n {
        let mut s = 0.0;
        for j in 0..n {
            s += k[i][j] * u[j];
        }
        r[i] = s - f[i];
    }

    let range = f.iter().fold(1.0f64, |m, &v| m.max(v.abs()));
    let mut free_res = 0.0f64;
    let mut reaction_res = 0.0f64;
    for i in 0..n {
        if fixed[i] {
            // Support residual must reproduce the production reaction.
            reaction_res = reaction_res.max((r[i] - reactions[i]).abs());
        } else {
            free_res = free_res.max(r[i].abs());
        }
    }

    // Energy: U = 1/2 uᵀ K u, W = 1/2 uᵀ f.
    let mut uku = 0.0;
    let mut uf = 0.0;
    for i in 0..n {
        let mut row = 0.0;
        for j in 0..n {
            row += k[i][j] * u[j];
        }
        uku += u[i] * row;
        uf += u[i] * f[i];
    }
    let (u_strain, w_ext) = (0.5 * uku, 0.5 * uf);
    let energy_rel = (u_strain - w_ext).abs() / u_strain.abs().max(w_ext.abs()).max(1e-30);

    Audit {
        free_res,
        reaction_res,
        energy_rel,
        range,
    }
}

/// Run one analytical case through every direct backend, checking the
/// analytical solution, the residual/energy audit, and cross-backend agreement.
fn run_direct_case<B, C>(label: &str, build: B, check: C)
where
    B: Fn() -> BeamModel,
    C: Fn(&BeamSolver, &BeamModel),
{
    let mut reference: Option<(Vec<f64>, Vec<f64>)> = None;
    for &backend in &DIRECT {
        let model = build();
        let solver = solve_backend(&model, backend);

        // (a) analytical checks for this case
        check(&solver, &model);

        // residual + energy audit (independent assembly)
        let a = audit(&model, &solver);
        assert!(
            a.free_res <= 1e-6 * a.range,
            "{} / {}: free-DOF residual {:.3e}",
            label,
            backend,
            a.free_res
        );
        assert!(
            a.reaction_res <= 1e-6 * a.range,
            "{} / {}: support residual vs reaction {:.3e}",
            label,
            backend,
            a.reaction_res
        );
        assert!(
            a.energy_rel <= 1e-9,
            "{} / {}: energy mismatch {:.3e}",
            label,
            backend,
            a.energy_rel
        );

        // (b) cross-solver agreement (dense is the reference)
        let u = solver.displacements().to_vec();
        let rr = solver.reactions();
        let (du, dr) = match &reference {
            None => {
                reference = Some((u.clone(), rr.clone()));
                (0.0, 0.0)
            }
            Some((ur, ref_r)) => {
                let mut du = 0.0f64;
                let mut dr = 0.0f64;
                for i in 0..u.len() {
                    assert_pair(u[i], ur[i], 1e-12, 1e-9, &format!("{} disp[{}]", label, i));
                    du = du.max((u[i] - ur[i]).abs());
                }
                for i in 0..rr.len() {
                    assert_pair(
                        rr[i],
                        ref_r[i],
                        1e-9,
                        1e-9,
                        &format!("{} react[{}]", label, i),
                    );
                    dr = dr.max((rr[i] - ref_r[i]).abs());
                }
                (du, dr)
            }
        };

        println!(
            "[{:14} / {:11}] free_res={:.2e} react_res={:.2e} energy_rel={:.2e} cross_dU={:.2e} cross_dR={:.2e}",
            label, backend, a.free_res, a.reaction_res, a.energy_rel, du, dr
        );
    }
}

// ===========================================================================
// A. Axial bar
// ===========================================================================

#[test]
fn test_backend_axial_bar() {
    let (l, e, a, i, p) = (1.0, E0, A0, i0(), 1000.0);
    let build = || {
        let mut m = BeamModel::new();
        m.add_node(BeamNode::new(0, 0.0, 0.0));
        m.add_node(BeamNode::new(1, l, 0.0));
        m.add_element(BeamElement::new(0, 1, mat(e), BeamSection::new(a, i)).unwrap());
        m.fix_node(0);
        m.add_nodal_force(1, 0, p);
        m
    };
    run_direct_case("axial", build, |s, _| {
        let ea = e * a;
        assert_pair(s.displacement(1, 0), p * l / ea, 1e-15, 1e-10, "u_tip");
        assert_pair(s.reactions()[0], -p, 1e-9, 1e-10, "Rx");
        assert_pair(
            s.element_section_forces(0, 0.5).unwrap().axial,
            p,
            1e-12,
            1e-10,
            "N",
        );
    });
}

// ===========================================================================
// B. Cantilever tip transverse force
// ===========================================================================

#[test]
fn test_backend_cantilever_tip_force() {
    let (l, e, i, p) = (1.0, E0, i0(), 1000.0);
    let ei = e * i;
    let build = || {
        let mut m = chain(1, l, e, A0, i);
        m.add_nodal_force(1, 1, -p);
        m
    };
    run_direct_case("tip force", build, |s, _| {
        assert_pair(
            s.displacement(1, 1),
            -an_tip_disp(p, l, ei),
            1e-15,
            1e-9,
            "uy",
        );
        assert_pair(
            s.displacement(1, 2),
            -an_tip_rot(p, l, ei),
            1e-15,
            1e-9,
            "rz",
        );
        assert_pair(s.reactions()[1], p, 1e-9, 1e-10, "Ry");
        assert_pair(s.reactions()[2], p * l, 1e-9, 1e-10, "Rz");
        for &xi in &[0.0, 0.5, 1.0] {
            let sf = s.element_section_forces(0, xi).unwrap();
            assert_pair(sf.shear, p, 1e-9, 1e-9, "V");
            assert_pair(sf.moment, -p * (l - xi * l), 1e-9, 1e-9, "M");
        }
    });
}

// ===========================================================================
// C. Cantilever tip moment
// ===========================================================================

#[test]
fn test_backend_cantilever_tip_moment() {
    let (l, e, i, m0) = (1.0, E0, i0(), 1000.0);
    let ei = e * i;
    let build = || {
        let mut m = chain(1, l, e, A0, i);
        m.add_applied_moment(1, m0).unwrap();
        m
    };
    run_direct_case("tip moment", build, |s, _| {
        assert_pair(s.displacement(1, 2), m0 * l / ei, 1e-15, 1e-9, "rz");
        assert_pair(
            s.displacement(1, 1),
            m0 * l * l / (2.0 * ei),
            1e-15,
            1e-9,
            "uy",
        );
        assert_pair(s.reactions()[2], -m0, 1e-9, 1e-10, "Rz");
        for &xi in &[0.0, 0.5, 1.0] {
            assert_pair(
                s.element_section_forces(0, xi).unwrap().moment,
                m0,
                1e-10,
                1e-10,
                "M",
            );
        }
    });
}

// ===========================================================================
// D. Uniform distributed load
// ===========================================================================

#[test]
fn test_backend_cantilever_udl() {
    let (l, e, i, q) = (1.0, E0, i0(), 1000.0);
    let ei = e * i;
    let build = || {
        let mut m = chain(1, l, e, A0, i);
        m.add_distributed_load(0, 0.0, -q).unwrap();
        m
    };
    run_direct_case("udl", build, |s, _| {
        assert_pair(s.reactions()[1], q * l, 1e-9, 1e-10, "Ry");
        assert_pair(s.reactions()[2], q * l * l / 2.0, 1e-9, 1e-10, "Rz");
        assert_pair(
            s.displacement(1, 1),
            -an_udl_disp(q, l, ei),
            1e-15,
            1e-9,
            "uy",
        );
        assert_pair(
            s.displacement(1, 2),
            -an_udl_rot(q, l, ei),
            1e-15,
            1e-9,
            "rz",
        );
        for &xi in &[0.0, 0.25, 0.5, 0.75, 1.0] {
            let sf = s.element_section_forces(0, xi).unwrap();
            assert_pair(sf.shear, q * (l - xi * l), 1e-9, 1e-8, "V");
            assert_pair(sf.moment, -q * (l - xi * l).powi(2) / 2.0, 1e-9, 1e-8, "M");
        }
    });
}

// ===========================================================================
// E. Combined loading
// ===========================================================================

#[test]
fn test_backend_combined_loading() {
    let (l, e, i) = (1.0, E0, i0());
    let (f_ax, p, m0, q) = (500.0, 1000.0, 800.0, 400.0);
    let build = || {
        let mut m = chain(1, l, e, A0, i);
        m.add_nodal_force(1, 0, f_ax);
        m.add_nodal_force(1, 1, -p);
        m.add_applied_moment(1, m0).unwrap();
        m.add_distributed_load(0, 0.0, -q).unwrap();
        m
    };
    run_direct_case("combined", build, |s, _| {
        let r = s.reactions();
        // Global equilibrium (analytical, about node 0).
        assert_pair(r[0] + f_ax, 0.0, 1e-9, 1e-9, "ΣFx");
        assert_pair(r[1] - p - q * l, 0.0, 1e-9, 1e-9, "ΣFy");
        assert_pair(
            r[2] + m0 - p * l - q * l * (l / 2.0),
            0.0,
            1e-9,
            1e-9,
            "ΣMz",
        );
        // Superposition of internal forces.
        let sf = s.element_section_forces(0, 0.5).unwrap();
        let n_an = f_ax;
        let v_an = p + q * (l - 0.5 * l); // tip shear + UDL shear
        let m_an = m0 - p * (l - 0.5 * l) - q * (l - 0.5 * l).powi(2) / 2.0;
        assert_pair(sf.axial, n_an, 1e-9, 1e-9, "N");
        assert_pair(sf.shear, v_an, 1e-9, 1e-9, "V");
        assert_pair(sf.moment, m_an, 1e-9, 1e-9, "M");
    });
}

// ===========================================================================
// Scaling audit across direct backends (E and load)
// ===========================================================================

#[test]
fn test_backend_scaling_audit() {
    let (l, i) = (1.0, i0());
    let p = 1000.0;

    let build = |e: f64, scale: f64| {
        let mut m = chain(1, l, e, A0, i);
        m.add_nodal_force(1, 1, -p * scale);
        m
    };

    for &backend in &DIRECT {
        // Reference E0, load 1.
        let s_ref = solve_backend(&build(E0, 1.0), backend);
        let uy_ref = s_ref.displacement(1, 1);
        let ry_ref = s_ref.reactions()[1];

        for &ef in &[0.1, 1.0, 10.0] {
            for &lf in &[0.1, 1.0, 10.0] {
                let s = solve_backend(&build(E0 * ef, lf), backend);
                let uy = s.displacement(1, 1);
                let ry = s.reactions()[1];
                // u ∝ load / E ; reactions ∝ load, independent of E.
                let expected_u = uy_ref * (lf / ef);
                assert_pair(
                    uy,
                    expected_u,
                    1e-18,
                    1e-9,
                    &format!("{} uy scaling", backend),
                );
                assert_pair(
                    ry,
                    ry_ref * lf,
                    1e-9,
                    1e-9,
                    &format!("{} Ry scaling", backend),
                );
                // Section moment scales with load, not with E.
                let m_an = -p * lf * (l - 0.5 * l);
                assert_pair(
                    s.element_section_forces(0, 0.5).unwrap().moment,
                    m_an,
                    1e-8,
                    1e-9,
                    &format!("{} M scaling", backend),
                );
            }
        }
    }
    println!("[scaling] E x0.1/x1/x10 and load x0.1/x1/x10 verified for all direct backends");
}

// ===========================================================================
// Rotation invariance across direct backends
// ===========================================================================

#[test]
fn test_backend_rotation_invariance() {
    let (l, e, i, p) = (1.0, E0, i0(), 1000.0);
    let ei = e * i;

    for &backend in &DIRECT {
        let mut local_ref: Option<(f64, f64, f64)> = None;
        for &deg in &[0.0_f64, 45.0, 90.0] {
            let th = deg.to_radians();
            let (c, s) = (th.cos(), th.sin());
            let mut m = BeamModel::new();
            m.add_node(BeamNode::new(0, 0.0, 0.0));
            m.add_node(BeamNode::new(1, l * c, l * s));
            m.add_element(BeamElement::new(0, 1, mat(e), BeamSection::new(A0, i)).unwrap());
            m.fix_node(0);
            // Global force equivalent to local (0, -P): f_global = Tᵀ f_local.
            m.add_nodal_force(1, 0, s * p);
            m.add_nodal_force(1, 1, -c * p);
            let solver = solve_backend(&m, backend);

            // Local tip deflection uy_l = -s·ux_g + c·uy_g.
            let uy_l = -s * solver.displacement(1, 0) + c * solver.displacement(1, 1);
            assert_pair(
                uy_l,
                -an_tip_disp(p, l, ei),
                1e-12,
                1e-9,
                &format!("{} local uy @{}", backend, deg),
            );

            let r = solver.reactions();
            let rmag = (r[0] * r[0] + r[1] * r[1]).sqrt();
            assert_pair(rmag, p, 1e-6, 1e-9, &format!("{} |R| @{}", backend, deg));
            assert_pair(r[2], p * l, 1e-6, 1e-9, &format!("{} Rz @{}", backend, deg));

            let sf = solver.element_section_forces(0, 0.5).unwrap();
            let key = (sf.axial, sf.shear, sf.moment);
            match local_ref {
                None => local_ref = Some(key),
                Some(k) => {
                    assert_pair(key.0, k.0, 1e-9, 1e-9, "N invariant");
                    assert_pair(key.1, k.1, 1e-9, 1e-9, "V invariant");
                    assert_pair(key.2, k.2, 1e-9, 1e-9, "M invariant");
                }
            }
        }
    }
    println!("[rotation] 0/45/90 deg invariant for all direct backends");
}

// ===========================================================================
// Unequal element lengths across direct backends
// ===========================================================================

#[test]
fn test_backend_unequal_element_lengths() {
    let (e, i, p) = (E0, i0(), 1000.0);
    let ei = e * i;
    let l_total = 6.0;

    let build = || {
        let mut m = BeamModel::new();
        m.add_node(BeamNode::new(0, 0.0, 0.0));
        m.add_node(BeamNode::new(1, 1.0, 0.0));
        m.add_node(BeamNode::new(2, 3.0, 0.0));
        m.add_node(BeamNode::new(3, 6.0, 0.0));
        m.add_element(BeamElement::new(0, 1, mat(e), BeamSection::new(A0, i)).unwrap());
        m.add_element(BeamElement::new(1, 2, mat(e), BeamSection::new(A0, i)).unwrap());
        m.add_element(BeamElement::new(2, 3, mat(e), BeamSection::new(A0, i)).unwrap());
        m.fix_node(0);
        m.add_nodal_force(3, 1, -p);
        m
    };

    run_direct_case("unequal mesh", build, |s, _| {
        // Phase 2.5 global x sampling is solver-independent.
        let samples = s.sample_beam_forces(3).unwrap();
        let expected_x = [0.0, 0.5, 1.0, 1.0, 2.0, 3.0, 3.0, 4.5, 6.0];
        for (sm, xe) in samples.iter().zip(expected_x.iter()) {
            assert_pair(sm.x, *xe, 1e-12, 1e-12, "global x");
        }
        assert_pair(s.reactions()[1], p, 1e-9, 1e-10, "Ry");
        assert_pair(s.reactions()[2], p * l_total, 1e-9, 1e-10, "Rz");
        assert_pair(
            s.displacement(3, 1),
            -an_tip_disp(p, l_total, ei),
            1e-15,
            1e-9,
            "tip uy",
        );
        for (elem, x) in [(0usize, 0.5), (1, 2.0), (2, 4.5)] {
            let dx = [1.0, 2.0, 3.0][elem];
            let x0 = [0.0, 1.0, 3.0][elem];
            let xi = (x - x0) / dx;
            let sf = s.element_section_forces(elem, xi).unwrap();
            assert_pair(sf.shear, p, 1e-9, 1e-9, "V");
            assert_pair(sf.moment, -p * (l_total - x), 1e-9, 1e-9, "M");
        }
    });
}

// ===========================================================================
// Result API consistency across backends
// ===========================================================================

#[test]
fn test_backend_result_api_consistency() {
    let build = || {
        let mut m = chain(3, 1.0, E0, A0, i0());
        m.add_nodal_force(3, 1, -1000.0);
        m.add_distributed_load(1, 0.0, -500.0).unwrap();
        m.add_point_moment(2, 0.5, 400.0).unwrap();
        m
    };

    for &backend in &DIRECT {
        let model = build();
        let solver = solve_backend(&model, backend);
        let r = solver.results();

        assert_eq!(r.displacements().len(), solver.displacements().len());
        for node in 0..r.n_nodes() {
            assert_eq!(
                r.displacement(node).unwrap().uy,
                solver.displacement(node, 1)
            );
        }
        let raw = solver.reactions();
        assert_pair(r.reaction(0).unwrap().fy, raw[1], 1e-12, 1e-12, "reaction");

        let direct_end = solver.element_end_forces().unwrap();
        for (elem, de) in direct_end.iter().enumerate() {
            let a = r.element_end_forces(elem).unwrap();
            for (k, (av, dv)) in a.iter().zip(de.iter()).enumerate() {
                assert_eq!(*av, *dv, "end force {}", k);
            }
        }
        for elem in 0..r.n_elements() {
            for &xi in &[0.0, 0.5, 1.0] {
                assert_eq!(
                    r.section_forces(elem, xi).unwrap(),
                    solver.element_section_forces(elem, xi).unwrap()
                );
            }
        }
        let a = r.sample_forces(4).unwrap();
        let b = solver.sample_beam_forces(4).unwrap();
        assert_eq!(a.len(), b.len());
        for (s1, s2) in a.iter().zip(b.iter()) {
            assert_eq!(s1.element_index, s2.element_index);
            assert_eq!(s1.xi, s2.xi);
            assert_eq!(s1.x, s2.x);
            assert_eq!(s1.section_forces, s2.section_forces);
        }
        let d = r.diagram(4).unwrap();
        assert_eq!(d.samples.len(), b.len());
    }
    println!("[result API] delegation consistent for all direct backends");
}

// ===========================================================================
// Iterative backends
// ===========================================================================

/// CG is applicable to these SPD condensed systems; validate it with explicit
/// tolerances and residual checks (it is iterative, so tolerances are slightly
/// looser than the direct solvers).
#[test]
fn test_backend_iterative_cg_applicable() {
    let (l, e, i) = (1.0, E0, i0());
    let cases: Vec<(&str, BeamModel)> = {
        let mut v = Vec::new();
        let mut a = chain(1, l, e, A0, i);
        a.add_nodal_force(1, 0, 1000.0);
        v.push(("axial", a));
        let mut b = chain(1, l, e, A0, i);
        b.add_nodal_force(1, 1, -1000.0);
        v.push(("tip force", b));
        let mut c = chain(1, l, e, A0, i);
        c.add_applied_moment(1, 1000.0).unwrap();
        v.push(("tip moment", c));
        let mut d = chain(1, l, e, A0, i);
        d.add_distributed_load(0, 0.0, -1000.0).unwrap();
        v.push(("udl", d));
        let mut e2 = chain(1, l, e, A0, i);
        e2.add_nodal_force(1, 0, 500.0);
        e2.add_nodal_force(1, 1, -1000.0);
        e2.add_applied_moment(1, 800.0).unwrap();
        e2.add_distributed_load(0, 0.0, -400.0).unwrap();
        v.push(("combined", e2));
        v
    };

    for (label, model) in &cases {
        let solver = solve_backend(model, "cg");
        let a = audit(model, &solver);
        assert!(
            a.free_res <= 1e-6 * a.range,
            "{} CG free residual {}",
            label,
            a.free_res
        );
        assert!(a.energy_rel <= 1e-9, "{} CG energy {}", label, a.energy_rel);
        println!(
            "[cg / {:9}] free_res={:.2e} energy_rel={:.2e}",
            label, a.free_res, a.energy_rel
        );
    }
    // Spot-check against the dense reference for a non-trivial case.
    let model = &cases[1].1;
    let cg = solve_backend(model, "cg");
    let dense = solve_backend(model, "dense");
    assert_pair(
        cg.displacement(1, 1),
        dense.displacement(1, 1),
        1e-12,
        1e-8,
        "cg vs dense uy",
    );
    assert_pair(
        cg.reactions()[1],
        dense.reactions()[1],
        1e-6,
        1e-8,
        "cg vs dense Ry",
    );
}

/// ICCG (`iccg`) is an optional iterative backend. It may fail on some beam
/// stiffness matrices (its IC(0) factorisation is not always applicable); when
/// it does, it must report an explicit solver error rather than a silently
/// wrong result. This test does **not** require ICCG to fail: every successful
/// result is validated against the dense reference plus the independent
/// residual/energy audit, and every failure must be an explicit
/// [`FemError::SolverError`]. The test stays valid if ICCG improves and all
/// cases begin to pass.
#[test]
fn test_backend_iccg_optional() {
    let (l, e, i) = (1.0, E0, i0());
    let mk = |kind: u32| {
        let mut m = chain(1, l, e, A0, i);
        match kind {
            0 => m.add_nodal_force(1, 0, 1000.0),
            1 => m.add_nodal_force(1, 1, -1000.0),
            2 => m.add_applied_moment(1, 1000.0).unwrap(),
            3 => m.add_distributed_load(0, 0.0, -1000.0).unwrap(),
            _ => panic!(),
        }
        m
    };

    let registry = SolverRegistry::default();
    let mut ok_count = 0;
    let mut err_count = 0;
    for kind in 0..4u32 {
        let model = mk(kind);
        let reference = solve_backend(&model, "dense");
        let mut solver = BeamSolver::from_model(&model).unwrap();
        let mut linear = registry.create("iccg").unwrap();
        match solver.solve(&mut *linear) {
            Ok(()) => {
                ok_count += 1;
                // A successful result is held to the same correctness bar.
                assert_pair(
                    solver.displacement(1, 1),
                    reference.displacement(1, 1),
                    1e-8,
                    1e-6,
                    "iccg vs dense uy",
                );
                assert_pair(
                    solver.reactions()[1],
                    reference.reactions()[1],
                    1e-4,
                    1e-6,
                    "iccg vs dense Ry",
                );
                let a = audit(&model, &solver);
                assert!(
                    a.free_res <= 1e-6 * a.range,
                    "iccg free-DOF residual {:.3e}",
                    a.free_res
                );
                assert!(
                    a.energy_rel <= 1e-9,
                    "iccg energy mismatch {:.3e}",
                    a.energy_rel
                );
            }
            Err(FemError::SolverError(_)) => err_count += 1,
            Err(other) => panic!("iccg returned an unexpected non-solver error: {:?}", other),
        }
    }
    println!(
        "[iccg] {}/4 validated successes, {}/4 explicit solver failures (no silently-wrong results)",
        ok_count, err_count
    );
}
