//! Beam FEM multi-solver cross-validation.
//!
//! Verifies that the unified `LinearSolver` backends produce consistent Beam FEM
//! physics, that reactions / element end forces obey the documented conventions,
//! and that the solver API contract (`factor` / `solve` / `solve_many`, error
//! propagation) is used correctly.
//!
//! Conventions under test (unchanged from the existing implementation):
//! * global DOFs `[ux, uy, rz]`, `rz` CCW positive;
//! * distributed loads are LOCAL (`qy` up +), point loads LOCAL, applied
//!   moments GLOBAL `θ` loads;
//! * `element_end_forces()` = element-on-node forces `f_equiv − K_local·u_local`;
//! * `reactions()` = `K_original·u − f_global` (external support reaction).

use section_properties::beam_fem::{
    BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver, FemError,
};
use section_properties::fea::SparseMatrix;
use section_properties::material::Material;
use section_properties::{SolverError, SolverRegistry, SolverSelection};

const DIRECT: [&str; 3] = ["dense", "skyline_ldlt", "sparse_lu"];

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn assert_close(a: f64, b: f64, abs: f64, rel: f64, label: &str) {
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

fn steel() -> Material {
    Material::new(200e9, 0.3, 7850.0, "Steel")
}

fn section() -> BeamSection {
    BeamSection::new(0.02, 0.1 * 0.2_f64.powi(3) / 12.0)
}

/// Horizontal cantilever of `n_elem` equal elements, fixed at node 0.
fn cantilever(n_elem: usize, length: f64) -> BeamModel {
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

fn solve_with(model: &BeamModel, name: &str) -> BeamSolver {
    let mut s = BeamSolver::from_model(model)
        .unwrap_or_else(|e| panic!("{}: from_model failed: {:?}", name, e));
    s.set_solver(SolverSelection::named(name));
    s.solve_configured()
        .unwrap_or_else(|e| panic!("{}: solve failed: {:?}", name, e));
    assert_eq!(s.solver_name(), Some(name), "{}: reported backend", name);
    s
}

/// Collected physics of a solved beam model.
struct Res {
    u: Vec<f64>,
    reactions: Vec<f64>,
    end_local: Vec<[f64; 6]>,
    end_global: Vec<[f64; 6]>,
}

fn collect(s: &BeamSolver) -> Res {
    Res {
        u: s.displacements().to_vec(),
        reactions: s.reactions(),
        end_local: s.element_end_forces().unwrap(),
        end_global: s.element_end_forces_global().unwrap(),
    }
}

fn compare(a: &Res, b: &Res, label: &str) {
    assert_eq!(a.u.len(), b.u.len());
    for i in 0..a.u.len() {
        assert_close(a.u[i], b.u[i], 1e-12, 1e-9, &format!("{} u[{}]", label, i));
    }
    for i in 0..a.reactions.len() {
        assert_close(
            a.reactions[i],
            b.reactions[i],
            1e-6,
            1e-9,
            &format!("{} R[{}]", label, i),
        );
    }
    for e in 0..a.end_local.len() {
        for k in 0..6 {
            assert_close(
                a.end_local[e][k],
                b.end_local[e][k],
                1e-6,
                1e-9,
                &format!("{} feL[{}][{}]", label, e, k),
            );
            assert_close(
                a.end_global[e][k],
                b.end_global[e][k],
                1e-6,
                1e-9,
                &format!("{} feG[{}][{}]", label, e, k),
            );
        }
    }
}

/// Solve the same model with every direct backend and assert agreement against
/// the first (dense). Returns the reference `Res`.
fn cross_validate(model: &BeamModel, label: &str) -> Res {
    let reference = collect(&solve_with(model, DIRECT[0]));
    for &name in &DIRECT[1..] {
        let other = collect(&solve_with(model, name));
        compare(&reference, &other, &format!("{} / {}", label, name));
    }
    reference
}

/// Local applied-load resultant on one element: `(fx, fy, m about node_i)`.
fn element_load_resultant(model: &BeamModel, elem: usize, length: f64) -> (f64, f64, f64) {
    let mut fx = 0.0;
    let mut fy = 0.0;
    let mut m = 0.0;
    for dl in &model.distributed_loads {
        if dl.element_idx == elem {
            let fy_tot = dl.qy * length;
            fx += dl.qx * length;
            fy += fy_tot;
            m += fy_tot * (length / 2.0); // transverse resultant at midspan
        }
    }
    for pl in &model.point_loads {
        if pl.element_idx == elem {
            let xp = pl.position * length;
            fx += pl.fx;
            fy += pl.fy;
            m += pl.fy * xp + pl.mz;
        }
    }
    (fx, fy, m)
}

// ===========================================================================
// Test A — Cantilever + transverse UDL
// ===========================================================================

#[test]
fn test_a_cantilever_udl() {
    let l = 1.0;
    let q = 1000.0;
    let e = 200e9;
    let i = 0.1 * 0.2_f64.powi(3) / 12.0;
    let mut model = cantilever(2, l);
    model.add_distributed_load(0, 0.0, -q).unwrap();
    model.add_distributed_load(1, 0.0, -q).unwrap();

    let r = cross_validate(&model, "A/udl");

    // Analytical reactions (ΣFy = 0, ΣMz = 0).
    let ry = r.reactions[1];
    let mz = r.reactions[2];
    assert_close(ry, q * l, 1e-9, 1e-10, "A Ry");
    assert_close(mz, q * l * l / 2.0, 1e-9, 1e-10, "A Rz");
    assert_close(ry + (-q * l), 0.0, 1e-6, 1e-9, "A ΣFy");
    assert_close(mz + (-q * l * l / 2.0), 0.0, 1e-6, 1e-9, "A ΣMz");

    // Tip DOF of node 2 (index 2*3+1 = 7).
    let tip_uy = r.u[7];
    assert_close(
        tip_uy,
        -q * l.powi(4) / (8.0 * e * i),
        1e-15,
        1e-9,
        "A tip uy",
    );
    println!(
        "[A udl] δ_tip={:.6e} Ry={:.6e} Rz={:.6e} (3 backends agree)",
        tip_uy, ry, mz
    );
}

// ===========================================================================
// Test B — Cantilever + tip point load
// ===========================================================================

#[test]
fn test_b_cantilever_point_load() {
    let l = 1.0;
    let p = 1000.0;
    let mut model = cantilever(1, l);
    model.add_nodal_force(1, 1, -p); // tip transverse point load

    let r = cross_validate(&model, "B/point");

    assert_close(
        r.u[4],
        -p * l.powi(3) / (3.0 * 200e9 * 0.1 * 0.2_f64.powi(3) / 12.0),
        1e-15,
        1e-9,
        "B tip uy",
    );
    assert_close(r.reactions[1], p, 1e-9, 1e-10, "B Ry");
    assert_close(r.reactions[2], p * l, 1e-9, 1e-10, "B Rz");
    // Column vectors describing the tip load.
    println!(
        "[B point] δ_tip={:.6e} Ry={:.6e} Rz={:.6e}",
        r.u[4], r.reactions[1], r.reactions[2]
    );
}

// ===========================================================================
// Test C — Cantilever + applied tip moment (contract: M_j = -M)
// ===========================================================================

#[test]
fn test_c_cantilever_tip_moment() {
    let l = 1.0;
    let m = 1000.0;
    let e = 200e9;
    let i = 0.1 * 0.2_f64.powi(3) / 12.0;
    let mut model = cantilever(1, l);
    model.add_applied_moment(1, m).unwrap();

    let r = cross_validate(&model, "C/moment");

    // Tip rotation = M·L/(EI); reaction moment balances the applied moment.
    assert_close(r.reactions[2], -m, 1e-9, 1e-10, "C Rz = -M");
    assert_close(r.u[5], m * l / (e * i), 1e-15, 1e-9, "C tip rotation");
    assert_close(r.reactions[2] + m, 0.0, 1e-6, 1e-9, "C ΣMz");

    // Fixed contract from the applied-moment handling: the free-end element
    // on-node moment is `-M` (and the other end is `+M`).
    assert_close(r.end_local[0][5], -m, 1e-9, 1e-10, "C M_j = -M");
    assert_close(r.end_local[0][2], m, 1e-9, 1e-10, "C M_i = +M");

    // Section-force convention: the internal moment is constant +M along the
    // element (the applied tip moment is carried by a constant sagging moment),
    // so M(0) = M(L) = +M.
    let s = solve_with(&model, "dense");
    assert_close(
        s.element_section_forces(0, 1.0).unwrap().moment,
        m,
        1e-9,
        1e-10,
        "C M(L) = +M",
    );
    assert_close(
        s.element_section_forces(0, 0.0).unwrap().moment,
        m,
        1e-9,
        1e-10,
        "C M(0) = +M",
    );

    println!(
        "[C moment] rz={:.6e} Rz={:.6e} M_j={:.6e} (M_j = -M preserved)",
        r.u[5], r.reactions[2], r.end_local[0][5]
    );
}

// ===========================================================================
// Test D — Combined loading
// ===========================================================================

#[test]
fn test_d_combined_loading() {
    // 2-element cantilever, L = 1.
    let mut model = cantilever(2, 1.0); // 2 elements of length 0.5 (total 1.0)
    model.add_distributed_load(0, 200.0, -800.0).unwrap(); // axial + transverse UDL
    model.add_point_load(1, 0.5, 300.0, -500.0, 200.0).unwrap(); // interior point load on elem 1
    model.add_applied_moment(1, 400.0).unwrap(); // nodal applied moment
    model.add_nodal_force(2, 1, -1000.0); // tip transverse force

    let r = cross_validate(&model, "D/combined");

    // Analytical global equilibrium about node 0.
    let rx = r.reactions[0];
    let ry = r.reactions[1];
    let rz = r.reactions[2];
    assert_close(rx + (200.0 * 0.5 + 300.0), 0.0, 1e-6, 1e-9, "D ΣFx");
    assert_close(
        ry + (-800.0 * 0.5 + -500.0 + -1000.0),
        0.0,
        1e-6,
        1e-9,
        "D ΣFy",
    );
    // ΣMz: applied +400; qy resultant -400 at x=0.25; point fy -500 at x=0.75
    // and mz +200; tip force -1000 at x=1.
    assert_close(
        rz + 400.0 + (-400.0 * 0.25) + (-500.0 * 0.75) + 200.0 + (-1000.0 * 1.0),
        0.0,
        1e-6,
        1e-9,
        "D ΣMz",
    );
    println!(
        "[D combined] Rx={:.4e} Ry={:.4e} Rz={:.4e} (all equilibria residual < 1e-6)",
        rx, ry, rz
    );
}

// ===========================================================================
// Test E — Rotated beam (0° / 45° / 90°)
// ===========================================================================

#[test]
fn test_e_rotated_beam() {
    let l = 1.0;
    let p = 1000.0;

    let mut local_ref: Option<Vec<[f64; 6]>> = None;
    for &deg in &[0.0_f64, 45.0, 90.0] {
        let th = deg.to_radians();
        let (c, s) = (th.cos(), th.sin());

        let mut model = BeamModel::new();
        model.add_node(BeamNode::new(0, 0.0, 0.0));
        model.add_node(BeamNode::new(1, l * c, l * s));
        model.add_element(BeamElement::new(0, 1, steel(), section()).unwrap());
        model.fix_node(0);
        // Global tip force equivalent to LOCAL (0, -P).
        model.add_nodal_force(1, 0, s * p);
        model.add_nodal_force(1, 1, -c * p);

        let r = cross_validate(&model, &format!("E/{:.0}deg", deg));

        // Reaction magnitude invariant; moment = P·L about z.
        let rmag = (r.reactions[0].powi(2) + r.reactions[1].powi(2)).sqrt();
        assert_close(rmag, p, 1e-9, 1e-9, &format!("E {:.0} |R|", deg));
        assert_close(
            r.reactions[2],
            p * l,
            1e-9,
            1e-9,
            &format!("E {:.0} Rz", deg),
        );

        // Local end forces are rotation-invariant.
        match &local_ref {
            None => local_ref = Some(r.end_local.clone()),
            Some(refv) => {
                for (e, fe) in r.end_local.iter().enumerate() {
                    for k in 0..6 {
                        assert_close(
                            fe[k],
                            refv[e][k],
                            1e-6,
                            1e-9,
                            &format!("E {:.0} local end force", deg),
                        );
                    }
                }
            }
        }

        // Global equilibrium with the applied tip force (s·P, -c·P):
        // ΣFx = Rx + sP = 0, ΣFy = Ry - cP = 0.
        assert_close(
            r.reactions[0] + s * p,
            0.0,
            1e-6,
            1e-9,
            &format!("E {:.0} ΣFx", deg),
        );
        assert_close(
            r.reactions[1] - c * p,
            0.0,
            1e-6,
            1e-9,
            &format!("E {:.0} ΣFy", deg),
        );

        // Local -> global transformation consistency: f_global = T^T · f_local.
        let node_i = model.nodes[0].point();
        let node_j = model.nodes[1].point();
        let t = model.elements[0].transformation_matrix(node_i, node_j);
        let mut g = [0.0f64; 6];
        for (j, tj) in t.iter().enumerate() {
            for (i, gv) in g.iter_mut().enumerate() {
                *gv += tj[i] * r.end_local[0][j];
            }
        }
        for (i, &gv) in g.iter().enumerate() {
            assert_close(
                gv,
                r.end_global[0][i],
                1e-9,
                1e-9,
                &format!("E {:.0} f_global = T^T f_local [{}]", deg, i),
            );
        }
        println!(
            "[E {:.0}deg] |R|={:.6e} Rz={:.6e} local end forces rotation-invariant",
            deg, rmag, r.reactions[2]
        );
    }
}

// ===========================================================================
// Phase 3 — end-force recovery vs equivalent applied loads
// ===========================================================================

#[test]
fn test_element_end_force_equilibrium_with_loads() {
    // Model with distributed + point loads (and an applied moment that must NOT
    // be treated as an element load).
    let mut model = cantilever(2, 1.0); // 2 elements of length 0.5 (total 1.0)
    model.add_distributed_load(0, 200.0, -800.0).unwrap();
    model.add_point_load(1, 0.5, 300.0, -500.0, 200.0).unwrap();
    model.add_applied_moment(1, 400.0).unwrap();
    model.add_nodal_force(2, 1, -1000.0);

    let s = solve_with(&model, "sparse_lu");
    let end = s.element_end_forces().unwrap();
    let lengths = [0.5, 0.5];

    for elem in 0..2 {
        let fe = end[elem];
        let l = lengths[elem];
        // Element-on-node end forces, summed: forces and moment about node i.
        let sum_fx = fe[0] + fe[3];
        let sum_fy = fe[1] + fe[4];
        let sum_m = fe[2] + fe[5] + fe[4] * l;

        let (lfx, lfy, lm) = element_load_resultant(&model, elem, l);
        assert_close(sum_fx, lfx, 1e-6, 1e-9, &format!("elem{} ΣFx = load", elem));
        assert_close(sum_fy, lfy, 1e-6, 1e-9, &format!("elem{} ΣFy = load", elem));
        assert_close(sum_m, lm, 1e-6, 1e-9, &format!("elem{} ΣM = load", elem));

        println!(
            "[elem {}] end-force resultant=({:.4e},{:.4e},{:.4e}) load resultant=({:.4e},{:.4e},{:.4e})",
            elem, sum_fx, sum_fy, sum_m, lfx, lfy, lm
        );
    }

    // The element carrying no span load (elem 1 has a point load; both do here)
    // — additionally verify that the applied nodal moment does NOT leak into any
    // element's equivalent load: it is a global external load only.
    // elem 0 has only the distributed load:
    let (_, lfy0, lm0) = element_load_resultant(&model, 0, 0.5);
    assert_close(lfy0, -800.0 * 0.5, 1e-9, 1e-12, "elem0 load fy");
    assert_close(lm0, (-800.0 * 0.5) * 0.25, 1e-9, 1e-12, "elem0 load m");
}

// ===========================================================================
// Phase 4 — global equilibrium: reactions and assembled end forces
// ===========================================================================

#[test]
fn test_global_equilibrium_force_and_moment() {
    let mut model = cantilever(2, 1.0); // 2 elements of length 0.5 (total 1.0)
    model.add_distributed_load(0, 200.0, -800.0).unwrap();
    model.add_point_load(1, 0.5, 300.0, -500.0, 200.0).unwrap();
    model.add_applied_moment(1, 400.0).unwrap();
    model.add_nodal_force(2, 1, -1000.0);

    let s = solve_with(&model, "sparse_lu");
    let r = collect(&s);

    // (1) Σ external + Σ reaction = 0 (computed above in Test D; re-assert here
    //     for a single backend).
    assert_close(
        r.reactions[0] + (200.0 * 0.5 + 300.0),
        0.0,
        1e-6,
        1e-9,
        "ΣFx",
    );
    assert_close(
        r.reactions[1] + (-800.0 * 0.5 + -500.0 + -1000.0),
        0.0,
        1e-6,
        1e-9,
        "ΣFy",
    );
    assert_close(
        r.reactions[2] + 400.0 + (-400.0 * 0.25) + (-375.0) + 200.0 + (-1000.0),
        0.0,
        1e-6,
        1e-9,
        "ΣMz",
    );

    // (2) Only the constrained DOFs carry a reaction; every free DOF (nodes 1
    //     and 2, DOF indices 3..9) must have a ~zero residual.
    for (k, &val) in r.reactions.iter().enumerate().skip(3) {
        assert_close(val, 0.0, 1e-6, 1e-9, &format!("free-DOF reaction R[{}]", k));
    }

    // (3) Assembled element-on-node end forces (global) satisfy the exact
    //     identity `Σ f_end + reactions + f_nodal_total = 0`, where
    //     `f_nodal_total` is the directly applied nodal load vector (nodal
    //     forces AND applied moments). Distributed/point loads live in the
    //     element equivalent loads and cancel through Σ f_end.
    let mut assembled = vec![0.0f64; r.reactions.len()];
    for (e, fe) in r.end_global.iter().enumerate() {
        let elem = &model.elements[e];
        let map = [
            model.dof_index(elem.node_i, 0),
            model.dof_index(elem.node_i, 1),
            model.dof_index(elem.node_i, 2),
            model.dof_index(elem.node_j, 0),
            model.dof_index(elem.node_j, 1),
            model.dof_index(elem.node_j, 2),
        ];
        for k in 0..6 {
            assembled[map[k]] += fe[k];
        }
    }
    let mut f_nodal_total = vec![0.0f64; r.reactions.len()];
    f_nodal_total[5] = 400.0; // applied moment at node 1 (rz DOF)
    f_nodal_total[7] = -1000.0; // tip nodal force at node 2 (uy DOF)
    for (k, &val) in r.reactions.iter().enumerate() {
        assert_close(
            assembled[k] + val + f_nodal_total[k],
            0.0,
            1e-6,
            1e-9,
            &format!("Σ f_end + R + f_nodal_total [{}]", k),
        );
    }

    println!(
        "[equilibrium] reactions=({:.4e},{:.4e},{:.4e}); ΣF/ΣM residual < 1e-6; free-DOF residual ≈ 0",
        r.reactions[0], r.reactions[1], r.reactions[2]
    );
}

// ===========================================================================
// Phase 5 — solver API contract
// ===========================================================================

#[test]
fn test_solve_many_matches_solve() {
    let registry = SolverRegistry::default();

    // Small SPD system.
    let mut a = SparseMatrix::new(3);
    a.add(0, 0, 4.0);
    a.add(0, 1, 1.0);
    a.add(1, 0, 1.0);
    a.add(1, 1, 4.0);
    a.add(1, 2, 1.0);
    a.add(2, 1, 1.0);
    a.add(2, 2, 4.0);
    a.compress();
    let b1 = vec![1.0, 2.0, 3.0];
    let b2 = vec![3.0, -1.0, 0.5];

    for &name in &DIRECT {
        let mut solver = registry.create(name).unwrap();
        solver.factor(&a).unwrap();

        // solve before factor is rejected (fresh solver).
        let fresh = registry.create(name).unwrap();
        assert!(
            matches!(fresh.solve(&b1), Err(SolverError::NotFactorized)),
            "{}: solve before factor must be NotFactorized",
            name
        );

        let x1 = solver.solve(&b1).unwrap();
        let x2 = solver.solve(&b2).unwrap();
        let many = solver.solve_many(&[b1.clone(), b2.clone()]).unwrap();
        assert_eq!(many.len(), 2);
        for (i, (&a_v, &b_v)) in many[0].iter().zip(x1.iter()).enumerate() {
            assert_close(a_v, b_v, 1e-12, 1e-12, &format!("{} many[0][{}]", name, i));
        }
        for (i, (&a_v, &b_v)) in many[1].iter().zip(x2.iter()).enumerate() {
            assert_close(a_v, b_v, 1e-12, 1e-12, &format!("{} many[1][{}]", name, i));
        }
        println!("[solve_many {:11}] matches per-RHS solve", name);
    }
}

#[test]
fn test_singular_beam_propagates_solver_error() {
    // Unsupported (free-free) beam: the condensed system is singular due to
    // rigid-body modes. The solver error must propagate as FemError::SolverError
    // and must not panic or be swallowed.
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, 1.0, 0.0));
    model.add_element(BeamElement::new(0, 1, steel(), section()).unwrap());
    model.add_nodal_force(1, 1, -1000.0);
    // No fix_node -> singular.

    let mut solver = BeamSolver::from_model(&model).unwrap();
    let err = solver
        .solve_configured()
        .expect_err("singular beam must not report success");
    assert!(
        matches!(err, FemError::SolverError(_)),
        "expected FemError::SolverError, got {:?}",
        err
    );
    assert_eq!(
        solver.solver_name(),
        None,
        "no backend may be reported after a failed solve"
    );
    println!("[singular beam] propagated: {:?}", err);
}

// ===========================================================================
// Element end-force recovery sign/behaviour regression (multi-solver)
// ===========================================================================

#[test]
fn test_end_force_recovery_consistent_across_solvers() {
    // A model where each load type is present, checking the end-force recovery
    // is identical across backends (already covered by cross_validate) AND that
    // the applied moment is not double-counted as an element load.
    let mut model = cantilever(1, 1.0);
    model.add_applied_moment(1, 500.0).unwrap();

    for &name in &DIRECT {
        let s = solve_with(&model, name);
        let fe = s.element_end_forces().unwrap()[0];
        // No element span load: the element-on-node force pair is a pure couple
        // that balances itself (+M_i, -M_j) and sums to zero.
        assert_close(fe[0], 0.0, 1e-9, 1e-12, "no axial");
        assert_close(fe[1], 0.0, 1e-9, 1e-12, "no shear");
        assert_close(fe[2], 500.0, 1e-9, 1e-10, "M_i = +M");
        assert_close(fe[5], -500.0, 1e-9, 1e-10, "M_j = -M");
        assert_close(fe[2] + fe[5], 0.0, 1e-9, 1e-12, "couple balances");
    }
    println!("[applied moment] not double-counted as an element load (all backends)");
}
