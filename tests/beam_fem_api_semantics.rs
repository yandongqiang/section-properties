//! Phase 4 — Beam FEM API semantics and boundary-condition audit.
//!
//! Focused on *valid* inputs: verifies the public API has correct, unambiguous
//! semantics for boundary conditions, loads, local/global transformation,
//! element end-force recovery and reactions. Pathological-input handling is
//! covered separately by `beam_fem_robustness.rs`.
//!
//! Conventions under audit (unchanged):
//! * DOFs per node `[ux, uy, rz]`, `rz` CCW positive (global); local x along
//!   node_i->node_j, local y up;
//! * distributed/point loads are LOCAL, applied moments are GLOBAL `theta` loads;
//! * `element_end_forces()` = `f_equiv - K_e u_e` (element-on-node);
//! * `reactions()` = `K_original u - f_global` (external support reaction).
//!
//! Tolerance policy: `|a-b| <= abs + rel*max(|a|,|b|)`; exact where exactness is
//! guaranteed (prescribed DOFs), abs 1e-9 elsewhere.

use section_properties::SolverSelection;
use section_properties::beam_fem::{BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver};
use section_properties::material::Material;

const DIRECT: [&str; 3] = ["dense", "skyline_ldlt", "sparse_lu"];
const L: f64 = 1.0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn mat() -> Material {
    Material::new(1.0, 0.3, 1.0, "U")
}
fn sec() -> BeamSection {
    BeamSection::new(1.0, 1.0)
}

/// Horizontal beam with `n` equal elements; no boundary conditions applied.
fn beam(n: usize) -> BeamModel {
    let mut m = BeamModel::new();
    let dx = L / n as f64;
    for k in 0..=n {
        m.add_node(BeamNode::new(k, k as f64 * dx, 0.0));
    }
    for k in 0..n {
        m.add_element(BeamElement::new(k, k + 1, mat(), sec()).unwrap());
    }
    m
}

fn solve_named(model: &BeamModel, name: &str) -> BeamSolver {
    let mut s = BeamSolver::from_model(model).unwrap();
    s.set_solver(SolverSelection::named(name));
    s.solve_configured()
        .unwrap_or_else(|e| panic!("{}: solve failed: {:?}", name, e));
    s
}

fn assert_mixed(a: f64, b: f64, abs: f64, rel: f64, label: &str) {
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

fn assert_finite(s: &BeamSolver) {
    for v in s.displacements() {
        assert!(v.is_finite());
    }
    for v in s.reactions() {
        assert!(v.is_finite());
    }
    for fe in s.element_end_forces().unwrap() {
        for v in fe {
            assert!(v.is_finite());
        }
    }
}

/// Independent re-derivation of the element end-force convention from public
/// APIs: `f_end = f_equiv - K_local · (T · u_global)`, where `f_equiv` sums only
/// distributed and point equivalent loads (applied moments excluded).
fn end_forces_from_principle(model: &BeamModel, solver: &BeamSolver, e: usize) -> [f64; 6] {
    let el = &model.elements[e];
    let ni = model.nodes[el.node_i].point();
    let nj = model.nodes[el.node_j].point();
    let map = [
        model.dof_index(el.node_i, 0),
        model.dof_index(el.node_i, 1),
        model.dof_index(el.node_i, 2),
        model.dof_index(el.node_j, 0),
        model.dof_index(el.node_j, 1),
        model.dof_index(el.node_j, 2),
    ];
    let u = solver.displacements();
    let ug = [
        u[map[0]], u[map[1]], u[map[2]], u[map[3]], u[map[4]], u[map[5]],
    ];
    let t = el.transformation_matrix(ni, nj);
    let mut ul = [0.0; 6];
    for i in 0..6 {
        for j in 0..6 {
            ul[i] += t[i][j] * ug[j];
        }
    }
    let k = el.local_stiffness(ni, nj);
    let mut ku = [0.0; 6];
    for i in 0..6 {
        for j in 0..6 {
            ku[i] += k[i][j] * ul[j];
        }
    }
    let mut feq = [0.0; 6];
    for dl in &model.distributed_loads {
        if dl.element_idx == e {
            let f = el.consistent_nodal_load(ni, nj, dl.qx, dl.qy).unwrap();
            for i in 0..6 {
                feq[i] += f[i];
            }
        }
    }
    for pl in &model.point_loads {
        if pl.element_idx == e {
            let f = el
                .consistent_nodal_load_point(ni, nj, pl.position, pl.fx, pl.fy, pl.mz)
                .unwrap();
            for i in 0..6 {
                feq[i] += f[i];
            }
        }
    }
    let mut end = [0.0; 6];
    for i in 0..6 {
        end[i] = feq[i] - ku[i];
    }
    end
}

// ===========================================================================
// 2. Boundary-condition semantics
// ===========================================================================

#[test]
fn test_fixed_node_bc_semantics() {
    let mut m = beam(1);
    m.fix_node(0);
    m.add_nodal_force(1, 1, -100.0);

    let s = solve_named(&m, "dense");
    assert_finite(&s);
    // Fixed node DOFs exactly zero.
    assert_eq!(s.displacements()[0], 0.0, "ux0");
    assert_eq!(s.displacements()[1], 0.0, "uy0");
    assert_eq!(s.displacements()[2], 0.0, "rz0");
    // Reaction satisfies equilibrium: Ry = P, Rz = P·L.
    assert_mixed(s.reactions()[1], 100.0, 1e-9, 1e-9, "Ry");
    assert_mixed(s.reactions()[2], 100.0 * L, 1e-9, 1e-9, "Rz");
    println!("[fixed node] ux=uy=rz=0 enforced; Ry=100, Rz=100");
}

#[test]
fn test_partial_constraints_enforced() {
    // Several solvable support schemes; each constrains 3+ DOFs to remove the
    // rigid-body modes.
    let cases: [(&str, &[(usize, usize)]); 4] = [
        ("pin-roller", &[(0, 0), (0, 1), (1, 1)]),
        ("cantilever", &[(0, 0), (0, 1), (0, 2)]),
        ("uy+rz / ux+uy", &[(0, 1), (0, 2), (1, 0), (1, 1)]),
        ("ux+uy / uy+rz", &[(0, 0), (0, 1), (1, 1), (1, 2)]),
    ];

    for (label, dofs) in cases {
        let mut m = beam(1);
        for &(node, dof) in dofs {
            m.try_fix_dof(node, dof, 0.0).unwrap();
        }
        m.add_nodal_force(1, 1, -50.0); // keep it loaded and well-posed

        let s = solve_named(&m, "dense");
        assert_finite(&s);
        for &(node, dof) in dofs {
            let idx = node * 3 + dof;
            assert_eq!(
                s.displacements()[idx],
                0.0,
                "{}: prescribed DOF node {} dof {} not exactly zero",
                label,
                node,
                dof
            );
        }
    }
    println!("[partial constraints] all prescribed DOFs exactly enforced (4 solvable schemes)");
}

#[test]
fn test_prescribed_displacement_enforced() {
    // Support settlement: a simply supported beam (pin + roller) with one
    // support prescribed a non-zero vertical displacement. This *strains* the
    // beam, so reactions are non-zero and self-equilibrated. (Prescribing a
    // rotation on a free-ended beam would instead be a strainless rigid-body
    // rotation with zero reaction.)
    let delta = 0.01;
    let mut m = beam(1);
    m.try_fix_dof(0, 0, 0.0).unwrap(); // pin: ux
    m.try_fix_dof(0, 1, delta).unwrap(); // prescribed settlement: uy
    m.try_fix_dof(1, 1, 0.0).unwrap(); // roller: uy

    let s = solve_named(&m, "dense");
    assert_finite(&s);
    assert_eq!(
        s.displacements()[1],
        delta,
        "prescribed settlement exactly enforced"
    );
    let r = s.reactions();
    assert!(
        r[1].abs() > 0.0 && r[2].abs() > 0.0,
        "settlement must induce reactions: {:?}",
        r
    );
    // No external load: the support reactions balance each other.
    assert_mixed(r[1] + r[4], 0.0, 1e-9, 1e-9, "ΣFy = 0");
    assert_mixed(r[2] + r[5] + r[4] * L, 0.0, 1e-9, 1e-9, "ΣMz = 0");
    println!(
        "[prescribed BC] settlement {} enforced exactly; induced reactions {:?}",
        delta, r
    );
}

// ===========================================================================
// 3. Boundary-condition equivalence
// ===========================================================================

#[test]
fn test_bc_api_equivalence() {
    // fix_node(i) must equal fix_dof(i, 0..2) and try_fix_node(i).
    let load = |m: &mut BeamModel| m.add_nodal_force(1, 1, -100.0);

    let mut a = beam(1);
    a.fix_node(0);
    load(&mut a);

    let mut b = beam(1);
    b.fix_dof(0, 0, 0.0);
    b.fix_dof(0, 1, 0.0);
    b.fix_dof(0, 2, 0.0);
    load(&mut b);

    let mut c = beam(1);
    c.try_fix_node(0).unwrap();
    load(&mut c);

    let (sa, sb, sc) = (
        solve_named(&a, "dense"),
        solve_named(&b, "dense"),
        solve_named(&c, "dense"),
    );
    for i in 0..sa.displacements().len() {
        assert_eq!(
            sa.displacements()[i],
            sb.displacements()[i],
            "fix_node vs fix_dof u[{}]",
            i
        );
        assert_eq!(
            sa.displacements()[i],
            sc.displacements()[i],
            "fix_node vs try_fix_node u[{}]",
            i
        );
    }
    for i in 0..sa.reactions().len() {
        assert_eq!(sa.reactions()[i], sb.reactions()[i], "reaction equivalence");
        assert_eq!(sa.reactions()[i], sc.reactions()[i], "reaction equivalence");
    }
    println!("[BC equivalence] fix_node == fix_dof x3 == try_fix_node (identical FEM semantics)");
}

// ===========================================================================
// 4. Load API semantics
// ===========================================================================

#[test]
fn test_nodal_force_semantics() {
    // Axial.
    let mut ax = beam(1);
    ax.fix_node(0);
    ax.add_nodal_force(1, 0, 100.0);
    let sa = solve_named(&ax, "dense");
    assert_mixed(sa.displacements()[3], 100.0, 1e-9, 1e-9, "axial u");
    assert_mixed(sa.reactions()[0], -100.0, 1e-9, 1e-9, "axial Rx");

    // Transverse.
    let mut tr = beam(1);
    tr.fix_node(0);
    tr.add_nodal_force(1, 1, -100.0);
    let st = solve_named(&tr, "dense");
    assert_mixed(
        st.displacements()[4],
        -100.0 / 3.0,
        1e-9,
        1e-9,
        "transverse delta",
    );
    assert_mixed(st.reactions()[1], 100.0, 1e-9, 1e-9, "Ry");
    assert_mixed(st.reactions()[2], 100.0, 1e-9, 1e-9, "Rz");
    println!("[nodal force] axial/transverse displacement, reaction, equilibrium OK");
}

#[test]
fn test_distributed_load_resultants() {
    let (qx, qy) = (60.0, -80.0);
    for (label, x, y) in [("qx", qx, 0.0), ("qy", 0.0, qy), ("qx+qy", qx, qy)] {
        let mut m = beam(2);
        m.fix_node(0);
        m.add_distributed_load(0, x, y).unwrap();
        m.add_distributed_load(1, x, y).unwrap();

        let s = solve_named(&m, "dense");
        // Total resultant reactions: Fx = qx·L, Fy = qy·L, M about node 0 = qy·L·L/2.
        assert_mixed(
            s.reactions()[0] + x * L,
            0.0,
            1e-8,
            1e-9,
            &format!("{} ΣFx", label),
        );
        assert_mixed(
            s.reactions()[1] + y * L,
            0.0,
            1e-8,
            1e-9,
            &format!("{} ΣFy", label),
        );
        assert_mixed(
            s.reactions()[2] + y * L * (L / 2.0),
            0.0,
            1e-8,
            1e-9,
            &format!("{} ΣMz", label),
        );
    }
    println!("[distributed] qx/qy/qx+qy resultants and moment correct");
}

#[test]
fn test_point_load_positions_and_boundary() {
    let p = 100.0;
    let a = L / 2.0;

    // Inside an element (xi = 0.5).
    let mut inside = beam(1);
    inside.fix_node(0);
    inside.add_point_load(0, 0.5, 0.0, -p, 0.0).unwrap();
    // At the element end (xi = 1) and at the start (xi = 0) of a 2-element beam.
    let mut at_end = beam(2);
    at_end.fix_node(0);
    at_end.add_point_load(0, 1.0, 0.0, -p, 0.0).unwrap();
    let mut at_start = beam(2);
    at_start.fix_node(0);
    at_start.add_point_load(1, 0.0, 0.0, -p, 0.0).unwrap();
    // Equivalent direct nodal load.
    let mut nodal = beam(2);
    nodal.fix_node(0);
    nodal.add_nodal_force(1, 1, -p);

    // Analytical tip response for a load at x = L/2 (cantilever):
    //   delta = P a^2 (3L - a) / 6EI, theta = P a^2 / 2EI.
    let an_delta = -p * a * a * (3.0 * L - a) / 6.0;
    let an_theta = -p * a * a / 2.0;

    let si = solve_named(&inside, "dense");
    assert_mixed(si.displacements()[4], an_delta, 1e-9, 1e-9, "inside delta");
    assert_mixed(si.displacements()[5], an_theta, 1e-9, 1e-9, "inside theta");
    assert_mixed(si.reactions()[1], p, 1e-9, 1e-9, "inside Ry");

    // Boundary loads: no double counting (Ry = P, not 2P) and equal to nodal.
    for (label, m) in [("xi=1", &at_end), ("xi=0", &at_start)] {
        let s = solve_named(m, "dense");
        assert_mixed(s.reactions()[1], p, 1e-9, 1e-9, &format!("{} Ry=P", label));
    }
    let (sn, se, ss) = (
        solve_named(&nodal, "dense"),
        solve_named(&at_end, "dense"),
        solve_named(&at_start, "dense"),
    );
    for i in 0..sn.displacements().len() {
        assert_mixed(
            se.displacements()[i],
            sn.displacements()[i],
            1e-10,
            1e-9,
            "xi=1 vs nodal",
        );
        assert_mixed(
            ss.displacements()[i],
            sn.displacements()[i],
            1e-10,
            1e-9,
            "xi=0 vs nodal",
        );
    }
    println!("[point load] inside element exact; boundary xi=0/1 == nodal (no double count)");
}

#[test]
fn test_applied_moment_semantics() {
    let m0 = 100.0;

    // Tip moment (free end).
    let mut tip = beam(1);
    tip.fix_node(0);
    tip.add_applied_moment(1, m0).unwrap();
    let st = solve_named(&tip, "dense");
    assert_mixed(
        st.displacements()[5],
        m0 * L,
        1e-9,
        1e-9,
        "tip theta = ML/EI",
    );
    assert_mixed(st.reactions()[2], -m0, 1e-9, 1e-9, "Rz = -M");
    // Established free-end element-on-node contract M_j = -M.
    assert_mixed(
        st.element_end_forces().unwrap()[0][5],
        -m0,
        1e-9,
        1e-9,
        "M_j = -M",
    );

    // Internal node moment: M_right - M_left + M_applied = 0.
    let mut im = beam(2);
    im.fix_node(0);
    im.add_applied_moment(1, m0).unwrap();
    let si = solve_named(&im, "dense");
    let left = si.element_section_forces(0, 1.0).unwrap().moment;
    let right = si.element_section_forces(1, 0.0).unwrap().moment;
    assert_mixed(
        right - left + m0,
        0.0,
        1e-9,
        1e-12,
        "M_right - M_left + M_app = 0",
    );
    assert_mixed(si.reactions()[2], -m0, 1e-9, 1e-9, "internal Rz = -M");
    println!("[applied moment] tip M_j=-M; internal M jump balances the applied moment");
}

// ===========================================================================
// 5. Load interactions
// ===========================================================================

#[test]
fn test_load_interactions_equilibrium() {
    let (f, p, q, m0) = (30.0, -50.0, -40.0, 25.0);

    let mut m = beam(2);
    m.fix_node(0);
    m.add_distributed_load(0, f, q).unwrap();
    m.add_point_load(1, 0.5, 0.0, p, 0.0).unwrap();
    m.add_applied_moment(1, m0).unwrap();
    m.add_nodal_force(2, 1, p);

    let s = solve_named(&m, "dense");
    assert_finite(&s);
    // ΣFx: distributed qx over elem 0 (length L/2).
    assert_mixed(s.reactions()[0] + f * (L / 2.0), 0.0, 1e-8, 1e-9, "ΣFx");
    // ΣFy: distributed qy (elem 0) + point (elem 1 at x=3L/4) + tip nodal force.
    let fy = q * (L / 2.0) + p + p;
    assert_mixed(s.reactions()[1] + fy, 0.0, 1e-8, 1e-9, "ΣFy");
    // ΣMz about node 0.
    let mz = m0 + (q * L / 2.0) * (L / 4.0) + p * (3.0 * L / 4.0) + p * L;
    assert_mixed(s.reactions()[2] + mz, 0.0, 1e-8, 1e-9, "ΣMz");
    println!("[interactions] distributed + point + applied moment + nodal force: ΣF/ΣM = 0");
}

// ===========================================================================
// 6. Multi-element load partitioning
// ===========================================================================

#[test]
fn test_multielement_load_partitioning() {
    // 4 nodes / 3 elements. One load per location; the global total must equal
    // the exact sum (no double counting, nothing missing).
    let p = 100.0;

    let mut m = beam(3);
    m.fix_node(0);
    m.add_nodal_force(0, 1, -p); // at node 0 (support)
    m.add_point_load(0, 0.5, 0.0, -p, 0.0).unwrap(); // inside element 0
    m.add_nodal_force(1, 1, -p); // exactly at node 1
    m.add_point_load(1, 0.5, 0.0, -p, 0.0).unwrap(); // inside element 1
    m.add_nodal_force(2, 1, -p); // exactly at node 2
    m.add_point_load(2, 0.5, 0.0, -p, 0.0).unwrap(); // inside element 2
    m.add_nodal_force(3, 1, -p); // at node 3

    let s = solve_named(&m, "dense");
    assert_finite(&s);
    // 7 transverse loads of magnitude P.
    assert_mixed(
        s.reactions()[1],
        7.0 * p,
        1e-8,
        1e-9,
        "Ry = 7P (each load once)",
    );

    // ΣMz about node 0 (dx = L/3): nodal at 0, 1/3, 2/3, 1 and point loads at
    // 1/6, 1/2, 5/6.
    let dx = L / 3.0;
    let x = [
        0.0,
        dx / 2.0,
        dx,
        dx + dx / 2.0,
        2.0 * dx,
        2.0 * dx + dx / 2.0,
        3.0 * dx,
    ];
    let mz: f64 = x.iter().map(|&xi| p * xi).sum();
    assert_mixed(s.reactions()[2], mz, 1e-8, 1e-9, "Rz = Σ P·x");

    // Internal force continuity where no external load sits: at the midpoints of
    // each element the section shear is constant between loads (piecewise).
    // Check the shear jumps by exactly one P across each nodal load.
    let v_left = s.element_section_forces(0, 0.999).unwrap().shear;
    let v_right = s.element_section_forces(1, 0.001).unwrap().shear;
    assert_mixed(v_right - v_left, -p, 1e-6, 1e-6, "shear jump at node 1 = P");
    println!("[partitioning] 7 loads each counted once; Ry = 7P; moment correct");
}

// ===========================================================================
// 7. Local/global transformation audit
// ===========================================================================

#[test]
fn test_coordinate_transformation_audit() {
    let p = 100.0;
    let an_v = -p * L.powi(3) / 3.0; // local tip deflection
    let an_rz = -p * L.powi(2) / 2.0;

    for &deg in &[0.0_f64, 45.0, 90.0, -45.0, 135.0] {
        let th = deg.to_radians();
        let (c, s) = (th.cos(), th.sin());
        let mut m = BeamModel::new();
        m.add_node(BeamNode::new(0, 0.0, 0.0));
        m.add_node(BeamNode::new(1, L * c, L * s));
        m.add_element(BeamElement::new(0, 1, mat(), sec()).unwrap());
        m.fix_node(0);
        m.add_nodal_force(1, 0, s * p); // global force = rotated local (0, -P)
        m.add_nodal_force(1, 1, -c * p);

        for &name in &DIRECT {
            let sol = solve_named(&m, name);
            let r = sol.results();
            // Global displacement = R(theta) · local displacement (0, an_v).
            assert_mixed(
                r.displacement(1).unwrap().ux,
                -s * an_v,
                1e-9,
                1e-9,
                &format!("{} {:.0}° ux", name, deg),
            );
            assert_mixed(
                r.displacement(1).unwrap().uy,
                c * an_v,
                1e-9,
                1e-9,
                &format!("{} {:.0}° uy", name, deg),
            );
            assert_mixed(
                r.displacement(1).unwrap().rz,
                an_rz,
                1e-9,
                1e-9,
                &format!("{} {:.0}° rz", name, deg),
            );
            // Reactions rotate the same way; moment invariant.
            assert_mixed(
                r.reaction(0).unwrap().fx,
                -s * p,
                1e-8,
                1e-9,
                &format!("{} {:.0}° Rx", name, deg),
            );
            assert_mixed(
                r.reaction(0).unwrap().fy,
                c * p,
                1e-8,
                1e-9,
                &format!("{} {:.0}° Ry", name, deg),
            );
            assert_mixed(
                r.reaction(0).unwrap().mz,
                p * L,
                1e-8,
                1e-9,
                &format!("{} {:.0}° Rz", name, deg),
            );
            // f_global = T^T f_local.
            let fe = sol.element_end_forces().unwrap()[0];
            let feg = sol.element_end_forces_global().unwrap()[0];
            let ni = m.nodes[0].point();
            let nj = m.nodes[1].point();
            let t = m.elements[0].transformation_matrix(ni, nj);
            let mut g = [0.0; 6];
            for (j, tj) in t.iter().enumerate() {
                for (i, gv) in g.iter_mut().enumerate() {
                    *gv += tj[i] * fe[j];
                }
            }
            for i in 0..6 {
                assert_mixed(g[i], feg[i], 1e-9, 1e-9, "f_global = Tᵀ f_local");
            }
        }
    }
    println!("[transformation] 0/45/90/-45/135°: u_global=R·u_local, reactions rotate, f_g=Tᵀ f_l");
}

// ===========================================================================
// 8. Element end-force audit
// ===========================================================================

#[test]
fn test_end_force_recovery_audit() {
    // Model exercising every load type on two elements of different lengths.
    let mut m = BeamModel::new();
    m.add_node(BeamNode::new(0, 0.0, 0.0));
    m.add_node(BeamNode::new(1, 0.4, 0.0));
    m.add_node(BeamNode::new(2, 1.0, 0.0)); // unequal elements (0.4, 0.6)
    m.add_element(BeamElement::new(0, 1, mat(), sec()).unwrap());
    m.add_element(BeamElement::new(1, 2, mat(), sec()).unwrap());
    m.fix_node(0);
    m.add_distributed_load(0, 20.0, -30.0).unwrap();
    m.add_point_load(1, 0.5, 5.0, -15.0, 7.0).unwrap();
    m.add_applied_moment(1, 11.0).unwrap(); // must NOT enter f_equiv
    m.add_nodal_force(2, 1, -25.0);

    for &name in &DIRECT {
        let s = solve_named(&m, name);
        let actual = s.element_end_forces().unwrap();
        for (e, ae) in actual.iter().enumerate() {
            let expected = end_forces_from_principle(&m, &s, e);
            for (k, (&a, &ex)) in ae.iter().zip(expected.iter()).enumerate() {
                assert_mixed(
                    a,
                    ex,
                    1e-10,
                    1e-9,
                    &format!(
                        "{} elem{} end force [{}]: f_end = f_equiv - K_e u_e",
                        name, e, k
                    ),
                );
            }
        }
    }
    println!(
        "[end-force audit] f_end == f_equiv - K_e u_e (applied moment excluded) for all backends"
    );
}

// ===========================================================================
// 9. Reaction audit
// ===========================================================================

#[test]
fn test_reaction_audit() {
    let mut m = beam(2);
    m.fix_node(0);
    m.add_distributed_load(0, 10.0, -20.0).unwrap();
    m.add_point_load(1, 0.5, 5.0, -30.0, 4.0).unwrap();
    m.add_applied_moment(1, 9.0).unwrap();
    m.add_nodal_force(2, 1, -40.0);

    let mut reference: Option<(Vec<f64>, Vec<f64>)> = None;
    for &name in &DIRECT {
        let s = solve_named(&m, name);
        let r = s.reactions();

        // Constrained DOF reactions balance the external loads.
        assert_mixed(r[0] + (10.0 * (L / 2.0) + 5.0), 0.0, 1e-8, 1e-9, "ΣFx");
        assert_mixed(
            r[1] + (-20.0 * (L / 2.0) - 30.0 - 40.0),
            0.0,
            1e-8,
            1e-9,
            "ΣFy",
        );
        assert_mixed(
            r[2] + 9.0 + (4.0) + (-30.0 * 0.75) + (-20.0 * (L / 2.0) * 0.25) + (-40.0 * L),
            0.0,
            1e-8,
            1e-9,
            "ΣMz",
        );
        // Free DOFs carry ~zero reaction.
        for (i, &v) in r.iter().enumerate().skip(3) {
            assert_mixed(v, 0.0, 1e-7, 1e-9, &format!("free-DOF R[{}]", i));
        }
        // Reactions do not depend on the solver choice.
        match &reference {
            None => reference = Some((r.clone(), s.displacements().to_vec())),
            Some((rr, uu)) => {
                for i in 0..r.len() {
                    assert_mixed(r[i], rr[i], 1e-7, 1e-9, "solver-independent reaction");
                }
                for (i, &v) in uu.iter().enumerate() {
                    assert_mixed(
                        s.displacements()[i],
                        v,
                        1e-10,
                        1e-9,
                        "solver-independent displacement",
                    );
                }
            }
        }
    }
    println!("[reaction audit] balances external loads; free DOFs ~0; solver-independent");
}

// ===========================================================================
// 10. Solver-independent semantics
// ===========================================================================

#[test]
fn test_solver_independent_semantics() {
    let mut m = beam(2);
    m.fix_node(0);
    m.add_distributed_load(0, 10.0, -20.0).unwrap();
    m.add_point_load(1, 0.5, 5.0, -30.0, 4.0).unwrap();
    m.add_applied_moment(1, 9.0).unwrap();
    m.add_nodal_force(2, 1, -40.0);

    let ref_solver = solve_named(&m, "dense");
    let u_ref = ref_solver.displacements().to_vec();
    let r_ref = ref_solver.reactions();
    let fe_ref = ref_solver.element_end_forces().unwrap();

    for &name in &["skyline_ldlt", "sparse_lu"] {
        let s = solve_named(&m, name);
        for (i, &v) in u_ref.iter().enumerate() {
            assert_mixed(s.displacements()[i], v, 1e-10, 1e-9, "displacement");
        }
        for (i, &v) in r_ref.iter().enumerate() {
            assert_mixed(s.reactions()[i], v, 1e-7, 1e-9, "reaction");
        }
        let fe = s.element_end_forces().unwrap();
        for e in 0..2 {
            for k in 0..6 {
                assert_mixed(fe[e][k], fe_ref[e][k], 1e-9, 1e-9, "end force");
            }
        }
    }
    println!("[solver independence] dense ≈ skyline_ldlt ≈ sparse_lu (u, R, f_end)");
}

// ===========================================================================
// 11. Repeated model mutation audit
// ===========================================================================

#[test]
fn test_mutation_after_solve_snapshot_semantics() {
    // A BeamSolver snapshots the model at from_model(): mutating the original
    // model afterwards does not affect an already-built solver, and the same
    // solver re-solves deterministically. Rebuilding reflects the mutation.
    let mut m = beam(1);
    m.fix_node(0);
    m.add_nodal_force(1, 1, -100.0);

    let mut s = BeamSolver::from_model(&m).unwrap();
    s.set_solver(SolverSelection::named("dense"));
    s.solve_configured().unwrap();
    let u_first = s.displacements().to_vec();
    let r_first = s.reactions();

    // Mutate the source model: add a second load.
    m.add_nodal_force(1, 1, -100.0);

    // (a) The already-built solver is unaffected (snapshot semantics).
    s.solve_configured().unwrap();
    for (i, &v) in u_first.iter().enumerate() {
        assert_eq!(
            s.displacements()[i],
            v,
            "snapshot: solver must not see model mutation"
        );
    }

    // (b) Rebuilding from the mutated model reflects the extra load exactly.
    let mut s2 = BeamSolver::from_model(&m).unwrap();
    s2.set_solver(SolverSelection::named("dense"));
    s2.solve_configured().unwrap();
    assert_mixed(
        s2.reactions()[1],
        2.0 * r_first[1],
        1e-9,
        1e-9,
        "rebuilt solver sees the new load",
    );
    for (i, &v) in u_first.iter().enumerate() {
        assert_mixed(
            s2.displacements()[i],
            2.0 * v,
            1e-9,
            1e-9,
            "rebuilt response doubles",
        );
    }
    println!("[mutation] BeamSolver snapshots the model; rebuild required to pick up mutations");
}

// ===========================================================================
// 12. API consistency (legacy panic vs checked)
// ===========================================================================

#[test]
#[should_panic]
fn test_legacy_add_nodal_force_panics_on_invalid_node() {
    // Compatibility contract: the legacy (non-fallible) API panics on
    // programmer error. The checked variant returns a structured error instead.
    let mut m = beam(1);
    m.add_nodal_force(9, 0, 1.0);
}

#[test]
#[should_panic]
fn test_legacy_fix_node_panics_on_invalid_node() {
    let mut m = beam(1);
    m.fix_node(9);
}
