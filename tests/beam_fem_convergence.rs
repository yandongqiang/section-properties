//! Beam FEM convergence and formulation coverage.
//!
//! Companion to `beam_fem_reference.rs` (which established the analytical
//! reference and the Case A-E contract). This file adds:
//! * systematic mesh convergence tables (n = 1, 2, 4, 8, 16);
//! * interior vs at-node point loads (no double counting);
//! * internal-node element end-force continuity (with and without nodal load);
//! * axial loading (tip and uniformly distributed `qx`);
//! * combined axial + bending and axial + UDL + applied moment;
//! * rotated beams at 0/30/45/60/90 with an explicit R(theta) reference;
//! * `qx`/`qy` equivalent nodal load and equilibrium;
//! * applied-moment contract for single, multi-element and internal nodes;
//! * extreme-but-valid scale invariance (E, A, I);
//! * every core case solved with dense / skyline_ldlt / sparse_lu and compared
//!   to the analytical reference.
//!
//! Conventions (Rust, under test): local x = node_i->node_j, local y up,
//! rotation CCW, shear V = dM/dx, moment sagging positive. Distributed/point
//! loads are LOCAL, applied moments are GLOBAL theta loads and are NOT element
//! equivalent loads. `element_end_forces()` is the element-on-node force
//! `r = f_eq - K_e u_e`; `reactions()` is the global support reaction
//! `K_original u - f_global`.
//!
//! Tolerance policy: `|a-b| <= abs + rel*max(|a|,|b|)`, abs scaled per quantity
//! (1e-9). Never loosened to make a case pass.

use section_properties::SolverSelection;
use section_properties::beam_fem::{BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver};
use section_properties::material::Material;

const DIRECT: [&str; 3] = ["dense", "skyline_ldlt", "sparse_lu"];
const L: f64 = 1.0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn model_with(n: usize, e: f64, a: f64, i: f64) -> BeamModel {
    let mut model = BeamModel::new();
    let dx = L / n as f64;
    for k in 0..=n {
        model.add_node(BeamNode::new(k, k as f64 * dx, 0.0));
    }
    for k in 0..n {
        model.add_element(
            BeamElement::new(
                k,
                k + 1,
                Material::new(e, 0.3, 1.0, "U"),
                BeamSection::new(a, i),
            )
            .unwrap(),
        );
    }
    model.fix_node(0);
    model
}

/// Unit model: E = A = I = 1, so EI = EA = 1.
fn unit(n: usize) -> BeamModel {
    model_with(n, 1.0, 1.0, 1.0)
}

fn solve_named(model: &BeamModel, name: &str) -> BeamSolver {
    let mut s = BeamSolver::from_model(model).unwrap();
    s.set_solver(SolverSelection::named(name));
    s.solve_configured()
        .unwrap_or_else(|e| panic!("{}: solve failed: {:?}", name, e));
    assert_eq!(s.solver_name(), Some(name));
    s
}

fn assert_mixed(a: f64, b: f64, abs: f64, rel: f64, label: &str) {
    let bound = abs + rel * a.abs().max(b.abs());
    assert!(
        (a - b).abs() <= bound,
        "{}: computed {} vs reference {} (|diff| = {:.3e} > {:.3e})",
        label,
        a,
        b,
        (a - b).abs(),
        bound
    );
}

fn reldiff(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-300)
}

// ===========================================================================
// 2. Mesh convergence
// ===========================================================================

#[test]
fn test_mesh_convergence_tip_load() {
    let p = 100.0;
    let an_uy = -p * L.powi(3) / 3.0; // d = PL^3 / 3EI
    let an_rz = -p * L.powi(2) / 2.0; // th = PL^2 / 2EI

    println!("[mesh tip-load]  n    tip_uy        tip_rz        uy_err     rz_err");
    for &n in &[1usize, 2, 4, 8, 16] {
        let mut model = unit(n);
        model.add_nodal_force(n, 1, -p);
        let mut worst = 0.0f64;
        for &name in &DIRECT {
            let s = solve_named(&model, name);
            let r = s.results();
            assert_mixed(
                r.displacement(n).unwrap().uy,
                an_uy,
                1e-9,
                1e-9,
                &format!("{} n={} uy", name, n),
            );
            assert_mixed(
                r.displacement(n).unwrap().rz,
                an_rz,
                1e-9,
                1e-9,
                &format!("{} n={} rz", name, n),
            );
            assert_mixed(
                r.reaction(0).unwrap().fy,
                p,
                1e-9,
                1e-9,
                &format!("{} n={} Ry", name, n),
            );
            assert_mixed(
                r.reaction(0).unwrap().mz,
                p * L,
                1e-9,
                1e-9,
                &format!("{} n={} Rz", name, n),
            );
            worst = worst.max(reldiff(r.displacement(n).unwrap().uy, an_uy));
        }
        let s = solve_named(&model, "dense");
        println!(
            "               {:2}   {:.9e}   {:.9e}   {:.2e}   {:.2e}",
            n,
            s.displacement(n, 1),
            s.displacement(n, 2),
            worst,
            reldiff(s.displacement(n, 2), an_rz)
        );
    }
}

#[test]
fn test_mesh_convergence_udl() {
    let q = 100.0;
    let an_uy = -q * L.powi(4) / 8.0; // d = qL^4 / 8EI
    let an_rz = -q * L.powi(3) / 6.0; // th = qL^3 / 6EI

    println!("[mesh udl]       n    tip_uy        tip_rz        uy_err     rz_err");
    for &n in &[1usize, 2, 4, 8, 16] {
        let mut model = unit(n);
        for e in 0..n {
            model.add_distributed_load(e, 0.0, -q).unwrap();
        }
        let mut worst = 0.0f64;
        for &name in &DIRECT {
            let s = solve_named(&model, name);
            let r = s.results();
            assert_mixed(
                r.displacement(n).unwrap().uy,
                an_uy,
                1e-9,
                1e-9,
                &format!("{} n={} uy", name, n),
            );
            assert_mixed(
                r.displacement(n).unwrap().rz,
                an_rz,
                1e-9,
                1e-9,
                &format!("{} n={} rz", name, n),
            );
            assert_mixed(
                r.reaction(0).unwrap().fy,
                q * L,
                1e-9,
                1e-9,
                &format!("{} n={} Ry", name, n),
            );
            assert_mixed(
                r.reaction(0).unwrap().mz,
                q * L * L / 2.0,
                1e-9,
                1e-9,
                &format!("{} n={} Rz", name, n),
            );
            worst = worst.max(reldiff(r.displacement(n).unwrap().uy, an_uy));
        }
        let s = solve_named(&model, "dense");
        println!(
            "               {:2}   {:.9e}   {:.9e}   {:.2e}   {:.2e}",
            n,
            s.displacement(n, 1),
            s.displacement(n, 2),
            worst,
            reldiff(s.displacement(n, 2), an_rz)
        );
    }
}

// ===========================================================================
// 3. Interior vs at-node point load
// ===========================================================================

#[test]
fn test_interior_point_load_inside_element() {
    let p = 100.0;
    let a = L / 2.0;
    // Cantilever, point load P downward at x = a = L/2.
    let an_uy = -p * a * a * (3.0 * L - a) / 6.0; // P a^2 (3L - a) / 6EI
    let an_rz = -p * a * a / 2.0; // P a^2 / 2EI

    let mut model = unit(1);
    model.add_point_load(0, 0.5, 0.0, -p, 0.0).unwrap();

    for &name in &DIRECT {
        let s = solve_named(&model, name);
        let r = s.results();
        assert_mixed(
            r.displacement(1).unwrap().uy,
            an_uy,
            1e-9,
            1e-9,
            &format!("{} tip uy", name),
        );
        assert_mixed(
            r.displacement(1).unwrap().rz,
            an_rz,
            1e-9,
            1e-9,
            &format!("{} tip rz", name),
        );
        assert_mixed(
            r.reaction(0).unwrap().fy,
            p,
            1e-9,
            1e-9,
            &format!("{} Ry", name),
        );
        assert_mixed(
            r.reaction(0).unwrap().mz,
            p * a,
            1e-9,
            1e-9,
            &format!("{} Rz = P*a", name),
        );
        // Element end forces: [0, -P, -P*a, 0, 0, 0].
        let fe = r.element_end_forces(0).unwrap();
        assert_mixed(fe[1], -p, 1e-9, 1e-9, "V_i = -P");
        assert_mixed(fe[2], -p * a, 1e-9, 1e-9, "M_i = -P*a");
        assert_mixed(fe[4], 0.0, 1e-9, 1e-9, "V_j = 0");
        assert_mixed(fe[5], 0.0, 1e-9, 1e-9, "M_j = 0");
        // Section: V = +P left of a, 0 right; M = -P(a-x) left, 0 right.
        assert_mixed(
            s.element_section_forces(0, 0.25).unwrap().shear,
            p,
            1e-8,
            1e-9,
            "V(<a)",
        );
        assert_mixed(
            s.element_section_forces(0, 0.75).unwrap().shear,
            0.0,
            1e-8,
            1e-9,
            "V(>a)",
        );
        assert_mixed(
            s.element_section_forces(0, 0.25).unwrap().moment,
            -p * (a - 0.25),
            1e-8,
            1e-9,
            "M(<a)",
        );
        assert_mixed(
            s.element_section_forces(0, 0.75).unwrap().moment,
            0.0,
            1e-8,
            1e-9,
            "M(>a)",
        );
    }
    println!(
        "[interior point load] δ={:.6e} θ={:.6e} Rz=P·a (analytical; 3 backends)",
        an_uy, an_rz
    );
}

#[test]
fn test_point_load_at_node_no_double_count() {
    let p = 100.0;
    let a = L / 2.0;
    let an_uy = -p * a * a * (3.0 * L - a) / 6.0;

    // (1) Nodal force directly at the mid node.
    let mut nodal = unit(2);
    nodal.add_nodal_force(1, 1, -p);
    // (2) Element point load at the boundary xi=1 of element 0.
    let mut at_end = unit(2);
    at_end.add_point_load(0, 1.0, 0.0, -p, 0.0).unwrap();
    // (3) Element point load at the boundary xi=0 of element 1.
    let mut at_start = unit(2);
    at_start.add_point_load(1, 0.0, 0.0, -p, 0.0).unwrap();

    for &name in &DIRECT {
        let sn = solve_named(&nodal, name);
        let se = solve_named(&at_end, name);
        let ss = solve_named(&at_start, name);

        // The tip response matches the analytical interior-point-load result.
        for (label, s) in [("nodal", &sn), ("xi=1", &se), ("xi=0", &ss)] {
            assert_mixed(
                s.results().displacement(2).unwrap().uy,
                an_uy,
                1e-9,
                1e-9,
                &format!("{} {} tip uy", name, label),
            );
        }
        // No double counting: all three applying the same single load agree.
        assert_mixed(
            se.results().displacement(2).unwrap().uy,
            sn.results().displacement(2).unwrap().uy,
            1e-12,
            1e-9,
            "xi=1 vs nodal",
        );
        assert_mixed(
            ss.results().displacement(2).unwrap().uy,
            sn.results().displacement(2).unwrap().uy,
            1e-12,
            1e-9,
            "xi=0 vs nodal",
        );
        // Total reaction is exactly -P (not -2P).
        assert_mixed(sn.reactions()[1], p, 1e-9, 1e-9, "Ry = P (single load)");
        assert_mixed(se.reactions()[1], p, 1e-9, 1e-9, "Ry = P (xi=1)");
        assert_mixed(ss.reactions()[1], p, 1e-9, 1e-9, "Ry = P (xi=0)");
    }
    println!("[point load at node] nodal == xi=1 == xi=0, Ry = P (no double counting)");
}

// ===========================================================================
// 4. Internal-node end-force continuity
// ===========================================================================

#[test]
fn test_internal_force_continuity_no_load() {
    let p = 100.0;
    let mut model = unit(2);
    model.add_nodal_force(2, 1, -p); // load at the tip, none at node 1

    for &name in &DIRECT {
        let fe = solve_named(&model, name).element_end_forces().unwrap();
        // Element-on-node forces at the shared node: elem0 j-end + elem1 i-end
        // (no external nodal load there).
        assert_mixed(fe[0][3] + fe[1][0], 0.0, 1e-9, 1e-12, "axial continuity");
        assert_mixed(fe[0][4] + fe[1][1], 0.0, 1e-9, 1e-12, "shear continuity");
        assert_mixed(fe[0][5] + fe[1][2], 0.0, 1e-9, 1e-12, "moment continuity");
    }
    // Section forces are continuous at the shared node (no applied moment).
    for &name in &DIRECT {
        let s = solve_named(&model, name);
        let left = s.element_section_forces(0, 1.0).unwrap();
        let right = s.element_section_forces(1, 0.0).unwrap();
        assert_mixed(left.shear, right.shear, 1e-8, 1e-9, "V continuous");
        assert_mixed(left.moment, right.moment, 1e-8, 1e-9, "M continuous");
        assert_mixed(left.axial, right.axial, 1e-8, 1e-9, "N continuous");
    }
    println!("[continuity/no-load] f_left + f_right = 0; section N/V/M continuous");
}

#[test]
fn test_internal_force_continuity_with_nodal_load() {
    let p = 100.0;
    let mut model = unit(2);
    model.add_nodal_force(1, 1, -p); // transverse load AT the shared node
    model.add_nodal_force(2, 1, -10.0);

    for &name in &DIRECT {
        let fe = solve_named(&model, name).element_end_forces().unwrap();
        // Node equilibrium at node 1: f_left + f_right + f_ext = 0, with the
        // external nodal load (0, -P, 0) in LOCAL coordinates (horizontal beam).
        assert_mixed(fe[0][3] + fe[1][0] + 0.0, 0.0, 1e-9, 1e-12, "axial eq");
        assert_mixed(
            fe[0][4] + fe[1][1] + (-p),
            0.0,
            1e-9,
            1e-12,
            "shear eq (+P_ext)",
        );
        assert_mixed(fe[0][5] + fe[1][2] + 0.0, 0.0, 1e-9, 1e-12, "moment eq");
    }
    println!("[continuity/nodal-load] f_left + f_right + P_ext = 0 at the loaded shared node");
}

// ===========================================================================
// 5. Axial loading
// ===========================================================================

#[test]
fn test_axial_tip_load() {
    let f = 100.0;
    let mut model = unit(1);
    model.add_nodal_force(1, 0, f); // tension +x

    let an_u = f * L; // F L / (E A), EA = 1
    for &name in &DIRECT {
        let s = solve_named(&model, name);
        let r = s.results();
        assert_mixed(
            r.displacement(1).unwrap().ux,
            an_u,
            1e-9,
            1e-9,
            &format!("{} ux", name),
        );
        assert_mixed(
            r.reaction(0).unwrap().fx,
            -f,
            1e-9,
            1e-9,
            &format!("{} Rx", name),
        );
        let fe = r.element_end_forces(0).unwrap();
        assert_mixed(fe[0], f, 1e-9, 1e-9, "N_i = +F");
        assert_mixed(fe[3], -f, 1e-9, 1e-9, "N_j = -F");
        assert_mixed(
            s.element_section_forces(0, 0.5).unwrap().axial,
            f,
            1e-9,
            1e-9,
            "N(x) = F",
        );
    }
    println!("[axial tip] u = FL/EA = {:.6e}", an_u);
}

#[test]
fn test_axial_distributed_load() {
    let qx = 100.0;
    let mut model = unit(2);
    model.add_distributed_load(0, qx, 0.0).unwrap();
    model.add_distributed_load(1, qx, 0.0).unwrap();

    // N(x) = qx (L - x); u_tip = qx L^2 / (2 EA); reaction Rx = -qx L.
    let an_u = qx * L * L / 2.0;
    for &name in &DIRECT {
        let s = solve_named(&model, name);
        let r = s.results();
        assert_mixed(
            r.displacement(2).unwrap().ux,
            an_u,
            1e-9,
            1e-9,
            &format!("{} u_tip", name),
        );
        assert_mixed(
            r.reaction(0).unwrap().fx,
            -qx * L,
            1e-9,
            1e-9,
            &format!("{} Rx", name),
        );
        // Section axial force diagram.
        let dx = L / 2.0;
        for (elem, x0) in [(0usize, 0.0), (1usize, dx)] {
            for &xi in &[0.0, 0.5, 1.0] {
                let x = x0 + xi * dx;
                let n = s.element_section_forces(elem, xi).unwrap().axial;
                assert_mixed(n, qx * (L - x), 1e-8, 1e-9, "N(x) = qx(L-x)");
            }
        }
        // Element 0 end forces: N_i = +N(0) = qx*L; N_j = -N(L/2) = -qx*L/2
        // (boundary relation N(xi=1) = -N_j).
        let fe = r.element_end_forces(0).unwrap();
        assert_mixed(fe[0], qx * L, 1e-8, 1e-9, "N_i = +N(0)");
        assert_mixed(fe[3], -qx * L / 2.0, 1e-8, 1e-9, "N_j = -N(L/2)");
        // Element longitudinal equilibrium: sum of end axial forces = load.
        assert_mixed(fe[0] + fe[3], qx * (L / 2.0), 1e-8, 1e-9, "ΣN = qx·(L/2)");
    }
    println!("[axial udl] u_tip = qxL²/(2EA) = {:.6e}", an_u);
}

// ===========================================================================
// 6. Combined axial + bending
// ===========================================================================

#[test]
fn test_combined_axial_bending() {
    let (f, p) = (100.0, 100.0);
    let mut model = unit(2);
    model.add_nodal_force(2, 0, f);
    model.add_nodal_force(2, 1, -p);

    for &name in &DIRECT {
        let s = solve_named(&model, name);
        let r = s.results();
        assert_mixed(
            r.displacement(2).unwrap().ux,
            f * L,
            1e-9,
            1e-9,
            &format!("{} ux", name),
        );
        assert_mixed(
            r.displacement(2).unwrap().uy,
            -p * L.powi(3) / 3.0,
            1e-9,
            1e-9,
            &format!("{} uy", name),
        );
        assert_mixed(
            r.displacement(2).unwrap().rz,
            -p * L.powi(2) / 2.0,
            1e-9,
            1e-9,
            &format!("{} rz", name),
        );
        assert_mixed(r.reaction(0).unwrap().fx, -f, 1e-9, 1e-9, "Rx");
        assert_mixed(r.reaction(0).unwrap().fy, p, 1e-9, 1e-9, "Ry");
        assert_mixed(r.reaction(0).unwrap().mz, p * L, 1e-9, 1e-9, "Rz");
        // Decoupling: axial displacement is unaffected by the transverse load.
        let axial_only = {
            let mut m = unit(2);
            m.add_nodal_force(2, 0, f);
            solve_named(&m, name).displacement(2, 0)
        };
        assert_mixed(
            r.displacement(2).unwrap().ux,
            axial_only,
            1e-12,
            1e-9,
            "axial decoupled from bending",
        );
    }
    println!("[combined axial+bending] axial and bending DOFs are decoupled");
}

#[test]
fn test_combined_axial_udl_moment() {
    let (f, q, m) = (100.0, 100.0, 100.0);
    let mut model = unit(2);
    model.add_nodal_force(2, 0, f);
    model.add_distributed_load(0, 0.0, -q).unwrap();
    model.add_distributed_load(1, 0.0, -q).unwrap();
    model.add_applied_moment(2, m).unwrap();

    // Superposition of individual responses.
    let mut ax = unit(2);
    ax.add_nodal_force(2, 0, f);
    let mut ud = unit(2);
    ud.add_distributed_load(0, 0.0, -q).unwrap();
    ud.add_distributed_load(1, 0.0, -q).unwrap();
    let mut mo = unit(2);
    mo.add_applied_moment(2, m).unwrap();

    for &name in &DIRECT {
        let s = solve_named(&model, name);
        let r = s.results();
        let (a, u, mm) = (
            solve_named(&ax, name),
            solve_named(&ud, name),
            solve_named(&mo, name),
        );
        let dc = r.displacement(2).unwrap();
        for (label, got, sum) in [
            (
                "ux",
                dc.ux,
                a.displacement(2, 0) + u.displacement(2, 0) + mm.displacement(2, 0),
            ),
            (
                "uy",
                dc.uy,
                a.displacement(2, 1) + u.displacement(2, 1) + mm.displacement(2, 1),
            ),
            (
                "rz",
                dc.rz,
                a.displacement(2, 2) + u.displacement(2, 2) + mm.displacement(2, 2),
            ),
        ] {
            assert_mixed(
                got,
                sum,
                1e-9,
                1e-9,
                &format!("{} superposed {}", name, label),
            );
        }
        // Equilibrium.
        assert_mixed(r.reaction(0).unwrap().fx + f, 0.0, 1e-8, 1e-9, "ΣFx");
        assert_mixed(r.reaction(0).unwrap().fy - q * L, 0.0, 1e-8, 1e-9, "ΣFy");
        assert_mixed(
            r.reaction(0).unwrap().mz + m - q * L * L / 2.0,
            0.0,
            1e-8,
            1e-9,
            "ΣMz",
        );
    }
    println!("[axial+udl+moment] superposition and ΣF/ΣM = 0 verified");
}

// ===========================================================================
// 7. Rotated beam coverage
// ===========================================================================

#[test]
fn test_rotated_beam_angles() {
    let p = 100.0;
    let an_v = -p * L.powi(3) / 3.0; // local tip deflection
    let an_rz = -p * L.powi(2) / 2.0;

    // 0-degree reference (global == local).
    let mut base_local_end: Option<Vec<[f64; 6]>> = None;
    let rot = |deg: f64| -> (f64, f64, f64, f64, f64, f64) {
        let th = deg.to_radians();
        let (c, s) = (th.cos(), th.sin());
        let mut model = BeamModel::new();
        model.add_node(BeamNode::new(0, 0.0, 0.0));
        model.add_node(BeamNode::new(1, L * c, L * s));
        model.add_element(
            BeamElement::new(
                0,
                1,
                Material::new(1.0, 0.3, 1.0, "U"),
                BeamSection::new(1.0, 1.0),
            )
            .unwrap(),
        );
        model.fix_node(0);
        model.add_nodal_force(1, 0, s * p); // global tip force = rotated local (0, -P)
        model.add_nodal_force(1, 1, -c * p);
        let s_ = solve_named(&model, "dense");
        let r = s_.results();
        let (ux, uy, rz) = (
            r.displacement(1).unwrap().ux,
            r.displacement(1).unwrap().uy,
            r.displacement(1).unwrap().rz,
        );
        let (rx, ry, rmz) = (
            r.reaction(0).unwrap().fx,
            r.reaction(0).unwrap().fy,
            r.reaction(0).unwrap().mz,
        );
        (ux, uy, rz, rx, ry, rmz)
    };

    for &deg in &[0.0_f64, 30.0, 45.0, 60.0, 90.0] {
        let (ux, uy, rz, rx, ry, rmz) = rot(deg);
        let th = deg.to_radians();
        let (c, s) = (th.cos(), th.sin());
        // Global response is the rigid rotation of the 0-degree LOCAL response
        // (0, an_v, an_rz): u_global = R(theta) * u_local.
        assert_mixed(ux, -s * an_v, 1e-9, 1e-9, &format!("{:.0}° global ux", deg));
        assert_mixed(uy, c * an_v, 1e-9, 1e-9, &format!("{:.0}° global uy", deg));
        assert_mixed(rz, an_rz, 1e-9, 1e-9, &format!("{:.0}° rz unchanged", deg));
        // Reactions rotate the same way; moment invariant.
        assert_mixed(rx, -s * p, 1e-8, 1e-9, &format!("{:.0}° Rx", deg));
        assert_mixed(ry, c * p, 1e-8, 1e-9, &format!("{:.0}° Ry", deg));
        assert_mixed(rmz, p * L, 1e-8, 1e-9, &format!("{:.0}° Rz", deg));

        // Local end forces identical to the 0-degree case for every solver.
        for &name in &DIRECT {
            let th = deg.to_radians();
            let (c, s) = (th.cos(), th.sin());
            let mut model = BeamModel::new();
            model.add_node(BeamNode::new(0, 0.0, 0.0));
            model.add_node(BeamNode::new(1, L * c, L * s));
            model.add_element(
                BeamElement::new(
                    0,
                    1,
                    Material::new(1.0, 0.3, 1.0, "U"),
                    BeamSection::new(1.0, 1.0),
                )
                .unwrap(),
            );
            model.fix_node(0);
            model.add_nodal_force(1, 0, s * p);
            model.add_nodal_force(1, 1, -c * p);
            let s_ = solve_named(&model, name);
            let fe = s_.element_end_forces().unwrap();
            match &base_local_end {
                None => base_local_end = Some(fe.clone()),
                Some(base) => {
                    for k in 0..6 {
                        assert_mixed(
                            fe[0][k],
                            base[0][k],
                            1e-8,
                            1e-9,
                            &format!("{:.0}° local end force", deg),
                        );
                    }
                }
            }
        }
    }
    println!("[rotated coverage] 0/30/45/60/90°: u_global = R·u_local, local end forces invariant");
}

// ===========================================================================
// 8. Distributed qx / qy combinations
// ===========================================================================

#[test]
fn test_distributed_qx_qy_combinations() {
    let (qx, qy) = (60.0, -80.0);

    for (label, use_x, use_y) in [
        ("qx", true, false),
        ("qy", false, true),
        ("qx+qy", true, true),
    ] {
        let mut model = unit(2);
        let (ax, ay) = if use_x { (qx, 0.0) } else { (0.0, 0.0) };
        let (bx, by) = if use_y { (0.0, qy) } else { (0.0, 0.0) };
        model.add_distributed_load(0, ax + bx, ay + by).unwrap();
        model.add_distributed_load(1, ax + bx, ay + by).unwrap();

        for &name in &DIRECT {
            let s = solve_named(&model, name);
            let r = s.results();
            let rx = r.reaction(0).unwrap().fx;
            let ry = r.reaction(0).unwrap().fy;
            let rmz = r.reaction(0).unwrap().mz;
            // ΣF: reaction + integrated load = 0.
            let fx_tot = if use_x { qx * L } else { 0.0 };
            let fy_tot = if use_y { qy * L } else { 0.0 };
            assert_mixed(rx + fx_tot, 0.0, 1e-8, 1e-9, &format!("{} ΣFx", label));
            assert_mixed(ry + fy_tot, 0.0, 1e-8, 1e-9, &format!("{} ΣFy", label));
            // ΣM about node 0: Rz + (transverse resultant at L/2).
            let m_load = if use_y { fy_tot * (L / 2.0) } else { 0.0 };
            assert_mixed(rmz + m_load, 0.0, 1e-8, 1e-9, &format!("{} ΣMz", label));

            // Global element end forces equal T^T * local (horizontal => equal).
            let fe = s.element_end_forces().unwrap();
            let feg = s.element_end_forces_global().unwrap();
            for e in 0..2 {
                for k in 0..6 {
                    assert_mixed(
                        feg[e][k],
                        fe[e][k],
                        1e-9,
                        1e-9,
                        &format!("{} f_global=T^T f_local", label),
                    );
                }
            }
        }
    }
    println!("[qx/qy combos] equilibrium and f_global = Tᵀ f_local verified");
}

// ===========================================================================
// 9. Applied moment regression
// ===========================================================================

#[test]
fn test_applied_moment_contract_multielement() {
    let m = 100.0;
    let mut model = unit(3);
    model.add_applied_moment(3, m).unwrap();

    for &name in &DIRECT {
        let s = solve_named(&model, name);
        let r = s.results();
        // Contract: constant internal moment +M along the beam.
        for e in 0..3 {
            for &xi in &[0.0, 0.5, 1.0] {
                assert_mixed(
                    s.element_section_forces(e, xi).unwrap().moment,
                    m,
                    1e-9,
                    1e-9,
                    "section M = +M",
                );
            }
        }
        // Free-end element-on-node moment M_j = -M; support reaction Rz = -M.
        let fe = s.element_end_forces().unwrap();
        assert_mixed(fe[2][5], -m, 1e-9, 1e-9, "free-end M_j = -M");
        assert_mixed(r.reaction(0).unwrap().mz, -m, 1e-9, 1e-9, "Rz = -M");
        assert_mixed(r.reaction(0).unwrap().mz + m, 0.0, 1e-9, 1e-12, "ΣMz");
        // Moment continuity at internal nodes (no applied moment there).
        for mid in 0..2 {
            let left = s.element_section_forces(mid, 1.0).unwrap().moment;
            let right = s.element_section_forces(mid + 1, 0.0).unwrap().moment;
            assert_mixed(left, right, 1e-8, 1e-9, "M continuous");
        }
    }
    println!("[applied moment / 3 elements] M_j=-M, section M=+M, Rz=-M, continuity OK");
}

#[test]
fn test_applied_moment_internal_node() {
    let m = 100.0;
    let mut model = unit(2);
    model.add_applied_moment(1, m).unwrap();

    for &name in &DIRECT {
        let s = solve_named(&model, name);
        let r = s.results();
        let left = s.element_section_forces(0, 1.0).unwrap().moment;
        let right = s.element_section_forces(1, 0.0).unwrap().moment;
        // Documented relation: M_right - M_left + M_applied = 0.
        assert_mixed(
            right - left + m,
            0.0,
            1e-9,
            1e-12,
            "M_right - M_left + M_app = 0",
        );
        // Element-on-node moments at the shared node also satisfy it.
        let fe = s.element_end_forces().unwrap();
        assert_mixed(
            fe[0][5] + fe[1][2] + m,
            0.0,
            1e-9,
            1e-12,
            "f_left + f_right + M_app = 0",
        );
        // Reaction moment balances the applied moment.
        assert_mixed(r.reaction(0).unwrap().mz + m, 0.0, 1e-9, 1e-12, "ΣMz");
    }
    println!("[applied moment / internal node] M_right - M_left + M_app = 0");
}

// ===========================================================================
// 11. Extreme-but-valid numerical scales
// ===========================================================================

#[test]
fn test_scale_invariance() {
    let p = 100.0;
    let f = 100.0;
    let m = 100.0;

    for &e in &[1e-6_f64, 1.0, 1e6] {
        // Bending δ = PL³/3EI, θ = PL²/2EI; axial u = FL/EA; reactions independent of E.
        let mut bend = model_with(1, e, 1.0, 1.0);
        bend.add_nodal_force(1, 1, -p);
        let mut ax = model_with(1, e, 1.0, 1.0);
        ax.add_nodal_force(1, 0, f);
        let mut mo = model_with(1, e, 1.0, 1.0);
        mo.add_applied_moment(1, m).unwrap();

        for &name in &DIRECT {
            let sb = solve_named(&bend, name);
            assert_mixed(
                sb.displacement(1, 1),
                -p / (3.0 * e),
                1e-12,
                1e-9,
                &format!("E={:e} {} δ ∝ 1/EI", e, name),
            );
            assert_mixed(
                sb.displacement(1, 2),
                -p / (2.0 * e),
                1e-12,
                1e-9,
                &format!("E={:e} {} θ ∝ 1/EI", e, name),
            );
            assert_mixed(
                sb.reactions()[1],
                p,
                1e-9,
                1e-9,
                &format!("E={:e} {} Ry indep of E", e, name),
            );
            assert_mixed(
                sb.reactions()[2],
                p * L,
                1e-9,
                1e-9,
                &format!("E={:e} {} Rz indep of E", e, name),
            );

            let sa = solve_named(&ax, name);
            assert_mixed(
                sa.displacement(1, 0),
                f / e,
                1e-12,
                1e-9,
                &format!("E={:e} {} u ∝ 1/EA", e, name),
            );
            assert_mixed(
                sa.reactions()[0],
                -f,
                1e-9,
                1e-9,
                &format!("E={:e} {} Rx indep of E", e, name),
            );

            let sm = solve_named(&mo, name);
            assert_mixed(
                sm.displacement(1, 2),
                m / e,
                1e-12,
                1e-9,
                &format!("E={:e} {} θ ∝ 1/EI", e, name),
            );
        }
    }

    // A / I scaling (E fixed at 1): u_axial ∝ 1/A, δ_bending ∝ 1/I.
    for &sc in &[0.5_f64, 1.0, 2.0] {
        let mut ax = model_with(1, 1.0, sc, 1.0);
        ax.add_nodal_force(1, 0, f);
        let mut bend = model_with(1, 1.0, 1.0, sc);
        bend.add_nodal_force(1, 1, -p);
        for &name in &DIRECT {
            assert_mixed(
                solve_named(&ax, name).displacement(1, 0),
                f / sc,
                1e-12,
                1e-9,
                &format!("A={} u ∝ 1/A", sc),
            );
            assert_mixed(
                solve_named(&bend, name).displacement(1, 1),
                -p / (3.0 * sc),
                1e-12,
                1e-9,
                &format!("I={} δ ∝ 1/I", sc),
            );
        }
    }
    println!(
        "[scale invariance] E ∈ {{1e-6,1,1e6}}, A/I ∈ {{0.5,1,2}}: physical scaling holds (3 backends)"
    );
}
