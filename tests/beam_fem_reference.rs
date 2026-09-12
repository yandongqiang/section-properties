//! Beam FEM reference validation.
//!
//! # Reference provenance
//!
//! The intended reference for this phase was a Python `section-properties`
//! Beam FEM implementation. Investigation of the available Python environment
//! found:
//!
//! ```text
//! Python:            3.14.2
//! Installed package: sectionproperties 3.10.2  (cross-section / warping library)
//! Beam FEM in it:    NONE
//! ```
//!
//! `sectionproperties` provides plane-stress (Tri6) section analysis, warping,
//! and `calculate_frame_properties()` — but it contains **no beam/frame finite
//! element** (no beam stiffness matrix, no cantilever/frame solver; the strings
//! `euler`, `bernoulli`, `truss`, `stiffness_matrix`, `cantilever` do not occur
//! in the package). There is therefore no Python Beam FEM result to compare
//! against, and one is deliberately **not** fabricated.
//!
//! # Reference used here
//!
//! Closed-form Euler–Bernoulli beam solutions. Each benchmark is solved with
//! every applicable Rust direct backend (`dense`, `skyline_ldlt`, `sparse_lu`)
//! and compared against the analytical result (not merely against each other).
//!
//! # Sign / formulation conventions (Rust, under test)
//!
//! ```text
//! local x      : node_i -> node_j
//! local y      : transverse, positive upward
//! rotation     : theta about +z, counter-clockwise positive
//! shear V      : dM/dx = V
//! moment M     : sagging positive (M = E·I·v'')
//! element end  : element-on-node forces [N_i,V_i,M_i,N_j,V_j,M_j], r = f_eq - K_e u_e
//! reaction     : external support reaction, R = K_original·u - f_global (global)
//! ```
//! Distributed/point loads are LOCAL; applied moments are GLOBAL nodal `theta`
//! loads. An applied moment is NOT an element equivalent load.
//!
//! # Tolerance policy
//!
//! `|a-b| <= abs + rel*max(|a|,|b|)`, with `abs` scaled per quantity class
//! (1e-9 for O(1)-O(100) displacements/rotations/forces/moments) and `rel`
//! 1e-9. Near-zero quantities use the absolute term.

use section_properties::SolverSelection;
use section_properties::beam_fem::{BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver};
use section_properties::material::Material;

const DIRECT: [&str; 3] = ["dense", "skyline_ldlt", "sparse_lu"];

// Benchmark parameters (Case A of the phase specification).
const L: f64 = 1.0;
const E: f64 = 1.0;
const A: f64 = 1.0;
const I: f64 = 1.0;
const EI: f64 = E * I;

fn steel() -> Material {
    Material::new(E, 0.3, 7850.0, "Unit")
}

fn section() -> BeamSection {
    BeamSection::new(A, I)
}

/// Cantilever of `n` equal elements, fixed at node 0.
fn cantilever(n: usize) -> BeamModel {
    let mut model = BeamModel::new();
    let dx = L / n as f64;
    for i in 0..=n {
        model.add_node(BeamNode::new(i, i as f64 * dx, 0.0));
    }
    for i in 0..n {
        model.add_element(BeamElement::new(i, i + 1, steel(), section()).unwrap());
    }
    model.fix_node(0);
    model
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
        "{}: {} vs {} (|diff| = {:.3e} > {:.3e})",
        label,
        a,
        b,
        (a - b).abs(),
        bound
    );
}

/// Run `f` for every direct backend and return the max abs difference against
/// the analytical value `an` over the quantities produced.
fn max_err_all_backends<F: Fn(&BeamSolver) -> Vec<(f64, f64)>>(model: &BeamModel, an: F) -> f64 {
    let mut worst = 0.0f64;
    for &name in &DIRECT {
        let s = solve_named(model, name);
        for (got, expected) in an(&s) {
            worst = worst.max((got - expected).abs());
        }
    }
    worst
}

// ===========================================================================
// Case A — Cantilever + tip transverse force
// ===========================================================================

#[test]
fn test_cantilever_tip_load_reference() {
    let p = 100.0;
    let mut model = cantilever(1);
    model.add_nodal_force(1, 1, -p);

    let u_tip = -p * L.powi(3) / (3.0 * EI);
    let th_tip = -p * L.powi(2) / (2.0 * EI);

    for &name in &DIRECT {
        let s = solve_named(&model, name);
        let r = s.results();
        assert_mixed(
            r.displacement(1).unwrap().uy,
            u_tip,
            1e-9,
            1e-9,
            &format!("{} tip uy", name),
        );
        assert_mixed(
            r.displacement(1).unwrap().rz,
            th_tip,
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
            p * L,
            1e-9,
            1e-9,
            &format!("{} Rz", name),
        );
        // Element end forces: [0, -P, -P·L, 0, P, 0].
        let fe = r.element_end_forces(0).unwrap();
        assert_mixed(fe[1], -p, 1e-9, 1e-9, &format!("{} N_i", name));
        assert_mixed(fe[2], -p * L, 1e-9, 1e-9, &format!("{} M_i", name));
        assert_mixed(fe[4], p, 1e-9, 1e-9, &format!("{} V_j", name));
        assert_mixed(fe[5], 0.0, 1e-9, 1e-9, &format!("{} M_j", name));
        // Section forces: V = +P, M(x) = -P(L-x).
        assert_mixed(
            s.element_section_forces(0, 0.5).unwrap().shear,
            p,
            1e-8,
            1e-9,
            "V",
        );
        assert_mixed(
            s.element_section_forces(0, 0.5).unwrap().moment,
            -p * L / 2.0,
            1e-8,
            1e-9,
            "M",
        );
    }
    println!(
        "[A tip load] δ={:.6e} θ={:.6e} (analytical; 3 backends agree)",
        u_tip, th_tip
    );
}

// ===========================================================================
// Case B — Cantilever + uniform transverse load
// ===========================================================================

#[test]
fn test_cantilever_udl_reference() {
    let q = 100.0;
    let mut model = cantilever(2);
    model.add_distributed_load(0, 0.0, -q).unwrap();
    model.add_distributed_load(1, 0.0, -q).unwrap();

    let u_tip = -q * L.powi(4) / (8.0 * EI);
    let th_tip = -q * L.powi(3) / (6.0 * EI);

    for &name in &DIRECT {
        let s = solve_named(&model, name);
        let r = s.results();
        assert_mixed(
            r.displacement(2).unwrap().uy,
            u_tip,
            1e-9,
            1e-9,
            &format!("{} tip uy", name),
        );
        assert_mixed(
            r.displacement(2).unwrap().rz,
            th_tip,
            1e-9,
            1e-9,
            &format!("{} tip rz", name),
        );
        assert_mixed(
            r.reaction(0).unwrap().fy,
            q * L,
            1e-9,
            1e-9,
            &format!("{} Ry", name),
        );
        assert_mixed(
            r.reaction(0).unwrap().mz,
            q * L * L / 2.0,
            1e-9,
            1e-9,
            &format!("{} Rz", name),
        );
        // Analytical section diagram V(x) = q(L-x), M(x) = -q(L-x)^2/2.
        let dx = L / 2.0;
        for (elem, x0) in [(0usize, 0.0), (1usize, dx)] {
            for &xi in &[0.0, 0.5, 1.0] {
                let x = x0 + xi * dx;
                let sf = s.element_section_forces(elem, xi).unwrap();
                assert_mixed(sf.shear, q * (L - x), 1e-8, 1e-9, "V(x)");
                assert_mixed(sf.moment, -q * (L - x).powi(2) / 2.0, 1e-8, 1e-9, "M(x)");
            }
        }
    }
    println!(
        "[B udl] δ={:.6e} θ={:.6e} Ry={:.6e} Rz={:.6e}",
        u_tip,
        th_tip,
        q * L,
        q * L * L / 2.0
    );
}

// ===========================================================================
// Case C — Cantilever + tip applied moment (contract)
// ===========================================================================

#[test]
fn test_cantilever_tip_moment_reference() {
    let m = 100.0;
    let mut model = cantilever(1);
    model.add_applied_moment(1, m).unwrap();

    let th_tip = m * L / EI;
    let u_tip = m * L * L / (2.0 * EI);

    for &name in &DIRECT {
        let s = solve_named(&model, name);
        let r = s.results();
        assert_mixed(
            r.displacement(1).unwrap().rz,
            th_tip,
            1e-9,
            1e-9,
            &format!("{} θ", name),
        );
        assert_mixed(
            r.displacement(1).unwrap().uy,
            u_tip,
            1e-9,
            1e-9,
            &format!("{} δ", name),
        );
        // Reaction moment balances the applied moment.
        assert_mixed(
            r.reaction(0).unwrap().mz,
            -m,
            1e-9,
            1e-9,
            &format!("{} Rz", name),
        );
        // Documented contract: element-on-node free-end moment M_j = -M,
        // fixed-end M_i = +M; the internal (sagging) moment is +M everywhere.
        let fe = r.element_end_forces(0).unwrap();
        assert_mixed(fe[2], m, 1e-9, 1e-9, &format!("{} M_i", name));
        assert_mixed(fe[5], -m, 1e-9, 1e-9, &format!("{} M_j = -M", name));
        assert_mixed(
            s.element_section_forces(0, 1.0).unwrap().moment,
            m,
            1e-9,
            1e-9,
            "section M(L)",
        );
        assert_mixed(
            s.element_section_forces(0, 0.0).unwrap().moment,
            m,
            1e-9,
            1e-9,
            "section M(0)",
        );
    }
    println!(
        "[C tip moment] θ={:.6e} δ={:.6e} Rz=-M; contract M_j=-M, section M=+M",
        th_tip, u_tip
    );
}

// ===========================================================================
// Case D — Combined loading (UDL + point + applied moment + nodal force)
// ===========================================================================

#[test]
fn test_combined_loading_reference() {
    let mut model = cantilever(2);
    model.add_distributed_load(0, 0.0, -40.0).unwrap();
    model.add_point_load(1, 0.5, 0.0, -50.0, 20.0).unwrap();
    model.add_applied_moment(1, 30.0).unwrap();
    model.add_nodal_force(2, 1, -100.0);

    // Analytical global equilibrium about node 0 (origin):
    //   ΣFy = Ry - 40·(L/2) - 50 - 100 = 0 -> Ry = 170
    //   ΣMz = Rz + 30 + (-40·L/2)(L/4) + (-50)(3L/4) + 20 + (-100)(L) = 0
    let ry_an = 40.0 * (L / 2.0) + 50.0 + 100.0;
    let rz_an =
        -(30.0 + (-40.0 * L / 2.0) * (L / 4.0) + (-50.0) * (3.0 * L / 4.0) + 20.0 + (-100.0) * L);

    for &name in &DIRECT {
        let s = solve_named(&model, name);
        let r = s.results();
        assert_mixed(
            r.reaction(0).unwrap().fy,
            ry_an,
            1e-8,
            1e-9,
            &format!("{} Ry", name),
        );
        assert_mixed(
            r.reaction(0).unwrap().mz,
            rz_an,
            1e-8,
            1e-9,
            &format!("{} Rz", name),
        );
        // ΣFy, ΣMz residuals.
        assert_mixed(r.reaction(0).unwrap().fy - ry_an, 0.0, 1e-8, 1e-12, "ΣFy");
        assert_mixed(r.reaction(0).unwrap().mz - rz_an, 0.0, 1e-8, 1e-12, "ΣMz");
        // Free DOFs carry no reaction.
        let reactions = s.reactions();
        for &val in reactions.iter().take(9).skip(3) {
            assert_mixed(val, 0.0, 1e-7, 1e-9, "free reaction");
        }
    }
    println!(
        "[D combined] Ry={:.6e} Rz={:.6e} (analytical, 3 backends)",
        ry_an, rz_an
    );
}

// ===========================================================================
// Case E — Rotated beam 0° / 45° / 90°
// ===========================================================================

#[test]
fn test_rotated_beam_reference() {
    let p = 100.0;

    for &deg in &[0.0_f64, 45.0, 90.0] {
        let th = deg.to_radians();
        let (c, s) = (th.cos(), th.sin());

        let mut model = BeamModel::new();
        model.add_node(BeamNode::new(0, 0.0, 0.0));
        model.add_node(BeamNode::new(1, L * c, L * s));
        model.add_element(BeamElement::new(0, 1, steel(), section()).unwrap());
        model.fix_node(0);
        // Global tip force equivalent to LOCAL (0, -P).
        model.add_nodal_force(1, 0, s * p);
        model.add_nodal_force(1, 1, -c * p);

        for &name in &DIRECT {
            let s_ = solve_named(&model, name);
            let r = s_.results();

            // Global displacement: u_global = Tᵀ u_local with local tip
            // (0, -P·L³/3EI, -P·L²/2EI).
            let v_l = -p * L.powi(3) / (3.0 * EI);
            let u_g = -s * v_l; // Tᵀ row 0: (c, -s); with u_local=(0,v_l): u_gx = -s·v_l
            let v_g = c * v_l;
            assert_mixed(
                r.displacement(1).unwrap().ux,
                u_g,
                1e-9,
                1e-9,
                &format!("{} {:.0}° global ux", name, deg),
            );
            assert_mixed(
                r.displacement(1).unwrap().uy,
                v_g,
                1e-9,
                1e-9,
                &format!("{} {:.0}° global uy", name, deg),
            );

            // Global reactions: |R| = P, Rz = P·L.
            let rmag =
                (r.reaction(0).unwrap().fx.powi(2) + r.reaction(0).unwrap().fy.powi(2)).sqrt();
            assert_mixed(rmag, p, 1e-8, 1e-9, &format!("{} {:.0}° |R|", name, deg));
            assert_mixed(
                r.reaction(0).unwrap().mz,
                p * L,
                1e-8,
                1e-9,
                &format!("{} {:.0}° Rz", name, deg),
            );
            assert_mixed(r.reaction(0).unwrap().fx + s * p, 0.0, 1e-8, 1e-12, "ΣFx");
            assert_mixed(r.reaction(0).unwrap().fy - c * p, 0.0, 1e-8, 1e-12, "ΣFy");

            // Local end forces are rotation-invariant.
            let fe = r.element_end_forces(0).unwrap();
            assert_mixed(
                fe[1],
                -p,
                1e-9,
                1e-9,
                &format!("{} {:.0}° local V_i", name, deg),
            );
            assert_mixed(
                fe[4],
                p,
                1e-9,
                1e-9,
                &format!("{} {:.0}° local V_j", name, deg),
            );

            // Global end forces = Tᵀ local end forces.
            let ni = model.nodes[0].point();
            let nj = model.nodes[1].point();
            let t = model.elements[0].transformation_matrix(ni, nj);
            let feg = s_.element_end_forces_global().unwrap()[0];
            for i in 0..6 {
                let mut g = 0.0;
                for j in 0..6 {
                    g += t[j][i] * fe[j];
                }
                assert_mixed(g, feg[i], 1e-9, 1e-9, "f_global = Tᵀ f_local");
            }
        }
    }
    println!("[E rotated] local invariance + global reactions verified at 0/45/90°");
}

// ===========================================================================
// End-force sign / semantics contract
// ===========================================================================

#[test]
fn test_end_force_sign_convention() {
    // (1) No span load + tip applied moment: a pure couple that balances itself
    //     and does NOT appear as an element equivalent load.
    let m = 100.0;
    let mut model = cantilever(1);
    model.add_applied_moment(1, m).unwrap();
    for &name in &DIRECT {
        let fe = solve_named(&model, name).element_end_forces().unwrap()[0];
        assert_mixed(fe[2], m, 1e-9, 1e-9, "M_i = +M");
        assert_mixed(fe[5], -m, 1e-9, 1e-9, "M_j = -M");
        assert_mixed(fe[2] + fe[5], 0.0, 1e-9, 1e-12, "couple balances");
    }

    // (2) Tip transverse force: element-on-node transverse forces are
    //     [-P, ..., +P] (the element pushes down on the fixed node, up on the
    //     free node), while the internal shear is +P.
    let p = 100.0;
    let mut model = cantilever(1);
    model.add_nodal_force(1, 1, -p);
    for &name in &DIRECT {
        let s = solve_named(&model, name);
        let fe = s.element_end_forces().unwrap()[0];
        assert_mixed(fe[1], -p, 1e-9, 1e-9, "V_i = -P");
        assert_mixed(fe[4], p, 1e-9, 1e-9, "V_j = +P");
        assert_mixed(
            s.element_section_forces(0, 0.5).unwrap().shear,
            p,
            1e-8,
            1e-9,
            "internal V = +P",
        );
    }

    // (3) Element end-force resultant equals the element's applied loads, and
    //     an applied moment is excluded from that load resultant.
    let mut model = cantilever(2);
    model.add_distributed_load(0, 0.0, -40.0).unwrap();
    model.add_point_load(1, 0.5, 0.0, -50.0, 20.0).unwrap();
    model.add_applied_moment(1, 30.0).unwrap();
    let dx = L / 2.0;
    for &name in &DIRECT {
        let fe = solve_named(&model, name).element_end_forces().unwrap();
        // elem 0: only the UDL -40 over dx -> resultant fy = -40*dx, m_about_i = -40*dx*(dx/2)
        let sum_fy = fe[0][1] + fe[0][4];
        let sum_m = fe[0][2] + fe[0][5] + fe[0][4] * dx;
        assert_mixed(sum_fy, -40.0 * dx, 1e-8, 1e-9, "e0 ΣFy = load");
        assert_mixed(sum_m, -40.0 * dx * (dx / 2.0), 1e-8, 1e-9, "e0 ΣM = load");
    }
    println!(
        "[end-force contract] applied moment excluded from element loads; couples/resultants verified"
    );
}

// ===========================================================================
// Reaction / equilibrium reference
// ===========================================================================

#[test]
fn test_reaction_equilibrium_reference() {
    let mut model = cantilever(3);
    model.add_distributed_load(1, 0.0, -60.0).unwrap();
    model.add_point_load(0, 0.25, 0.0, -30.0, 0.0).unwrap();
    model.add_applied_moment(2, 45.0).unwrap();
    model.add_nodal_force(3, 1, -80.0);
    let dx = L / 3.0;

    // Analytical reactions from global equilibrium.
    let q = 60.0;
    let ry = 30.0 + q * dx + 80.0;
    // Moments about origin: applied moment, UDL resultant at element-1 midspan,
    // point load at x=dx/4, tip force at x=L.
    let rz = -(45.0 + (-30.0) * (dx / 4.0) + (-q * dx) * (dx + dx / 2.0) + (-80.0) * L);

    for &name in &DIRECT {
        let s = solve_named(&model, name);
        let reactions = s.reactions();
        let sum_fy: f64 = reactions[1];
        let sum_mz: f64 = reactions[2];
        assert_mixed(sum_fy, ry, 1e-8, 1e-9, &format!("{} Ry", name));
        assert_mixed(sum_mz, rz, 1e-8, 1e-9, &format!("{} Rz", name));
        // ΣFy = 0 and ΣMz = 0 residuals (external + reaction).
        assert_mixed(sum_fy - ry, 0.0, 1e-8, 1e-12, "ΣFy residual");
        assert_mixed(sum_mz - rz, 0.0, 1e-8, 1e-12, "ΣMz residual");
    }
    println!(
        "[reaction reference] Ry={:.6e} Rz={:.6e} (analytical, 3 backends)",
        ry, rz
    );
}

// ===========================================================================
// Mesh refinement study (determinate -> nodal-exact; errors recorded)
// ===========================================================================

#[test]
fn test_mesh_refinement_convergence() {
    let p = 100.0;
    let q = 100.0;

    println!("[mesh study] relative error vs analytical, per mesh (3 backends)");
    for &n in &[1usize, 2, 4, 8] {
        // Tip force.
        let mut mt = cantilever(n);
        mt.add_nodal_force(n, 1, -p);
        let uy_an = -p * L.powi(3) / (3.0 * EI);
        let e_tip = max_err_all_backends(&mt, |s| {
            vec![
                (s.displacement(n, 1), uy_an),
                (s.reactions()[1], p),
                (s.reactions()[2], p * L),
            ]
        });

        // UDL.
        let mut mu = cantilever(n);
        for e in 0..n {
            mu.add_distributed_load(e, 0.0, -q).unwrap();
        }
        let uy_an_u = -q * L.powi(4) / (8.0 * EI);
        let e_udl = max_err_all_backends(&mu, |s| {
            vec![
                (s.displacement(n, 1), uy_an_u),
                (s.reactions()[1], q * L),
                (s.reactions()[2], q * L * L / 2.0),
            ]
        });

        println!(
            "  n={:2}  tip-force max|Δ|={:.3e}   udl max|Δ|={:.3e}",
            n, e_tip, e_udl
        );

        // These statically determinate quantities are nodal-exact for every
        // mesh (equilibrium-determined), so a uniform tight bound is justified.
        assert!(e_tip < 1e-9, "n={}: tip-force error {:.3e}", n, e_tip);
        assert!(e_udl < 1e-9, "n={}: udl error {:.3e}", n, e_udl);
    }
}
