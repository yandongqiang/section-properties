//! Phase 2.6 — Unified beam analysis result / reaction summary API tests.
//!
//! These tests verify that [`BeamSolver::results`] is a pure delegation layer
//! over the existing solver (displacements, reactions, element end forces,
//! section forces, Phase 2.5 sampling), and that reactions satisfy independent
//! global equilibrium — not just self-consistency.

use section_properties::beam_fem::{BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver};
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

fn solve(model: &BeamModel) -> BeamSolver {
    let mut solver = BeamSolver::from_model(model).unwrap();
    let registry = SolverRegistry::default();
    let mut linear = registry.create("dense").unwrap();
    solver.solve(&mut *linear).unwrap();
    solver
}

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

// ===========================================================================
// 1 — single-element cantilever displacement
// ===========================================================================

#[test]
fn test_result_displacement_single_element() {
    let l = 1.0;
    let p = 1000.0;
    let ei = E * second_moment();

    let mut model = cantilever_model(1, l);
    model.add_nodal_force(1, 1, -p);
    let solver = solve(&model);
    let r = solver.results();

    assert_eq!(r.n_nodes(), 2);
    assert_eq!(r.n_elements(), 1);
    assert_eq!(r.displacements().len(), 6);

    let d0 = r.displacement(0).unwrap();
    assert_close(d0.ux, 0.0, 1e-12, "node0 ux");
    assert_close(d0.uy, 0.0, 1e-12, "node0 uy");
    assert_close(d0.rz, 0.0, 1e-12, "node0 rz");

    let d1 = r.displacement(1).unwrap();
    assert_close(d1.ux, 0.0, 1e-12, "tip ux");
    assert_close(d1.uy, -p * l.powi(3) / (3.0 * ei), 1e-9, "tip uy");
    assert_close(d1.rz, -p * l.powi(2) / (2.0 * ei), 1e-9, "tip rz");

    // Raw vector mirrors the typed accessors.
    assert_eq!(r.displacements()[4], d1.uy);
    assert_eq!(r.displacements()[5], d1.rz);
}

// ===========================================================================
// 2 — support reactions (fixed node)
// ===========================================================================

#[test]
fn test_result_reactions_cantilever_tip_force() {
    let l = 1.0;
    let p = 1000.0;

    let mut model = cantilever_model(1, l);
    model.add_nodal_force(1, 1, -p);
    let solver = solve(&model);
    let r = solver.results();

    let r0 = r.reaction(0).unwrap();
    assert_close(r0.fx, 0.0, 1e-9, "R0 fx");
    assert_close(r0.fy, p, 1e-9, "R0 fy");
    assert_close(r0.mz, p * l, 1e-9, "R0 mz");

    // Free node carries no support reaction (exactly zero, not a residual).
    let r1 = r.reaction(1).unwrap();
    assert_eq!(r1.fx, 0.0);
    assert_eq!(r1.fy, 0.0);
    assert_eq!(r1.mz, 0.0);

    // Consistency with the raw solver reaction at constrained DOFs.
    let raw = solver.reactions();
    assert_close(r0.fy, raw[1], 1e-12, "R0 fy == solver.reactions()[1]");
    assert_close(r0.mz, raw[2], 1e-12, "R0 mz == solver.reactions()[2]");
}

// ===========================================================================
// 3 — tip transverse force equilibrium
// ===========================================================================

#[test]
fn test_result_equilibrium_tip_force() {
    let l = 1.0;
    let p = 1000.0;

    let mut model = cantilever_model(1, l);
    model.add_nodal_force(1, 1, -p);
    let solver = solve(&model);
    let r = solver.results();

    let r0 = r.reaction(0).unwrap();
    // ΣFy = 0 and ΣMz about node 0 = 0.
    assert_close(r0.fy + (-p), 0.0, 1e-9, "ΣFy");
    assert_close(r0.mz + (-p) * l, 0.0, 1e-9, "ΣMz");
}

// ===========================================================================
// 4 — tip applied moment equilibrium
// ===========================================================================

#[test]
fn test_result_equilibrium_tip_moment() {
    let l = 1.0;
    let m = 1000.0;

    let mut model = cantilever_model(1, l);
    model.add_applied_moment(1, m).unwrap();
    let solver = solve(&model);
    let r = solver.results();

    let r0 = r.reaction(0).unwrap();
    assert_close(r0.fy, 0.0, 1e-9, "R0 fy");
    assert_close(r0.mz + m, 0.0, 1e-9, "Rz + M = 0");
}

// ===========================================================================
// 5 — axial force equilibrium
// ===========================================================================

#[test]
fn test_result_equilibrium_axial_force() {
    let l = 1.0;
    let f = 500.0;

    let mut model = cantilever_model(1, l);
    model.add_nodal_force(1, 0, f); // global +x at tip
    let solver = solve(&model);
    let r = solver.results();

    let r0 = r.reaction(0).unwrap();
    assert_close(r0.fx + f, 0.0, 1e-9, "Rx + Fx = 0");
    assert_close(r0.fy, 0.0, 1e-9, "R0 fy");
    assert_close(r0.mz, 0.0, 1e-9, "R0 mz");
}

// ===========================================================================
// 6 — distributed load equilibrium
// ===========================================================================

#[test]
fn test_result_equilibrium_distributed_load() {
    let l = 1.0;
    let q = 1000.0; // downward intensity

    let mut model = cantilever_model(1, l);
    model.add_distributed_load(0, 0.0, -q).unwrap();
    let solver = solve(&model);
    let r = solver.results();

    let r0 = r.reaction(0).unwrap();
    // Total upward reaction balances the integrated downward load q·L, and the
    // reaction moment balances the load resultant at L/2.
    assert_close(r0.fy, q * l, 1e-9, "Ry = qL");
    assert_close(r0.fy + (-q * l), 0.0, 1e-9, "ΣFy");
    assert_close(r0.mz, q * l * l / 2.0, 1e-9, "Rz = qL²/2");
    assert_close(r0.mz + (-q * l * l / 2.0), 0.0, 1e-9, "ΣMz");
}

// ===========================================================================
// 7 — multi-element reactions (unequal lengths), supports only at constraints
// ===========================================================================

#[test]
fn test_result_multi_element_reactions() {
    // L0 = 1, L1 = 2, L2 = 3 (total 6).
    let p = 1000.0;
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, 1.0, 0.0));
    model.add_node(BeamNode::new(2, 3.0, 0.0));
    model.add_node(BeamNode::new(3, 6.0, 0.0));
    model.add_element(BeamElement::new(0, 1, steel(), section()).unwrap());
    model.add_element(BeamElement::new(1, 2, steel(), section()).unwrap());
    model.add_element(BeamElement::new(2, 3, steel(), section()).unwrap());
    model.fix_node(0);
    model.add_nodal_force(3, 1, -p);
    let solver = solve(&model);
    let r = solver.results();

    assert_eq!(r.n_nodes(), 4);
    assert_eq!(r.n_elements(), 3);

    // Only the constrained node reports a reaction.
    let r0 = r.reaction(0).unwrap();
    assert_close(r0.fy, p, 1e-9, "Ry");
    assert_close(r0.mz, p * 6.0, 1e-9, "Rz");
    for n in 1..4 {
        let rn = r.reaction(n).unwrap();
        assert_eq!(rn.fx, 0.0, "node {} fx", n);
        assert_eq!(rn.fy, 0.0, "node {} fy", n);
        assert_eq!(rn.mz, 0.0, "node {} mz", n);
    }

    // Global equilibrium with the applied tip force.
    assert_close(r0.fy + (-p), 0.0, 1e-9, "ΣFy");
    assert_close(r0.mz + (-p) * 6.0, 0.0, 1e-9, "ΣMz");

    // Element end forces at the shared node 1 remain two separate results and
    // balance (no external load there).
    let f0 = r.element_end_forces(0).unwrap();
    let f1 = r.element_end_forces(1).unwrap();
    assert_close(f0[3] + f1[0], 0.0, 1e-9, "shared node N");
    assert_close(f0[4] + f1[1], 0.0, 1e-9, "shared node V");
    assert_close(f0[5] + f1[2], 0.0, 1e-9, "shared node M");
}

// ===========================================================================
// 8 — internal-node applied moment regression
// ===========================================================================

#[test]
fn test_result_internal_node_applied_moment() {
    let m = 1000.0;

    let mut model = cantilever_model(2, 1.0);
    model.add_applied_moment(1, m).unwrap();
    let solver = solve(&model);
    let r = solver.results();

    let r0 = r.reaction(0).unwrap();
    assert_close(r0.fy, 0.0, 1e-9, "Ry");
    assert_close(r0.mz + m, 0.0, 1e-9, "Rz + M = 0");

    // Same section forces as the authoritative API (no extra correction).
    let left = r.section_forces(0, 1.0).unwrap();
    let right = r.section_forces(1, 0.0).unwrap();
    let direct_left = solver.element_section_forces(0, 1.0).unwrap();
    let direct_right = solver.element_section_forces(1, 0.0).unwrap();
    assert_close(left.moment, direct_left.moment, 1e-15, "left == direct");
    assert_close(right.moment, direct_right.moment, 1e-15, "right == direct");
    // Equilibrium: M_right - M_left + M_ext = 0.
    assert_close(
        right.moment - left.moment + m,
        0.0,
        1e-9,
        "nodal equilibrium",
    );
}

// ===========================================================================
// 9 — multiple applied moments accumulate
// ===========================================================================

#[test]
fn test_result_multiple_applied_moments() {
    let (m1, m2) = (600.0, 400.0);

    let mut model = cantilever_model(1, 1.0);
    model.add_applied_moment(1, m1).unwrap();
    model.add_applied_moment(1, m2).unwrap();
    let solver = solve(&model);
    let r = solver.results();

    let r0 = r.reaction(0).unwrap();
    assert_close(r0.mz + (m1 + m2), 0.0, 1e-9, "Rz + (M1+M2) = 0");
}

// ===========================================================================
// 10 — element end-force consistency with BeamSolver
// ===========================================================================

#[test]
fn test_result_element_end_force_consistency() {
    let mut model = cantilever_model(3, 1.0);
    model.add_nodal_force(3, 1, -1000.0);
    model.add_distributed_load(1, 0.0, -500.0).unwrap();
    let solver = solve(&model);
    let r = solver.results();

    let direct = solver.element_end_forces().unwrap();
    for (e, de) in direct.iter().enumerate() {
        let a = r.element_end_forces(e).unwrap();
        for (i, (av, dv)) in a.iter().zip(de.iter()).enumerate() {
            assert_close(*av, *dv, 1e-15, &format!("end force e{}[{}]", e, i));
        }
    }
}

// ===========================================================================
// 11 — section-force consistency with BeamSolver
// ===========================================================================

#[test]
fn test_result_section_force_consistency() {
    let mut model = cantilever_model(2, 1.0);
    model.add_point_load(0, 0.5, 0.0, -1000.0, 0.0).unwrap();
    model.add_distributed_load(1, 0.0, -300.0).unwrap();
    let solver = solve(&model);
    let r = solver.results();

    for e in 0..2 {
        for &xi in &[0.0, 0.25, 0.5, 0.75, 1.0] {
            let a = r.section_forces(e, xi).unwrap();
            let b = solver.element_section_forces(e, xi).unwrap();
            assert_close(a.axial, b.axial, 1e-15, "N");
            assert_close(a.shear, b.shear, 1e-15, "V");
            assert_close(a.moment, b.moment, 1e-15, "M");
        }
    }
}

// ===========================================================================
// 12 — Phase 2.5 sampling consistency
// ===========================================================================

#[test]
fn test_result_sampling_consistency() {
    let mut model = cantilever_model(3, 1.0);
    model.add_nodal_force(3, 1, -1000.0);
    model.add_point_moment(1, 0.5, 500.0).unwrap();
    let solver = solve(&model);
    let r = solver.results();

    let n = 5;
    let a = r.sample_forces(n).unwrap();
    let b = solver.sample_beam_forces(n).unwrap();
    assert_eq!(a.len(), b.len());
    for (sa, sb) in a.iter().zip(b.iter()) {
        assert_eq!(sa.element_index, sb.element_index);
        assert_close(sa.xi, sb.xi, 1e-15, "xi");
        assert_close(sa.x, sb.x, 1e-15, "x");
        assert_close(sa.section_forces.axial, sb.section_forces.axial, 1e-15, "N");
        assert_close(sa.section_forces.shear, sb.section_forces.shear, 1e-15, "V");
        assert_close(
            sa.section_forces.moment,
            sb.section_forces.moment,
            1e-15,
            "M",
        );
    }

    // Diagram accessor delegates too.
    let d = r.diagram(n).unwrap();
    assert_eq!(d.samples.len(), b.len());
    assert_close(
        d.moment()[0],
        b[0].section_forces.moment,
        1e-15,
        "diagram M",
    );
}

// ===========================================================================
// 13 — invalid node / element access
// ===========================================================================

#[test]
fn test_result_invalid_node_and_element_access() {
    let mut model = cantilever_model(2, 1.0);
    model.add_nodal_force(2, 1, -1000.0);
    let solver = solve(&model);
    let r = solver.results();

    // Valid (3 nodes: 0, 1, 2).
    assert!(r.displacement(1).is_ok());
    assert!(r.displacement(2).is_ok());
    assert!(r.reaction(0).is_ok());
    assert!(r.element_end_forces(1).is_ok());

    // Invalid node index.
    assert!(r.displacement(3).is_err());
    assert!(r.displacement(999).is_err());
    assert!(r.reaction(3).is_err());
    assert!(r.reaction(999).is_err());

    // Invalid element index.
    assert!(r.element_end_forces(2).is_err());
    assert!(r.element_end_forces(999).is_err());
    assert!(r.section_forces(2, 0.5).is_err());
    assert!(r.section_forces(999, 0.5).is_err());
}

// ===========================================================================
// 14 — invalid xi
// ===========================================================================

#[test]
fn test_result_invalid_xi() {
    let mut model = cantilever_model(1, 1.0);
    model.add_nodal_force(1, 1, -1000.0);
    let solver = solve(&model);
    let r = solver.results();

    assert!(r.section_forces(0, 0.0).is_ok());
    assert!(r.section_forces(0, 1.0).is_ok());
    assert!(r.section_forces(0, -1e-9).is_err());
    assert!(r.section_forces(0, 1.0 + 1e-9).is_err());
    assert!(r.section_forces(0, f64::NAN).is_err());
}

// ===========================================================================
// 15 — invalid sampling count
// ===========================================================================

#[test]
fn test_result_invalid_sampling_count() {
    let mut model = cantilever_model(2, 1.0);
    model.add_nodal_force(2, 1, -1000.0);
    let solver = solve(&model);
    let r = solver.results();

    assert!(r.sample_forces(0).is_err());
    assert!(r.sample_forces(1).is_err());
    assert!(r.sample_forces(2).is_ok());
    assert!(r.diagram(1).is_err());
    assert!(r.diagram(2).is_ok());
}

// ===========================================================================
// 16 — zero-length element is rejected by the model-validation path
// ===========================================================================

#[test]
fn test_result_zero_length_element_rejected() {
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, 0.0, 0.0)); // coincident
    model.add_element(BeamElement::new(0, 1, steel(), section()).unwrap());
    model.fix_node(0);

    assert!(
        BeamSolver::from_model(&model).is_err(),
        "from_model must reject a zero-length element"
    );
}

// ===========================================================================
// Result construction does not solve: an unsolved solver keeps zero
// displacements (documented semantic).
// ===========================================================================

#[test]
fn test_result_is_view_not_solve() {
    let mut model = cantilever_model(1, 1.0);
    model.add_nodal_force(1, 1, -1000.0);

    // Build but do NOT solve.
    let solver = BeamSolver::from_model(&model).unwrap();
    let r = solver.results();
    assert_eq!(r.displacements().iter().filter(|v| **v != 0.0).count(), 0);
}
