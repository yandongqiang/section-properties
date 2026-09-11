//! Phase 2.5 — Beam internal-force diagram / sampling API tests.
//!
//! These tests exercise the thin sampling layer over
//! `BeamSolver::element_section_forces` (`sample_element_forces`,
//! `sample_beam_forces`, `beam_force_diagram`). They verify coordinates,
//! ordering, shared-node duplication and discontinuity preservation — never a
//! second N/V/M formulation.
//!
//! Sign convention (unchanged from Phase 2.4; local element axes):
//! `axial` tension positive, `shear` with `d(moment)/dx = shear`,
//! `moment` sagging positive.

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

/// Uniform horizontal cantilever of `n_elem` equal elements, fixed at node 0.
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
// Test 1 — single element, no loads
// ===========================================================================

#[test]
fn test_diagram_single_element_no_loads() {
    let l = 1.0;
    let model = cantilever_model(1, l);
    let solver = solve(&model);

    let samples = solver.sample_element_forces(0, 5).unwrap();
    assert_eq!(samples.len(), 5);
    for (k, s) in samples.iter().enumerate() {
        assert_eq!(s.element_index, 0);
        let xi_expected = k as f64 / 4.0;
        assert_close(s.xi, xi_expected, 1e-15, "xi");
        assert_close(s.x, xi_expected * l, 1e-15, "x");
        assert_close(s.section_forces.axial, 0.0, 1e-12, "N");
        assert_close(s.section_forces.shear, 0.0, 1e-12, "V");
        assert_close(s.section_forces.moment, 0.0, 1e-12, "M");
    }
}

// ===========================================================================
// Test 2 — cantilever tip transverse force
// ===========================================================================

#[test]
fn test_diagram_cantilever_tip_force() {
    let l = 1.0;
    let p = 1000.0; // downward

    let mut model = cantilever_model(1, l);
    model.add_nodal_force(1, 1, -p);
    let solver = solve(&model);

    let samples = solver.sample_element_forces(0, 9).unwrap();
    for s in &samples {
        let x = s.xi * l;
        // Existing convention: V(x) = +P, M(x) = -P(L - x), N = 0.
        assert_close(s.section_forces.axial, 0.0, 1e-12, "N");
        assert_close(s.section_forces.shear, p, 1e-9, "V");
        assert_close(s.section_forces.moment, -p * (l - x), 1e-9, "M");
        // Cross-check that the sample equals the authoritative API directly.
        let direct = solver.element_section_forces(0, s.xi).unwrap();
        assert_close(s.section_forces.shear, direct.shear, 1e-15, "V==direct");
        assert_close(s.section_forces.moment, direct.moment, 1e-15, "M==direct");
    }
}

// ===========================================================================
// Test 3 — uniform transverse load: V linear, M quadratic, dV/dx = q_y, dM/dx = V
// ===========================================================================

#[test]
fn test_diagram_uniform_transverse_load_kinematics() {
    let l = 1.0;
    let q = 1000.0; // downward intensity, stores qy = -q

    let mut model = cantilever_model(1, l);
    model.add_distributed_load(0, 0.0, -q).unwrap();
    let solver = solve(&model);

    let n = 101;
    let samples = solver.sample_element_forces(0, n).unwrap();
    let dx = l / (n - 1) as f64;

    // Analytical for this convention: V(x) = q(L - x), M(x) = -q(L - x)^2 / 2.
    for s in &samples {
        let x = s.xi * l;
        assert_close(s.section_forces.shear, q * (l - x), 1e-8, "V analytic");
        assert_close(
            s.section_forces.moment,
            -q * (l - x).powi(2) / 2.0,
            1e-8,
            "M analytic",
        );
    }

    // dV/dx = q_y = -q (V is linear): consecutive slope is exact.
    for k in 0..(n - 1) {
        let slope = (samples[k + 1].section_forces.shear - samples[k].section_forces.shear) / dx;
        assert_close(slope, -q, 1e-8, &format!("dV/dx at k={}", k));
    }

    // dM/dx = V (M quadratic): central difference is exact.
    for k in 1..(n - 1) {
        let slope = (samples[k + 1].section_forces.moment - samples[k - 1].section_forces.moment)
            / (2.0 * dx);
        assert_close(
            slope,
            samples[k].section_forces.shear,
            1e-8,
            &format!("dM/dx at k={}", k),
        );
    }
}

// ===========================================================================
// Test 4 — axial distributed load: dN/dx = -q_x
// ===========================================================================

#[test]
fn test_diagram_axial_distributed_load() {
    let l = 1.0;
    let qx = 1000.0; // tensile (+x) intensity

    let mut model = cantilever_model(1, l);
    model.add_distributed_load(0, qx, 0.0).unwrap();
    let solver = solve(&model);

    let n = 101;
    let samples = solver.sample_element_forces(0, n).unwrap();
    let dx = l / (n - 1) as f64;

    // N(x) = qx (L - x): tension at the fixed end, zero at the free end.
    for s in &samples {
        let x = s.xi * l;
        assert_close(s.section_forces.axial, qx * (l - x), 1e-8, "N analytic");
        assert_close(s.section_forces.shear, 0.0, 1e-9, "V");
        assert_close(s.section_forces.moment, 0.0, 1e-9, "M");
    }

    // dN/dx = -qx (N linear): consecutive slope is exact.
    for k in 0..(n - 1) {
        let slope = (samples[k + 1].section_forces.axial - samples[k].section_forces.axial) / dx;
        assert_close(slope, -qx, 1e-8, &format!("dN/dx at k={}", k));
    }
}

// ===========================================================================
// Test 5 — internal point force: discontinuity both sides, no dedup
// ===========================================================================

#[test]
fn test_diagram_internal_point_force_discontinuity() {
    let l = 1.0;
    let p = 1000.0; // downward at midspan

    let mut model = cantilever_model(1, l);
    model.add_point_load(0, 0.5, 0.0, -p, 0.0).unwrap();
    let solver = solve(&model);

    // n = 11 hits xi = 0.5 exactly.
    let samples = solver.sample_element_forces(0, 11).unwrap();
    let at_half = samples.iter().find(|s| (s.xi - 0.5).abs() < 1e-15).unwrap();

    // Exact hit uses the existing left-limit convention.
    let left_limit = solver.element_section_forces(0, 0.5).unwrap();
    assert_close(
        at_half.section_forces.shear,
        left_limit.shear,
        1e-15,
        "V at 0.5",
    );
    assert_close(at_half.section_forces.shear, p, 1e-6, "V left limit = +P");

    // Right-hand value (not sampled exactly, evaluated just after).
    let right = solver.element_section_forces(0, 0.5 + 1e-6).unwrap();
    assert_close(right.shear, 0.0, 1e-6, "V right = 0");
    assert_close(
        right.shear - left_limit.shear,
        -p,
        1e-6,
        "shear jump = fy_p",
    );

    // Both sides remain representable in a finer sweep (samples around 0.5).
    let fine = solver.sample_element_forces(0, 21).unwrap();
    let before = fine.iter().find(|s| (s.xi - 0.45).abs() < 1e-15).unwrap();
    let after = fine.iter().find(|s| (s.xi - 0.55).abs() < 1e-15).unwrap();
    assert_close(before.section_forces.shear, p, 1e-6, "V before load");
    assert_close(after.section_forces.shear, 0.0, 1e-6, "V after load");
}

// ===========================================================================
// Test 6 — internal point moment: moment discontinuity both sides
// ===========================================================================

#[test]
fn test_diagram_internal_point_moment_discontinuity() {
    let l = 1.0;
    let m = 1000.0; // CCW applied at midspan

    let mut model = cantilever_model(1, l);
    model.add_point_moment(0, 0.5, m).unwrap();
    let solver = solve(&model);

    let left = solver.element_section_forces(0, 0.5).unwrap(); // left limit
    let right = solver.element_section_forces(0, 0.5 + 1e-6).unwrap();
    assert_close(left.moment, m, 1e-6, "M left limit = +M");
    assert_close(right.moment, 0.0, 1e-6, "M right = 0");
    assert_close(right.moment - left.moment, -m, 1e-6, "moment jump = -mz_p");

    // The exact hit in a uniform sweep equals the left limit.
    let samples = solver.sample_element_forces(0, 11).unwrap();
    let at_half = samples.iter().find(|s| (s.xi - 0.5).abs() < 1e-15).unwrap();
    assert_close(
        at_half.section_forces.moment,
        left.moment,
        1e-15,
        "M at 0.5",
    );
}

// ===========================================================================
// Test 7 — nodal applied moment preserved across shared node, not double-counted
// ===========================================================================

#[test]
fn test_diagram_nodal_applied_moment_shared_node() {
    let l = 1.0;
    let m = 1000.0;

    let mut model = cantilever_model(2, l);
    model.add_applied_moment(1, m).unwrap();
    let solver = solve(&model);

    // n = 2 per element → samples: e0@0, e0@1, e1@0, e1@1.
    let s = solver.sample_beam_forces(2).unwrap();
    assert_eq!(s.len(), 4);

    let e0_right = s[1]; // element 0, xi = 1, x = 0.5
    let e1_left = s[2]; // element 1, xi = 0, x = 0.5
    assert_eq!(e0_right.element_index, 0);
    assert_eq!(e1_left.element_index, 1);
    assert_close(e0_right.x, 0.5, 1e-15, "shared x (left sample)");
    assert_close(e1_left.x, 0.5, 1e-15, "shared x (right sample)");

    // Values equal the authoritative API (no double counting).
    let a = solver.element_section_forces(0, 1.0).unwrap();
    let b = solver.element_section_forces(1, 0.0).unwrap();
    assert_close(
        e0_right.section_forces.moment,
        a.moment,
        1e-15,
        "e0 right M",
    );
    assert_close(e1_left.section_forces.moment, b.moment, 1e-15, "e1 left M");

    // Nodal equilibrium in the section representation: M_right - M_left + M = 0.
    assert_close(
        e1_left.section_forces.moment - e0_right.section_forces.moment + m,
        0.0,
        1e-9,
        "nodal equilibrium",
    );
}

// ===========================================================================
// Test 8 — multi-element beam with unequal lengths, global x
// ===========================================================================

#[test]
fn test_diagram_unequal_element_lengths_global_x() {
    // L0 = 1, L1 = 2, L2 = 3, total = 6 along global x.
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, 1.0, 0.0));
    model.add_node(BeamNode::new(2, 3.0, 0.0));
    model.add_node(BeamNode::new(3, 6.0, 0.0));
    model.add_element(BeamElement::new(0, 1, steel(), section()).unwrap());
    model.add_element(BeamElement::new(1, 2, steel(), section()).unwrap());
    model.add_element(BeamElement::new(2, 3, steel(), section()).unwrap());
    model.fix_node(0);
    model.add_nodal_force(3, 1, -1000.0);
    let solver = solve(&model);

    let samples = solver.sample_beam_forces(3).unwrap();
    // 3 elements × 3 samples.
    assert_eq!(samples.len(), 9);

    // Element boundaries in physical x.
    let expected_x = [0.0, 0.5, 1.0, 1.0, 2.0, 3.0, 3.0, 4.5, 6.0];
    let expected_elem = [0, 0, 0, 1, 1, 1, 2, 2, 2];
    for (s, (&xe, &ee)) in samples
        .iter()
        .zip(expected_x.iter().zip(expected_elem.iter()))
    {
        assert_eq!(s.element_index, ee, "element index order");
        assert_close(s.x, xe, 1e-12, "global x");
    }

    // Total span ends at 6.
    assert_close(samples.last().unwrap().x, 6.0, 1e-12, "beam end x");

    // Independent check: element 2 spans exactly [3, 6].
    let e2: Vec<f64> = samples
        .iter()
        .filter(|s| s.element_index == 2)
        .map(|s| s.x)
        .collect();
    assert_close(e2[0], 3.0, 1e-12, "e2 start");
    assert_close(e2[2], 6.0, 1e-12, "e2 end");
}

// ===========================================================================
// Test 9 — shared-node duplication (same x, different element_index)
// ===========================================================================

#[test]
fn test_diagram_shared_node_duplicates_x() {
    let mut model = cantilever_model(2, 1.0);
    model.add_nodal_force(2, 1, -1000.0);
    let solver = solve(&model);

    let samples = solver.sample_beam_forces(4).unwrap();

    let mut found_shared = false;
    for w in samples.windows(2) {
        let (a, b) = (w[0], w[1]);
        if a.x == b.x {
            found_shared = true;
            // Same physical location, adjacent elements, distinct sides.
            assert_eq!(
                b.element_index,
                a.element_index + 1,
                "shared x must join adjacent elements"
            );
            assert!((a.xi - 1.0).abs() < 1e-15, "left sample is xi=1");
            assert!(b.xi.abs() < 1e-15, "right sample is xi=0");
        }
    }
    assert!(
        found_shared,
        "expected a duplicated shared-node x coordinate"
    );

    // Specifically, sample[3] (e0, xi=1) and sample[4] (e1, xi=0) share x=0.5.
    // (Both x values are produced by exactly representable arithmetic.)
    assert_eq!(samples[3].element_index, 0);
    assert_eq!(samples[4].element_index, 1);
    assert_eq!(samples[3].x, samples[4].x);
}

// ===========================================================================
// Test 10 — rotated beam: physical x, local section forces
// ===========================================================================

#[test]
fn test_diagram_rotated_beam() {
    use std::f64::consts::FRAC_1_SQRT_2;

    let l = 1.0;
    let p = 1000.0;
    let c = FRAC_1_SQRT_2;
    let s = FRAC_1_SQRT_2;

    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, l * c, l * s));
    model.add_element(BeamElement::new(0, 1, steel(), section()).unwrap());
    model.fix_node(0);
    model.add_nodal_force(1, 1, -p); // global downward tip force
    let solver = solve(&model);

    // Local decomposition of the global tip force F = (0, -P).
    let fx = -p * FRAC_1_SQRT_2;
    let fy = -p * FRAC_1_SQRT_2;

    let samples = solver.sample_element_forces(0, 5).unwrap();
    for st in &samples {
        // x is physical beam arclength regardless of orientation.
        assert_close(st.x, st.xi * l, 1e-15, "x is beam arclength");
        let x = st.xi * l;
        assert_close(st.section_forces.axial, fx, 1e-9, "N local");
        assert_close(st.section_forces.shear, -fy, 1e-9, "V local");
        assert_close(st.section_forces.moment, fy * (l - x), 1e-9, "M local");
        // No accidental global/local conversion.
        let direct = solver.element_section_forces(0, st.xi).unwrap();
        assert_close(st.section_forces.shear, direct.shear, 1e-15, "V==direct");
        assert_close(st.section_forces.moment, direct.moment, 1e-15, "M==direct");
    }
}

// ===========================================================================
// Test 11 — invalid inputs (Err, never panic)
// ===========================================================================

#[test]
fn test_diagram_invalid_inputs() {
    let mut model = cantilever_model(2, 1.0);
    model.add_nodal_force(2, 1, -1000.0);
    let solver = solve(&model);

    // Valid baseline.
    assert!(solver.sample_element_forces(0, 2).is_ok());
    assert!(solver.sample_beam_forces(2).is_ok());

    // Invalid element index.
    assert!(solver.sample_element_forces(2, 4).is_err());
    assert!(solver.sample_element_forces(999, 4).is_err());

    // n < 2 for element sampling.
    assert!(solver.sample_element_forces(0, 0).is_err());
    assert!(solver.sample_element_forces(0, 1).is_err());

    // n_per_element < 2 for beam sampling.
    assert!(solver.sample_beam_forces(0).is_err());
    assert!(solver.sample_beam_forces(1).is_err());

    // Diagram wrapper follows the same validation.
    assert!(solver.beam_force_diagram(1).is_err());
    assert!(solver.beam_force_diagram(2).is_ok());
}

/// A zero-length element is rejected by the existing model-validation path
/// (`from_model`), which is why the sampling layer cannot be handed one. This
/// confirms the error path exists and does not panic.
#[test]
fn test_diagram_zero_length_element_rejected() {
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, 0.0, 0.0)); // coincident coordinates
    model.add_element(BeamElement::new(0, 1, steel(), section()).unwrap());
    model.fix_node(0);

    assert!(
        BeamSolver::from_model(&model).is_err(),
        "from_model must reject a zero-length element"
    );
}

// ===========================================================================
// Diagram container basics
// ===========================================================================

#[test]
fn test_beam_force_diagram_container() {
    let mut model = cantilever_model(1, 1.0);
    model.add_nodal_force(1, 1, -1000.0);
    let solver = solve(&model);

    let diagram = solver.beam_force_diagram(5).unwrap();
    assert_eq!(diagram.samples.len(), 5);
    assert_eq!(diagram.axial().len(), 5);
    assert_eq!(diagram.shear().len(), 5);
    assert_eq!(diagram.moment().len(), 5);
    assert_eq!(diagram.x().len(), 5);

    // Container accessors mirror the sample fields.
    for (i, s) in diagram.samples.iter().enumerate() {
        assert_eq!(diagram.shear()[i], s.section_forces.shear);
        assert_eq!(diagram.moment()[i], s.section_forces.moment);
        assert_eq!(diagram.x()[i], s.x);
    }
}
