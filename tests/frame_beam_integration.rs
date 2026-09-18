//! Phase 42 — Frame/Beam integration regression tests.
//!
//! End-to-end integration cases covering complete structural models:
//! - Simply supported beam with central point load (analytical benchmark)
//! - Simply supported beam with UDL (analytical benchmark)
//! - Frame API simply supported beam (cross-API consistency)

use section_properties::beam_fem::{BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver};
use section_properties::frame::FrameModel;
use section_properties::{Dof, Material};

// ===========================================================================
// Simply supported beam — central point load
// ===========================================================================

/// Two-element simply supported beam (pin at left, roller at right).
/// Central point load P downward at the mid-span node.
fn simply_supported_central_load(n_elem: usize, p: f64) -> BeamSolver {
    let l = 1.0_f64;
    let e = 1.0_f64;
    let a = 1.0_f64;
    let i = 1.0_f64;
    let mut model = BeamModel::new();
    for k in 0..=n_elem {
        model.add_node(BeamNode::new(k, k as f64 * (l / n_elem as f64), 0.0));
    }
    for k in 0..n_elem {
        model.add_element(
            BeamElement::new(
                k,
                k + 1,
                Material::new(e, 0.3, 1.0, "unit"),
                BeamSection::new(a, i),
            )
            .unwrap(),
        );
    }
    // Simply supported: fix u,v at left; fix v at right (allow axial sliding + rotation).
    model.fix_dof(0, 0, 0.0);
    model.fix_dof(0, 1, 0.0);
    model.fix_dof(n_elem, 1, 0.0);
    // Central point load at mid-span.
    let mid = n_elem / 2;
    model.add_nodal_force(mid, 1, -p);
    let mut s = BeamSolver::from_model(&model).unwrap();
    s.solve_configured().unwrap();
    s
}

#[test]
fn simply_supported_central_point_load_deflection() {
    let p = 100.0_f64;
    let l = 1.0_f64;
    let e = 1.0_f64;
    let i = 1.0_f64;
    let ei = e * i;

    // Analytical: δ_mid = P*L^3 / (48*E*I)  (downward)
    let an_delta = p * l.powi(3) / (48.0 * ei);

    // Use 2 elements so mid-span is a node.
    let s = simply_supported_central_load(2, p);
    let r = s.results();
    let delta = r.displacement(1).unwrap().uy;

    assert!(
        (delta + an_delta).abs() < 1e-9 * an_delta,
        "mid-span deflection: got {}, analytical {}",
        delta,
        -an_delta
    );
}

#[test]
fn simply_supported_central_point_load_reactions() {
    let p = 100.0_f64;

    let s = simply_supported_central_load(2, p);
    let r = s.results();

    // R_A = R_B = P/2 (upward)
    let ry_left = r.reaction(0).unwrap().fy;
    let ry_right = r.reaction(2).unwrap().fy;

    assert!(
        (ry_left - p / 2.0).abs() < 1e-9,
        "left reaction: got {}, expected {}",
        ry_left,
        p / 2.0
    );
    assert!(
        (ry_right - p / 2.0).abs() < 1e-9,
        "right reaction: got {}, expected {}",
        ry_right,
        p / 2.0
    );
}

#[test]
fn simply_supported_central_point_load_end_rotations() {
    let p = 100.0_f64;
    let l = 1.0_f64;
    let e = 1.0_f64;
    let i = 1.0_f64;
    let ei = e * i;

    // Analytical: θ_A = -P*L^2 / (16*E*I), θ_B = +P*L^2 / (16*E*I)
    let an_theta = p * l * l / (16.0 * ei);

    let s = simply_supported_central_load(2, p);
    let r = s.results();
    let theta_a = r.displacement(0).unwrap().rz;
    let theta_b = r.displacement(2).unwrap().rz;

    assert!(
        (theta_a + an_theta).abs() < 1e-9 * an_theta,
        "left rotation: got {}, analytical {}",
        theta_a,
        -an_theta
    );
    assert!(
        (theta_b - an_theta).abs() < 1e-9 * an_theta,
        "right rotation: got {}, analytical {}",
        theta_b,
        an_theta
    );
}

#[test]
fn simply_supported_central_point_load_equilibrium() {
    let p = 100.0_f64;
    let s = simply_supported_central_load(2, p);
    let r = s.results();

    // Global equilibrium: ΣFy = 0, ΣM = 0
    let ry_left = r.reaction(0).unwrap().fy;
    let ry_right = r.reaction(2).unwrap().fy;
    let mz_left = r.reaction(0).unwrap().mz;
    let mz_right = r.reaction(2).unwrap().mz;

    assert!(
        (ry_left + ry_right - p).abs() < 1e-9,
        "vertical equilibrium"
    );
    // Moments about left support: R_B * L - P * L/2 + M_A + M_B = 0
    let l = 1.0_f64;
    assert!(
        (ry_right * l - p * l / 2.0 + mz_left + mz_right).abs() < 1e-9,
        "moment equilibrium"
    );
}

// ===========================================================================
// Simply supported beam — UDL
// ===========================================================================

fn simply_supported_udl(n_elem: usize, q: f64) -> BeamSolver {
    let l = 1.0_f64;
    let e = 1.0_f64;
    let a = 1.0_f64;
    let i = 1.0_f64;
    let mut model = BeamModel::new();
    for k in 0..=n_elem {
        model.add_node(BeamNode::new(k, k as f64 * (l / n_elem as f64), 0.0));
    }
    for k in 0..n_elem {
        model.add_element(
            BeamElement::new(
                k,
                k + 1,
                Material::new(e, 0.3, 1.0, "unit"),
                BeamSection::new(a, i),
            )
            .unwrap(),
        );
    }
    model.fix_dof(0, 0, 0.0);
    model.fix_dof(0, 1, 0.0);
    model.fix_dof(n_elem, 1, 0.0);
    for e in 0..n_elem {
        model.add_distributed_load(e, 0.0, -q).unwrap();
    }
    let mut s = BeamSolver::from_model(&model).unwrap();
    s.solve_configured().unwrap();
    s
}

#[test]
fn simply_supported_udl_deflection() {
    let q = 100.0_f64;
    let l = 1.0_f64;
    let e = 1.0_f64;
    let i = 1.0_f64;
    let ei = e * i;

    // Analytical: δ_mid = 5*q*L^4 / (384*E*I)  (downward)
    let an_delta = 5.0 * q * l.powi(4) / (384.0 * ei);

    let s = simply_supported_udl(2, q);
    let r = s.results();
    let delta = r.displacement(1).unwrap().uy;

    assert!(
        (delta + an_delta).abs() < 1e-9 * an_delta,
        "mid-span UDL deflection: got {}, analytical {}",
        delta,
        -an_delta
    );
}

#[test]
fn simply_supported_udl_reactions() {
    let q = 100.0_f64;
    let l = 1.0_f64;

    let s = simply_supported_udl(2, q);
    let r = s.results();

    // R_A = R_B = q*L/2
    let an_r = q * l / 2.0;
    let ry_left = r.reaction(0).unwrap().fy;
    let ry_right = r.reaction(2).unwrap().fy;

    assert!(
        (ry_left - an_r).abs() < 1e-9,
        "left UDL reaction: got {}, expected {}",
        ry_left,
        an_r
    );
    assert!(
        (ry_right - an_r).abs() < 1e-9,
        "right UDL reaction: got {}, expected {}",
        ry_right,
        an_r
    );
}

// ===========================================================================
// Frame API simply supported beam — cross-API consistency
// ===========================================================================

#[test]
fn frame_api_simply_supported_central_load() {
    let l = 2.0_f64;
    let e = 200e9;
    let a = 5e-3;
    let i = 2e-5;
    let p = 1.0e4;
    let ei = e * i;

    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0).unwrap();
    let n1 = f.add_node(l / 2.0, 0.0).unwrap();
    let n2 = f.add_node(l, 0.0).unwrap();
    f.add_member(
        n0,
        n1,
        Material::new(e, 0.3, 7850.0, "steel"),
        BeamSection::new(a, i),
    )
    .unwrap();
    f.add_member(
        n1,
        n2,
        Material::new(e, 0.3, 7850.0, "steel"),
        BeamSection::new(a, i),
    )
    .unwrap();
    // Pin at left (ux=uy=0, rotation free), roller at right (uy=0).
    f.pin(n0).unwrap();
    f.roller_y(n2).unwrap();
    // Central point load downward.
    f.nodal_load(n1, 0.0, -p).unwrap();

    let r = f.solve().unwrap();

    // Analytical: δ = P*L^3 / (48*E*I)
    let an_delta = p * l.powi(3) / (48.0 * ei);
    let delta = r.displacement(n1, Dof::Uy).unwrap();
    assert!(
        (delta + an_delta).abs() < 1e-6 * an_delta,
        "Frame API mid-span deflection: got {}, analytical {}",
        delta,
        -an_delta
    );

    // Reactions: R_A = R_B = P/2
    let ry_left = r.reaction(n0, Dof::Uy).unwrap();
    let ry_right = r.reaction(n2, Dof::Uy).unwrap();
    assert!(
        (ry_left - p / 2.0).abs() < 1e-6 * p,
        "Frame API left reaction: got {}, expected {}",
        ry_left,
        p / 2.0
    );
    assert!(
        (ry_right - p / 2.0).abs() < 1e-6 * p,
        "Frame API right reaction: got {}, expected {}",
        ry_right,
        p / 2.0
    );

    // Equilibrium.
    let eq = r.equilibrium();
    assert!(eq.is_balanced(), "Frame API equilibrium must be balanced");
}

// ===========================================================================
// Simply supported beam — mesh convergence
// ===========================================================================

#[test]
fn simply_supported_mesh_convergence() {
    let p = 100.0_f64;
    let l = 1.0_f64;
    let e = 1.0_f64;
    let i = 1.0_f64;
    let ei = e * i;
    let an_delta = p * l.powi(3) / (48.0 * ei);

    // With Euler-Bernoulli elements, nodal values are exact even for 2 elements.
    // Verify mesh refinement doesn't degrade.
    for &n in &[2usize, 4, 8] {
        let s = simply_supported_central_load(n, p);
        let r = s.results();
        let mid = n / 2;
        let delta = r.displacement(mid).unwrap().uy;
        assert!(
            (delta + an_delta).abs() < 1e-9 * an_delta,
            "n={}: mid-span deflection = {}, analytical = {}",
            n,
            delta,
            -an_delta
        );
    }
}
