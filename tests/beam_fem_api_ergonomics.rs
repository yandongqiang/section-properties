//! Phase 6A — Beam FEM API ergonomics.
//!
//! Tests **only** the new additive API: the typed [`Dof`] enum, the typed
//! constraint helpers and the local/global force-direction helper. Existing
//! semantics are covered by `beam_fem_api_semantics.rs` / `beam_fem_contract.rs`
//! and are not re-tested here, except for one backward-compatibility check
//! confirming the pre-existing workflow still behaves identically.

use section_properties::beam_fem::{
    BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver, Dof, FemError,
};
use section_properties::material::Material;

const L: f64 = 1.0;

fn mat() -> Material {
    Material::new(1.0, 0.3, 1.0, "unit")
}
fn sec() -> BeamSection {
    BeamSection::new(1.0, 1.0)
}

/// Unit cantilever (E = A = I = 1, L = 1); no boundary conditions applied.
fn beam() -> BeamModel {
    let mut m = BeamModel::new();
    m.add_node(BeamNode::new(0, 0.0, 0.0));
    m.add_node(BeamNode::new(1, L, 0.0));
    m.add_element(BeamElement::new(0, 1, mat(), sec()).unwrap());
    m
}

fn solve(model: &BeamModel) -> BeamSolver {
    let mut s = BeamSolver::from_model(model).unwrap();
    s.solve_configured().unwrap();
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

// ===========================================================================
// Typed DOF
// ===========================================================================

#[test]
fn test_dof_mapping_and_conversions() {
    // Frozen contract mapping.
    assert_eq!(Dof::Ux.index(), 0);
    assert_eq!(Dof::Uy.index(), 1);
    assert_eq!(Dof::Rz.index(), 2);
    assert_eq!(Dof::Ux.name(), "ux");
    assert_eq!(Dof::Uy.name(), "uy");
    assert_eq!(Dof::Rz.name(), "rz");

    // Canonical ordering.
    assert_eq!(
        Dof::ALL,
        [Dof::Ux, Dof::Uy, Dof::Rz],
        "Dof::ALL must match [ux, uy, rz]"
    );
    assert_eq!(Dof::ALL.len(), 3);

    // usize round-trip.
    for dof in Dof::ALL {
        let raw: usize = dof.into();
        assert_eq!(raw, dof.index());
        let back = Dof::try_from(raw).unwrap();
        assert_eq!(back, dof);
    }

    // Invalid raw DOF.
    assert!(matches!(Dof::try_from(3), Err(FemError::InvalidInput(_))));
    assert!(matches!(
        Dof::try_from(usize::MAX),
        Err(FemError::InvalidInput(_))
    ));
}

#[test]
fn test_typed_fix_matches_fix_dof() {
    // Typed: try_fix(node, Dof, value).
    let mut typed = beam();
    typed.try_fix(0, Dof::Ux, 0.0).unwrap();
    typed.try_fix(0, Dof::Uy, 0.0).unwrap();
    typed.try_fix(0, Dof::Rz, 0.0).unwrap();
    typed.add_nodal_force(1, 1, -100.0);

    // Existing: try_fix_dof(node, raw dof, value).
    let mut legacy = beam();
    legacy.try_fix_dof(0, 0, 0.0).unwrap();
    legacy.try_fix_dof(0, 1, 0.0).unwrap();
    legacy.try_fix_dof(0, 2, 0.0).unwrap();
    legacy.add_nodal_force(1, 1, -100.0);

    let (a, b) = (solve(&typed), solve(&legacy));
    for i in 0..a.displacements().len() {
        assert_eq!(
            a.displacements()[i],
            b.displacements()[i],
            "displacement[{}] must be identical",
            i
        );
    }
    for i in 0..a.reactions().len() {
        assert_eq!(a.reactions()[i], b.reactions()[i], "reaction[{}]", i);
    }
    // And the known analytical result still holds.
    assert_mixed(a.displacements()[4], -100.0 / 3.0, 1e-12, 1e-9, "tip uy");
}

// ===========================================================================
// Prescribed-value convenience
// ===========================================================================

#[test]
fn test_fix_node_with_values_matches_per_dof_and_settlement() {
    let delta = -0.01; // support settlement

    // Convenience API.
    let mut conv = beam();
    conv.try_fix_node_with_values(0, 0.0, delta, 0.0).unwrap();
    conv.try_fix(1, Dof::Uy, 0.0).unwrap(); // roller

    // Equivalent per-DOF calls on the existing API.
    let mut per_dof = beam();
    per_dof.try_fix_dof(0, 0, 0.0).unwrap();
    per_dof.try_fix_dof(0, 1, delta).unwrap();
    per_dof.try_fix_dof(0, 2, 0.0).unwrap();
    per_dof.try_fix_dof(1, 1, 0.0).unwrap();

    let (a, b) = (solve(&conv), solve(&per_dof));
    for i in 0..a.displacements().len() {
        assert_eq!(a.displacements()[i], b.displacements()[i], "u[{}]", i);
    }
    for i in 0..a.reactions().len() {
        assert_eq!(a.reactions()[i], b.reactions()[i], "R[{}]", i);
    }

    // Known settlement behaviour: prescribed value exact, reactions induced and
    // self-equilibrated (no external load).
    assert_eq!(a.displacements()[1], delta, "prescribed settlement exact");
    let r = a.reactions();
    assert!(
        r[1].abs() > 0.0 && r[2].abs() > 0.0,
        "settlement must induce reactions"
    );
    assert_mixed(r[1] + r[4], 0.0, 1e-9, 1e-9, "ΣFy = 0");
    assert_mixed(r[2] + r[5] + r[4] * L, 0.0, 1e-9, 1e-9, "ΣMz = 0");
}

// ===========================================================================
// Error handling
// ===========================================================================

#[test]
fn test_ergonomics_error_handling() {
    // Invalid node.
    let mut m = beam();
    assert!(matches!(
        m.try_fix(9, Dof::Ux, 0.0),
        Err(FemError::InvalidInput(_))
    ));
    assert!(matches!(
        m.try_fix_node_with_values(9, 0.0, 0.0, 0.0),
        Err(FemError::InvalidInput(_))
    ));
    // An invalid node must not partially apply the constraint.
    assert!(
        m.fixed_dofs.is_empty(),
        "no BC may be recorded for an invalid node"
    );

    // Non-finite prescribed values.
    assert!(matches!(
        m.try_fix(0, Dof::Uy, f64::NAN),
        Err(FemError::InvalidInput(_))
    ));
    assert!(matches!(
        m.try_fix(0, Dof::Rz, f64::INFINITY),
        Err(FemError::InvalidInput(_))
    ));
    for (ux, uy, rz) in [
        (f64::NAN, 0.0, 0.0),
        (0.0, f64::NAN, 0.0),
        (0.0, 0.0, f64::INFINITY),
    ] {
        assert!(matches!(
            m.try_fix_node_with_values(0, ux, uy, rz),
            Err(FemError::InvalidInput(_))
        ));
    }
    assert!(
        m.fixed_dofs.is_empty(),
        "no BC may be recorded for invalid values"
    );

    // A valid call still works after the rejected ones.
    m.try_fix_node_with_values(0, 0.0, 0.0, 0.0).unwrap();
    assert_eq!(m.fixed_dofs.len(), 3);
}

// ===========================================================================
// Local/global force-direction helper
// ===========================================================================

#[test]
fn test_to_local_force_rotation() {
    for &deg in &[0.0_f64, 45.0, 90.0, -45.0] {
        let th = deg.to_radians();
        let (c, s) = (th.cos(), th.sin());
        let mut m = BeamModel::new();
        m.add_node(BeamNode::new(0, 0.0, 0.0));
        m.add_node(BeamNode::new(1, L * c, L * s));
        m.add_element(BeamElement::new(0, 1, mat(), sec()).unwrap());
        let (ni, nj) = (m.nodes[0].point(), m.nodes[1].point());
        let el = &m.elements[0];

        // Explicit expectations: fx_local = c·gx + s·gy, fy_local = -s·gx + c·gy.
        for (gx, gy) in [(1.0, 0.0), (0.0, 1.0), (3.0, -7.0)] {
            let (fx, fy) = el.to_local_force(ni, nj, gx, gy);
            assert_mixed(
                fx,
                c * gx + s * gy,
                1e-14,
                1e-12,
                &format!("{:.0}° fx", deg),
            );
            assert_mixed(
                fy,
                -s * gx + c * gy,
                1e-14,
                1e-12,
                &format!("{:.0}° fy", deg),
            );
        }

        // Round trip: local -> global -> local is the identity.
        for (fx, fy) in [(50.0, -100.0), (-12.0, 7.5)] {
            let (gx, gy) = (c * fx - s * fy, s * fx + c * fy);
            let (back_x, back_y) = el.to_local_force(ni, nj, gx, gy);
            assert_mixed(
                back_x,
                fx,
                1e-13,
                1e-12,
                &format!("{:.0}° round-trip fx", deg),
            );
            assert_mixed(
                back_y,
                fy,
                1e-13,
                1e-12,
                &format!("{:.0}° round-trip fy", deg),
            );
        }
    }
    println!("[to_local_force] global -> local rotation verified at 0/45/90/-45°");
}

// ===========================================================================
// Backward compatibility
// ===========================================================================

#[test]
fn test_backward_compatibility_existing_workflow() {
    // The pre-existing (pre-6A) workflow must compile and behave identically.
    let mut m = beam();
    m.fix_node(0);
    m.add_nodal_force(1, 1, -100.0);
    let s = solve(&m);

    assert_mixed(s.displacements()[4], -100.0 / 3.0, 1e-12, 1e-9, "tip uy");
    assert_mixed(s.reactions()[1], 100.0, 1e-9, 1e-9, "Ry");
    assert_mixed(s.reactions()[2], 100.0, 1e-9, 1e-9, "Rz");
    assert_eq!(s.solver_name(), Some("dense"));
}
