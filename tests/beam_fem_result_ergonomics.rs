//! Phase 6B — Beam FEM result-access ergonomics.
//!
//! Tests **only** the new typed result accessors (`displacement_dof`,
//! `reaction_dof`): equivalence with the raw-index API, `Dof::ALL` ordering,
//! numerical identity with the existing result structs, pre-solve behaviour and
//! invalid-node safety. Existing FEM semantics are covered by
//! `beam_fem_api_semantics.rs` / `beam_fem_contract.rs` and are not duplicated
//! here.

use section_properties::beam_fem::{
    BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver, Dof, FemError,
};
use section_properties::material::Material;

const L: f64 = 1.0;

/// Two-element unit cantilever (E = A = I = 1) with mixed loading.
fn model() -> BeamModel {
    let mut m = BeamModel::new();
    let n = 2;
    for k in 0..=n {
        m.add_node(BeamNode::new(k, k as f64 * (L / n as f64), 0.0));
    }
    for k in 0..n {
        m.add_element(
            BeamElement::new(
                k,
                k + 1,
                Material::new(1.0, 0.3, 1.0, "unit"),
                BeamSection::new(1.0, 1.0),
            )
            .unwrap(),
        );
    }
    m.fix_node(0);
    m.add_distributed_load(0, 0.0, -100.0).unwrap();
    m.add_nodal_force(2, 1, -50.0);
    m.add_applied_moment(2, 10.0).unwrap();
    m
}

fn solved() -> BeamSolver {
    let mut s = BeamSolver::from_model(&model()).unwrap();
    s.solve_configured().unwrap();
    s
}

// ===========================================================================
// 7.1 Typed displacement equivalence
// ===========================================================================

#[test]
fn test_typed_displacement_matches_raw() {
    let s = solved();
    for node in 0..3 {
        assert_eq!(
            s.displacement_dof(node, Dof::Ux).unwrap(),
            s.displacement(node, 0),
            "node {} Ux",
            node
        );
        assert_eq!(
            s.displacement_dof(node, Dof::Uy).unwrap(),
            s.displacement(node, 1),
            "node {} Uy",
            node
        );
        assert_eq!(
            s.displacement_dof(node, Dof::Rz).unwrap(),
            s.displacement(node, 2),
            "node {} Rz",
            node
        );
    }
    // Sanity: the tip actually deflected (not all zeros).
    assert!(s.displacement_dof(2, Dof::Uy).unwrap().abs() > 1e-6);
}

// ===========================================================================
// 7.2 Typed reaction equivalence
// ===========================================================================

#[test]
fn test_typed_reaction_matches_raw() {
    let s = solved();
    for node in 0..3 {
        assert_eq!(
            s.reaction_dof(node, Dof::Ux).unwrap(),
            s.reaction(node, 0),
            "node {} Ux",
            node
        );
        assert_eq!(
            s.reaction_dof(node, Dof::Uy).unwrap(),
            s.reaction(node, 1),
            "node {} Uy",
            node
        );
        assert_eq!(
            s.reaction_dof(node, Dof::Rz).unwrap(),
            s.reaction(node, 2),
            "node {} Rz",
            node
        );
    }
    // Free DOFs carry ~zero reaction; constrained DOFs do not.
    assert!(s.reaction_dof(2, Dof::Uy).unwrap().abs() < 1e-9);
    assert!(s.reaction_dof(0, Dof::Uy).unwrap().abs() > 1.0);
}

// ===========================================================================
// 7.3 Dof::ALL ordering
// ===========================================================================

#[test]
fn test_dof_all_ordering_and_iteration() {
    assert_eq!(
        Dof::ALL,
        [Dof::Ux, Dof::Uy, Dof::Rz],
        "ordering must stay ux, uy, rz"
    );
    assert_eq!(
        Dof::ALL.iter().map(|d| d.index()).collect::<Vec<_>>(),
        vec![0, 1, 2]
    );

    let s = solved();
    // Iterating Dof::ALL reproduces the raw 0..3 loop for every node.
    for node in 0..3 {
        for (k, dof) in Dof::ALL.iter().enumerate() {
            assert_eq!(
                s.displacement_dof(node, *dof).unwrap(),
                s.displacement(node, k),
                "node {} dof {}",
                node,
                dof.name()
            );
            assert_eq!(
                s.reaction_dof(node, *dof).unwrap(),
                s.reaction(node, k),
                "node {} reaction {}",
                node,
                dof.name()
            );
        }
    }
}

// ===========================================================================
// 7.4 Numerical identity with the existing result structs
// ===========================================================================

#[test]
fn test_typed_access_matches_result_structs() {
    let s = solved();
    let res = s.results();

    for node in 0..3 {
        let d = res.displacement(node).unwrap();
        assert_eq!(s.displacement_dof(node, Dof::Ux).unwrap(), d.ux);
        assert_eq!(s.displacement_dof(node, Dof::Uy).unwrap(), d.uy);
        assert_eq!(s.displacement_dof(node, Dof::Rz).unwrap(), d.rz);
    }

    let reactions = s.reactions();
    for node in 0..3 {
        let r = res.reaction(node).unwrap();
        // `BeamAnalysisResult::reaction` sanitises free-DOF residuals to exactly
        // 0.0 (they are round-off, not physical support reactions), whereas
        // `BeamSolver::reactions()` returns the raw `K·u - f` values. So the
        // comparison is exact at supports and within round-off at free DOFs.
        for (got, expected, name) in [
            (s.reaction_dof(node, Dof::Ux).unwrap(), r.fx, "fx"),
            (s.reaction_dof(node, Dof::Uy).unwrap(), r.fy, "fy"),
            (s.reaction_dof(node, Dof::Rz).unwrap(), r.mz, "mz"),
        ] {
            assert!(
                (got - expected).abs() <= 1e-9 + 1e-9 * expected.abs().max(got.abs()),
                "node {} {}: {} vs {}",
                node,
                name,
                got,
                expected
            );
        }
        // And identical (exactly) to the corresponding slice of the raw vector.
        let base = 3 * node;
        assert_eq!(s.reaction_dof(node, Dof::Ux).unwrap(), reactions[base]);
        assert_eq!(s.reaction_dof(node, Dof::Uy).unwrap(), reactions[base + 1]);
        assert_eq!(s.reaction_dof(node, Dof::Rz).unwrap(), reactions[base + 2]);
    }
}

// ===========================================================================
// Result access before solving
// ===========================================================================

#[test]
fn test_result_access_before_solve() {
    // Documented pre-solve behaviour: the displacement vector is initialised to
    // zero, so displacement access returns 0.0 (no panic, no error). Reactions
    // are evaluated as K·u - f with u = 0, i.e. they are NOT physical support
    // reactions until a solve has succeeded; `solver_name()` is `Some(..)` only
    // after a successful solve.
    let mut s = BeamSolver::from_model(&model()).unwrap();
    assert_eq!(s.solver_name(), None, "no solve yet");

    for node in 0..3 {
        for dof in Dof::ALL {
            assert_eq!(
                s.displacement_dof(node, dof).unwrap(),
                0.0,
                "pre-solve displacement must be zero (node {}, {})",
                node,
                dof.name()
            );
            assert!(
                s.reaction_dof(node, dof).unwrap().is_finite(),
                "pre-solve reaction access must not panic"
            );
        }
    }

    s.solve_configured().unwrap();
    assert_eq!(s.solver_name(), Some("dense"));
    assert!(s.displacement_dof(2, Dof::Uy).unwrap().abs() > 1e-6);
}

// ===========================================================================
// Invalid node handling
// ===========================================================================

#[test]
fn test_invalid_node_returns_error() {
    let s = solved();

    for dof in Dof::ALL {
        assert!(matches!(
            s.displacement_dof(9, dof),
            Err(FemError::InvalidInput(_))
        ));
        assert!(matches!(
            s.reaction_dof(9, dof),
            Err(FemError::InvalidInput(_))
        ));
        assert!(matches!(
            s.displacement_dof(usize::MAX, dof),
            Err(FemError::InvalidInput(_))
        ));
    }

    // Valid access still works after rejected calls (no state corruption).
    assert!(s.displacement_dof(0, Dof::Uy).unwrap().is_finite());
    assert!(s.reaction_dof(0, Dof::Rz).unwrap().is_finite());
}
