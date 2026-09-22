//! Phase 63: P1 API robustness regression tests.
//!
//! Verifies fixes for:
//! - P1-01: legacy displacement/reaction out-of-range node_idx returns Err
//! - P1-02: try_fix_node_with_values override semantics (intentional, documented)
//! - P1-03: CompositeComponent pub-field bypass (resolved by v0.2.0 private fields)

use section_properties::beam_fem::{
    BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver, Dof,
};
use section_properties::material::Material;

// ─────────────────────────────────────────────────────────────────────────
// Helper: minimal 2-node cantilever
// ─────────────────────────────────────────────────────────────────────────

fn make_cantilever() -> (BeamModel, BeamSolver) {
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, 1.0, 0.0));
    let element = BeamElement::new(
        0,
        1,
        Material::new(1.0, 0.3, 1.0, "unit"),
        BeamSection::new(1.0, 1.0),
    )
    .unwrap();
    model.add_element(element);
    model.fix_node(0);
    model.add_nodal_force(1, 1, -100.0);
    let solver = BeamSolver::from_model(&model).unwrap();
    (model, solver)
}

fn make_solved_cantilever() -> BeamSolver {
    let (_, mut solver) = make_cantilever();
    solver.solve_configured().unwrap();
    solver
}

// ─────────────────────────────────────────────────────────────────────────
// P1-01: displacement — invalid node_idx returns Err (not Ok(0.0))
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn displacement_valid_node_valid_dof() {
    let solver = make_solved_cantilever();
    let uy = solver.displacement(1, 1).unwrap();
    assert!((uy + 100.0 / 3.0).abs() < 1e-9);
}

#[test]
fn displacement_invalid_node_valid_dof() {
    let solver = make_solved_cantilever();
    let result = solver.displacement(999, 0);
    assert!(result.is_err(), "invalid node_idx must return Err");
}

#[test]
fn displacement_valid_node_invalid_dof() {
    let solver = make_solved_cantilever();
    assert!(solver.displacement(0, 3).is_err());
    assert!(solver.displacement(0, 100).is_err());
}

#[test]
fn displacement_invalid_node_invalid_dof() {
    let solver = make_solved_cantilever();
    let result = solver.displacement(999, 3);
    assert!(result.is_err());
}

#[test]
fn displacement_dof_invalid_node() {
    let solver = make_solved_cantilever();
    assert!(solver.displacement_dof(999, Dof::Uy).is_err());
}

// ─────────────────────────────────────────────────────────────────────────
// P1-01: reaction — invalid node_idx returns Err (not Ok(0.0))
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn reaction_valid_node_valid_dof() {
    let solver = make_solved_cantilever();
    let ry = solver.reaction(0, 1).unwrap();
    assert!((ry - 100.0).abs() < 1e-9);
}

#[test]
fn reaction_invalid_node_valid_dof() {
    let solver = make_solved_cantilever();
    let result = solver.reaction(999, 0);
    assert!(result.is_err(), "invalid node_idx must return Err");
}

#[test]
fn reaction_valid_node_invalid_dof() {
    let solver = make_solved_cantilever();
    assert!(solver.reaction(0, 3).is_err());
    assert!(solver.reaction(0, 99).is_err());
}

#[test]
fn reaction_invalid_node_invalid_dof() {
    let solver = make_solved_cantilever();
    let result = solver.reaction(999, 3);
    assert!(result.is_err());
}

#[test]
fn reaction_dof_invalid_node() {
    let solver = make_solved_cantilever();
    assert!(solver.reaction_dof(999, Dof::Uy).is_err());
}

// ─────────────────────────────────────────────────────────────────────────
// P1-01: pre-solve state — valid queries return Ok(0.0), invalid return Err
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn displacement_presolve_valid_returns_zero() {
    let (_, solver) = make_cantilever();
    assert_eq!(solver.displacement(0, 0).unwrap(), 0.0);
    assert_eq!(solver.displacement(1, 1).unwrap(), 0.0);
}

#[test]
fn displacement_presolve_invalid_node_returns_err() {
    let (_, solver) = make_cantilever();
    assert!(solver.displacement(999, 0).is_err());
}

// ─────────────────────────────────────────────────────────────────────────
// P1-02: try_fix_node_with_values — override semantics
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn fix_node_with_values_overrides_existing_constraint() {
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, 1.0, 0.0));

    // First, fully fix node 0
    model.try_fix_node(0).unwrap();

    // Then override with a support settlement
    model.try_fix_node_with_values(0, 0.0, -0.01, 0.0).unwrap();

    // The uy constraint should be -0.01, not 0.0
    let uy = model
        .fixed_dofs
        .iter()
        .find(|&&(n, d, _)| n == 0 && d == 1)
        .map(|&(_, _, v)| v)
        .unwrap();
    assert_eq!(uy, -0.01);
    assert_eq!(
        model.fixed_dofs.len(),
        3,
        "should still have exactly 3 constraints"
    );
}

#[test]
fn fix_node_with_values_does_not_conflict() {
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, 1.0, 0.0));

    // try_fix_node then try_fix_node_with_values with different value — no error
    model.try_fix_node(0).unwrap();
    let result = model.try_fix_node_with_values(0, 0.0, -0.01, 0.0);
    assert!(result.is_ok(), "override should not conflict");
}

#[test]
fn fix_node_then_try_fix_dof_same_dof_different_value_conflicts() {
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, 1.0, 0.0));

    // try_fix_node then try_fix_dof with different value — conflict error
    model.try_fix_node(0).unwrap();
    let result = model.try_fix_dof(0, 1, -0.01);
    assert!(result.is_err(), "try_fix_dof should conflict");
}

#[test]
fn fix_node_with_values_presolves_validates_all_before_applying() {
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, 1.0, 0.0));

    // NaN in uy should reject the entire call, leaving no partial constraints
    let result = model.try_fix_node_with_values(0, 0.0, f64::NAN, 0.0);
    assert!(result.is_err());
    assert_eq!(
        model.fixed_dofs.len(),
        0,
        "no partial constraints after rejected call"
    );
}

#[test]
fn fix_node_with_values_invalid_node_returns_err() {
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, 1.0, 0.0));

    let result = model.try_fix_node_with_values(999, 0.0, 0.0, 0.0);
    assert!(result.is_err());
}
