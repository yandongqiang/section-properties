//! Phase 27 — Public API misuse tests.
//!
//! Verify that invalid inputs produce structured errors (not panics) for
//! fallible APIs, and documented panics for legacy APIs.

use section_properties::{Material, Point, Polygon, Section, SectionProperties, SolverSelection};
use structural_analysis::{BeamSection, Dof, FemError, FrameModel};

// ===========================================================================
// Frame API — invalid handles
// ===========================================================================

#[test]
fn frame_invalid_node_handle_returns_error() {
    let mut frame = FrameModel::new();
    let n1 = frame.add_node(0.0, 0.0).unwrap();
    let n2 = frame.add_node(1.0, 0.0).unwrap();
    frame
        .add_member(
            n1,
            n2,
            Material::new(200e9, 0.3, 7850.0, "Steel"),
            BeamSection::new(1e-3, 1e-6),
        )
        .unwrap();
    frame.fix(n1).unwrap();
    frame.nodal_load(n2, 0.0, -1.0e3).unwrap();
    let result = frame.solve().unwrap();

    // Handle with out-of-bounds index should be rejected.
    let mut other = FrameModel::new();
    other.add_node(0.0, 0.0).unwrap();
    other.add_node(1.0, 0.0).unwrap();
    let foreign_node = other.add_node(2.0, 0.0).unwrap(); // index 2, out of bounds for 2-node result
    assert!(result.displacement(foreign_node, Dof::Uy).is_err());
}

#[test]
fn frame_zero_length_member_returns_error() {
    let mut frame = FrameModel::new();
    let n1 = frame.add_node(1.0, 2.0).unwrap();
    let n2 = frame.add_node(1.0, 2.0).unwrap();
    let result = frame.add_member(
        n1,
        n2,
        Material::new(200e9, 0.3, 7850.0, "Steel"),
        BeamSection::new(1e-3, 1e-6),
    );
    assert!(matches!(result, Err(FemError::ZeroLengthMember(_))));
}

#[test]
fn frame_duplicate_member_returns_error() {
    let mut frame = FrameModel::new();
    let n1 = frame.add_node(0.0, 0.0).unwrap();
    let n2 = frame.add_node(1.0, 0.0).unwrap();
    frame
        .add_member(
            n1,
            n2,
            Material::new(200e9, 0.3, 7850.0, "Steel"),
            BeamSection::new(1e-3, 1e-6),
        )
        .unwrap();
    let result = frame.add_member(
        n1,
        n2,
        Material::new(200e9, 0.3, 7850.0, "Steel"),
        BeamSection::new(1e-3, 1e-6),
    );
    assert!(matches!(result, Err(FemError::DuplicateMember(_))));
}

#[test]
fn frame_unconstrained_solve_returns_error() {
    let mut frame = FrameModel::new();
    let n1 = frame.add_node(0.0, 0.0).unwrap();
    let n2 = frame.add_node(1.0, 0.0).unwrap();
    frame
        .add_member(
            n1,
            n2,
            Material::new(200e9, 0.3, 7850.0, "Steel"),
            BeamSection::new(1e-3, 1e-6),
        )
        .unwrap();
    frame.nodal_load(n2, 0.0, -1.0e3).unwrap();
    // No supports → singular system
    let result = frame.solve();
    assert!(result.is_err());
}

// ===========================================================================
// Frame API — diagnostic does not mutate model
// ===========================================================================

#[test]
fn frame_diagnostic_does_not_mutate_model() {
    let mut frame = FrameModel::new();
    let n1 = frame.add_node(0.0, 0.0).unwrap();
    let n2 = frame.add_node(1.0, 0.0).unwrap();
    frame
        .add_member(
            n1,
            n2,
            Material::new(200e9, 0.3, 7850.0, "Steel"),
            BeamSection::new(1e-3, 1e-6),
        )
        .unwrap();
    frame.fix(n1).unwrap();

    let n_nodes_before = frame.n_nodes();
    let n_members_before = frame.n_members();

    let _diag = frame.diagnostic().unwrap();

    assert_eq!(frame.n_nodes(), n_nodes_before);
    assert_eq!(frame.n_members(), n_members_before);
}

// ===========================================================================
// Solver API — explicit solver selection
// ===========================================================================

#[test]
fn solver_explicit_unknown_name_returns_error() {
    let mut frame = FrameModel::new();
    let n1 = frame.add_node(0.0, 0.0).unwrap();
    let n2 = frame.add_node(1.0, 0.0).unwrap();
    frame
        .add_member(
            n1,
            n2,
            Material::new(200e9, 0.3, 7850.0, "Steel"),
            BeamSection::new(1e-3, 1e-6),
        )
        .unwrap();
    frame.fix(n1).unwrap();
    frame.nodal_load(n2, 0.0, -1.0e3).unwrap();

    let result = frame.solve_with(SolverSelection::Named("nonexistent_solver".into()));
    assert!(result.is_err());
}

#[test]
fn solver_auto_selection_succeeds() {
    let mut frame = FrameModel::new();
    let n1 = frame.add_node(0.0, 0.0).unwrap();
    let n2 = frame.add_node(1.0, 0.0).unwrap();
    frame
        .add_member(
            n1,
            n2,
            Material::new(200e9, 0.3, 7850.0, "Steel"),
            BeamSection::new(1e-3, 1e-6),
        )
        .unwrap();
    frame.fix(n1).unwrap();
    frame.nodal_load(n2, 0.0, -1.0e3).unwrap();

    let result = frame.solve_with(SolverSelection::Auto);
    assert!(result.is_ok());
}

// ===========================================================================
// Geometry API — Polygon::try_new error variants
// ===========================================================================

#[test]
fn polygon_try_new_too_few_returns_error_not_panic() {
    let result = Polygon::try_new(vec![Point::new(0.0, 0.0), Point::new(1.0, 0.0)]);
    assert!(result.is_err());
}

#[test]
fn polygon_try_new_collinear_returns_error_not_panic() {
    let result = Polygon::try_new(vec![
        Point::new(0.0, 0.0),
        Point::new(1.0, 0.0),
        Point::new(2.0, 0.0),
    ]);
    assert!(result.is_err());
}

#[test]
fn polygon_try_new_valid_succeeds() {
    let result = Polygon::try_new(vec![
        Point::new(0.0, 0.0),
        Point::new(1.0, 0.0),
        Point::new(0.0, 1.0),
    ]);
    assert!(result.is_ok());
}

// ===========================================================================
// Section API — try_centroid on valid and invalid
// ===========================================================================

#[test]
fn section_try_centroid_valid_returns_some() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 5.0),
        Point::new(0.0, 5.0),
    ]);
    let sec = Section::new(outer, Vec::new());
    assert!(sec.try_centroid().is_some());
}

#[test]
fn section_try_centroid_degenerate_returns_none() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 10.0),
        Point::new(0.0, 10.0),
    ]);
    let hole = Polygon::new(vec![
        Point::new(20.0, 0.0),
        Point::new(30.0, 0.0),
        Point::new(30.0, 10.0),
        Point::new(20.0, 10.0),
    ]);
    let sec = Section::new(outer, vec![hole]);
    assert!(sec.try_centroid().is_none());
}

// ===========================================================================
// SectionProperties — try_from_section on valid and invalid
// ===========================================================================

#[test]
fn section_properties_try_from_section_valid_returns_ok() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 5.0),
        Point::new(0.0, 5.0),
    ]);
    let sec = Section::new(outer, Vec::new());
    let result = SectionProperties::try_from_section(&sec);
    assert!(result.is_ok());
    let props = result.unwrap();
    assert!((props.area - 50.0).abs() < 1e-10);
}

#[test]
fn section_properties_try_from_section_degenerate_returns_err() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 10.0),
        Point::new(0.0, 10.0),
    ]);
    let hole = Polygon::new(vec![
        Point::new(20.0, 0.0),
        Point::new(30.0, 0.0),
        Point::new(30.0, 10.0),
        Point::new(20.0, 10.0),
    ]);
    let sec = Section::new(outer, vec![hole]);
    let result = SectionProperties::try_from_section(&sec);
    assert!(result.is_err());
}
