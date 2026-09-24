//! Phase 109 — displacement and reaction envelope tests.
//!
//! Tests A–H from the Phase 109 specification:
//!   A. Node displacement min/max across two load cases.
//!   B. Reaction min/max at a fixed support.
//!   C. Governing load source tracking.
//!   D. Three load cases — extrema not just first/last.
//!   E. Empty input — no panic.
//!   F. Incompatible results — error.
//!   G. Existing member force envelope regression.
//!   H. Truss decision (documented, not tested here).

use section_properties::Material;
use structural_analysis::{BeamSection, Dof, Envelope, FrameModel, LoadCase, LoadCombination};

const E: f64 = 200e9;
const NU: f64 = 0.3;
const RHO: f64 = 7850.0;

fn steel() -> Material {
    Material::new(E, NU, RHO, "Steel")
}

/// Cantilever: fixed at A, free at B. Non-zero displacement at B.
fn make_cantilever() -> (
    FrameModel,
    structural_analysis::NodeHandle,
    structural_analysis::NodeHandle,
) {
    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(5.0, 0.0).unwrap();
    frame
        .add_member(a, b, steel(), BeamSection::new(5e-3, 2e-5))
        .unwrap();
    frame.fix(a).unwrap();
    (frame, a, b)
}

/// Simply supported: pin at A, pin at B, with a midspan node C for loads.
fn make_simply_supported() -> (
    FrameModel,
    structural_analysis::NodeHandle,
    structural_analysis::NodeHandle,
    structural_analysis::NodeHandle,
) {
    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let c = frame.add_node(2.5, 0.0).unwrap();
    let b = frame.add_node(5.0, 0.0).unwrap();
    frame
        .add_member(a, c, steel(), BeamSection::new(5e-3, 2e-5))
        .unwrap();
    frame
        .add_member(c, b, steel(), BeamSection::new(5e-3, 2e-5))
        .unwrap();
    frame.pin(a).unwrap();
    frame.pin(b).unwrap();
    (frame, a, c, b)
}

// ---------------------------------------------------------------------------
// A. Node displacement min/max
// ---------------------------------------------------------------------------

#[test]
fn a_node_displacement_envelope() {
    let (frame, a, b) = make_cantilever();

    let mut case_down = LoadCase::new("down");
    case_down.nodal_load(b, 0.0, -5000.0).unwrap();

    let mut case_up = LoadCase::new("up");
    case_up.nodal_load(b, 0.0, 3000.0).unwrap();

    let r_down = frame.solve_case(&case_down).unwrap();
    let r_up = frame.solve_case(&case_up).unwrap();

    let envelope = Envelope::from_frame_results(&[&r_down, &r_up], 5).unwrap();

    let disp_b = envelope.node_displacement(b.index()).unwrap();
    let uy_down = r_down.displacement(b, Dof::Uy).unwrap();
    let uy_up = r_up.displacement(b, Dof::Uy).unwrap();

    assert!(
        (disp_b.uy.min - uy_down.min(uy_up)).abs() < 1e-12,
        "min Uy at B"
    );
    assert!(
        (disp_b.uy.max - uy_down.max(uy_up)).abs() < 1e-12,
        "max Uy at B"
    );
    assert!(disp_b.uy.min < 0.0, "min Uy is negative (downward)");
    assert!(disp_b.uy.max > 0.0, "max Uy is positive (upward)");

    let disp_a = envelope.node_displacement(a.index()).unwrap();
    assert!(disp_a.uy.min.abs() < 1e-12, "fixed support A has zero Uy");
    assert!(disp_a.uy.max.abs() < 1e-12, "fixed support A has zero Uy");
}

// ---------------------------------------------------------------------------
// B. Reaction min/max
// ---------------------------------------------------------------------------

#[test]
fn b_reaction_envelope() {
    let (frame, a, b) = make_cantilever();

    let mut case1 = LoadCase::new("case1");
    case1.nodal_load(b, 0.0, -4000.0).unwrap();

    let mut case2 = LoadCase::new("case2");
    case2.nodal_load(b, -2000.0, -1000.0).unwrap();

    let r1 = frame.solve_case(&case1).unwrap();
    let r2 = frame.solve_case(&case2).unwrap();

    let envelope = Envelope::from_frame_results(&[&r1, &r2], 5).unwrap();

    let react_a = envelope.support_reaction(a.index()).unwrap();
    let ry_a1 = r1.reaction(a, Dof::Uy).unwrap();
    let ry_a2 = r2.reaction(a, Dof::Uy).unwrap();

    assert!(
        (react_a.uy.min - ry_a1.min(ry_a2)).abs() < 1e-9,
        "min Ry at A"
    );
    assert!(
        (react_a.uy.max - ry_a1.max(ry_a2)).abs() < 1e-9,
        "max Ry at A"
    );

    let rx_a1 = r1.reaction(a, Dof::Ux).unwrap();
    let rx_a2 = r2.reaction(a, Dof::Ux).unwrap();
    assert!(
        (react_a.ux.min - rx_a1.min(rx_a2)).abs() < 1e-9,
        "min Rx at A"
    );
    assert!(
        (react_a.ux.max - rx_a1.max(rx_a2)).abs() < 1e-9,
        "max Rx at A"
    );
}

// ---------------------------------------------------------------------------
// C. Governing load source
// ---------------------------------------------------------------------------

#[test]
fn c_governing_load_source() {
    let (frame, a, b) = make_cantilever();

    let mut case_down = LoadCase::new("heavy_down");
    case_down.nodal_load(b, 0.0, -10000.0).unwrap();

    let mut case_up = LoadCase::new("light_up");
    case_up.nodal_load(b, 0.0, 1000.0).unwrap();

    let r_down = frame.solve_case(&case_down).unwrap();
    let r_up = frame.solve_case(&case_up).unwrap();

    let envelope = Envelope::from_frame_results(&[&r_down, &r_up], 5).unwrap();

    let disp_b = envelope.node_displacement(b.index()).unwrap();

    assert_eq!(
        disp_b.uy.min_source.as_deref(),
        Some("case:heavy_down"),
        "min Uy governed by heavy_down"
    );
    assert_eq!(
        disp_b.uy.max_source.as_deref(),
        Some("case:light_up"),
        "max Uy governed by light_up"
    );

    let react_a = envelope.support_reaction(a.index()).unwrap();
    assert!(
        react_a.uy.min_source.is_some(),
        "reaction min source is tracked"
    );
    assert!(
        react_a.uy.max_source.is_some(),
        "reaction max source is tracked"
    );
}

// ---------------------------------------------------------------------------
// D. Three load cases — extrema not just first/last
// ---------------------------------------------------------------------------

#[test]
fn d_three_load_cases() {
    let (frame, _a, b) = make_cantilever();

    let mut case_a = LoadCase::new("A");
    case_a.nodal_load(b, 0.0, -1000.0).unwrap();

    let mut case_b = LoadCase::new("B");
    case_b.nodal_load(b, 0.0, -5000.0).unwrap();

    let mut case_c = LoadCase::new("C");
    case_c.nodal_load(b, 0.0, -3000.0).unwrap();

    let r_a = frame.solve_case(&case_a).unwrap();
    let r_b = frame.solve_case(&case_b).unwrap();
    let r_c = frame.solve_case(&case_c).unwrap();

    let envelope = Envelope::from_frame_results(&[&r_a, &r_b, &r_c], 5).unwrap();

    let disp_b = envelope.node_displacement(b.index()).unwrap();
    let uy_b = r_b.displacement(b, Dof::Uy).unwrap();

    assert!(
        (disp_b.uy.min - uy_b).abs() < 1e-12,
        "min Uy from case B (middle case, largest downward)"
    );
    assert_eq!(
        disp_b.uy.min_source.as_deref(),
        Some("case:B"),
        "governing case is B, not first or last"
    );
}

// ---------------------------------------------------------------------------
// E. Empty input — no panic
// ---------------------------------------------------------------------------

#[test]
fn e_empty_input_returns_error() {
    let result = Envelope::from_frame_results(&[], 5);
    assert!(result.is_err());
    let err = result.unwrap_err();
    assert!(matches!(
        err,
        structural_analysis::FemError::InvalidInput(_)
    ));
}

// ---------------------------------------------------------------------------
// F. Incompatible results — error
// ---------------------------------------------------------------------------

#[test]
fn f_incompatible_node_count_rejected() {
    let (frame1, _a1, b1) = make_cantilever();

    let mut frame2 = FrameModel::new();
    let c = frame2.add_node(0.0, 0.0).unwrap();
    let d = frame2.add_node(3.0, 0.0).unwrap();
    let e = frame2.add_node(6.0, 0.0).unwrap();
    frame2
        .add_member(c, d, steel(), BeamSection::new(5e-3, 2e-5))
        .unwrap();
    frame2
        .add_member(d, e, steel(), BeamSection::new(5e-3, 2e-5))
        .unwrap();
    frame2.fix(c).unwrap();

    let mut case1 = LoadCase::new("case1");
    case1.nodal_load(b1, 0.0, -1000.0).unwrap();
    let r1 = frame1.solve_case(&case1).unwrap();

    let mut case2 = LoadCase::new("case2");
    case2.nodal_load(d, 0.0, -1000.0).unwrap();
    let r2 = frame2.solve_case(&case2).unwrap();

    let result = Envelope::from_frame_results(&[&r1, &r2], 5);
    assert!(
        result.is_err(),
        "incompatible node/member counts must error"
    );
}

// ---------------------------------------------------------------------------
// G. Existing member force envelope regression
// ---------------------------------------------------------------------------

#[test]
fn g_member_force_envelope_regression() {
    let (frame, _a, c, _b) = make_simply_supported();

    let mut case1 = LoadCase::new("case1");
    case1.nodal_load(c, 0.0, -4000.0).unwrap();

    let mut case2 = LoadCase::new("case2");
    case2.nodal_load(c, 0.0, -2000.0).unwrap();

    let r1 = frame.solve_case(&case1).unwrap();
    let r2 = frame.solve_case(&case2).unwrap();

    let envelope = Envelope::from_frame_results(&[&r1, &r2], 10).unwrap();

    assert_eq!(envelope.n_results, 2);
    assert_eq!(envelope.n_members, 2);
    assert_eq!(envelope.n_per_member, 10);
    assert_eq!(envelope.samples.len(), 20);

    let max_abs_m = envelope.max_abs_moment();
    assert!(max_abs_m > 0.0, "non-zero moment envelope");

    let mid = envelope.member_samples(0)[5];
    assert!(mid.min.moment <= mid.max.moment, "min <= max for M");
    assert!(mid.min.shear <= mid.max.shear, "min <= max for V");
}

// ---------------------------------------------------------------------------
// LoadCombination provenance in envelope
// ---------------------------------------------------------------------------

#[test]
fn combination_provenance_in_envelope() {
    let (frame, _a, b) = make_cantilever();

    let mut dead = LoadCase::new("dead");
    dead.nodal_load(b, 0.0, -2000.0).unwrap();

    let mut live = LoadCase::new("live");
    live.nodal_load(b, 0.0, -1000.0).unwrap();

    let mut combo = LoadCombination::new("1.2D+1.6L");
    combo.add_case(&dead, 1.2).unwrap();
    combo.add_case(&live, 1.6).unwrap();

    let r_dead = frame.solve_case(&dead).unwrap();
    let r_combo = frame.solve_combination(&combo).unwrap();

    let envelope = Envelope::from_frame_results(&[&r_dead, &r_combo], 5).unwrap();

    let disp_b = envelope.node_displacement(b.index()).unwrap();
    assert!(disp_b.uy.min_source.is_some(), "min source tracked");
    let source = disp_b.uy.min_source.as_deref().unwrap();
    assert!(
        source.contains("combination"),
        "combination provenance preserved: {source}"
    );
}

// ---------------------------------------------------------------------------
// Envelope node count matches model
// ---------------------------------------------------------------------------

#[test]
fn envelope_node_count_matches_model() {
    let (frame, _a, _c, b) = make_simply_supported();

    let mut case = LoadCase::new("case");
    case.nodal_load(b, 0.0, -1000.0).unwrap();

    let r = frame.solve_case(&case).unwrap();
    let envelope = Envelope::from_frame_results(&[&r], 5).unwrap();

    assert_eq!(envelope.n_nodes, 3);
    assert_eq!(envelope.node_displacements.len(), 3);
    assert_eq!(envelope.support_reactions.len(), 3);
}

// ---------------------------------------------------------------------------
// Single result envelope — min == max
// ---------------------------------------------------------------------------

#[test]
fn single_result_envelope_min_equals_max() {
    let (frame, _a, b) = make_cantilever();

    let mut case = LoadCase::new("only");
    case.nodal_load(b, 0.0, -3000.0).unwrap();

    let r = frame.solve_case(&case).unwrap();
    let envelope = Envelope::from_frame_results(&[&r], 5).unwrap();

    let disp_b = envelope.node_displacement(b.index()).unwrap();
    assert!(
        (disp_b.uy.min - disp_b.uy.max).abs() < 1e-12,
        "single result: min == max"
    );
    assert_eq!(
        disp_b.uy.min_source.as_deref(),
        Some("case:only"),
        "source tracked for single result"
    );
}
