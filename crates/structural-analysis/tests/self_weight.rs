//! Phase 108 — self-weight / body-force load tests.
//!
//! Tests A–F from the Phase 108 specification:
//!   A. Single horizontal beam — equivalent nodal loads (via reactions).
//!   B. Simply supported beam — reactions, total weight, symmetry, equilibrium.
//!   C. Inclined beam — gravity stays in global direction, not local −y.
//!   D. Multi-member frame — each member contributes ρ·A·L.
//!   E. LoadCase isolation — self-weight in one case does not pollute another.
//!   F. LoadCombination — factor × self-weight matches scaled solve.

use section_properties::Material;
use structural_analysis::{BeamSection, Dof, FrameModel, LoadCase, LoadCombination};

const E: f64 = 200e9;
const NU: f64 = 0.3;
const RHO: f64 = 7850.0;
const G: f64 = 9.81;

fn steel() -> Material {
    Material::new(E, NU, RHO, "Steel")
}

// ---------------------------------------------------------------------------
// A. Single horizontal beam — equivalent nodal loads (via fixed-end reactions)
// ---------------------------------------------------------------------------

#[test]
fn a_single_beam_equivalent_nodal_loads() {
    let l = 4.0;
    let area = 5e-3;
    let i = 2e-5;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(l, 0.0).unwrap();
    frame
        .add_member(a, b, steel(), BeamSection::new(area, i))
        .unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();

    let sw = frame.self_weight(0.0, -G).unwrap();
    let result = frame.solve_case(&sw).unwrap();
    assert!(result.equilibrium().is_balanced());

    let w = RHO * area * G;
    let ry_a = result.reaction(a, Dof::Uy).unwrap();
    let ry_b = result.reaction(b, Dof::Uy).unwrap();
    let rz_a = result.reaction(a, Dof::Rz).unwrap();
    let rz_b = result.reaction(b, Dof::Rz).unwrap();

    let expected_ry = w * l / 2.0;
    let expected_m = w * l * l / 12.0;

    assert!(
        (ry_a - expected_ry).abs() < 1e-6 * expected_ry,
        "R_Ay = wL/2"
    );
    assert!(
        (ry_b - expected_ry).abs() < 1e-6 * expected_ry,
        "R_By = wL/2"
    );
    assert!(
        (rz_a.abs() - expected_m).abs() < 1e-6 * expected_m,
        "|M_A| = wL²/12"
    );
    assert!(
        (rz_b.abs() - expected_m).abs() < 1e-6 * expected_m,
        "|M_B| = wL²/12"
    );
    assert!(rz_a * rz_b < 0.0, "M_A and M_B have opposite signs");

    let rx_a = result.reaction(a, Dof::Ux).unwrap();
    let rx_b = result.reaction(b, Dof::Ux).unwrap();
    assert!(rx_a.abs() < 1e-9, "no axial reaction at A");
    assert!(rx_b.abs() < 1e-9, "no axial reaction at B");
}

// ---------------------------------------------------------------------------
// B. Simply supported beam — reactions, total weight, symmetry, equilibrium
// ---------------------------------------------------------------------------

#[test]
fn b_simply_supported_reactions_and_equilibrium() {
    let l = 6.0;
    let area = 5e-3;
    let i = 2e-5;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(l, 0.0).unwrap();
    frame
        .add_member(a, b, steel(), BeamSection::new(area, i))
        .unwrap();
    frame.pin(a).unwrap();
    frame.pin(b).unwrap();

    let sw = frame.self_weight(0.0, -G).unwrap();
    let result = frame.solve_case(&sw).unwrap();

    let total_weight = RHO * area * l * G;
    let ry_a = result.reaction(a, Dof::Uy).unwrap();
    let ry_b = result.reaction(b, Dof::Uy).unwrap();

    assert!(
        (ry_a + ry_b - total_weight).abs() < 1e-6 * total_weight,
        "total reaction != total weight"
    );
    assert!(
        (ry_a - total_weight / 2.0).abs() < 1e-6 * total_weight,
        "symmetric A"
    );
    assert!(
        (ry_b - total_weight / 2.0).abs() < 1e-6 * total_weight,
        "symmetric B"
    );
    assert!(result.equilibrium().is_balanced());
}

// ---------------------------------------------------------------------------
// C. Inclined beam — gravity stays in global direction
// ---------------------------------------------------------------------------

#[test]
fn c_inclined_beam_global_gravity_direction() {
    let l = 4.0;
    let dx = l / 2.0_f64.sqrt();
    let dy = l / 2.0_f64.sqrt();
    let area = 5e-3;
    let i = 2e-5;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(dx, dy).unwrap();
    frame
        .add_member(a, b, steel(), BeamSection::new(area, i))
        .unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();

    let sw = frame.self_weight(0.0, -G).unwrap();
    let result = frame.solve_case(&sw).unwrap();
    assert!(result.equilibrium().is_balanced());

    let total_weight = RHO * area * l * G;
    let ry_a = result.reaction(a, Dof::Uy).unwrap();
    let ry_b = result.reaction(b, Dof::Uy).unwrap();
    assert!(
        (ry_a + ry_b - total_weight).abs() < 1e-6 * total_weight,
        "vertical reactions sum to total weight"
    );

    let rx_a = result.reaction(a, Dof::Ux).unwrap();
    let rx_b = result.reaction(b, Dof::Ux).unwrap();
    assert!(
        (rx_a + rx_b).abs() < 1e-6 * total_weight,
        "horizontal reactions sum to zero for vertical gravity"
    );
}

// ---------------------------------------------------------------------------
// D. Multi-member frame — each member contributes ρ·A·L
// ---------------------------------------------------------------------------

#[test]
fn d_multi_member_total_weight() {
    let l1 = 3.0;
    let l2 = 4.0;
    let area1 = 5e-3;
    let area2 = 8e-3;
    let i1 = 2e-5;
    let i2 = 5e-5;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(l1, 0.0).unwrap();
    let c = frame.add_node(l1, l2).unwrap();
    frame
        .add_member(a, b, steel(), BeamSection::new(area1, i1))
        .unwrap();
    frame
        .add_member(b, c, steel(), BeamSection::new(area2, i2))
        .unwrap();
    frame.fix(a).unwrap();
    frame.fix(c).unwrap();

    let sw = frame.self_weight(0.0, -G).unwrap();
    let result = frame.solve_case(&sw).unwrap();
    assert!(result.equilibrium().is_balanced());

    let total_weight = RHO * G * (area1 * l1 + area2 * l2);
    let ry_a = result.reaction(a, Dof::Uy).unwrap();
    let ry_c = result.reaction(c, Dof::Uy).unwrap();
    assert!(
        (ry_a + ry_c - total_weight).abs() < 1e-6 * total_weight,
        "total vertical reaction != total weight"
    );

    let rx_a = result.reaction(a, Dof::Ux).unwrap();
    let rx_c = result.reaction(c, Dof::Ux).unwrap();
    assert!(
        (rx_a + rx_c).abs() < 1e-6 * total_weight,
        "total horizontal reaction should be zero"
    );
}

// ---------------------------------------------------------------------------
// E. LoadCase isolation — no cross-case contamination
// ---------------------------------------------------------------------------

#[test]
fn e_loadcase_isolation() {
    let l = 5.0;
    let area = 6e-3;
    let i = 3e-5;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(l, 0.0).unwrap();
    frame
        .add_member(a, b, steel(), BeamSection::new(area, i))
        .unwrap();
    frame.pin(a).unwrap();
    frame.pin(b).unwrap();

    let mut case_a = LoadCase::new("A");
    case_a.add_self_weight(&frame, 0.0, -G).unwrap();
    case_a.nodal_load(b, 0.0, -2000.0).unwrap();

    let mut case_b = LoadCase::new("B");
    case_b.nodal_load(b, 0.0, -500.0).unwrap();

    let result_a = frame.solve_case(&case_a).unwrap();
    let result_b = frame.solve_case(&case_b).unwrap();

    let sw_weight = RHO * area * l * G;
    let total_a = result_a.reaction(a, Dof::Uy).unwrap() + result_a.reaction(b, Dof::Uy).unwrap();
    let total_b = result_b.reaction(a, Dof::Uy).unwrap() + result_b.reaction(b, Dof::Uy).unwrap();

    assert!(
        (total_a - (sw_weight + 2000.0)).abs() < 1e-6 * (sw_weight + 2000.0),
        "case A includes self-weight"
    );
    assert!(
        (total_b - 500.0).abs() < 1e-6 * 500.0,
        "case B does NOT include self-weight"
    );
    assert!(result_a.equilibrium().is_balanced());
    assert!(result_b.equilibrium().is_balanced());
}

// ---------------------------------------------------------------------------
// F. LoadCombination — factor × self-weight
// ---------------------------------------------------------------------------

#[test]
fn f_load_combination_self_weight() {
    let l = 5.0;
    let area = 6e-3;
    let i = 3e-5;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(l, 0.0).unwrap();
    frame
        .add_member(a, b, steel(), BeamSection::new(area, i))
        .unwrap();
    frame.pin(a).unwrap();
    frame.pin(b).unwrap();

    let mut dead = LoadCase::new("dead");
    dead.add_self_weight(&frame, 0.0, -G).unwrap();

    let mut live = LoadCase::new("live");
    live.nodal_load(b, 0.0, -3000.0).unwrap();

    let mut combo = LoadCombination::new("1.2D + 1.6L");
    combo.add_case(&dead, 1.2).unwrap();
    combo.add_case(&live, 1.6).unwrap();

    let result_combo = frame.solve_combination(&combo).unwrap();
    assert!(result_combo.equilibrium().is_balanced());

    let sw_weight = RHO * area * l * G;
    let expected_total = 1.2 * sw_weight + 1.6 * 3000.0;
    let total =
        result_combo.reaction(a, Dof::Uy).unwrap() + result_combo.reaction(b, Dof::Uy).unwrap();
    assert!(
        (total - expected_total).abs() < 1e-6 * expected_total,
        "combination total != expected"
    );

    let result_dead = frame.solve_case(&dead).unwrap();
    let result_live = frame.solve_case(&live).unwrap();
    let uy_dead = result_dead.displacement(b, Dof::Uy).unwrap();
    let uy_live = result_live.displacement(b, Dof::Uy).unwrap();
    let uy_combo = result_combo.displacement(b, Dof::Uy).unwrap();
    let uy_superposed = 1.2 * uy_dead + 1.6 * uy_live;
    assert!(
        (uy_combo - uy_superposed).abs() < 1e-9 * uy_combo.abs().max(1.0),
        "superposition check"
    );
}

// ---------------------------------------------------------------------------
// Input validation
// ---------------------------------------------------------------------------

#[test]
fn validation_nan_gravity_rejected() {
    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(1.0, 0.0).unwrap();
    frame
        .add_member(a, b, steel(), BeamSection::new(1e-3, 1e-6))
        .unwrap();

    let err = frame.self_weight(f64::NAN, -G).unwrap_err();
    assert!(matches!(
        err,
        structural_analysis::FemError::InvalidInput(_)
    ));

    let err = frame.self_weight(0.0, f64::INFINITY).unwrap_err();
    assert!(matches!(
        err,
        structural_analysis::FemError::InvalidInput(_)
    ));
}

#[test]
fn validation_zero_density_produces_zero_reaction() {
    let l = 2.0;
    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(l, 0.0).unwrap();
    let mat = Material::new(E, NU, 0.0, "massless");
    frame
        .add_member(a, b, mat, BeamSection::new(1e-3, 1e-6))
        .unwrap();
    frame.pin(a).unwrap();
    frame.pin(b).unwrap();

    let sw = frame.self_weight(0.0, -G).unwrap();
    let result = frame.solve_case(&sw).unwrap();

    let ry = result.reaction(a, Dof::Uy).unwrap() + result.reaction(b, Dof::Uy).unwrap();
    assert!(ry.abs() < 1e-12, "zero density → zero reaction");
}

// ---------------------------------------------------------------------------
// FrameModel::self_weight vs LoadCase::add_self_weight consistency
// ---------------------------------------------------------------------------

#[test]
fn self_weight_method_consistency() {
    let l = 4.0;
    let area = 5e-3;
    let i = 2e-5;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(l, 0.0).unwrap();
    frame
        .add_member(a, b, steel(), BeamSection::new(area, i))
        .unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();

    let sw1 = frame.self_weight(0.0, -G).unwrap();

    let mut sw2 = LoadCase::new("sw2");
    sw2.add_self_weight(&frame, 0.0, -G).unwrap();

    let r1 = frame.solve_case(&sw1).unwrap();
    let r2 = frame.solve_case(&sw2).unwrap();
    let uy1 = r1.displacement(b, Dof::Uy).unwrap();
    let uy2 = r2.displacement(b, Dof::Uy).unwrap();
    assert!(
        (uy1 - uy2).abs() < 1e-12,
        "both APIs produce identical results"
    );
}

// ---------------------------------------------------------------------------
// Phase 107 factorization reuse compatibility
// ---------------------------------------------------------------------------

#[test]
fn factorization_reuse_with_self_weight() {
    let l = 5.0;
    let area = 6e-3;
    let i = 3e-5;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(l, 0.0).unwrap();
    frame
        .add_member(a, b, steel(), BeamSection::new(area, i))
        .unwrap();
    frame.pin(a).unwrap();
    frame.pin(b).unwrap();

    let mut case_a = LoadCase::new("A");
    case_a.add_self_weight(&frame, 0.0, -G).unwrap();
    case_a.nodal_load(b, 0.0, -1000.0).unwrap();

    let mut case_b = LoadCase::new("B");
    case_b.add_self_weight(&frame, 0.0, -G).unwrap();
    case_b.nodal_load(b, -500.0, 0.0).unwrap();

    let prepared = frame.prepare().unwrap();
    let results = prepared.solve_cases(&[case_a, case_b]).unwrap();
    assert_eq!(results.len(), 2);

    let mut case_a2 = LoadCase::new("A");
    case_a2.add_self_weight(&frame, 0.0, -G).unwrap();
    case_a2.nodal_load(b, 0.0, -1000.0).unwrap();

    let mut case_b2 = LoadCase::new("B");
    case_b2.add_self_weight(&frame, 0.0, -G).unwrap();
    case_b2.nodal_load(b, -500.0, 0.0).unwrap();

    let r_a = frame.solve_case(&case_a2).unwrap();
    let r_b = frame.solve_case(&case_b2).unwrap();

    let uy_prepared = results[0].displacement(b, Dof::Uy).unwrap();
    let uy_individual = r_a.displacement(b, Dof::Uy).unwrap();
    assert!(
        (uy_prepared - uy_individual).abs() < 1e-10,
        "prepared A matches individual"
    );

    let ux_prepared = results[1].displacement(b, Dof::Ux).unwrap();
    let ux_individual = r_b.displacement(b, Dof::Ux).unwrap();
    assert!(
        (ux_prepared - ux_individual).abs() < 1e-10,
        "prepared B matches individual"
    );

    assert!(results[0].equilibrium().is_balanced());
    assert!(results[1].equilibrium().is_balanced());
}

// ---------------------------------------------------------------------------
// Vertical member — self-weight is axial, not transverse
// ---------------------------------------------------------------------------

#[test]
fn vertical_member_self_weight_is_axial() {
    let l = 3.0;
    let area = 4e-3;
    let i = 1e-5;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(0.0, l).unwrap();
    frame
        .add_member(a, b, steel(), BeamSection::new(area, i))
        .unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();

    let sw = frame.self_weight(0.0, -G).unwrap();
    let result = frame.solve_case(&sw).unwrap();
    assert!(result.equilibrium().is_balanced());

    let total_weight = RHO * area * l * G;
    let ry_a = result.reaction(a, Dof::Uy).unwrap();
    let ry_b = result.reaction(b, Dof::Uy).unwrap();
    assert!(
        (ry_a + ry_b - total_weight).abs() < 1e-6 * total_weight,
        "vertical reactions = total weight"
    );

    let expected_ry = total_weight / 2.0;
    assert!(
        (ry_a - expected_ry).abs() < 1e-6 * expected_ry,
        "R_Ay = wL/2"
    );
    assert!(
        (ry_b - expected_ry).abs() < 1e-6 * expected_ry,
        "R_By = wL/2"
    );

    let rx_a = result.reaction(a, Dof::Ux).unwrap();
    let rx_b = result.reaction(b, Dof::Ux).unwrap();
    assert!(rx_a.abs() < 1e-9, "no horizontal reaction at A");
    assert!(rx_b.abs() < 1e-9, "no horizontal reaction at B");

    let rz_a = result.reaction(a, Dof::Rz).unwrap();
    let rz_b = result.reaction(b, Dof::Rz).unwrap();
    assert!(rz_a.abs() < 1e-9, "no moment at A");
    assert!(rz_b.abs() < 1e-9, "no moment at B");
}

// ---------------------------------------------------------------------------
// Horizontal gravity (gx != 0)
// ---------------------------------------------------------------------------

#[test]
fn horizontal_gravity_component() {
    let l = 4.0;
    let area = 5e-3;
    let i = 2e-5;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(l, 0.0).unwrap();
    frame
        .add_member(a, b, steel(), BeamSection::new(area, i))
        .unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();

    let sw = frame.self_weight(-G, 0.0).unwrap();
    let result = frame.solve_case(&sw).unwrap();
    assert!(result.equilibrium().is_balanced());

    let total_force = RHO * area * l * G;
    let rx_a = result.reaction(a, Dof::Ux).unwrap();
    let rx_b = result.reaction(b, Dof::Ux).unwrap();
    let ry_a = result.reaction(a, Dof::Uy).unwrap();
    let ry_b = result.reaction(b, Dof::Uy).unwrap();

    assert!(
        (rx_a + rx_b - total_force).abs() < 1e-6 * total_force,
        "horizontal reactions = total horizontal force"
    );
    assert!(
        ry_a.abs() < 1e-9,
        "no vertical reaction from horizontal gravity"
    );
    assert!(
        ry_b.abs() < 1e-9,
        "no vertical reaction from horizontal gravity"
    );
}
