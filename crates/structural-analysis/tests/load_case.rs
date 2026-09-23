//! Phase 92 — Load case / load combination tests.

#![allow(non_snake_case)]

use section_properties::material::Material;
use structural_analysis::{
    BeamSection, Dof, FemError, FrameModel, LoadCase, LoadCombination, MemberHandle, NodeHandle,
};

fn steel() -> Material {
    Material::new(200e9, 0.3, 7850.0, "Steel")
}

fn section() -> BeamSection {
    BeamSection::new(5e-3, 2e-5)
}

const E: f64 = 200e9;
const I: f64 = 2e-5;

fn cantilever() -> (FrameModel, NodeHandle, NodeHandle, MemberHandle) {
    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(2.0, 0.0).unwrap();
    let m = frame.add_member(a, b, steel(), section()).unwrap();
    frame.fix(a).unwrap();
    (frame, a, b, m)
}

fn fixed_beam() -> (FrameModel, NodeHandle, NodeHandle, MemberHandle) {
    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(4.0, 0.0).unwrap();
    let m = frame.add_member(a, b, steel(), section()).unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();
    (frame, a, b, m)
}

// §1 — Single LoadCase matches direct loading

#[test]
fn test_loadcase_matches_direct_nodal_load() {
    let (frame1, _, b, _) = cantilever();
    let frame2 = frame1.clone();

    let mut f1 = frame1.clone();
    f1.nodal_load(b, 0.0, -1.0e4).unwrap();
    let r1 = f1.solve().unwrap();

    let mut case = LoadCase::new("P");
    case.nodal_load(b, 0.0, -1.0e4).unwrap();
    let r2 = frame2.solve_case(&case).unwrap();

    let uy1 = r1.displacement(b, Dof::Uy).unwrap();
    let uy2 = r2.displacement(b, Dof::Uy).unwrap();
    assert!((uy1 - uy2).abs() < 1e-12, "uy: direct={uy1}, case={uy2}");
}

#[test]
fn test_loadcase_matches_direct_udl() {
    let (frame1, _, _, m) = fixed_beam();
    let frame2 = frame1.clone();

    let mut f1 = frame1.clone();
    f1.member_udl(m, 0.0, -1000.0).unwrap();
    let r1 = f1.solve().unwrap();

    let mut case = LoadCase::new("udl");
    case.member_udl(m, 0.0, -1000.0).unwrap();
    let r2 = frame2.solve_case(&case).unwrap();

    let forces1 = r1.member_end_forces(m).unwrap();
    let forces2 = r2.member_end_forces(m).unwrap();
    for i in 0..6 {
        assert!(
            (forces1[i] - forces2[i]).abs() < 1e-6,
            "force[{i}]: direct={}, case={}",
            forces1[i],
            forces2[i]
        );
    }
}

#[test]
fn test_loadcase_matches_direct_point_load() {
    let (frame1, _, _, m) = fixed_beam();
    let frame2 = frame1.clone();

    let mut f1 = frame1.clone();
    f1.member_point_load(m, 0.5, 0.0, -5000.0, 0.0).unwrap();
    let r1 = f1.solve().unwrap();

    let mut case = LoadCase::new("P_mid");
    case.member_point_load(m, 0.5, 0.0, -5000.0, 0.0).unwrap();
    let r2 = frame2.solve_case(&case).unwrap();

    let uy1 = r1.displacement(NodeHandle::from_index(1), Dof::Uy).unwrap();
    let uy2 = r2.displacement(NodeHandle::from_index(1), Dof::Uy).unwrap();
    assert!((uy1 - uy2).abs() < 1e-12, "uy: direct={uy1}, case={uy2}");
}

#[test]
fn test_loadcase_matches_direct_moment() {
    let (frame1, _, b, _) = cantilever();
    let frame2 = frame1.clone();

    let mut f1 = frame1.clone();
    f1.nodal_moment(b, 500.0).unwrap();
    let r1 = f1.solve().unwrap();

    let mut case = LoadCase::new("M");
    case.nodal_moment(b, 500.0).unwrap();
    let r2 = frame2.solve_case(&case).unwrap();

    let rz1 = r1.displacement(b, Dof::Rz).unwrap();
    let rz2 = r2.displacement(b, Dof::Rz).unwrap();
    assert!((rz1 - rz2).abs() < 1e-12, "rz: direct={rz1}, case={rz2}");
}

#[test]
fn test_loadcase_provenance() {
    let (frame, _, b, _) = cantilever();
    let mut case = LoadCase::new("dead");
    case.nodal_load(b, 0.0, -1000.0).unwrap();
    let result = frame.solve_case(&case).unwrap();
    assert_eq!(result.load_source(), Some("case:dead"));
}

// §2 — Two independent LoadCases solve independently

#[test]
fn test_two_independent_cases() {
    let (frame, _, b, _) = cantilever();

    let mut case_a = LoadCase::new("A");
    case_a.nodal_load(b, 0.0, -1000.0).unwrap();

    let mut case_b = LoadCase::new("B");
    case_b.nodal_load(b, 0.0, -2000.0).unwrap();

    let ra = frame.solve_case(&case_a).unwrap();
    let rb = frame.solve_case(&case_b).unwrap();

    let uya = ra.displacement(b, Dof::Uy).unwrap();
    let uyb = rb.displacement(b, Dof::Uy).unwrap();

    let p = 1000.0_f64;
    let l = 2.0_f64;
    let uy_a_exact = -p * l * l * l / (3.0 * E * I);
    let uy_b_exact = -2.0 * p * l * l * l / (3.0 * E * I);

    assert!(
        (uya - uy_a_exact).abs() < 1e-9,
        "uy_a={uya}, exact={uy_a_exact}"
    );
    assert!(
        (uyb - uy_b_exact).abs() < 1e-9,
        "uy_b={uyb}, exact={uy_b_exact}"
    );
}

// §3 — Linear superposition: solve(1.0A + 1.0B) ≈ solve(A) + solve(B)

#[test]
fn test_superposition_displacement() {
    let (frame, _, b, m) = fixed_beam();

    let mut case_a = LoadCase::new("A");
    case_a.nodal_load(b, 0.0, -3000.0).unwrap();

    let mut case_b = LoadCase::new("B");
    case_b.member_udl(m, 0.0, -1000.0).unwrap();

    let ra = frame.solve_case(&case_a).unwrap();
    let rb = frame.solve_case(&case_b).unwrap();

    let mut combo = LoadCombination::new("A+B");
    combo.add_case(&case_a, 1.0).unwrap();
    combo.add_case(&case_b, 1.0).unwrap();
    let rc = frame.solve_combination(&combo).unwrap();

    for dof in [Dof::Ux, Dof::Uy, Dof::Rz] {
        let da = ra.displacement(b, dof).unwrap();
        let db = rb.displacement(b, dof).unwrap();
        let dc = rc.displacement(b, dof).unwrap();
        assert!(
            (dc - (da + db)).abs() < 1e-9,
            "superposition failed for {dof:?}: combo={dc}, sum={}",
            da + db
        );
    }
}

#[test]
fn test_superposition_reactions() {
    let (frame, _, b, m) = fixed_beam();

    let mut case_a = LoadCase::new("A");
    case_a.nodal_load(b, 0.0, -3000.0).unwrap();

    let mut case_b = LoadCase::new("B");
    case_b.member_udl(m, 0.0, -1000.0).unwrap();

    let ra = frame.solve_case(&case_a).unwrap();
    let rb = frame.solve_case(&case_b).unwrap();

    let mut combo = LoadCombination::new("A+B");
    combo.add_case(&case_a, 1.0).unwrap();
    combo.add_case(&case_b, 1.0).unwrap();
    let rc = frame.solve_combination(&combo).unwrap();

    let a = NodeHandle::from_index(0);
    for dof in [Dof::Ux, Dof::Uy, Dof::Rz] {
        let ra_a = ra.reaction(a, dof).unwrap();
        let rb_a = rb.reaction(a, dof).unwrap();
        let rc_a = rc.reaction(a, dof).unwrap();
        assert!(
            (rc_a - (ra_a + rb_a)).abs() < 1e-6,
            "reaction superposition failed for {dof:?}"
        );
    }
}

#[test]
fn test_superposition_member_forces() {
    let (frame, _, _, m) = fixed_beam();

    let mut case_a = LoadCase::new("A");
    case_a
        .nodal_load(NodeHandle::from_index(1), 0.0, -3000.0)
        .unwrap();

    let mut case_b = LoadCase::new("B");
    case_b.member_udl(m, 0.0, -1000.0).unwrap();

    let ra = frame.solve_case(&case_a).unwrap();
    let rb = frame.solve_case(&case_b).unwrap();

    let mut combo = LoadCombination::new("A+B");
    combo.add_case(&case_a, 1.0).unwrap();
    combo.add_case(&case_b, 1.0).unwrap();
    let rc = frame.solve_combination(&combo).unwrap();

    let fa = ra.member_end_forces(m).unwrap();
    let fb = rb.member_end_forces(m).unwrap();
    let fc = rc.member_end_forces(m).unwrap();
    for i in 0..6 {
        assert!(
            (fc[i] - (fa[i] + fb[i])).abs() < 1e-6,
            "member force[{i}] superposition failed"
        );
    }
}

// §4 — Load factors: 2.0×A ≈ 2×solve(A)

#[test]
fn test_load_factor_2x() {
    let (frame, _, b, _) = cantilever();

    let mut case = LoadCase::new("P");
    case.nodal_load(b, 0.0, -1000.0).unwrap();

    let r1 = frame.solve_case(&case).unwrap();

    let mut combo = LoadCombination::new("2P");
    combo.add_case(&case, 2.0).unwrap();
    let r2 = frame.solve_combination(&combo).unwrap();

    let uy1 = r1.displacement(b, Dof::Uy).unwrap();
    let uy2 = r2.displacement(b, Dof::Uy).unwrap();
    assert!(
        (uy2 - 2.0 * uy1).abs() < 1e-12,
        "2×uy={uy2}, expected={}",
        2.0 * uy1
    );
}

#[test]
fn test_load_factor_negative() {
    let (frame, _, b, _) = cantilever();

    let mut case = LoadCase::new("P");
    case.nodal_load(b, 0.0, -1000.0).unwrap();

    let r_pos = frame.solve_case(&case).unwrap();

    let mut combo = LoadCombination::new("-P");
    combo.add_case(&case, -1.0).unwrap();
    let r_neg = frame.solve_combination(&combo).unwrap();

    let uy_pos = r_pos.displacement(b, Dof::Uy).unwrap();
    let uy_neg = r_neg.displacement(b, Dof::Uy).unwrap();
    assert!(
        (uy_neg + uy_pos).abs() < 1e-12,
        "neg factor should flip sign"
    );
}

#[test]
fn test_load_factor_zero() {
    let (frame, _, b, _) = cantilever();

    let mut case = LoadCase::new("P");
    case.nodal_load(b, 0.0, -1000.0).unwrap();

    let mut combo = LoadCombination::new("0P");
    combo.add_case(&case, 0.0).unwrap();
    let r = frame.solve_combination(&combo).unwrap();

    let uy = r.displacement(b, Dof::Uy).unwrap();
    assert!(
        uy.abs() < 1e-15,
        "zero factor should give zero displacement, got {uy}"
    );
}

#[test]
fn test_multiple_factors() {
    let (frame, _, b, m) = fixed_beam();

    let mut dead = LoadCase::new("dead");
    dead.member_udl(m, 0.0, -1000.0).unwrap();

    let mut live = LoadCase::new("live");
    live.nodal_load(b, 0.0, -2000.0).unwrap();

    let mut combo = LoadCombination::new("1.2D+1.6L");
    combo.add_case(&dead, 1.2).unwrap();
    combo.add_case(&live, 1.6).unwrap();
    let r = frame.solve_combination(&combo).unwrap();

    let rd = frame.solve_case(&dead).unwrap();
    let rl = frame.solve_case(&live).unwrap();

    let uy = r.displacement(b, Dof::Uy).unwrap();
    let uy_expected =
        1.2 * rd.displacement(b, Dof::Uy).unwrap() + 1.6 * rl.displacement(b, Dof::Uy).unwrap();
    assert!(
        (uy - uy_expected).abs() < 1e-9,
        "factor combination mismatch"
    );
}

// §5 — Mixed load types in one case

#[test]
fn test_mixed_load_types() {
    let (frame, _, b, m) = fixed_beam();

    let mut case = LoadCase::new("mixed");
    case.nodal_load(b, 500.0, -3000.0).unwrap();
    case.nodal_moment(b, 200.0).unwrap();
    case.member_udl(m, 0.0, -800.0).unwrap();
    case.member_point_load(m, 0.3, 100.0, -500.0, 50.0).unwrap();

    let result = frame.solve_case(&case).unwrap();
    assert!(
        result.equilibrium().is_balanced(),
        "equilibrium failed for mixed loads"
    );
}

// §6 — Local/global transformation (angled member)

#[test]
fn test_angled_member_udl() {
    let L = 4.0_f64;
    let angle = std::f64::consts::FRAC_PI_4;
    let (dx, dy) = (L * angle.cos(), L * angle.sin());

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(dx, dy).unwrap();
    let m = frame.add_member(a, b, steel(), section()).unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();

    let qy = -1000.0_f64;

    let mut f1 = frame.clone();
    f1.member_udl(m, 0.0, qy).unwrap();
    let r1 = f1.solve().unwrap();

    let mut case = LoadCase::new("udl_angled");
    case.member_udl(m, 0.0, qy).unwrap();
    let r2 = frame.solve_case(&case).unwrap();

    let f1_forces = r1.member_end_forces(m).unwrap();
    let f2_forces = r2.member_end_forces(m).unwrap();
    for i in 0..6 {
        assert!(
            (f1_forces[i] - f2_forces[i]).abs() < 1e-6,
            "angled member force[{i}]: direct={}, case={}",
            f1_forces[i],
            f2_forces[i]
        );
    }
}

// §7 — End release + LoadCase/Combination

#[test]
fn test_end_release_with_loadcase() {
    use structural_analysis::EndRelease;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(4.0, 0.0).unwrap();
    let m = frame
        .add_member_with_release(a, b, steel(), section(), EndRelease::end_pin())
        .unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();

    let mut case = LoadCase::new("udl");
    case.member_udl(m, 0.0, -1000.0).unwrap();
    let result = frame.solve_case(&case).unwrap();

    let forces = result.member_end_forces(m).unwrap();
    assert!(
        forces[5].abs() < 1.0,
        "released end moment should be ~0, got {}",
        forces[5]
    );
    assert!(result.equilibrium().is_balanced());
}

#[test]
fn test_end_release_with_combination() {
    use structural_analysis::EndRelease;

    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(4.0, 0.0).unwrap();
    let m = frame
        .add_member_with_release(a, b, steel(), section(), EndRelease::end_pin())
        .unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();

    let mut dead = LoadCase::new("dead");
    dead.member_udl(m, 0.0, -1000.0).unwrap();

    let mut live = LoadCase::new("live");
    live.nodal_load(b, 0.0, -500.0).unwrap();

    let mut combo = LoadCombination::new("1.4D+1.6L");
    combo.add_case(&dead, 1.4).unwrap();
    combo.add_case(&live, 1.6).unwrap();

    let result = frame.solve_combination(&combo).unwrap();
    let forces = result.member_end_forces(m).unwrap();
    assert!(
        forces[5].abs() < 1.0,
        "released end moment should be ~0 with combination, got {}",
        forces[5]
    );
    assert!(result.equilibrium().is_balanced());
}

// §8 — Equilibrium

#[test]
fn test_equilibrium_single_case() {
    let (frame, _, b, m) = fixed_beam();

    let mut case = LoadCase::new("eq");
    case.nodal_load(b, 1000.0, -2000.0).unwrap();
    case.nodal_moment(b, 300.0).unwrap();
    case.member_udl(m, 200.0, -500.0).unwrap();
    case.member_point_load(m, 0.4, 100.0, -300.0, 50.0).unwrap();

    let result = frame.solve_case(&case).unwrap();
    assert!(result.equilibrium().is_balanced());
}

#[test]
fn test_equilibrium_combination() {
    let (frame, _, b, m) = fixed_beam();

    let mut dead = LoadCase::new("dead");
    dead.member_udl(m, 0.0, -1000.0).unwrap();

    let mut live = LoadCase::new("live");
    live.nodal_load(b, 0.0, -2000.0).unwrap();
    live.nodal_moment(b, 500.0).unwrap();

    let mut combo = LoadCombination::new("combo");
    combo.add_case(&dead, 1.4).unwrap();
    combo.add_case(&live, 1.6).unwrap();

    let result = frame.solve_combination(&combo).unwrap();
    assert!(result.equilibrium().is_balanced());
}

// §9 — Invalid input

#[test]
fn test_nan_nodal_load_rejected() {
    let (_, _, b, _) = cantilever();
    let mut case = LoadCase::new("bad");
    assert!(case.nodal_load(b, f64::NAN, 0.0).is_err());
    assert!(case.nodal_load(b, 0.0, f64::NAN).is_err());
}

#[test]
fn test_inf_nodal_load_rejected() {
    let (_, _, b, _) = cantilever();
    let mut case = LoadCase::new("bad");
    assert!(case.nodal_load(b, f64::INFINITY, 0.0).is_err());
    assert!(case.nodal_load(b, 0.0, f64::NEG_INFINITY).is_err());
}

#[test]
fn test_nan_udl_rejected() {
    let (_, _, _, m) = fixed_beam();
    let mut case = LoadCase::new("bad");
    assert!(case.member_udl(m, f64::NAN, 0.0).is_err());
    assert!(case.member_udl(m, 0.0, f64::NAN).is_err());
}

#[test]
fn test_nan_point_load_rejected() {
    let (_, _, _, m) = fixed_beam();
    let mut case = LoadCase::new("bad");
    assert!(case.member_point_load(m, 0.5, f64::NAN, 0.0, 0.0).is_err());
    assert!(
        case.member_point_load(m, 0.5, 0.0, f64::INFINITY, 0.0)
            .is_err()
    );
}

#[test]
fn test_nan_moment_rejected() {
    let (_, _, b, _) = cantilever();
    let mut case = LoadCase::new("bad");
    assert!(case.nodal_moment(b, f64::NAN).is_err());
    assert!(case.nodal_moment(b, f64::INFINITY).is_err());
}

#[test]
fn test_nan_factor_rejected() {
    let (_, _, b, _) = cantilever();
    let mut case = LoadCase::new("ok");
    case.nodal_load(b, 0.0, -1000.0).unwrap();

    let mut combo = LoadCombination::new("bad");
    assert!(combo.add_case(&case, f64::NAN).is_err());
    assert!(combo.add_case(&case, f64::INFINITY).is_err());
    assert!(combo.add_case(&case, f64::NEG_INFINITY).is_err());
}

#[test]
fn test_invalid_node_in_loadcase() {
    let (frame, _, _, _) = cantilever();
    let mut case = LoadCase::new("bad_node");
    case.nodal_load(NodeHandle::from_index(999), 0.0, -1000.0)
        .unwrap();
    let err = frame.solve_case(&case).unwrap_err();
    assert!(matches!(err, FemError::InvalidModel(_)));
}

#[test]
fn test_invalid_member_in_loadcase() {
    let (frame, _, _, _) = cantilever();
    let mut case = LoadCase::new("bad_member");
    case.member_udl(MemberHandle::from_index(999), 0.0, -1000.0)
        .unwrap();
    let err = frame.solve_case(&case).unwrap_err();
    assert!(matches!(err, FemError::InvalidModel(_)));
}

#[test]
fn test_empty_loadcase() {
    let (frame, _, b, _) = cantilever();
    let case = LoadCase::new("empty");
    assert!(case.is_empty());
    let result = frame.solve_case(&case).unwrap();
    let uy = result.displacement(b, Dof::Uy).unwrap();
    assert!(uy.abs() < 1e-15, "empty case should give zero displacement");
}

#[test]
fn test_empty_combination() {
    let (frame, _, b, _) = cantilever();
    let combo = LoadCombination::new("empty");
    assert!(combo.is_empty());
    let result = frame.solve_combination(&combo).unwrap();
    let uy = result.displacement(b, Dof::Uy).unwrap();
    assert!(
        uy.abs() < 1e-15,
        "empty combination should give zero displacement"
    );
}

// §10 — Combination provenance and metadata

#[test]
fn test_combination_provenance() {
    let (frame, _, b, _) = cantilever();
    let mut case = LoadCase::new("P");
    case.nodal_load(b, 0.0, -1000.0).unwrap();

    let mut combo = LoadCombination::new("1.4P");
    combo.add_case(&case, 1.4).unwrap();
    let result = frame.solve_combination(&combo).unwrap();
    assert_eq!(result.load_source(), Some("combination:1.4P"));
}

#[test]
fn test_direct_solve_no_provenance() {
    let (frame, _, b, _) = cantilever();
    let mut f = frame.clone();
    f.nodal_load(b, 0.0, -1000.0).unwrap();
    let result = f.solve().unwrap();
    assert_eq!(result.load_source(), None);
}

#[test]
fn test_loadcase_name() {
    let case = LoadCase::new("my_case");
    assert_eq!(case.name(), "my_case");
}

#[test]
fn test_combination_name_and_terms() {
    let (_, _, b, _) = cantilever();
    let mut case = LoadCase::new("P");
    case.nodal_load(b, 0.0, -1000.0).unwrap();

    let mut combo = LoadCombination::new("combo");
    assert_eq!(combo.name(), "combo");
    assert_eq!(combo.n_terms(), 0);
    assert!(combo.is_empty());

    combo.add_case(&case, 1.0).unwrap();
    combo.add_case(&case, 0.5).unwrap();
    assert_eq!(combo.n_terms(), 2);
    assert!(!combo.is_empty());
}
