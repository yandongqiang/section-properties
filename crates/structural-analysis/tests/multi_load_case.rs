//! Tests for multi-load-case factorisation reuse (Phase 107).
//!
//! These tests verify that `FrameModel::prepare()` + `solve_cases()` produces
//! results that are numerically identical to individual `solve_case()` calls,
//! while reusing a single stiffness factorisation.

use section_properties::Material;
use structural_analysis::{
    BeamSection, Dof, FrameModel, LoadCase, LoadCombination, LoadSource, MemberHandle, NodeHandle,
};

const E: f64 = 200e9;
const NU: f64 = 0.3;
const RHO: f64 = 7850.0;
const A: f64 = 5e-3;
const I: f64 = 2e-5;

fn steel() -> Material {
    Material::new(E, NU, RHO, "Steel")
}

fn sec() -> BeamSection {
    BeamSection::new(A, I)
}

const TOL: f64 = 1e-10;

/// Compare two FrameAnalysisResults for numerical equivalence.
fn assert_results_match(
    a: &structural_analysis::FrameAnalysisResult,
    b: &structural_analysis::FrameAnalysisResult,
    label: &str,
) {
    let n_nodes = a.n_nodes();
    assert_eq!(b.n_nodes(), n_nodes, "{label}: node count mismatch");
    let n_members = a.n_members();
    assert_eq!(b.n_members(), n_members, "{label}: member count mismatch");

    for i in 0..n_nodes {
        let node = NodeHandle::from_index(i);
        for dof in [Dof::Ux, Dof::Uy, Dof::Rz] {
            let ua = a.displacement(node, dof).unwrap();
            let ub = b.displacement(node, dof).unwrap();
            assert!(
                (ua - ub).abs() < TOL,
                "{label}: node {i} {:?} displacement: {ua} vs {ub} (diff {})",
                dof,
                (ua - ub).abs()
            );
        }
    }

    let ra = a.reactions();
    let rb = b.reactions();
    assert_eq!(ra.len(), rb.len(), "{label}: reaction length mismatch");
    for i in 0..ra.len() {
        assert!(
            (ra[i] - rb[i]).abs() < TOL,
            "{label}: reaction[{i}]: {} vs {} (diff {})",
            ra[i],
            rb[i],
            (ra[i] - rb[i]).abs()
        );
    }

    for m in 0..n_members {
        let member = MemberHandle::from_index(m);
        let fa = a.member_end_forces(member).unwrap();
        let fb = b.member_end_forces(member).unwrap();
        for i in 0..6 {
            assert!(
                (fa[i] - fb[i]).abs() < TOL,
                "{label}: member {m} end force[{i}]: {} vs {} (diff {})",
                fa[i],
                fb[i],
                (fa[i] - fb[i]).abs()
            );
        }
        for xi in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let sa = a.section_forces(member, xi).unwrap();
            let sb = b.section_forces(member, xi).unwrap();
            assert!(
                (sa.axial - sb.axial).abs() < TOL,
                "{label}: member {m} xi={xi} axial: {} vs {}",
                sa.axial,
                sb.axial
            );
            assert!(
                (sa.shear - sb.shear).abs() < TOL,
                "{label}: member {m} xi={xi} shear: {} vs {}",
                sa.shear,
                sb.shear
            );
            assert!(
                (sa.moment - sb.moment).abs() < TOL,
                "{label}: member {m} xi={xi} moment: {} vs {}",
                sa.moment,
                sb.moment
            );
        }
    }
}

/// Build a simple portal frame for testing.
fn portal_frame() -> Result<FrameModel, Box<dyn std::error::Error>> {
    let mut frame = FrameModel::new();
    let b1 = frame.add_node(0.0, 0.0)?;
    let t1 = frame.add_node(0.0, 3.0)?;
    let t2 = frame.add_node(4.0, 3.0)?;
    let b2 = frame.add_node(4.0, 0.0)?;
    frame.add_member(b1, t1, steel(), sec())?;
    frame.add_member(t1, t2, steel(), sec())?;
    frame.add_member(b2, t2, steel(), sec())?;
    frame.fix(b1)?;
    frame.fix(b2)?;
    Ok(frame)
}

fn make_cases(
    frame: &FrameModel,
) -> Result<(LoadCase, LoadCase, LoadCase), Box<dyn std::error::Error>> {
    let nodes: Vec<NodeHandle> = (0..frame.n_nodes()).map(NodeHandle::from_index).collect();
    let members: Vec<MemberHandle> = (0..frame.n_members())
        .map(MemberHandle::from_index)
        .collect();

    let mut dead = LoadCase::new("dead");
    dead.member_udl(members[1], 0.0, -1000.0)?;

    let mut live = LoadCase::new("live");
    live.nodal_load(nodes[1], 5000.0, 0.0)?;
    live.nodal_load(nodes[2], 5000.0, 0.0)?;

    let mut wind = LoadCase::new("wind");
    wind.nodal_load(nodes[1], 0.0, 2000.0)?;
    wind.nodal_moment(nodes[1], 500.0)?;

    Ok((dead, live, wind))
}

#[test]
fn numerical_equivalence_individual_vs_prepared() -> Result<(), Box<dyn std::error::Error>> {
    let frame = portal_frame()?;
    let (dead, live, wind) = make_cases(&frame)?;

    let r_dead_indiv = frame.solve_case(&dead)?;
    let r_live_indiv = frame.solve_case(&live)?;
    let r_wind_indiv = frame.solve_case(&wind)?;

    let prepared = frame.prepare()?;
    let r_dead_prep = prepared.solve_case(&dead)?;
    let r_live_prep = prepared.solve_case(&live)?;
    let r_wind_prep = prepared.solve_case(&wind)?;

    assert_results_match(&r_dead_indiv, &r_dead_prep, "dead");
    assert_results_match(&r_live_indiv, &r_live_prep, "live");
    assert_results_match(&r_wind_indiv, &r_wind_prep, "wind");
    Ok(())
}

#[test]
fn solve_cases_preserves_order() -> Result<(), Box<dyn std::error::Error>> {
    let frame = portal_frame()?;
    let (dead, live, wind) = make_cases(&frame)?;

    let prepared = frame.prepare()?;
    let results = prepared.solve_cases(&[dead.clone(), live.clone(), wind.clone()])?;

    assert_eq!(results.len(), 3);
    assert_eq!(results[0].load_source().name(), Some("dead"));
    assert_eq!(results[1].load_source().name(), Some("live"));
    assert_eq!(results[2].load_source().name(), Some("wind"));
    assert!(matches!(
        results[0].load_source(),
        LoadSource::LoadCase {
            has_prescribed_displacements: false,
            ..
        }
    ));
    Ok(())
}

#[test]
fn solve_cases_convenience_method() -> Result<(), Box<dyn std::error::Error>> {
    let frame = portal_frame()?;
    let (dead, live, wind) = make_cases(&frame)?;

    let results_convenience = frame.solve_cases(&[dead.clone(), live.clone(), wind.clone()])?;

    let prepared = frame.prepare()?;
    let results_prepared = prepared.solve_cases(&[dead, live, wind])?;

    assert_eq!(results_convenience.len(), results_prepared.len());
    for (i, (c, p)) in results_convenience
        .iter()
        .zip(&results_prepared)
        .enumerate()
    {
        assert_results_match(c, p, &format!("case {i}"));
    }
    Ok(())
}

#[test]
fn load_combination_equivalence() -> Result<(), Box<dyn std::error::Error>> {
    let frame = portal_frame()?;
    let (dead, live, wind) = make_cases(&frame)?;

    let mut combo = LoadCombination::new("ULS");
    combo.add_case(&dead, 1.35)?;
    combo.add_case(&live, 1.5)?;
    combo.add_case(&wind, 1.0)?;

    let r_indiv = frame.solve_combination(&combo)?;

    let prepared = frame.prepare()?;
    let r_prep = prepared.solve_combination(&combo)?;

    assert_results_match(&r_indiv, &r_prep, "combination");
    assert_eq!(r_prep.load_source().name(), Some("ULS"));
    Ok(())
}

#[test]
fn load_combination_vs_superposition() -> Result<(), Box<dyn std::error::Error>> {
    let frame = portal_frame()?;
    let (dead, live, wind) = make_cases(&frame)?;

    let factors = [1.35, 1.5, 1.0];
    let cases = [&dead, &live, &wind];

    let mut combo = LoadCombination::new("ULS");
    for (case, &f) in cases.iter().zip(&factors) {
        combo.add_case(case, f)?;
    }

    let r_combo = frame.solve_combination(&combo)?;

    let prepared = frame.prepare()?;
    let r_individual: Vec<_> = cases
        .iter()
        .map(|c| prepared.solve_case(c).unwrap())
        .collect();

    let n_nodes = r_combo.n_nodes();
    for i in 0..n_nodes {
        let node = NodeHandle::from_index(i);
        for dof in [Dof::Ux, Dof::Uy, Dof::Rz] {
            let u_combo = r_combo.displacement(node, dof)?;
            let u_super: f64 = r_individual
                .iter()
                .zip(&factors)
                .map(|(r, &f)| r.displacement(node, dof).unwrap() * f)
                .sum();
            assert!(
                (u_combo - u_super).abs() < TOL,
                "node {i} {:?}: combo={u_combo} super={u_super} diff={}",
                dof,
                (u_combo - u_super).abs()
            );
        }
    }
    Ok(())
}

#[test]
fn spring_support_multi_case() -> Result<(), Box<dyn std::error::Error>> {
    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0)?;
    let b = frame.add_node(5.0, 0.0)?;
    let c = frame.add_node(10.0, 0.0)?;
    frame.add_member(a, b, steel(), sec())?;
    frame.add_member(b, c, steel(), sec())?;
    frame.fix(a)?;
    frame.spring(c, Dof::Uy, 1e6)?;
    frame.spring(c, Dof::Ux, 1e8)?;

    let mut case1 = LoadCase::new("case1");
    case1.nodal_load(b, 0.0, -10000.0)?;
    let mut case2 = LoadCase::new("case2");
    case2.nodal_load(c, 0.0, -5000.0)?;
    let mut case3 = LoadCase::new("case3");
    case3.member_udl(MemberHandle::from_index(0), 0.0, -2000.0)?;
    case3.member_udl(MemberHandle::from_index(1), 0.0, -2000.0)?;

    let r1_indiv = frame.solve_case(&case1)?;
    let r2_indiv = frame.solve_case(&case2)?;
    let r3_indiv = frame.solve_case(&case3)?;

    let prepared = frame.prepare()?;
    let results = prepared.solve_cases(&[case1, case2, case3])?;

    assert_results_match(&r1_indiv, &results[0], "spring case1");
    assert_results_match(&r2_indiv, &results[1], "spring case2");
    assert_results_match(&r3_indiv, &results[2], "spring case3");

    for r in &results {
        assert!(r.equilibrium().is_balanced());
    }
    Ok(())
}

#[test]
fn inclined_roller_multi_case() -> Result<(), Box<dyn std::error::Error>> {
    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0)?;
    let b = frame.add_node(4.0, 0.0)?;
    let c = frame.add_node(8.0, 0.0)?;
    frame.add_member(a, b, steel(), sec())?;
    frame.add_member(b, c, steel(), sec())?;
    frame.fix(a)?;
    frame.inclined_roller(c, 1.0, 1.0, 0.0)?;

    let mut case1 = LoadCase::new("vertical");
    case1.nodal_load(b, 0.0, -8000.0)?;
    let mut case2 = LoadCase::new("horizontal");
    case2.nodal_load(b, 3000.0, 0.0)?;
    let mut case3 = LoadCase::new("moment");
    case3.nodal_moment(b, 2000.0)?;

    let r1_indiv = frame.solve_case(&case1)?;
    let r2_indiv = frame.solve_case(&case2)?;
    let r3_indiv = frame.solve_case(&case3)?;

    let prepared = frame.prepare()?;
    let results = prepared.solve_cases(&[case1, case2, case3])?;

    assert_results_match(&r1_indiv, &results[0], "roller case1");
    assert_results_match(&r2_indiv, &results[1], "roller case2");
    assert_results_match(&r3_indiv, &results[2], "roller case3");

    for r in &results {
        assert!(r.equilibrium().is_balanced());
    }
    Ok(())
}

#[test]
fn end_release_multi_case() -> Result<(), Box<dyn std::error::Error>> {
    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0)?;
    let b = frame.add_node(4.0, 0.0)?;
    let c = frame.add_node(8.0, 0.0)?;
    frame.add_member_with_release(
        a,
        b,
        steel(),
        sec(),
        structural_analysis::EndRelease::end_pin(),
    )?;
    frame.add_member(b, c, steel(), sec())?;
    frame.fix(a)?;
    frame.fix(c)?;

    let mut case1 = LoadCase::new("udl");
    case1.member_udl(MemberHandle::from_index(0), 0.0, -1500.0)?;
    case1.member_udl(MemberHandle::from_index(1), 0.0, -1500.0)?;
    let mut case2 = LoadCase::new("point");
    case2.nodal_load(b, 0.0, -12000.0)?;
    let mut case3 = LoadCase::new("moment");
    case3.nodal_moment(b, 3000.0)?;

    let r1_indiv = frame.solve_case(&case1)?;
    let r2_indiv = frame.solve_case(&case2)?;
    let r3_indiv = frame.solve_case(&case3)?;

    let prepared = frame.prepare()?;
    let results = prepared.solve_cases(&[case1, case2, case3])?;

    assert_results_match(&r1_indiv, &results[0], "release case1");
    assert_results_match(&r2_indiv, &results[1], "release case2");
    assert_results_match(&r3_indiv, &results[2], "release case3");

    let forces = results[0].member_end_forces(MemberHandle::from_index(0))?;
    assert!(
        forces[5].abs() < 1.0,
        "released end moment at b (member 0) should be ~0, got {}",
        forces[5]
    );
    Ok(())
}

#[test]
fn prepared_solver_name() -> Result<(), Box<dyn std::error::Error>> {
    let frame = portal_frame()?;
    let prepared = frame.prepare()?;
    assert!(prepared.solver_name().is_some());

    let (dead, _, _) = make_cases(&frame)?;
    let result = prepared.solve_case(&dead)?;
    assert!(result.solver_name().is_some());
    assert_eq!(result.solver_name(), prepared.solver_name());
    Ok(())
}

#[test]
fn prepared_with_explicit_solver() -> Result<(), Box<dyn std::error::Error>> {
    let frame = portal_frame()?;
    let prepared = frame.prepare_with(section_properties::SolverSelection::dense())?;

    let (dead, live, _) = make_cases(&frame)?;
    let results = prepared.solve_cases(&[dead, live])?;

    assert_eq!(results.len(), 2);
    assert_eq!(results[0].solver_name(), Some("dense"));
    Ok(())
}

#[test]
fn equilibrium_all_cases() -> Result<(), Box<dyn std::error::Error>> {
    let frame = portal_frame()?;
    let (dead, live, wind) = make_cases(&frame)?;

    let prepared = frame.prepare()?;
    let results = prepared.solve_cases(&[dead, live, wind])?;

    for (i, r) in results.iter().enumerate() {
        assert!(
            r.equilibrium().is_balanced(),
            "case {i} equilibrium not balanced: {:?}",
            r.equilibrium()
        );
    }
    Ok(())
}
