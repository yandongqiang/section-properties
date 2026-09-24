//! Phase 103 tests: envelope, force diagrams, equilibrium, geometry accessors.

use section_properties::Material;
use structural_analysis::{
    BeamModel, BeamSection, BeamSolver, Dof, Envelope, FrameModel, LoadCase, LoadCombination,
    beam_fem::{BeamElement, BeamNode, DistributedLoad},
    frame::{MemberHandle, NodeHandle},
    truss::{TrussElement, TrussModel, TrussNode, TrussSolver},
};

fn steel() -> Material {
    Material::new(200e9, 0.3, 7850.0, "Steel")
}

fn sec() -> BeamSection {
    BeamSection::new(0.01, 1e-4)
}

// ---------------------------------------------------------------------------
// Frame sample_forces / diagram
// ---------------------------------------------------------------------------

fn cantilever_with_tip_load() -> (FrameModel, NodeHandle, MemberHandle) {
    let mut f = FrameModel::new();
    let a = f.add_node(0.0, 0.0).unwrap();
    let b = f.add_node(1.0, 0.0).unwrap();
    let m = f.add_member(a, b, steel(), sec()).unwrap();
    f.fix(a).unwrap();
    f.nodal_load(b, 0.0, -1000.0).unwrap();
    (f, a, m)
}

#[test]
fn frame_sample_forces_basic() {
    let (f, _, _) = cantilever_with_tip_load();
    let result = f.solve().unwrap();
    let samples = result.sample_forces(5).unwrap();
    assert_eq!(samples.len(), 5);
    assert_eq!(samples[0].element_index, 0);
    assert!((samples[0].xi - 0.0).abs() < 1e-12);
    assert!((samples[4].xi - 1.0).abs() < 1e-12);
}

#[test]
fn frame_diagram_basic() {
    let (f, _, _) = cantilever_with_tip_load();
    let result = f.solve().unwrap();
    let diagram = result.diagram(3).unwrap();
    assert_eq!(diagram.samples.len(), 3);
    let moments = diagram.moment();
    assert_eq!(moments.len(), 3);
}

#[test]
fn frame_sample_forces_min_2() {
    let (f, _, _) = cantilever_with_tip_load();
    let result = f.solve().unwrap();
    assert!(result.sample_forces(1).is_err());
}

// ---------------------------------------------------------------------------
// Geometry accessors
// ---------------------------------------------------------------------------

#[test]
fn frame_geometry_accessors() {
    let (f, a, m) = cantilever_with_tip_load();
    let result = f.solve().unwrap();
    let p = result.node_position(a).unwrap();
    assert!((p.x - 0.0).abs() < 1e-12);
    assert!((p.y - 0.0).abs() < 1e-12);
    let (ni, nj) = result.member_nodes(m).unwrap();
    assert_eq!(ni.index(), 0);
    assert_eq!(nj.index(), 1);
    let len = result.member_length(m).unwrap();
    assert!((len - 1.0).abs() < 1e-12);
}

#[test]
fn frame_geometry_out_of_range() {
    let (f, _, _) = cantilever_with_tip_load();
    let result = f.solve().unwrap();
    assert!(result.node_position(NodeHandle::from_index(99)).is_none());
    assert!(result.member_nodes(MemberHandle::from_index(99)).is_none());
    assert!(result.member_length(MemberHandle::from_index(99)).is_none());
}

// ---------------------------------------------------------------------------
// Beam equilibrium
// ---------------------------------------------------------------------------

#[test]
fn beam_equilibrium_cantilever() {
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, 2.0, 0.0));
    model.add_element(BeamElement::new(0, 1, steel(), BeamSection::new(0.01, 1e-4)).unwrap());
    model.fixed_dofs.push((0, 0, 0.0));
    model.fixed_dofs.push((0, 1, 0.0));
    model.fixed_dofs.push((0, 2, 0.0));
    model.nodal_forces.push((1, 1, -5000.0));
    let mut solver = BeamSolver::from_model(&model).unwrap();
    solver.solve_configured().unwrap();
    let result = solver.results();
    let eq = result.equilibrium();
    assert!(
        eq.is_balanced(),
        "beam equilibrium should be balanced: {eq:?}"
    );
}

#[test]
fn beam_equilibrium_with_udl() {
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, 4.0, 0.0));
    model.add_element(BeamElement::new(0, 1, steel(), BeamSection::new(0.01, 1e-4)).unwrap());
    model.fixed_dofs.push((0, 0, 0.0));
    model.fixed_dofs.push((0, 1, 0.0));
    model.fixed_dofs.push((0, 2, 0.0));
    model.fixed_dofs.push((1, 0, 0.0));
    model.fixed_dofs.push((1, 1, 0.0));
    model
        .distributed_loads
        .push(DistributedLoad::trapezoidal(0, 0.0, -2000.0, 0.0, -2000.0));
    let mut solver = BeamSolver::from_model(&model).unwrap();
    solver.solve_configured().unwrap();
    let result = solver.results();
    let eq = result.equilibrium();
    assert!(
        eq.is_balanced(),
        "beam equilibrium with UDL should be balanced: {eq:?}"
    );
}

// ---------------------------------------------------------------------------
// Truss equilibrium
// ---------------------------------------------------------------------------

#[test]
fn truss_equilibrium_basic() {
    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, 3.0, 0.0));
    model.add_node(TrussNode::new(2, 3.0, 4.0));
    model.add_element(TrussElement::new(0, 2, &steel(), 0.001).unwrap());
    model.add_element(TrussElement::new(1, 2, &steel(), 0.001).unwrap());
    model.fixed_dofs.push((0, 0, 0.0));
    model.fixed_dofs.push((0, 1, 0.0));
    model.fixed_dofs.push((1, 0, 0.0));
    model.fixed_dofs.push((1, 1, 0.0));
    model.nodal_forces.push((2, 1, -10000.0));
    let mut solver = TrussSolver::from_model(&model).unwrap();
    solver.solve_configured().unwrap();
    let result = solver.results().unwrap();
    let eq = result.equilibrium();
    assert!(
        eq.is_balanced(),
        "truss equilibrium should be balanced: {eq:?}"
    );
}

#[test]
fn truss_geometry_in_result() {
    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 1.0, 2.0));
    model.add_node(TrussNode::new(1, 3.0, 4.0));
    model.add_node(TrussNode::new(2, 5.0, 2.0));
    model.add_element(TrussElement::new(0, 1, &steel(), 0.001).unwrap());
    model.add_element(TrussElement::new(1, 2, &steel(), 0.001).unwrap());
    model.fixed_dofs.push((0, 0, 0.0));
    model.fixed_dofs.push((0, 1, 0.0));
    model.fixed_dofs.push((2, 0, 0.0));
    model.fixed_dofs.push((2, 1, 0.0));
    model.nodal_forces.push((1, 1, -100.0));
    let mut solver = TrussSolver::from_model(&model).unwrap();
    solver.solve_configured().unwrap();
    let result = solver.results().unwrap();
    let p0 = result.node_position(0).unwrap();
    assert!((p0.0 - 1.0).abs() < 1e-12);
    assert!((p0.1 - 2.0).abs() < 1e-12);
    let (ni, nj) = result.element_endpoints(0).unwrap();
    assert_eq!(ni, 0);
    assert_eq!(nj, 1);
}

// ---------------------------------------------------------------------------
// Envelope tests
// ---------------------------------------------------------------------------

fn portal_frame() -> FrameModel {
    let mut f = FrameModel::new();
    let a = f.add_node(0.0, 0.0).unwrap();
    let b = f.add_node(0.0, 3.0).unwrap();
    let c = f.add_node(4.0, 3.0).unwrap();
    let d = f.add_node(4.0, 0.0).unwrap();
    f.add_member(a, b, steel(), sec()).unwrap();
    f.add_member(b, c, steel(), sec()).unwrap();
    f.add_member(d, c, steel(), sec()).unwrap();
    f.fix(a).unwrap();
    f.fix(d).unwrap();
    f
}

#[test]
fn envelope_two_load_cases() {
    let f = portal_frame();
    let n1 = NodeHandle::from_index(1);
    let n2 = NodeHandle::from_index(2);
    let mut case_a = LoadCase::new("dead");
    case_a.nodal_load(n1, 0.0, -1000.0).unwrap();
    let mut case_b = LoadCase::new("live");
    case_b.nodal_load(n2, 0.0, -500.0).unwrap();
    let ra = f.solve_case(&case_a).unwrap();
    let rb = f.solve_case(&case_b).unwrap();
    let env = Envelope::from_frame_results(&[&ra, &rb], 5).unwrap();
    assert_eq!(env.n_results, 2);
    assert_eq!(env.n_members, 3);
    assert_eq!(env.samples.len(), 15);
}

#[test]
fn envelope_min_not_abs_max() {
    let f = portal_frame();
    let n1 = NodeHandle::from_index(1);
    let mut case_a = LoadCase::new("down");
    case_a.nodal_load(n1, 0.0, -10000.0).unwrap();
    let mut case_b = LoadCase::new("up");
    case_b.nodal_load(n1, 0.0, 4000.0).unwrap();
    let ra = f.solve_case(&case_a).unwrap();
    let rb = f.solve_case(&case_b).unwrap();
    let env = Envelope::from_frame_results(&[&ra, &rb], 5).unwrap();
    let m0 = env.member_samples(0);
    let min_m = m0
        .iter()
        .map(|s| s.min.moment)
        .fold(f64::INFINITY, f64::min);
    let max_m = m0
        .iter()
        .map(|s| s.max.moment)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(min_m < 0.0, "min moment should be negative: {min_m}");
    assert!(max_m > 0.0, "max moment should be positive: {max_m}");
    assert!(min_m.abs() != max_m.abs(), "min should differ from abs max");
}

#[test]
fn envelope_with_combination() {
    let f = portal_frame();
    let n1 = NodeHandle::from_index(1);
    let n2 = NodeHandle::from_index(2);
    let mut case_a = LoadCase::new("dead");
    case_a.nodal_load(n1, 0.0, -1000.0).unwrap();
    let mut case_b = LoadCase::new("live");
    case_b.nodal_load(n2, 0.0, -800.0).unwrap();
    let mut combo = LoadCombination::new("combo");
    combo.add_case(&case_a, 1.2).unwrap();
    combo.add_case(&case_b, 1.6).unwrap();
    let ra = f.solve_case(&case_a).unwrap();
    let rc = f.solve_combination(&combo).unwrap();
    let env = Envelope::from_frame_results(&[&ra, &rc], 3).unwrap();
    assert_eq!(env.n_results, 2);
    assert_eq!(env.samples.len(), 9);
}

#[test]
fn envelope_trapezoidal_load() {
    let mut f = FrameModel::new();
    let a = f.add_node(0.0, 0.0).unwrap();
    let b = f.add_node(0.0, 5.0).unwrap();
    let c = f.add_node(5.0, 5.0).unwrap();
    let d = f.add_node(5.0, 0.0).unwrap();
    f.add_member(a, b, steel(), sec()).unwrap();
    f.add_member(b, c, steel(), sec()).unwrap();
    f.add_member(d, c, steel(), sec()).unwrap();
    f.fix(a).unwrap();
    f.fix(d).unwrap();
    let m1 = MemberHandle::from_index(1);
    let mut case_a = LoadCase::new("trap_a");
    case_a
        .member_trapezoidal(m1, 0.0, -3000.0, 0.0, -1000.0)
        .unwrap();
    let mut case_b = LoadCase::new("trap_b");
    case_b
        .member_trapezoidal(m1, 0.0, -1000.0, 0.0, -3000.0)
        .unwrap();
    let ra = f.solve_case(&case_a).unwrap();
    let rb = f.solve_case(&case_b).unwrap();
    let env = Envelope::from_frame_results(&[&ra, &rb], 7).unwrap();
    assert_eq!(env.n_members, 3);
    assert_eq!(env.samples.len(), 21);
    for s in &env.samples {
        assert!(s.min.axial.is_finite());
        assert!(s.max.axial.is_finite());
        assert!(s.min.moment.is_finite());
        assert!(s.max.moment.is_finite());
    }
}

#[test]
fn envelope_multiple_members_no_confusion() {
    let f = portal_frame();
    let n1 = NodeHandle::from_index(1);
    let n2 = NodeHandle::from_index(2);
    let mut case_a = LoadCase::new("a");
    case_a.nodal_load(n1, 0.0, -1000.0).unwrap();
    let mut case_b = LoadCase::new("b");
    case_b.nodal_load(n2, 0.0, -2000.0).unwrap();
    let ra = f.solve_case(&case_a).unwrap();
    let rb = f.solve_case(&case_b).unwrap();
    let env = Envelope::from_frame_results(&[&ra, &rb], 3).unwrap();
    for (i, s) in env.samples.iter().enumerate() {
        assert_eq!(s.member_index, i / 3, "member_index at sample {i}");
    }
}

#[test]
fn envelope_empty_input_errors() {
    let env = Envelope::from_frame_results(&[], 5);
    assert!(env.is_err());
}

#[test]
fn envelope_n_per_member_too_small() {
    let f = portal_frame();
    let n1 = NodeHandle::from_index(1);
    let mut case_a = LoadCase::new("a");
    case_a.nodal_load(n1, 0.0, -1000.0).unwrap();
    let ra = f.solve_case(&case_a).unwrap();
    let env = Envelope::from_frame_results(&[&ra], 1);
    assert!(env.is_err());
}

#[test]
fn envelope_max_abs_moment() {
    let f = portal_frame();
    let n1 = NodeHandle::from_index(1);
    let mut case_a = LoadCase::new("down");
    case_a.nodal_load(n1, 0.0, -10000.0).unwrap();
    let mut case_b = LoadCase::new("up");
    case_b.nodal_load(n1, 0.0, 5000.0).unwrap();
    let ra = f.solve_case(&case_a).unwrap();
    let rb = f.solve_case(&case_b).unwrap();
    let env = Envelope::from_frame_results(&[&ra, &rb], 5).unwrap();
    let mam = env.max_abs_moment();
    assert!(mam > 0.0);
}

// ---------------------------------------------------------------------------
// Inclined member diagram
// ---------------------------------------------------------------------------

#[test]
fn frame_diagram_inclined_member() {
    let mut f = FrameModel::new();
    let a = f.add_node(0.0, 0.0).unwrap();
    let b = f.add_node(3.0, 4.0).unwrap();
    f.add_member(a, b, steel(), sec()).unwrap();
    f.fix(a).unwrap();
    f.nodal_load(b, -1000.0, 0.0).unwrap();
    let result = f.solve().unwrap();
    let diagram = result.diagram(5).unwrap();
    assert_eq!(diagram.samples.len(), 5);
    for s in &diagram.samples {
        assert!(s.section_forces.axial.is_finite());
        assert!(s.section_forces.shear.is_finite());
        assert!(s.section_forces.moment.is_finite());
    }
    let eq = result.equilibrium();
    assert!(eq.is_balanced(), "inclined member equilibrium: {eq:?}");
}

// ---------------------------------------------------------------------------
// End release diagram
// ---------------------------------------------------------------------------

#[test]
fn frame_diagram_with_end_release() {
    let mut f = FrameModel::new();
    let a = f.add_node(0.0, 0.0).unwrap();
    let b = f.add_node(2.0, 0.0).unwrap();
    let c = f.add_node(4.0, 0.0).unwrap();
    f.add_member(a, b, steel(), sec()).unwrap();
    f.add_member_with_release(
        b,
        c,
        steel(),
        sec(),
        structural_analysis::EndRelease::start_pin(),
    )
    .unwrap();
    f.fix(a).unwrap();
    f.fix(c).unwrap();
    f.nodal_load(b, 0.0, -5000.0).unwrap();
    let result = f.solve().unwrap();
    let diagram = result.diagram(5).unwrap();
    assert_eq!(diagram.samples.len(), 10);
    let sf_at_b_from_m1 = result
        .section_forces(MemberHandle::from_index(1), 0.0)
        .unwrap();
    assert!(
        sf_at_b_from_m1.moment.abs() < 1e-6,
        "released end moment should be ~0, got {}",
        sf_at_b_from_m1.moment
    );
}
