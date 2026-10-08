//! Phase 112 — result and post-processing consistency tests.

use section_properties::Material;
use structural_analysis::{
    BeamSection, Dof, Envelope, FemError, FrameModel, LoadCase, LoadCombination, LoadSource,
    NodeHandle,
};

fn steel() -> Material {
    Material::new(200e9, 0.3, 7850.0, "Steel")
}

fn cantilever() -> (FrameModel, NodeHandle, NodeHandle) {
    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(5.0, 0.0).unwrap();
    frame
        .add_member(a, b, steel(), BeamSection::new(5e-3, 2e-5))
        .unwrap();
    frame.fix(a).unwrap();
    (frame, a, b)
}

fn fixed_beam() -> (FrameModel, NodeHandle, NodeHandle) {
    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0).unwrap();
    let b = frame.add_node(4.0, 0.0).unwrap();
    frame
        .add_member(a, b, steel(), BeamSection::new(5e-3, 2e-5))
        .unwrap();
    frame.fix(a).unwrap();
    frame.fix(b).unwrap();
    (frame, a, b)
}

#[test]
fn load_source_identity_is_typed_and_self_describing() {
    let (mut frame, _, b) = cantilever();
    frame.nodal_load(b, 0.0, -1000.0).unwrap();
    let direct = frame.solve().unwrap();
    assert_eq!(direct.load_source(), &LoadSource::ModelLoads);

    let mut case = LoadCase::new("dead");
    case.nodal_load(b, 0.0, -1000.0).unwrap();
    let case_result = frame.solve_case(&case).unwrap();
    assert_eq!(case_result.load_source().name(), Some("dead"));
    assert!(!case_result.load_source().has_prescribed_displacements());

    let mut combo = LoadCombination::new("1.4D");
    combo.add_case(&case, 1.4).unwrap();
    let combo_result = frame.solve_combination(&combo).unwrap();
    match combo_result.load_source() {
        LoadSource::LoadCombination { name, terms } => {
            assert_eq!(name, "1.4D");
            assert_eq!(terms.len(), 1);
            assert_eq!(terms[0].case_name, "dead");
            assert_eq!(terms[0].factor, 1.4);
        }
        other => panic!("expected combination source, got {other:?}"),
    }
}

#[test]
fn duplicate_case_names_remain_unambiguous_in_envelope() {
    let (frame, _, b) = cantilever();

    let mut first = LoadCase::new("same");
    first.nodal_load(b, 0.0, -1000.0).unwrap();
    let mut second = LoadCase::new("same");
    second.nodal_load(b, 0.0, -4000.0).unwrap();

    let r1 = frame.solve_case(&first).unwrap();
    let r2 = frame.solve_case(&second).unwrap();
    let envelope = Envelope::from_frame_results(&[&r1, &r2], 5).unwrap();
    let tip = envelope.node_displacement(b.index()).unwrap();

    assert_eq!(tip.uy.min_source, Some(1));
    assert_eq!(tip.uy.max_source, Some(0));
    assert_eq!(envelope.source(0).unwrap().name(), Some("same"));
    assert_eq!(envelope.source(1).unwrap().name(), Some("same"));
}

#[test]
fn displacement_and_reaction_envelopes_carry_source_indices() {
    let (frame, a, b) = cantilever();

    let mut light = LoadCase::new("light");
    light.nodal_load(b, 0.0, -1000.0).unwrap();
    let mut heavy = LoadCase::new("heavy");
    heavy.nodal_load(b, 0.0, -4000.0).unwrap();
    let light_result = frame.solve_case(&light).unwrap();
    let heavy_result = frame.solve_case(&heavy).unwrap();
    let envelope = Envelope::from_frame_results(&[&light_result, &heavy_result], 5).unwrap();

    let tip = envelope.node_displacement(b.index()).unwrap();
    assert_eq!(tip.uy.min_source, Some(1));
    assert_eq!(tip.uy.max_source, Some(0));

    let support = envelope.support_reaction(a.index()).unwrap();
    assert_eq!(support.ry.min_source, Some(0));
    assert_eq!(support.ry.max_source, Some(1));
    let expected = heavy_result.reactions()[a.index() * 3 + Dof::Uy.index()];
    assert!((support.ry.max - expected).abs() < 1e-9);
}

#[test]
fn reaction_envelope_excludes_free_dof_residuals() {
    let (frame, a, b) = cantilever();
    let mut case = LoadCase::new("tip");
    case.nodal_load(b, 0.0, -4000.0).unwrap();
    let result = frame.solve_case(&case).unwrap();
    let envelope = Envelope::from_frame_results(&[&result], 5).unwrap();

    let tip = envelope.support_reaction(b.index()).unwrap();
    assert!(!tip.rx.is_populated());
    assert!(!tip.ry.is_populated());
    assert!(!tip.mz.is_populated());

    let support = envelope.support_reaction(a.index()).unwrap();
    assert!(support.rx.is_populated());
    assert!(support.ry.is_populated());
    assert!(support.mz.is_populated());
}

#[test]
fn spring_and_inclined_roller_reactions_are_included() {
    let (mut spring_frame, _spring_a, spring_b) = cantilever();
    spring_frame.spring(spring_b, Dof::Uy, 1.0e6).unwrap();
    let mut spring_case = LoadCase::new("spring");
    spring_case.nodal_load(spring_b, 0.0, -4000.0).unwrap();
    let spring_result = spring_frame.solve_case(&spring_case).unwrap();
    let spring_envelope = Envelope::from_frame_results(&[&spring_result], 5).unwrap();
    let spring_node = spring_envelope.support_reaction(spring_b.index()).unwrap();
    assert!(spring_node.ry.is_populated());
    assert!(!spring_node.rx.is_populated());

    let mut roller_frame = FrameModel::new();
    let roller_a = roller_frame.add_node(0.0, 0.0).unwrap();
    let roller_b = roller_frame.add_node(4.0, 0.0).unwrap();
    roller_frame
        .add_member(roller_a, roller_b, steel(), BeamSection::new(5e-3, 2e-5))
        .unwrap();
    roller_frame.fix(roller_a).unwrap();
    roller_frame
        .inclined_roller(roller_b, 1.0, 1.0, 0.0)
        .unwrap();
    let mut roller_case = LoadCase::new("roller");
    roller_case.nodal_load(roller_b, 0.0, -4000.0).unwrap();
    let roller_result = roller_frame.solve_case(&roller_case).unwrap();
    let roller_envelope = Envelope::from_frame_results(&[&roller_result], 5).unwrap();
    let roller_node = roller_envelope.support_reaction(roller_b.index()).unwrap();
    assert!(roller_node.rx.is_populated());
    assert!(roller_node.ry.is_populated());
    assert!(!roller_node.mz.is_populated());
}

#[test]
fn prescribed_displacement_envelope_is_flagged_and_isolated() {
    let (frame, a, b) = fixed_beam();

    let mut settlement = LoadCase::new("settlement");
    settlement
        .prescribed_displacement(b, Dof::Uy, -0.002)
        .unwrap();
    let mut gravity = LoadCase::new("gravity");
    gravity.nodal_load(b, 0.0, -1000.0).unwrap();

    let settlement_result = frame.solve_case(&settlement).unwrap();
    let gravity_result = frame.solve_case(&gravity).unwrap();
    let envelope = Envelope::from_frame_results(&[&settlement_result, &gravity_result], 5).unwrap();
    let tip = envelope.node_displacement(b.index()).unwrap();

    assert_eq!(tip.uy.min_source, Some(0));
    assert!((tip.uy.min - (-0.002)).abs() < 1e-12);
    assert!(
        envelope
            .source(tip.uy.min_source.unwrap())
            .unwrap()
            .has_prescribed_displacements()
    );

    let support = envelope.support_reaction(a.index()).unwrap();
    assert!(support.ry.is_populated());
    assert_ne!(
        settlement_result.displacement(b, Dof::Uy).unwrap(),
        gravity_result.displacement(b, Dof::Uy).unwrap()
    );
}

#[test]
fn prescribed_displacement_reaction_matches_result_vector() {
    let (frame, _, b) = fixed_beam();
    let mut case = LoadCase::new("settlement");
    case.prescribed_displacement(b, Dof::Ux, 0.001).unwrap();
    let result = frame.solve_case(&case).unwrap();
    let envelope = Envelope::from_frame_results(&[&result], 5).unwrap();
    let reaction = envelope.support_reaction(b.index()).unwrap();
    let expected = result.reactions()[b.index() * 3 + Dof::Ux.index()];
    assert!((reaction.rx.min - expected).abs() / expected.abs() < 1e-9);
    assert_eq!(reaction.rx.min_source, Some(0));
}

#[test]
fn member_force_envelope_reports_component_sources() {
    let (frame, _, b) = cantilever();
    let mut light = LoadCase::new("light");
    light.nodal_load(b, 0.0, -1000.0).unwrap();
    let mut heavy = LoadCase::new("heavy");
    heavy.nodal_load(b, 0.0, -4000.0).unwrap();
    let light_result = frame.solve_case(&light).unwrap();
    let heavy_result = frame.solve_case(&heavy).unwrap();
    let envelope = Envelope::from_frame_results(&[&light_result, &heavy_result], 5).unwrap();
    let tip_sample = envelope.member_samples(0)[4];

    assert_eq!(tip_sample.max_sources.moment, Some(0));
    assert_eq!(tip_sample.min_sources.moment, Some(1));
    assert_eq!(envelope.source(1).unwrap().name(), Some("heavy"));
    assert!(tip_sample.max.moment > tip_sample.min.moment);
}

#[test]
fn envelope_ties_keep_the_first_source() {
    let (frame, _, b) = cantilever();
    let mut first = LoadCase::new("first");
    first.nodal_load(b, 0.0, -2000.0).unwrap();
    let mut second = LoadCase::new("second");
    second.nodal_load(b, 0.0, -2000.0).unwrap();
    let r1 = frame.solve_case(&first).unwrap();
    let r2 = frame.solve_case(&second).unwrap();
    let envelope = Envelope::from_frame_results(&[&r1, &r2], 5).unwrap();
    let tip = envelope.node_displacement(b.index()).unwrap();

    assert_eq!(tip.uy.min_source, Some(0));
    assert_eq!(tip.uy.max_source, Some(0));
}

#[test]
fn envelope_rejects_degenerate_input_and_counts_node_mismatch() {
    assert!(matches!(
        Envelope::from_frame_results(&[], 5).unwrap_err(),
        FemError::InvalidInput(_)
    ));

    let (frame, _, b) = cantilever();
    let mut case = LoadCase::new("case");
    case.nodal_load(b, 0.0, -1000.0).unwrap();
    let result = frame.solve_case(&case).unwrap();
    assert!(matches!(
        Envelope::from_frame_results(&[&result], 1).unwrap_err(),
        FemError::InvalidInput(_)
    ));

    let mut triangle = FrameModel::new();
    let a1 = triangle.add_node(0.0, 0.0).unwrap();
    let b1 = triangle.add_node(5.0, 0.0).unwrap();
    let c1 = triangle.add_node(0.0, 5.0).unwrap();
    for (start, end) in [(a1, b1), (b1, c1), (c1, a1)] {
        triangle
            .add_member(start, end, steel(), BeamSection::new(5e-3, 2e-5))
            .unwrap();
    }
    triangle.fix(a1).unwrap();
    triangle.pin(b1).unwrap();

    let mut chain = FrameModel::new();
    let a2 = chain.add_node(0.0, 0.0).unwrap();
    let b2 = chain.add_node(5.0, 0.0).unwrap();
    let c2 = chain.add_node(10.0, 0.0).unwrap();
    let d2 = chain.add_node(15.0, 0.0).unwrap();
    for (start, end) in [(a2, b2), (b2, c2), (c2, d2)] {
        chain
            .add_member(start, end, steel(), BeamSection::new(5e-3, 2e-5))
            .unwrap();
    }
    chain.fix(a2).unwrap();
    chain.pin(d2).unwrap();

    let mut triangle_case = LoadCase::new("triangle");
    triangle_case.nodal_load(c1, 0.0, -1000.0).unwrap();
    let mut chain_case = LoadCase::new("chain");
    chain_case.nodal_load(c2, 0.0, -1000.0).unwrap();
    let triangle_result = triangle.solve_case(&triangle_case).unwrap();
    let chain_result = chain.solve_case(&chain_case).unwrap();
    assert_eq!(triangle_result.n_members(), chain_result.n_members());
    assert_ne!(triangle_result.n_nodes(), chain_result.n_nodes());

    let error = Envelope::from_frame_results(&[&triangle_result, &chain_result], 5).unwrap_err();
    match error {
        FemError::InvalidInput(message) => assert!(message.contains("same node count")),
        other => panic!("expected node-count error, got {other:?}"),
    }

    let (frame, _, b) = cantilever();
    let mut case = LoadCase::new("case");
    case.nodal_load(b, 0.0, -1000.0).unwrap();
    let result = frame.solve_case(&case).unwrap();
    let envelope = Envelope::from_frame_results(&[&result], 5).unwrap();
    assert_eq!(envelope.n_results, 1);
    assert_eq!(envelope.n_members, 1);
    assert_eq!(envelope.n_per_member, 5);
    assert_eq!(envelope.samples.len(), 5);
    assert_eq!(envelope.n_nodes, 2);
    assert_eq!(envelope.node_displacements.len(), 2);
    assert_eq!(envelope.support_reactions.len(), 2);
    assert!(envelope.member_samples(1).is_empty());
}

#[test]
fn combination_source_is_preserved_in_envelope() {
    let (frame, _, b) = cantilever();
    let mut dead = LoadCase::new("dead");
    dead.nodal_load(b, 0.0, -1000.0).unwrap();
    let mut live = LoadCase::new("live");
    live.nodal_load(b, 0.0, -4000.0).unwrap();
    let mut combo = LoadCombination::new("1.4D+1.6L");
    combo.add_case(&dead, 1.4).unwrap();
    combo.add_case(&live, 1.6).unwrap();

    let combo_result = frame.solve_combination(&combo).unwrap();
    let case_result = frame.solve_case(&dead).unwrap();
    let envelope = Envelope::from_frame_results(&[&case_result, &combo_result], 5).unwrap();
    let tip = envelope.node_displacement(b.index()).unwrap();

    assert_eq!(tip.uy.min_source, Some(1));
    match envelope.source(1).unwrap() {
        LoadSource::LoadCombination { name, terms } => {
            assert_eq!(name, "1.4D+1.6L");
            assert_eq!(terms.len(), 2);
        }
        other => panic!("expected combination source, got {other:?}"),
    }
}

#[test]
fn envelope_rejects_member_count_mismatch() {
    let (frame1, _, b1) = cantilever();
    let mut frame2 = FrameModel::new();
    let a = frame2.add_node(0.0, 0.0).unwrap();
    let b = frame2.add_node(5.0, 0.0).unwrap();
    let c = frame2.add_node(10.0, 0.0).unwrap();
    frame2
        .add_member(a, b, steel(), BeamSection::new(5e-3, 2e-5))
        .unwrap();
    frame2
        .add_member(b, c, steel(), BeamSection::new(5e-3, 2e-5))
        .unwrap();
    frame2.fix(a).unwrap();

    let mut case1 = LoadCase::new("one");
    case1.nodal_load(b1, 0.0, -1000.0).unwrap();
    let mut case2 = LoadCase::new("two");
    case2.nodal_load(c, 0.0, -1000.0).unwrap();
    let r1 = frame1.solve_case(&case1).unwrap();
    let r2 = frame2.solve_case(&case2).unwrap();

    let error = Envelope::from_frame_results(&[&r1, &r2], 5).unwrap_err();
    match error {
        FemError::InvalidInput(message) => assert!(message.contains("same member count")),
        other => panic!("expected member-count error, got {other:?}"),
    }
}
