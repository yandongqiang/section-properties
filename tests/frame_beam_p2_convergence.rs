//! Phase 39 — P2 convergence regression tests.
//!
//! Covers the five P2 items from the Phase 37/38 audit:
//! - P2-1: `EquilibriumReport::force_tolerance()` / `moment_tolerance()`
//! - P2-2: `FemError::SolverError` struct variant with `solver_error()` accessor
//! - P2-3: `BeamSection::from_section_properties_x` / `_y`
//! - P2-4: UDL implementation dedup (`element_equivalent_nodal_forces` → `consistent_nodal_load`)
//! - P2-5: Raw DOF accessor bounds checking (`displacement` / `reaction` reject `dof >= 3`)

use section_properties::Material;
use section_properties::beam_fem::{
    BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver, Dof, FemError,
};
use section_properties::frame::FrameModel;
use section_properties::section::Section;
use section_properties::section_library::ParametricSection;
use section_properties::section_library::primitive::RectangularSection;
use section_properties::section_properties::SectionProperties;
use section_properties::solver::SolverError;

// ===========================================================================
// P2-1: EquilibriumReport tolerance accessors
// ===========================================================================

#[test]
fn p2_1_force_and_moment_tolerance_exposed() {
    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0).unwrap();
    let n1 = f.add_node(2.0, 0.0).unwrap();
    f.add_member(
        n0,
        n1,
        Material::new(200e9, 0.3, 7850.0, "steel"),
        BeamSection::new(5e-3, 2e-5),
    )
    .unwrap();
    f.fix(n0).unwrap();
    f.nodal_load(n1, 0.0, -1.0e4).unwrap();

    let r = f.solve().unwrap();
    let eq = r.equilibrium();

    let ft = eq.force_tolerance();
    let mt = eq.moment_tolerance();

    assert!(ft > 0.0, "force_tolerance must be positive");
    assert!(mt > 0.0, "moment_tolerance must be positive");
    assert!(
        (ft - eq.tolerance).abs() < 1e-15 * (1.0 + ft.abs()),
        "force_tolerance must equal the tolerance field"
    );
    assert!(eq.is_balanced(), "equilibrium must be balanced");
}

#[test]
fn p2_1_moment_tolerance_differs_from_force_for_nonunit_length() {
    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0).unwrap();
    let n1 = f.add_node(5.0, 0.0).unwrap();
    f.add_member(
        n0,
        n1,
        Material::new(200e9, 0.3, 7850.0, "steel"),
        BeamSection::new(5e-3, 2e-5),
    )
    .unwrap();
    f.fix(n0).unwrap();
    f.nodal_load(n1, 0.0, -1.0e4).unwrap();

    let r = f.solve().unwrap();
    let eq = r.equilibrium();

    let ft = eq.force_tolerance();
    let mt = eq.moment_tolerance();

    assert!(
        (ft - mt).abs() > 1e-6 * ft,
        "for l_char=5 the moment tolerance must differ from force tolerance"
    );
}

// ===========================================================================
// P2-2: FemError::SolverError struct variant
// ===========================================================================

#[test]
fn p2_2_solver_error_preserves_source() {
    let se = SolverError::SingularMatrix("test singular".to_string());
    let fe: FemError = se.clone().into();

    assert!(fe.solver_error().is_some(), "solver_error must return Some");
    let recovered = fe.solver_error().unwrap();
    assert!(
        matches!(recovered, SolverError::SingularMatrix(s) if s == "test singular"),
        "source must be preserved"
    );
}

#[test]
fn p2_2_std_error_source_chain_returns_solver_error() {
    let se = SolverError::SingularMatrix("chain test".to_string());
    let fe: FemError = se.clone().into();

    let source = std::error::Error::source(&fe);
    assert!(
        source.is_some(),
        "std::error::Error::source() must return Some for SolverError variant"
    );
    let downcasted = source
        .unwrap()
        .downcast_ref::<SolverError>();
    assert!(
        downcasted.is_some(),
        "source must be downcastable to SolverError"
    );
    assert!(
        matches!(downcasted.unwrap(), SolverError::SingularMatrix(s) if s == "chain test"),
        "source chain must preserve the original SolverError payload"
    );
}

#[test]
fn p2_2_std_error_source_returns_none_for_non_solver_error() {
    let fe = FemError::InvalidInput("not solver".to_string());
    assert!(
        std::error::Error::source(&fe).is_none(),
        "source() must return None for non-SolverError variants"
    );
}

#[test]
fn p2_2_solver_error_message_matches_display() {
    let se = SolverError::DimensionMismatch {
        expected: 6,
        actual: 3,
    };
    let fe: FemError = se.into();

    if let FemError::SolverError { source, message } = &fe {
        assert_eq!(
            message,
            &source.to_string(),
            "message must equal source Display"
        );
    } else {
        panic!("must be SolverError variant");
    }
}

#[test]
fn p2_2_non_solver_error_returns_none() {
    let fe = FemError::InvalidInput("not a solver error".to_string());
    assert!(
        fe.solver_error().is_none(),
        "non-SolverError must return None"
    );
}

#[test]
fn p2_2_solver_error_display_uses_message() {
    let se = SolverError::SolveFailed("convergence issue".to_string());
    let fe: FemError = se.into();
    let display = format!("{}", fe);
    assert!(
        display.contains("convergence issue"),
        "Display must contain the original message: got {}",
        display
    );
}

// ===========================================================================
// P2-3: BeamSection from SectionProperties
// ===========================================================================

#[test]
fn p2_3_from_section_properties_x_uses_ix() {
    let b = 0.1_f64;
    let h = 0.2_f64;
    let rect = RectangularSection::new(b, h);
    let section: Section = rect.build();
    let props = SectionProperties::from_section(&section);

    let bs = BeamSection::from_section_properties_x(&props);

    assert!((bs.area - props.area).abs() < 1e-12, "area must match");
    assert!(
        (bs.second_moment - props.ix).abs() < 1e-12,
        "second_moment must be ix for x-aligned beam"
    );
    let expected_ix = b * h.powi(3) / 12.0;
    assert!(
        (bs.second_moment - expected_ix).abs() < 1e-12,
        "second_moment must equal b*h^3/12 = {}: got {}",
        expected_ix,
        bs.second_moment
    );
}

#[test]
fn p2_3_from_section_properties_y_uses_iy() {
    let b = 0.1_f64;
    let h = 0.2_f64;
    let rect = RectangularSection::new(b, h);
    let section: Section = rect.build();
    let props = SectionProperties::from_section(&section);

    let bs = BeamSection::from_section_properties_y(&props);

    assert!((bs.area - props.area).abs() < 1e-12, "area must match");
    assert!(
        (bs.second_moment - props.iy).abs() < 1e-12,
        "second_moment must be iy for y-aligned beam"
    );
    let expected_iy = h * b.powi(3) / 12.0;
    assert!(
        (bs.second_moment - expected_iy).abs() < 1e-12,
        "second_moment must equal h*b^3/12 = {}: got {}",
        expected_iy,
        bs.second_moment
    );
}

#[test]
fn p2_3_x_and_y_differ_for_non_square() {
    let b = 0.1_f64;
    let h = 0.3_f64;
    let rect = RectangularSection::new(b, h);
    let section: Section = rect.build();
    let props = SectionProperties::from_section(&section);

    let bs_x = BeamSection::from_section_properties_x(&props);
    let bs_y = BeamSection::from_section_properties_y(&props);

    assert!(
        (bs_x.second_moment - bs_y.second_moment).abs() > 1e-8,
        "ix and iy must differ for non-square section"
    );
}

// ===========================================================================
// P2-4: UDL dedup — numerical identity via analytical benchmark
// ===========================================================================

fn unit_cantilever_with_udl(n_elem: usize, q: f64) -> BeamSolver {
    let l = 1.0_f64;
    let e = 1.0_f64;
    let i = 1.0_f64;
    let a = 1.0_f64;
    let mut model = BeamModel::new();
    for k in 0..=n_elem {
        model.add_node(BeamNode::new(k, k as f64 * (l / n_elem as f64), 0.0));
    }
    for k in 0..n_elem {
        model.add_element(
            BeamElement::new(
                k,
                k + 1,
                Material::new(e, 0.3, 1.0, "unit"),
                BeamSection::new(a, i),
            )
            .unwrap(),
        );
    }
    model.fix_node(0);
    for e in 0..n_elem {
        model.add_distributed_load(e, 0.0, -q).unwrap();
    }
    let mut solver = BeamSolver::from_model(&model).unwrap();
    solver.solve_configured().unwrap();
    solver
}

#[test]
fn p2_4_udl_tip_displacement_matches_analytical() {
    let q = 100.0_f64;
    let l = 1.0_f64;
    let e = 1.0_f64;
    let i = 1.0_f64;
    let ei = e * i;
    let an_uy = -q * l.powi(4) / (8.0 * ei);
    let an_rz = -q * l.powi(3) / (6.0 * ei);

    for &n in &[1usize, 2, 4, 8] {
        let s = unit_cantilever_with_udl(n, q);
        let r = s.results();
        let uy = r.displacement(n).unwrap().uy;
        let rz = r.displacement(n).unwrap().rz;
        assert!(
            (uy - an_uy).abs() < 1e-9 * an_uy.abs(),
            "n={}: tip uy = {}, analytical = {}",
            n,
            uy,
            an_uy
        );
        assert!(
            (rz - an_rz).abs() < 1e-9 * an_rz.abs(),
            "n={}: tip rz = {}, analytical = {}",
            n,
            rz,
            an_rz
        );
    }
}

#[test]
fn p2_4_udl_reactions_match_analytical() {
    let q = 100.0_f64;
    let l = 1.0_f64;
    let an_ry = q * l;
    let an_mz = q * l * l / 2.0;

    let s = unit_cantilever_with_udl(1, q);
    let r = s.results();
    let ry = r.reaction(0).unwrap().fy;
    let mz = r.reaction(0).unwrap().mz;

    assert!(
        (ry - an_ry).abs() < 1e-9,
        "vertical reaction = {}, analytical = {}",
        ry,
        an_ry
    );
    assert!(
        (mz - an_mz).abs() < 1e-9,
        "moment reaction = {}, analytical = {}",
        mz,
        an_mz
    );
}

#[test]
fn p2_4_udl_with_axial_component() {
    let qx = 50.0_f64;
    let qy = 100.0_f64;
    let l = 1.0_f64;
    let e = 1.0_f64;
    let a = 1.0_f64;
    let i = 1.0_f64;

    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, l, 0.0));
    model.add_element(
        BeamElement::new(
            0,
            1,
            Material::new(e, 0.3, 1.0, "unit"),
            BeamSection::new(a, i),
        )
        .unwrap(),
    );
    model.fix_node(0);
    model.add_distributed_load(0, qx, qy).unwrap();

    let mut s = BeamSolver::from_model(&model).unwrap();
    s.solve_configured().unwrap();
    let r = s.results();

    let an_ux = qx * l.powi(2) / (2.0 * e * a);
    let an_uy = qy * l.powi(4) / (8.0 * e * i);
    let ux = r.displacement(1).unwrap().ux;
    let uy = r.displacement(1).unwrap().uy;

    assert!(
        (ux - an_ux).abs() < 1e-9 * an_ux.abs(),
        "axial tip disp = {}, analytical = {}",
        ux,
        an_ux
    );
    assert!(
        (uy - an_uy).abs() < 1e-9 * an_uy.abs(),
        "transverse tip disp = {}, analytical = {}",
        uy,
        an_uy
    );
}

// ===========================================================================
// P2-5: Raw DOF accessor bounds checking
// ===========================================================================

fn solved_cantilever() -> BeamSolver {
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, 1.0, 0.0));
    model.add_element(
        BeamElement::new(
            0,
            1,
            Material::new(1.0, 0.3, 1.0, "unit"),
            BeamSection::new(1.0, 1.0),
        )
        .unwrap(),
    );
    model.fix_node(0);
    model.add_nodal_force(1, 1, -10.0);
    let mut s = BeamSolver::from_model(&model).unwrap();
    s.solve_configured().unwrap();
    s
}

#[test]
fn p2_5_valid_dof_returns_ok() {
    let s = solved_cantilever();
    for node in 0..2 {
        for dof in 0..3 {
            assert!(
                s.displacement(node, dof).is_ok(),
                "displacement(node={}, dof={}) must be Ok",
                node,
                dof
            );
            assert!(
                s.reaction(node, dof).is_ok(),
                "reaction(node={}, dof={}) must be Ok",
                node,
                dof
            );
        }
    }
}

#[test]
fn p2_5_invalid_dof_displacement_returns_err() {
    let s = solved_cantilever();
    for dof in [3usize, 4, 100, usize::MAX] {
        let err = s.displacement(0, dof);
        assert!(err.is_err(), "displacement(dof={}) must be Err", dof);
        let e = err.unwrap_err();
        assert!(
            matches!(e, FemError::InvalidInput(_)),
            "error must be InvalidInput for dof={}: got {:?}",
            dof,
            e
        );
    }
}

#[test]
fn p2_5_invalid_dof_reaction_returns_err() {
    let s = solved_cantilever();
    for dof in [3usize, 4, 100] {
        let err = s.reaction(0, dof);
        assert!(err.is_err(), "reaction(dof={}) must be Err", dof);
        let e = err.unwrap_err();
        assert!(
            matches!(e, FemError::InvalidInput(_)),
            "error must be InvalidInput for dof={}: got {:?}",
            dof,
            e
        );
    }
}

#[test]
fn p2_5_raw_matches_typed_for_valid_dof() {
    let s = solved_cantilever();
    let dofs = [Dof::Ux, Dof::Uy, Dof::Rz];
    for node in 0..2 {
        for (k, dof) in dofs.iter().enumerate() {
            assert_eq!(
                s.displacement_dof(node, *dof).unwrap(),
                s.displacement(node, k).unwrap(),
                "node {} dof {:?}",
                node,
                dof
            );
            assert_eq!(
                s.reaction_dof(node, *dof).unwrap(),
                s.reaction(node, k).unwrap(),
                "node {} reaction {:?}",
                node,
                dof
            );
        }
    }
}
