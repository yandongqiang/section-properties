//! Phase 46 — Frame solver selection API tests.
//!
//! Verifies that explicit solver selection is respected, produces numerically
//! equivalent results across backends, and that failures are propagated without
//! silent fallback.

use section_properties::{Material, SolverSelection};
use structural_analysis::{BeamSection, Dof, FemError, FrameModel, NodeHandle};

fn steel() -> Material {
    Material::new(200e9, 0.3, 7850.0, "S355")
}

fn sec() -> BeamSection {
    BeamSection::new(5e-3, 2e-5)
}

struct Portal {
    model: FrameModel,
    loaded_node: NodeHandle,
    support_a: NodeHandle,
}

fn portal() -> Result<Portal, FemError> {
    let mut f = FrameModel::new();
    let a = f.add_node(0.0, 0.0)?;
    let b = f.add_node(2.0, 0.0)?;
    let c = f.add_node(2.0, 3.0)?;
    let d = f.add_node(0.0, 3.0)?;
    f.add_member(a, d, steel(), sec())?;
    f.add_member(d, c, steel(), sec())?;
    f.add_member(c, b, steel(), sec())?;
    f.fix(a)?;
    f.fix(b)?;
    f.nodal_load(d, 2.0e4, -1.0e4)?;
    Ok(Portal {
        model: f,
        loaded_node: d,
        support_a: a,
    })
}

fn solvers() -> [SolverSelection; 3] {
    [
        SolverSelection::dense(),
        SolverSelection::skyline_ldlt(),
        SolverSelection::sparse_lu(),
    ]
}

#[test]
fn explicit_solver_is_respected() -> Result<(), FemError> {
    let p = portal()?;
    for sel in &solvers() {
        let r = p.model.solve_with(sel.clone())?;
        let expected = sel.requested_name().unwrap();
        assert_eq!(
            r.solver_name(),
            Some(expected),
            "solver_name mismatch for {:?}",
            sel
        );
    }
    Ok(())
}

#[test]
fn auto_solver_reports_name() -> Result<(), FemError> {
    let p = portal()?;
    let r = p.model.solve()?;
    assert!(
        r.solver_name().is_some(),
        "Auto must report which backend was used"
    );
    Ok(())
}

#[test]
fn all_solvers_numerically_equivalent() -> Result<(), FemError> {
    let p = portal()?;

    let auto = p.model.solve()?;
    let auto_uy = auto.displacement(p.loaded_node, Dof::Uy)?;
    let auto_rx = auto.reaction(p.support_a, Dof::Ux)?;

    for sel in &solvers() {
        let r = p.model.solve_with(sel.clone())?;
        let uy = r.displacement(p.loaded_node, Dof::Uy)?;
        let rx = r.reaction(p.support_a, Dof::Ux)?;

        let uy_rel = (uy - auto_uy).abs() / auto_uy.abs().max(1e-30);
        assert!(
            uy_rel < 1e-10,
            "{:?}: Uy rel diff = {:.3e} (auto = {:.6e}, got = {:.6e})",
            sel,
            uy_rel,
            auto_uy,
            uy
        );

        let rx_rel = (rx - auto_rx).abs() / auto_rx.abs().max(1e-30);
        assert!(
            rx_rel < 1e-8,
            "{:?}: Rx rel diff = {:.3e} (auto = {:.6e}, got = {:.6e})",
            sel,
            rx_rel,
            auto_rx,
            rx
        );

        assert!(
            r.equilibrium().is_balanced(),
            "{:?}: equilibrium not balanced",
            sel
        );
    }
    Ok(())
}

#[test]
fn convenience_constructors_match_named() {
    assert_eq!(SolverSelection::dense(), SolverSelection::named("dense"));
    assert_eq!(
        SolverSelection::skyline_ldlt(),
        SolverSelection::named("skyline_ldlt")
    );
    assert_eq!(
        SolverSelection::sparse_lu(),
        SolverSelection::named("sparse_lu")
    );
}

#[test]
fn unavailable_solver_returns_error() -> Result<(), FemError> {
    let p = portal()?;
    let result = p
        .model
        .solve_with(SolverSelection::named("nonexistent_solver"));
    assert!(
        result.is_err(),
        "Unavailable solver must return Err, not silently fall back"
    );
    let err = result.unwrap_err();
    assert!(
        err.solver_error().is_some(),
        "Error must be a SolverError variant, got: {:?}",
        err
    );
    Ok(())
}

#[test]
fn builder_pattern_works() -> Result<(), FemError> {
    let p = portal()?;
    let r = p
        .model
        .solver()
        .with_selection(SolverSelection::dense())
        .solve()?;
    assert_eq!(r.solver_name(), Some("dense"));
    assert!(r.equilibrium().is_balanced());
    let _uy = r.displacement(p.loaded_node, Dof::Uy)?;
    Ok(())
}

#[test]
fn default_is_auto() {
    let sel = SolverSelection::default();
    assert!(sel.is_auto());
    assert!(sel.requested_name().is_none());
}
