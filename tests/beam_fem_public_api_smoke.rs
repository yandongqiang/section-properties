//! External-style Beam FEM smoke test.
//!
//! Uses **only** root-level public exports (`use section_properties::*;`), i.e.
//! exactly what an external crate user sees. It verifies that the complete
//! public workflow is reachable without touching `section_properties::beam_fem`
//! or any private/internal item:
//!
//! ```text
//! construct model -> create beam -> apply BC -> apply load -> solve
//!   -> typed displacement -> typed reaction -> element result
//! ```

use ::section_properties::*;

#[test]
fn public_api_workflow_smoke() -> Result<(), Box<dyn std::error::Error>> {
    // Realistic magnitudes: E = 200 GPa, A = 5e-3 m^2, I = 2e-5 m^4, L = 1 m.
    let e = 200e9;
    let i = 2e-5;
    let ei = e * i;
    let p = 10e3;

    // --- model ---------------------------------------------------------------
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, 1.0, 0.0));
    model.add_element(BeamElement::new(
        0,
        1,
        Material::new(e, 0.3, 7850.0, "Steel"),
        BeamSection::new(5e-3, i),
    )?);

    // --- boundary condition (typed, fallible) --------------------------------
    model.try_fix_node_with_values(0, 0.0, 0.0, 0.0)?;

    // --- load ----------------------------------------------------------------
    model.add_nodal_force(1, 1, -p);

    // --- solve with an explicitly selected backend ---------------------------
    let mut solver = BeamSolver::from_model(&model)?;
    solver.set_solver(SolverSelection::named("sparse_lu"));
    solver.solve_configured()?;
    assert_eq!(solver.solver_name(), Some("sparse_lu"));

    // --- typed result access -------------------------------------------------
    let uy = solver.displacement_dof(1, Dof::Uy)?;
    let rz = solver.displacement_dof(1, Dof::Rz)?;
    let ry = solver.reaction_dof(0, Dof::Uy)?;
    let moment = solver.reaction_dof(0, Dof::Rz)?;

    // v(L) = -P L^3 / (3 E I),  theta(L) = -P L^2 / (2 E I)
    assert!((uy + p / (3.0 * ei)).abs() < 1e-9, "uy = {}", uy);
    assert!((rz + p / (2.0 * ei)).abs() < 1e-9, "rz = {}", rz);
    // Ry = +P,  Rz = +P L
    assert!((ry - p).abs() < 1e-6, "Ry = {}", ry);
    assert!((moment - p).abs() < 1e-6, "Rz = {}", moment);

    // --- element result (local element-on-node forces) -----------------------
    let end = solver.element_end_forces()?;
    assert_eq!(end.len(), 1);
    assert!((end[0][1] + p).abs() < 1e-6, "V_i = {}", end[0][1]);
    assert!((end[0][2] + p).abs() < 1e-6, "M_i = {}", end[0][2]);
    assert!((end[0][4] - p).abs() < 1e-6, "V_j = {}", end[0][4]);

    // --- section result ------------------------------------------------------
    let mid = solver.element_section_forces(0, 0.5)?;
    assert!((mid.shear - p).abs() < 1e-6, "V(0.5) = {}", mid.shear);
    assert!(
        (mid.moment + p * 0.5).abs() < 1e-6,
        "M(0.5) = {}",
        mid.moment
    );

    Ok(())
}
