//! Cantilever with a transverse tip load — the simplest end-to-end workflow.
//!
//! Demonstrates: build model → add nodes/elements → fix a support → apply a
//! load → solve → read displacement, reactions and element end forces.
//!
//! Unit properties (`E = A = I = 1`, `L = 1`) are used so the results are easy
//! to verify by hand:
//!
//! ```text
//! tip deflection  delta = -P L^3 / (3 E I) = -33.333333
//! tip rotation    theta = -P L^2 / (2 E I) = -50.0
//! reaction        Ry = +P = 100,  Rz = +P L = 100
//! ```

use section_properties::beam_fem::{
    BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver, Dof,
};
use section_properties::material::Material;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (e, a, i, l, p) = (1.0, 1.0, 1.0, 1.0, 100.0);

    // --- model ---------------------------------------------------------------
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, l, 0.0));
    model.add_element(BeamElement::new(
        0,
        1,
        Material::new(e, 0.3, 1.0, "unit"),
        BeamSection::new(a, i),
    )?);
    model.fix_node(0); // clamp node 0 (ux = uy = rz = 0)
    model.add_nodal_force(1, 1, -p); // global uy = -P at the free tip

    // --- solve ---------------------------------------------------------------
    let mut solver = BeamSolver::from_model(&model)?;
    solver.solve_configured()?; // default SolverSelection::Auto
    let res = solver.results();

    // --- results -------------------------------------------------------------
    // Typed DOF accessors on the solver (equivalent to the raw-index API):
    //   solver.displacement_dof(1, Dof::Uy) == solver.displacement(1, 1)
    let uy = solver.displacement_dof(1, Dof::Uy)?;
    let rz = solver.displacement_dof(1, Dof::Rz)?;
    let ry = solver.reaction_dof(0, Dof::Uy)?;
    let moment = solver.reaction_dof(0, Dof::Rz)?;

    println!("solver backend : {:?}", res.solver_name());
    println!(
        "tip uy         : {:+.6}   (analytical {:+.6})",
        uy,
        -p * l.powi(3) / (3.0 * e * i)
    );
    println!(
        "tip rz         : {:+.6}   (analytical {:+.6})",
        rz,
        -p * l.powi(2) / (2.0 * e * i)
    );
    println!("reaction Ry    : {:+.6}   (analytical {:+.6})", ry, p);
    println!(
        "reaction Rz    : {:+.6}   (analytical {:+.6})",
        moment,
        p * l
    );

    let end = res.element_end_forces(0)?;
    println!("element end forces [N_i,V_i,M_i,N_j,V_j,M_j] = {end:?}");
    println!(
        "internal section moment at the fixed end M(0) = {:+.6} (analytical -P L = {:+.6})",
        res.section_forces(0, 0.0)?.moment,
        -p * l
    );

    Ok(())
}
