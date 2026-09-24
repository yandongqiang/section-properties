//! 2D truss analysis — triangular truss with a vertical tip load.
//!
//! Demonstrates: create model → add nodes → add truss members → supports
//! → nodal load → solve → displacements → reactions → axial forces
//! → mechanism diagnostic.
//!
//! Model: a symmetric triangular truss with a 10 kN downward load at the apex.
//!
//! ```text
//!        node 2 (1, 1)
//!          / \
//!         /   \
//!        /     \
//!  node 0 ----- node 1
//!  (0, 0)       (2, 0)
//!  fixed        roller (uy=0)
//! ```

use section_properties::Material;
use structural_analysis::truss::{TrussDof, TrussElement, TrussModel, TrussNode, TrussSolver};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (e, a) = (200e9, 5e-3);
    let p = -10e3; // 10 kN downward

    let mut model = TrussModel::new();
    model.add_node(TrussNode::new(0, 0.0, 0.0));
    model.add_node(TrussNode::new(1, 2.0, 0.0));
    model.add_node(TrussNode::new(2, 1.0, 1.0));

    let mat = Material::new(e, 0.3, 7850.0, "Steel");
    let m02 = TrussElement::new(0, 2, &mat, a)?;
    let m12 = TrussElement::new(1, 2, &mat, a)?;
    let m01 = TrussElement::new(0, 1, &mat, a)?;
    model.add_element(m02);
    model.add_element(m12);
    model.add_element(m01);

    model.fix_node(0)?;
    model.fix_dof(1, TrussDof::Uy, 0.0)?;
    model.add_nodal_force(2, 0.0, p)?;

    let mut solver = TrussSolver::from_model(&model)?;
    solver.solve_configured()?;

    println!("solver backend : {:?}", solver.solver_name());

    println!(
        "apex displacement : ux={:.6e} m  uy={:.6e} m",
        solver.displacement(2, TrussDof::Ux)?,
        solver.displacement(2, TrussDof::Uy)?
    );

    println!(
        "support reactions : node0 Rx={:.4e}  Ry={:.4e}  |  node1 Ry={:.4e}",
        solver.reaction(0, TrussDof::Ux)?,
        solver.reaction(0, TrussDof::Uy)?,
        solver.reaction(1, TrussDof::Uy)?
    );

    let forces = solver.axial_forces()?;
    println!("axial forces (tension +) :");
    println!("  member 0-2 : {:+.4e} N", forces[0]);
    println!("  member 1-2 : {:+.4e} N", forces[1]);
    println!("  member 0-1 : {:+.4e} N", forces[2]);

    let diag = solver.diagnostic()?;
    println!("diagnostic : {:?}", diag);
    println!("stable     : {}", diag.is_stable());

    let rx0 = solver.reaction(0, TrussDof::Ux)?;
    let ry0 = solver.reaction(0, TrussDof::Uy)?;
    let rx1 = solver.reaction(1, TrussDof::Ux)?;
    let ry1 = solver.reaction(1, TrussDof::Uy)?;
    println!(
        "equilibrium : sumFx={:+.3e}  sumFy={:+.3e}  sumM={:+.3e}",
        rx0 + rx1,
        ry0 + ry1 + p,
        2.0 * ry1 + p * 1.0
    );

    Ok(())
}
