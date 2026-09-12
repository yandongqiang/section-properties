//! Cantilever with a uniform transverse load, modelled with several elements.
//!
//! Demonstrates a multi-element model with a distributed load and shows that
//! the complete external load is represented exactly once in the reactions.
//!
//! Unit properties (`E = A = I = 1`), `L = 1`, `q = 100` (downward):
//!
//! ```text
//! tip deflection  delta = -q L^4 / (8 E I) = -12.5
//! tip rotation    theta = -q L^3 / (6 E I) = -16.666667
//! reactions       Ry = q L = 100,  Rz = q L^2 / 2 = 50
//! ```

use section_properties::beam_fem::{BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver};
use section_properties::material::Material;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (e, a, i, l, q) = (1.0, 1.0, 1.0, 1.0, 100.0);
    let n_elem = 4;

    // --- model ---------------------------------------------------------------
    let mut model = BeamModel::new();
    let dx = l / n_elem as f64;
    for k in 0..=n_elem {
        model.add_node(BeamNode::new(k, k as f64 * dx, 0.0));
    }
    for k in 0..n_elem {
        model.add_element(BeamElement::new(
            k,
            k + 1,
            Material::new(e, 0.3, 1.0, "unit"),
            BeamSection::new(a, i),
        )?);
        // uniform downward load, local qy < 0, applied to every element
        model.add_distributed_load(k, 0.0, -q)?;
    }
    model.fix_node(0);

    // --- solve ---------------------------------------------------------------
    let mut solver = BeamSolver::from_model(&model)?;
    solver.solve_configured()?;
    let res = solver.results();

    // --- results -------------------------------------------------------------
    let tip = res.displacement(n_elem)?;
    println!("solver backend : {:?}", res.solver_name());
    println!(
        "tip uy         : {:+.6}   (analytical {:+.6})",
        tip.uy,
        -q * l.powi(4) / (8.0 * e * i)
    );
    println!(
        "tip rz         : {:+.6}   (analytical {:+.6})",
        tip.rz,
        -q * l.powi(3) / (6.0 * e * i)
    );

    let r0 = res.reaction(0)?;
    println!(
        "reaction Ry    : {:+.6}   (analytical q L = {:+.6})",
        r0.fy,
        q * l
    );
    println!(
        "reaction Rz    : {:+.6}   (analytical q L^2 / 2 = {:+.6})",
        r0.mz,
        q * l * l / 2.0
    );

    // Each element's end forces are element-on-node forces; the total transverse
    // end-force resultant across the beam balances the applied load q·L.
    let mut total_vy = 0.0;
    for k in 0..n_elem {
        let fe = res.element_end_forces(k)?;
        total_vy += fe[1] + fe[4];
    }
    println!(
        "Σ element end shear = {:+.6}; applied load = {:+.6} (counted once)",
        total_vy,
        -q * l
    );

    Ok(())
}
