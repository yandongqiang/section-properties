//! Rotated (45°) cantilever with combined axial + transverse tip loads and a
//! tip applied moment — showing the local/global transformation.
//!
//! Demonstrates: a non-axis-aligned element, local tip loads expressed in
//! global coordinates, an applied moment, and the relationship between global
//! displacement and the local element response.
//!
//! Unit properties (`E = A = I = 1`, `L = 1`). Local tip loads `fx = 50`
//! (tension), `fy = -100` (transverse) and a tip applied moment `M = +20` give
//! the analytical local tip response
//!
//! ```text
//! u_local = fx L / (E A)                             = +50
//! v_local = fy L^3 / (3 E I) + M L^2 / (2 E I)       = -23.333333
//! theta   = fy L^2 / (2 E I) + M L / (E I)           = -30
//! ```

use section_properties::beam_fem::{BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver};
use section_properties::material::Material;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (e, a, i, l) = (1.0, 1.0, 1.0, 1.0);
    let (fx, fy, m) = (50.0, -100.0, 20.0);
    let angle: f64 = 45.0_f64.to_radians();
    let (c, s) = (angle.cos(), angle.sin());

    // --- model (element along direction (cos, sin)) --------------------------
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, l * c, l * s));
    model.add_element(BeamElement::new(
        0,
        1,
        Material::new(e, 0.3, 1.0, "unit"),
        BeamSection::new(a, i),
    )?);
    model.fix_node(0);

    // Local tip force (fx, fy) expressed in global coordinates: Fg = R(theta)·F.
    let gx = c * fx - s * fy;
    let gy = s * fx + c * fy;
    model.add_nodal_force(1, 0, gx);
    model.add_nodal_force(1, 1, gy);
    model.add_applied_moment(1, m)?; // applied moment is a global theta load

    // --- solve ---------------------------------------------------------------
    let mut solver = BeamSolver::from_model(&model)?;
    solver.solve_configured()?;
    let res = solver.results();

    // --- global results ------------------------------------------------------
    let dg = res.displacement(1)?;
    println!("solver backend : {:?}", res.solver_name());
    println!("global tip ux,uy : {:+.6}, {:+.6}", dg.ux, dg.uy);

    // Local tip displacement: u_local = T · u_global.
    let el = &model.elements[0];
    let ni = model.nodes[0].point();
    let nj = model.nodes[1].point();
    let t = el.transformation_matrix(ni, nj);
    let ug = [
        solver.displacements()[3],
        solver.displacements()[4],
        solver.displacements()[5],
    ];
    let ul = [
        t[0][0] * ug[0] + t[0][1] * ug[1],
        t[1][0] * ug[0] + t[1][1] * ug[1],
        ug[2],
    ];
    println!(
        "local tip  u,v   : {:+.6}, {:+.6}   (analytical {:+.6}, {:+.6})",
        ul[0],
        ul[1],
        fx * l / (e * a),
        fy * l.powi(3) / (3.0 * e * i) + m * l * l / (2.0 * e * i)
    );
    println!(
        "local tip theta  : {:+.6}   (analytical {:+.6})",
        ul[2],
        fy * l.powi(2) / (2.0 * e * i) + m * l / (e * i)
    );

    // --- reactions and equilibrium ------------------------------------------
    let r0 = res.reaction(0)?;
    println!("reaction Rx,Ry   : {:+.6}, {:+.6}", r0.fx, r0.fy);
    println!("reaction Rz      : {:+.6}", r0.mz);
    println!("ΣFx residual     : {:+.3e}", r0.fx + gx);
    println!("ΣFy residual     : {:+.3e}", r0.fy + gy);
    println!(
        "ΣMz residual     : {:+.3e}   (about the fixed node)",
        r0.mz + m + l * c * gy - l * s * gx
    );

    // --- element end forces --------------------------------------------------
    println!("local end forces : {:?}", res.element_end_forces(0)?);
    println!(
        "global end forces: {:?}",
        solver.element_end_forces_global()?[0]
    );

    Ok(())
}
