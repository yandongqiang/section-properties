//! Advanced supports: spring, inclined roller, and end release.
//!
//! Demonstrates three support features beyond the basic fix/pin/roller
//! vocabulary:
//!
//! 1. **Spring support** — a translational spring at a node provides finite
//!    restraint. `reaction = -k · displacement`.
//! 2. **Inclined roller** — constrains displacement along a specified
//!    direction `(nx, ny)`, leaving the orthogonal direction free.
//! 3. **End release (hinge)** — removes the moment transfer at a member end
//!    without removing the node's rotational DOF.
//!
//! Model: a two-span continuous beam on a spring support at the middle.
//! The right end uses an inclined roller at 30° from vertical. The left
//! span has a hinge at its right end.

use section_properties::Material;
use structural_analysis::{BeamSection, Dof, EndRelease, FrameModel};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (e, a, i) = (200e9, 5e-3, 2e-5);
    let l = 3.0;

    let mut frame = FrameModel::new();
    let n0 = frame.add_node(0.0, 0.0)?;
    let n1 = frame.add_node(l, 0.0)?;
    let n2 = frame.add_node(2.0 * l, 0.0)?;

    let mat = Material::new(e, 0.3, 7850.0, "Steel");
    let sec = BeamSection::new(a, i);

    // Span 0–1: hinge at node 1 (end release on rotation at node_j)
    let m01 = frame.add_member_with_release(n0, n1, mat, sec, EndRelease::end_pin())?;
    // Span 1–2: fully rigid
    let m12 = frame.add_member(n1, n2, mat, sec)?;

    // --- supports ---
    // Left: pinned
    frame.pin(n0)?;

    // Middle: translational spring in uy (k = 1e6 N/m)
    let k_spring = 1.0e6;
    frame.spring(n1, Dof::Uy, k_spring)?;

    // Right: inclined roller constraining along (sin30°, cos30°) = (0.5, 0.866)
    // This allows movement along the orthogonal direction.
    let angle = 30.0_f64.to_radians();
    frame.inclined_roller(n2, angle.sin(), angle.cos(), 0.0)?;

    // --- loads ---
    // Uniform downward load on both spans
    frame.member_udl(m01, 0.0, -1000.0)?;
    frame.member_udl(m12, 0.0, -1000.0)?;

    // --- solve ---
    let result = frame.solve()?;
    println!("solver backend : {:?}", result.solver_name());

    // --- displacements ---
    println!(
        "node 0  ux={:.6e}  uy={:.6e}  rz={:.6e}",
        result.displacement(n0, Dof::Ux)?,
        result.displacement(n0, Dof::Uy)?,
        result.displacement(n0, Dof::Rz)?
    );
    println!(
        "node 1  ux={:.6e}  uy={:.6e}  rz={:.6e}",
        result.displacement(n1, Dof::Ux)?,
        result.displacement(n1, Dof::Uy)?,
        result.displacement(n1, Dof::Rz)?
    );
    println!(
        "node 2  ux={:.6e}  uy={:.6e}  rz={:.6e}",
        result.displacement(n2, Dof::Ux)?,
        result.displacement(n2, Dof::Uy)?,
        result.displacement(n2, Dof::Rz)?
    );

    // --- spring reaction ---
    // reaction = -k * displacement
    let uy1 = result.displacement(n1, Dof::Uy)?;
    let spring_reaction = -k_spring * uy1;
    println!(
        "spring reaction : {:.4e} N  (=-k*uy = -{:.0e}*{:.6e})",
        spring_reaction, k_spring, uy1
    );

    // --- member end forces ---
    let f01 = result.member_end_forces(m01)?;
    let f12 = result.member_end_forces(m12)?;
    println!("span 0-1 end forces (local) : {:?}", f01);
    println!("span 1-2 end forces (local) : {:?}", f12);

    // The hinge at node 1 (end of member 0-1) should give M_j ≈ 0
    println!(
        "hinge check: M_j(span 0-1) = {:.6e}  (should be ≈ 0)",
        f01[5]
    );
    assert!(f01[5].abs() < 1.0, "hinge moment should be near zero");

    // --- equilibrium ---
    let eq = result.equilibrium();
    println!(
        "equilibrium : sumFx={:+.3e}  sumFy={:+.3e}  sumMz={:+.3e}",
        eq.fx_residual, eq.fy_residual, eq.mz_residual
    );
    println!("balanced     : {}", eq.is_balanced());
    assert!(eq.is_balanced());

    Ok(())
}
