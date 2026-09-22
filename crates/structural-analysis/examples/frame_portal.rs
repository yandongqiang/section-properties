//! 2D frame analysis with the `FrameModel` façade.
//!
//! Demonstrates: node/member handles, support vocabulary (fixed bases), a
//! global nodal load, a **local** member distributed load, solving with an
//! explicit backend, typed result access, and the global equilibrium report.
//!
//! Conventions (unchanged from the Beam FEM core):
//! nodal loads and moments are GLOBAL; member loads are LOCAL.

use section_properties::{Material, SolverSelection};
use structural_analysis::frame::FrameModel;
use structural_analysis::{BeamSection, Dof};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (e, a, i) = (200e9, 5e-3, 2e-5);
    let (h, w) = (3.0, 4.0);
    let lateral = 20e3; // N, +X
    let udl = -8e3; // N/m, downward in member-local +y

    // --- model -----------------------------------------------------------------
    let mut frame = FrameModel::new();
    let base_left = frame.add_node(0.0, 0.0)?;
    let top_left = frame.add_node(0.0, h)?;
    let top_right = frame.add_node(w, h)?;
    let base_right = frame.add_node(w, 0.0)?;

    let section = BeamSection::new(a, i);
    let mat = Material::new(e, 0.3, 7850.0, "Steel");
    let col_left = frame.add_member(base_left, top_left, mat, section)?;
    let beam = frame.add_member(top_left, top_right, mat, section)?;
    let col_right = frame.add_member(top_right, base_right, mat, section)?;

    frame.fix(base_left)?;
    frame.fix(base_right)?;
    frame.nodal_load(top_left, lateral, 0.0)?; // global
    frame.member_udl(beam, 0.0, udl)?; // local

    // --- solve -----------------------------------------------------------------
    let result = frame.solve_with(SolverSelection::named("sparse_lu"))?;
    println!("backend              : {:?}", result.solver_name());
    println!(
        "nodes / members      : {} / {}",
        result.n_nodes(),
        result.n_members()
    );

    // --- results ---------------------------------------------------------------
    println!(
        "roof sway (ux)       : {:+.6e} m",
        result.displacement(top_left, Dof::Ux)?
    );
    println!(
        "base reactions       : Rx={:+.6e}  Ry={:+.6e}  Mz={:+.6e}",
        result.reaction(base_left, Dof::Ux)?,
        result.reaction(base_left, Dof::Uy)?,
        result.reaction(base_left, Dof::Rz)?
    );
    println!(
        "right base reaction  : Rx={:+.6e}  Ry={:+.6e}",
        result.reaction(base_right, Dof::Ux)?,
        result.reaction(base_right, Dof::Uy)?
    );

    // Member end forces are element-on-node forces in LOCAL axes:
    // [N_i, V_i, M_i, N_j, V_j, M_j] = f_equiv - K_e u_e.
    println!(
        "beam end forces (local)  : {:?}",
        result.member_end_forces(beam)?
    );
    println!(
        "left column global ends  : {:?}",
        result.member_end_forces_global(col_left)?
    );
    println!(
        "right column global ends : {:?}",
        result.member_end_forces_global(col_right)?
    );

    // --- equilibrium -----------------------------------------------------------
    let eq = result.equilibrium();
    println!(
        "equilibrium residual : sumFx={:+.3e}  sumFy={:+.3e}  sumMz={:+.3e}  (about the origin)",
        eq.fx_residual, eq.fy_residual, eq.mz_residual
    );
    println!("balanced             : {}", eq.is_balanced());

    Ok(())
}
