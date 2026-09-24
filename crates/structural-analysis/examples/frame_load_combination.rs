//! Load cases and load combinations on a simply supported beam.
//!
//! Demonstrates: `LoadCase`, `LoadCombination`, `solve_case`,
//! `solve_combination`, and `load_source` result tagging.
//!
//! A 4 m simply supported beam carries:
//! - **Dead load**: self-weight UDL of 500 N/m
//! - **Live load**: point load of 2000 N at midspan
//!
//! Combined as `1.4·D + 1.6·L`.

use section_properties::Material;
use structural_analysis::{BeamSection, Dof, FrameModel, LoadCase, LoadCombination};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (e, a, i, l) = (200e9, 5e-3, 2e-5, 4.0);

    let mut frame = FrameModel::new();
    let left = frame.add_node(0.0, 0.0)?;
    let right = frame.add_node(l, 0.0)?;
    let member = frame.add_member(
        left,
        right,
        Material::new(e, 0.3, 7850.0, "Steel"),
        BeamSection::new(a, i),
    )?;
    frame.pin(left)?;
    frame.roller_y(right)?;

    let mut dead = LoadCase::new("dead");
    dead.member_udl(member, 0.0, -500.0)?;

    let mut live = LoadCase::new("live");
    live.member_point_load(member, 0.5, 0.0, -2000.0, 0.0)?;

    println!("--- dead load only ---");
    let result_d = frame.solve_case(&dead)?;
    println!("load source : {:?}", result_d.load_source());
    println!("end forces  : {:?}", result_d.member_end_forces(member)?);
    assert!(result_d.equilibrium().is_balanced());

    println!("--- live load only ---");
    let result_l = frame.solve_case(&live)?;
    println!("load source : {:?}", result_l.load_source());
    println!("end forces  : {:?}", result_l.member_end_forces(member)?);
    assert!(result_l.equilibrium().is_balanced());

    println!("--- 1.4D + 1.6L ---");
    let mut combo = LoadCombination::new("1.4D + 1.6L");
    combo.add_case(&dead, 1.4)?;
    combo.add_case(&live, 1.6)?;

    let result = frame.solve_combination(&combo)?;
    println!("load source : {:?}", result.load_source());
    println!(
        "left reaction  : Rx={:.4e}  Ry={:.4e}",
        result.reaction(left, Dof::Ux)?,
        result.reaction(left, Dof::Uy)?
    );
    println!(
        "right reaction : Rx={:.4e}  Ry={:.4e}",
        result.reaction(right, Dof::Ux)?,
        result.reaction(right, Dof::Uy)?
    );
    println!(
        "member end forces (local) : {:?}",
        result.member_end_forces(member)?
    );

    let sf_mid = result.section_forces(member, 0.5)?;
    println!(
        "midspan section forces : N={:.4e}  V={:.4e}  M={:.4e}",
        sf_mid.axial, sf_mid.shear, sf_mid.moment
    );

    assert!(result.equilibrium().is_balanced());
    println!("\nequilibrium: balanced");

    Ok(())
}
