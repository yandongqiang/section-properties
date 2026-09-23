//! Phase 93 - Inclined roller tests.

use section_properties::{Material, SolverSelection};
use structural_analysis::frame::{FrameModel, MemberHandle, NodeHandle};
use structural_analysis::{BeamSection, Dof, FemError};

const E: f64 = 200e9;
const A: f64 = 5e-3;
const I: f64 = 2e-5;
const EA: f64 = E * A;
const EI: f64 = E * I;

fn steel() -> Material {
    Material::new(E, 0.3, 7850.0, "Steel")
}
fn sec() -> BeamSection {
    BeamSection::new(A, I)
}

fn assert_close(a: f64, b: f64, abs: f64, rel: f64, label: &str) {
    let bound = abs + rel * a.abs().max(b.abs());
    assert!(
        (a - b).abs() <= bound,
        "{}: {} vs {} (|diff| = {:.3e} > {:.3e})",
        label,
        a,
        b,
        (a - b).abs(),
        bound
    );
}

#[test]
fn inclined_roller_vertical_direction_matches_roller_y() -> Result<(), FemError> {
    let (l, p) = (2.0, 1.0e4);

    let mut f_ref = FrameModel::new();
    let n0 = f_ref.add_node(0.0, 0.0)?;
    let n1 = f_ref.add_node(l, 0.0)?;
    f_ref.add_member(n0, n1, steel(), sec())?;
    f_ref.fix(n0)?;
    f_ref.roller_y(n1)?;
    f_ref.nodal_load(n1, p, 0.0)?;
    let r_ref = f_ref.solve()?;

    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(l, 0.0)?;
    f.add_member(n0, n1, steel(), sec())?;
    f.fix(n0)?;
    f.inclined_roller(n1, 0.0, 1.0, 0.0)?;
    f.nodal_load(n1, p, 0.0)?;
    let r = f.solve()?;

    assert_close(
        r.displacement(n1, Dof::Ux)?,
        r_ref.displacement(n1, Dof::Ux)?,
        1e-14,
        1e-12,
        "Ux matches roller_y",
    );
    assert_close(
        r.displacement(n1, Dof::Uy)?,
        r_ref.displacement(n1, Dof::Uy)?,
        1e-14,
        1e-12,
        "Uy matches roller_y",
    );
    assert_close(
        r.reaction(n1, Dof::Uy)?,
        r_ref.reaction(n1, Dof::Uy)?,
        1e-10,
        1e-10,
        "Ry matches roller_y",
    );
    Ok(())
}

#[test]
fn inclined_roller_horizontal_direction_matches_roller_x() -> Result<(), FemError> {
    let (l, p) = (2.0, 1.0e4);

    let mut f_ref = FrameModel::new();
    let n0 = f_ref.add_node(0.0, 0.0)?;
    let n1 = f_ref.add_node(l, 0.0)?;
    f_ref.add_member(n0, n1, steel(), sec())?;
    f_ref.fix(n0)?;
    f_ref.roller_x(n1)?;
    f_ref.nodal_load(n1, 0.0, p)?;
    let r_ref = f_ref.solve()?;

    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(l, 0.0)?;
    f.add_member(n0, n1, steel(), sec())?;
    f.fix(n0)?;
    f.inclined_roller(n1, 1.0, 0.0, 0.0)?;
    f.nodal_load(n1, 0.0, p)?;
    let r = f.solve()?;

    assert_close(
        r.displacement(n1, Dof::Ux)?,
        r_ref.displacement(n1, Dof::Ux)?,
        1e-14,
        1e-12,
        "Ux matches roller_x",
    );
    assert_close(
        r.displacement(n1, Dof::Uy)?,
        r_ref.displacement(n1, Dof::Uy)?,
        1e-14,
        1e-12,
        "Uy matches roller_x",
    );
    Ok(())
}

#[test]
fn forty_five_degree_roller_axial_bar() -> Result<(), FemError> {
    let (l, px, py) = (2.0, 8.0e3, 3.0e3);
    let k_bar = EA / l;

    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(l, 0.0)?;
    f.add_member(n0, n1, steel(), sec())?;
    f.fix(n0)?;
    f.inclined_roller(n1, 1.0, 1.0, 0.0)?;
    f.nodal_load(n1, px, py)?;

    let r = f.solve()?;

    let ux_exact = (px - py) / k_bar;
    let uy_exact = (-px + py) / k_bar;
    assert_close(
        r.displacement(n1, Dof::Ux)?,
        ux_exact,
        1e-6,
        1e-3,
        "Ux = (Px - Py) / k_bar (axial-dominant)",
    );
    assert_close(
        r.displacement(n1, Dof::Uy)?,
        uy_exact,
        1e-6,
        1e-3,
        "Uy = (-Px + Py) / k_bar (axial-dominant)",
    );

    assert_close(
        r.displacement(n1, Dof::Ux)? + r.displacement(n1, Dof::Uy)?,
        0.0,
        1e-12,
        1e-12,
        "Ux + Uy = 0 (roller constraint)",
    );

    let rx1 = r.reaction(n1, Dof::Ux)?;
    let ry1 = r.reaction(n1, Dof::Uy)?;
    assert_close(rx1, -py, 1.0, 1e-2, "Rx1 ≈ -Py (axial-dominant)");
    assert_close(ry1, -py, 1.0, 1e-2, "Ry1 ≈ -Py (axial-dominant)");
    assert_close(rx1, ry1, 1e-8, 1e-10, "reaction in (1,1) direction");

    let rx0 = r.reaction(n0, Dof::Ux)?;
    let ry0 = r.reaction(n0, Dof::Uy)?;
    assert_close(rx0 + rx1 + px, 0.0, 1e-6, 1e-9, "global Fx equilibrium");
    assert_close(ry0 + ry1 + py, 0.0, 1e-6, 1e-9, "global Fy equilibrium");
    Ok(())
}

#[test]
fn inclined_roller_reaction_along_constraint_direction() -> Result<(), FemError> {
    let l = 2.0;
    let angle = 30.0_f64.to_radians();
    let nx = angle.cos();
    let ny = angle.sin();

    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(l, 0.0)?;
    f.add_member(n0, n1, steel(), sec())?;
    f.fix(n0)?;
    f.inclined_roller(n1, nx, ny, 0.0)?;
    f.nodal_load(n1, 5.0e3, 7.0e3)?;

    let r = f.solve()?;

    let rx = r.reaction(n1, Dof::Ux)?;
    let ry = r.reaction(n1, Dof::Uy)?;
    let r_normal = rx * nx + ry * ny;
    let r_tangent = -rx * ny + ry * nx;
    assert_close(
        r_tangent,
        0.0,
        1e-8,
        1e-9,
        "reaction tangent component = 0 (reaction along constraint normal)",
    );
    assert!(
        r_normal.abs() > 1.0,
        "reaction normal component should be non-trivial"
    );

    let ux = r.displacement(n1, Dof::Ux)?;
    let uy = r.displacement(n1, Dof::Uy)?;
    let u_normal = ux * nx + uy * ny;
    assert_close(
        u_normal,
        0.0,
        1e-12,
        1e-12,
        "displacement along constraint direction = 0",
    );
    Ok(())
}

#[test]
fn inclined_roller_with_prescribed_displacement() -> Result<(), FemError> {
    let l = 2.0;
    let delta = 0.001;

    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(l, 0.0)?;
    f.add_member(n0, n1, steel(), sec())?;
    f.fix(n0)?;
    f.inclined_roller(n1, 0.0, 1.0, delta)?;

    let r = f.solve()?;

    assert_close(
        r.displacement(n1, Dof::Uy)?,
        delta,
        1e-14,
        1e-12,
        "Uy = prescribed settlement",
    );
    assert_close(
        r.displacement(n1, Dof::Ux)?,
        0.0,
        1e-14,
        1e-12,
        "Ux = 0 (no axial load)",
    );
    Ok(())
}

#[test]
fn inclined_roller_validation() -> Result<(), FemError> {
    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(1.0, 0.0)?;
    f.add_member(n0, n1, steel(), sec())?;
    f.fix(n0)?;

    assert!(
        f.inclined_roller(n1, 0.0, 0.0, 0.0).is_err(),
        "zero direction rejected"
    );
    assert!(
        f.inclined_roller(n1, f64::NAN, 1.0, 0.0).is_err(),
        "NaN direction rejected"
    );
    assert!(
        f.inclined_roller(n1, 1.0, 0.0, f64::NAN).is_err(),
        "NaN settlement rejected"
    );
    assert!(
        f.inclined_roller(n1, 1.0, 1.0, 0.0).is_ok(),
        "valid roller accepted"
    );
    Ok(())
}

#[test]
fn inclined_roller_direction_auto_normalized() -> Result<(), FemError> {
    let (l, px, py) = (2.0, 8.0e3, 3.0e3);

    let mut f1 = FrameModel::new();
    let n0 = f1.add_node(0.0, 0.0)?;
    let n1 = f1.add_node(l, 0.0)?;
    f1.add_member(n0, n1, steel(), sec())?;
    f1.fix(n0)?;
    f1.inclined_roller(n1, 1.0, 1.0, 0.0)?;
    f1.nodal_load(n1, px, py)?;
    let r1 = f1.solve()?;

    let mut f2 = FrameModel::new();
    let n0 = f2.add_node(0.0, 0.0)?;
    let n1 = f2.add_node(l, 0.0)?;
    f2.add_member(n0, n1, steel(), sec())?;
    f2.fix(n0)?;
    f2.inclined_roller(n1, 5.0, 5.0, 0.0)?;
    f2.nodal_load(n1, px, py)?;
    let r2 = f2.solve()?;

    assert_close(
        r1.displacement(n1, Dof::Ux)?,
        r2.displacement(n1, Dof::Ux)?,
        1e-12,
        1e-10,
        "Ux: (1,1) vs (5,5) same result",
    );
    assert_close(
        r1.displacement(n1, Dof::Uy)?,
        r2.displacement(n1, Dof::Uy)?,
        1e-12,
        1e-10,
        "Uy: (1,1) vs (5,5) same result",
    );
    Ok(())
}

#[test]
fn inclined_roller_on_inclined_member() -> Result<(), FemError> {
    let angle = 45.0_f64.to_radians();
    let l = 3.0;
    let (dx, dy) = (l * angle.cos(), l * angle.sin());
    let p = 1.0e4;

    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(dx, dy)?;
    f.add_member(n0, n1, steel(), sec())?;
    f.fix(n0)?;

    let nx = angle.cos();
    let ny = angle.sin();
    f.inclined_roller(n1, -ny, nx, 0.0)?;

    f.nodal_load(n1, p * nx, p * ny)?;

    let r = f.solve()?;

    let k_bar = EA / l;
    let u_axial = p / k_bar;
    let ux_exact = u_axial * nx;
    let uy_exact = u_axial * ny;
    assert_close(
        r.displacement(n1, Dof::Ux)?,
        ux_exact,
        1e-10,
        1e-9,
        "axial displacement along member",
    );
    assert_close(
        r.displacement(n1, Dof::Uy)?,
        uy_exact,
        1e-10,
        1e-9,
        "axial displacement along member",
    );
    Ok(())
}

#[test]
fn inclined_roller_global_equilibrium() -> Result<(), FemError> {
    let l = 2.0;
    let angle = 60.0_f64.to_radians();
    let nx = angle.cos();
    let ny = angle.sin();

    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(l, 0.0)?;
    let n2 = f.add_node(l, l)?;
    f.add_member(n0, n1, steel(), sec())?;
    f.add_member(n1, n2, steel(), sec())?;
    f.fix(n0)?;
    f.inclined_roller(n2, nx, ny, 0.0)?;
    f.nodal_load(n1, 3.0e3, -4.0e3)?;
    f.nodal_load(n2, 2.0e3, 1.0e3)?;

    let r = f.solve()?;

    let nodes = [(0.0_f64, 0.0_f64), (l, 0.0), (l, l)];
    let loads = [(0.0_f64, 0.0_f64), (3.0e3, -4.0e3), (2.0e3, 1.0e3)];
    let node_handles = [n0, n1, n2];

    let mut sum_fx = 0.0;
    let mut sum_fy = 0.0;
    let mut sum_m = 0.0;
    for i in 0..3 {
        let rx = r.reaction(node_handles[i], Dof::Ux)?;
        let ry = r.reaction(node_handles[i], Dof::Uy)?;
        let rz = r.reaction(node_handles[i], Dof::Rz)?;
        let fx = loads[i].0;
        let fy = loads[i].1;
        sum_fx += rx + fx;
        sum_fy += ry + fy;
        sum_m += nodes[i].0 * (ry + fy) - nodes[i].1 * (rx + fx) + rz;
    }

    assert_close(sum_fx, 0.0, 1e-6, 1e-9, "global Fx equilibrium");
    assert_close(sum_fy, 0.0, 1e-6, 1e-9, "global Fy equilibrium");
    assert_close(sum_m, 0.0, 1e-3, 1e-9, "global moment equilibrium");
    Ok(())
}
