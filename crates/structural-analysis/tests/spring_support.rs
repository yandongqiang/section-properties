//! Phase 93 - Spring support tests.

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

/// Axial bar: node 0 fixed, node 1 with a spring in Ux, roller in Uy.
fn axial_bar_with_spring(
    len: f64,
    k: f64,
) -> Result<(FrameModel, NodeHandle, NodeHandle, MemberHandle), FemError> {
    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(len, 0.0)?;
    let m = f.add_member(n0, n1, steel(), sec())?;
    f.fix(n0)?;
    f.roller_y(n1)?;
    f.spring(n1, Dof::Ux, k)?;
    Ok((f, n0, n1, m))
}

#[test]
fn axial_spring_displacement_and_reaction() -> Result<(), FemError> {
    let (l, p, k) = (2.0, 1.0e4, 5.0e6);
    let (f, n0, n1, _m) = axial_bar_with_spring(l, k)?;
    let mut f2 = f.clone();
    f2.nodal_load(n1, p, 0.0)?;
    let r = f2.solve()?;

    let k_bar = EA / l;
    let u_exact = p / (k_bar + k);
    assert_close(
        r.displacement(n1, Dof::Ux)?,
        u_exact,
        1e-12,
        1e-9,
        "u = P / (EA/L + k)",
    );

    let rx0 = -k_bar * u_exact;
    assert_close(
        r.reaction(n0, Dof::Ux)?,
        rx0,
        1e-6,
        1e-9,
        "Rx0 = -(EA/L) * u",
    );

    let spring_reaction = -k * u_exact;
    assert_close(
        r.reaction(n1, Dof::Ux)?,
        spring_reaction,
        1e-6,
        1e-9,
        "spring reaction = -k * u",
    );

    assert_close(
        r.reaction(n0, Dof::Ux)? + r.reaction(n1, Dof::Ux)? + p,
        0.0,
        1e-6,
        1e-9,
        "global equilibrium: Rx0 + Rx1 + P = 0",
    );
    Ok(())
}

#[test]
fn spring_approaches_fixed_as_stiffness_grows() -> Result<(), FemError> {
    let (l, p) = (2.0, 1.0e4);
    let k_bar = EA / l;
    let u_free = p / k_bar;

    for &k in &[1.0e8, 1.0e10, 1.0e12, 1.0e14] {
        let (f, _n0, n1, _m) = axial_bar_with_spring(l, k)?;
        let mut f2 = f.clone();
        f2.nodal_load(n1, p, 0.0)?;
        let r = f2.solve()?;
        let u = r.displacement(n1, Dof::Ux)?;
        assert_close(u, p / (k_bar + k), 1e-12, 1e-9, "spring displacement");
        assert!(
            u < u_free,
            "spring should reduce displacement below free value (k={k})"
        );
    }

    let (f, _n0, n1, _m) = axial_bar_with_spring(l, 1.0e18)?;
    let mut f2 = f.clone();
    f2.nodal_load(n1, p, 0.0)?;
    let r = f2.solve()?;
    assert!(
        r.displacement(n1, Dof::Ux)?.abs() < 1e-10,
        "very stiff spring → zero displacement"
    );

    let (f, _n0, n1, _m) = axial_bar_with_spring(l, 1.0e-3)?;
    let mut f2 = f.clone();
    f2.nodal_load(n1, p, 0.0)?;
    let r = f2.solve()?;
    assert_close(
        r.displacement(n1, Dof::Ux)?,
        u_free,
        1e-10,
        1e-9,
        "very soft spring → free displacement",
    );
    Ok(())
}

#[test]
fn spring_stiffness_validation() -> Result<(), FemError> {
    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(1.0, 0.0)?;
    f.add_member(n0, n1, steel(), sec())?;
    f.fix(n0)?;

    assert!(
        f.spring(n1, Dof::Ux, -1.0).is_err(),
        "negative stiffness rejected"
    );
    assert!(
        f.spring(n1, Dof::Ux, 0.0).is_err(),
        "zero stiffness rejected"
    );
    assert!(
        f.spring(n1, Dof::Ux, f64::NAN).is_err(),
        "NaN stiffness rejected"
    );
    assert!(
        f.spring(n1, Dof::Ux, f64::INFINITY).is_err(),
        "infinite stiffness rejected"
    );
    assert!(
        f.spring(n1, Dof::Ux, f64::NEG_INFINITY).is_err(),
        "neg-infinity rejected"
    );
    assert!(
        f.spring(n1, Dof::Ux, 1.0e6).is_ok(),
        "positive finite stiffness accepted"
    );
    Ok(())
}

#[test]
fn transverse_spring_on_cantilever() -> Result<(), FemError> {
    let (l, p, k) = (3.0, 5.0e3, 2.0e5);
    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(l, 0.0)?;
    f.add_member(n0, n1, steel(), sec())?;
    f.fix(n0)?;
    f.spring(n1, Dof::Uy, k)?;
    f.nodal_load(n1, 0.0, p)?;

    let r = f.solve()?;

    let k_cant = 3.0 * EI / l.powi(3);
    let u_exact = p / (k_cant + k);
    assert_close(
        r.displacement(n1, Dof::Uy)?,
        u_exact,
        1e-10,
        1e-9,
        "cantilever tip with spring: u = P / (3EI/L³ + k)",
    );

    let spring_reaction = -k * u_exact;
    assert_close(
        r.reaction(n1, Dof::Uy)?,
        spring_reaction,
        1e-6,
        1e-9,
        "spring reaction = -k * u",
    );
    Ok(())
}

#[test]
fn spring_reaction_equals_negative_k_times_u() -> Result<(), FemError> {
    let (l, p, k) = (2.0, 1.0e4, 3.0e6);
    let (f, _n0, n1, _m) = axial_bar_with_spring(l, k)?;
    let mut f2 = f.clone();
    f2.nodal_load(n1, p, 0.0)?;
    let r = f2.solve()?;

    let u = r.displacement(n1, Dof::Ux)?;
    let reaction = r.reaction(n1, Dof::Ux)?;
    assert_close(
        reaction,
        -k * u,
        1e-6,
        1e-9,
        "R = -k*u (spring restoring force)",
    );
    Ok(())
}

#[test]
fn multiple_springs_equilibrium() -> Result<(), FemError> {
    let (l, p) = (1.0, 1.0e4);
    let (k0, k1) = (2.0e6, 3.0e6);

    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(l, 0.0)?;
    let n2 = f.add_node(2.0 * l, 0.0)?;
    f.add_member(n0, n1, steel(), sec())?;
    f.add_member(n1, n2, steel(), sec())?;
    f.spring(n0, Dof::Ux, k0)?;
    f.spring(n2, Dof::Ux, k1)?;
    f.roller_y(n0)?;
    f.roller_y(n1)?;
    f.roller_y(n2)?;
    f.restrain(n1, Dof::Rz, 0.0)?;
    f.nodal_load(n1, p, 0.0)?;

    let r = f.solve()?;

    let u0 = r.displacement(n0, Dof::Ux)?;
    let u1 = r.displacement(n1, Dof::Ux)?;
    let u2 = r.displacement(n2, Dof::Ux)?;

    let r0 = r.reaction(n0, Dof::Ux)?;
    let r2 = r.reaction(n2, Dof::Ux)?;
    assert_close(r0, -k0 * u0, 1e-6, 1e-9, "spring 0: R = -k*u");
    assert_close(r2, -k1 * u2, 1e-6, 1e-9, "spring 2: R = -k*u");

    assert_close(
        r0 + r2 + p,
        0.0,
        1e-6,
        1e-9,
        "global equilibrium: R0 + R2 + P = 0",
    );

    assert!(u1 > 0.0, "node 1 should move in +x");
    assert!(u0 > 0.0, "node 0 should move in +x (spring allows)");
    assert!(u2 > 0.0, "node 2 should move in +x (spring allows)");
    Ok(())
}

#[test]
fn spring_with_distributed_load() -> Result<(), FemError> {
    let (l, q, k) = (4.0, 2.0e3, 1.0e6);

    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(l, 0.0)?;
    let m = f.add_member(n0, n1, steel(), sec())?;
    f.fix(n0)?;
    f.spring(n1, Dof::Uy, k)?;
    f.member_udl(m, 0.0, q)?;

    let r = f.solve()?;

    let u_tip = r.displacement(n1, Dof::Uy)?;
    assert!(u_tip > 0.0, "tip should deflect downward (positive qy)");

    let total_load = q * l;
    let r_fixed = r.reaction(n0, Dof::Uy)?;
    let r_spring = r.reaction(n1, Dof::Uy)?;
    assert_close(
        r_fixed + r_spring + total_load,
        0.0,
        1e-6,
        1e-9,
        "equilibrium: R_fixed + R_spring + q*L = 0",
    );

    assert_close(r_spring, -k * u_tip, 1e-6, 1e-9, "spring reaction = -k*u");
    Ok(())
}

#[test]
fn spring_does_not_overconstrain_dof() -> Result<(), FemError> {
    let l = 2.0;
    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(l, 0.0)?;
    f.add_member(n0, n1, steel(), sec())?;
    f.fix(n0)?;
    f.spring(n1, Dof::Ux, 1.0e6)?;
    f.spring(n1, Dof::Uy, 2.0e6)?;
    f.nodal_load(n1, 1.0e3, 2.0e3)?;

    let r = f.solve()?;
    let ux = r.displacement(n1, Dof::Ux)?;
    let uy = r.displacement(n1, Dof::Uy)?;
    assert!(
        ux.abs() > 1e-15,
        "Ux should be non-zero (spring allows motion)"
    );
    assert!(
        uy.abs() > 1e-15,
        "Uy should be non-zero (spring allows motion)"
    );
    assert_close(
        r.reaction(n1, Dof::Ux)?,
        -1.0e6 * ux,
        1e-6,
        1e-9,
        "Ux spring",
    );
    assert_close(
        r.reaction(n1, Dof::Uy)?,
        -2.0e6 * uy,
        1e-6,
        1e-9,
        "Uy spring",
    );
    Ok(())
}

#[test]
fn spring_in_rotation_dof() -> Result<(), FemError> {
    let l = 2.0;
    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(l, 0.0)?;
    f.add_member(n0, n1, steel(), sec())?;
    f.pin(n0)?;
    f.pin(n1)?;
    f.spring(n1, Dof::Rz, 5.0e4)?;
    f.nodal_moment(n1, 1.0e3)?;

    let r = f.solve()?;
    let theta = r.displacement(n1, Dof::Rz)?;
    assert!(theta.abs() > 1e-15, "rotation should be non-zero");

    let k_beam = 3.0 * EI / l;
    let theta_exact = 1.0e3 / (k_beam + 5.0e4);
    assert_close(
        theta,
        theta_exact,
        1e-10,
        1e-9,
        "rotational spring: theta = M / (3EI/L + k)",
    );

    assert_close(
        r.reaction(n1, Dof::Rz)?,
        -5.0e4 * theta,
        1e-6,
        1e-9,
        "spring moment = -k*theta",
    );
    Ok(())
}
