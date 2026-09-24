//! Phase 93 - Trapezoidal distributed load tests.

use section_properties::Material;
use structural_analysis::frame::{FrameModel, MemberHandle, NodeHandle};
use structural_analysis::load::{LoadCase, LoadCombination};
use structural_analysis::{BeamSection, Dof, FemError};

const E: f64 = 200e9;
const A: f64 = 5e-3;
const I: f64 = 2e-5;
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
fn trapezoidal_uniform_matches_udl() -> Result<(), FemError> {
    let (l, qy) = (4.0, 2.0e3);

    let mut f_ref = FrameModel::new();
    let n0 = f_ref.add_node(0.0, 0.0)?;
    let n1 = f_ref.add_node(l, 0.0)?;
    let m = f_ref.add_member(n0, n1, steel(), sec())?;
    f_ref.pin(n0)?;
    f_ref.pin(n1)?;
    f_ref.member_udl(m, 0.0, qy)?;
    let r_ref = f_ref.solve()?;

    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(l, 0.0)?;
    let m = f.add_member(n0, n1, steel(), sec())?;
    f.pin(n0)?;
    f.pin(n1)?;
    f.member_trapezoidal(m, 0.0, qy, 0.0, qy)?;
    let r = f.solve()?;

    for &ni in &[n0, n1] {
        for dof in [Dof::Ux, Dof::Uy, Dof::Rz] {
            assert_close(
                r.displacement(ni, dof)?,
                r_ref.displacement(ni, dof)?,
                1e-14,
                1e-12,
                "trapezoidal(uniform) matches UDL",
            );
        }
    }
    Ok(())
}

#[test]
fn triangular_load_simply_supported_reactions() -> Result<(), FemError> {
    let (l, q) = (6.0, 3.0e3);

    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(l, 0.0)?;
    let m = f.add_member(n0, n1, steel(), sec())?;
    f.pin(n0)?;
    f.pin(n1)?;
    f.member_trapezoidal(m, 0.0, q, 0.0, 0.0)?;

    let r = f.solve()?;

    let r_left = -q * l / 3.0;
    let r_right = -q * l / 6.0;
    let total = q * l / 2.0;

    assert_close(
        r.reaction(n0, Dof::Uy)?,
        r_left,
        1e-6,
        1e-9,
        "R_left = -qL/3 (support reaction, downward for upward load)",
    );
    assert_close(
        r.reaction(n1, Dof::Uy)?,
        r_right,
        1e-6,
        1e-9,
        "R_right = -qL/6",
    );
    assert_close(
        r.reaction(n0, Dof::Uy)? + r.reaction(n1, Dof::Uy)? + total,
        0.0,
        1e-6,
        1e-9,
        "total reaction + total load = 0",
    );
    Ok(())
}

#[test]
fn trapezoidal_total_load_equilibrium() -> Result<(), FemError> {
    let (l, q_start, q_end) = (5.0, 4.0e3, 1.0e3);

    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(l, 0.0)?;
    let m = f.add_member(n0, n1, steel(), sec())?;
    f.fix(n0)?;
    f.pin(n1)?;
    f.member_trapezoidal(m, 0.0, q_start, 0.0, q_end)?;

    let r = f.solve()?;

    let total_load = (q_start + q_end) * l / 2.0;
    let r0 = r.reaction(n0, Dof::Uy)?;
    let r1 = r.reaction(n1, Dof::Uy)?;
    assert_close(
        r0 + r1 + total_load,
        0.0,
        1e-6,
        1e-9,
        "equilibrium: R0 + R1 + total_load = 0",
    );
    Ok(())
}

#[test]
fn trapezoidal_axial_load_equilibrium() -> Result<(), FemError> {
    let (l, qx_start, qx_end) = (3.0, 2.0e3, 5.0e3);

    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(l, 0.0)?;
    let m = f.add_member(n0, n1, steel(), sec())?;
    f.fix(n0)?;
    f.roller_y(n1)?;
    f.member_trapezoidal(m, qx_start, 0.0, qx_end, 0.0)?;

    let r = f.solve()?;

    let total_axial = (qx_start + qx_end) * l / 2.0;
    let r0 = r.reaction(n0, Dof::Ux)?;
    let r1 = r.reaction(n1, Dof::Ux)?;
    assert_close(
        r0 + r1 + total_axial,
        0.0,
        1e-6,
        1e-9,
        "axial equilibrium: R0 + R1 + total_axial = 0",
    );
    Ok(())
}

#[test]
fn trapezoidal_section_forces_satisfy_equilibrium() -> Result<(), FemError> {
    let (l, q_start, q_end) = (4.0, 3.0e3, 1.0e3);

    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(l, 0.0)?;
    let m = f.add_member(n0, n1, steel(), sec())?;
    f.pin(n0)?;
    f.pin(n1)?;
    f.member_trapezoidal(m, 0.0, q_start, 0.0, q_end)?;

    let r = f.solve()?;

    let s0 = r.section_forces(m, 0.0)?;
    let v_i = -s0.shear;
    let m_i = s0.moment;

    let n_steps = 20;
    for k in 0..=n_steps {
        let xi = k as f64 / n_steps as f64;
        let s = r.section_forces(m, xi)?;

        let x = xi * l;
        let v_expected = -v_i + q_start * x + (q_end - q_start) * x * x / (2.0 * l);
        let m_expected =
            m_i - x * v_i + q_start * x * x / 2.0 + (q_end - q_start) * x * x * x / (6.0 * l);

        assert_close(s.shear, v_expected, 1e-3, 1e-6, "V(xi) equilibrium");
        assert_close(s.moment, m_expected, 1e-2, 1e-6, "M(xi) equilibrium");
    }

    let xi_mid = 0.5;
    let s_mid = r.section_forces(m, xi_mid)?;
    let q_mid = q_start + (q_end - q_start) * xi_mid;
    let h = 1e-6;
    let s_plus = r.section_forces(m, xi_mid + h)?;
    let s_minus = r.section_forces(m, xi_mid - h)?;
    let dV_dx = (s_plus.shear - s_minus.shear) / (2.0 * h * l);
    let dM_dx = (s_plus.moment - s_minus.moment) / (2.0 * h * l);
    assert_close(dV_dx, q_mid, 1.0, 1e-3, "dV/dx = q(x) at mid");
    assert_close(dM_dx, s_mid.shear, 1.0, 1e-3, "dM/dx = V at mid");
    Ok(())
}

#[test]
fn trapezoidal_in_load_case() -> Result<(), FemError> {
    let (l, q_start, q_end) = (4.0, 2.0e3, 0.0);

    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(l, 0.0)?;
    let m = f.add_member(n0, n1, steel(), sec())?;
    f.pin(n0)?;
    f.pin(n1)?;

    let mut case = LoadCase::new("triangular");
    case.member_trapezoidal(m, 0.0, q_start, 0.0, q_end)?;

    let r = f.solve_case(&case)?;

    let total_load = (q_start + q_end) * l / 2.0;
    assert_close(
        r.reaction(n0, Dof::Uy)? + r.reaction(n1, Dof::Uy)? + total_load,
        0.0,
        1e-6,
        1e-9,
        "LoadCase trapezoidal equilibrium",
    );

    let r_left = -q_start * l / 3.0;
    assert_close(
        r.reaction(n0, Dof::Uy)?,
        r_left,
        1e-6,
        1e-9,
        "LoadCase triangular R_left = -qL/3",
    );
    Ok(())
}

#[test]
fn trapezoidal_validation() -> Result<(), FemError> {
    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(1.0, 0.0)?;
    let m = f.add_member(n0, n1, steel(), sec())?;

    assert!(
        f.member_trapezoidal(m, f64::NAN, 0.0, 0.0, 0.0).is_err(),
        "NaN qx rejected"
    );
    assert!(
        f.member_trapezoidal(m, 0.0, 0.0, f64::INFINITY, 0.0)
            .is_err(),
        "infinite qx_end rejected"
    );
    assert!(
        f.member_trapezoidal(m, 1.0, 2.0, 3.0, 4.0).is_ok(),
        "valid trapezoidal accepted"
    );
    Ok(())
}

#[test]
fn trapezoidal_reversed_triangle() -> Result<(), FemError> {
    let (l, q) = (6.0, 3.0e3);

    let mut f1 = FrameModel::new();
    let n0 = f1.add_node(0.0, 0.0)?;
    let n1 = f1.add_node(l, 0.0)?;
    let m = f1.add_member(n0, n1, steel(), sec())?;
    f1.pin(n0)?;
    f1.pin(n1)?;
    f1.member_trapezoidal(m, 0.0, q, 0.0, 0.0)?;
    let r1 = f1.solve()?;

    let mut f2 = FrameModel::new();
    let n0 = f2.add_node(0.0, 0.0)?;
    let n1 = f2.add_node(l, 0.0)?;
    let m = f2.add_member(n0, n1, steel(), sec())?;
    f2.pin(n0)?;
    f2.pin(n1)?;
    f2.member_trapezoidal(m, 0.0, 0.0, 0.0, q)?;
    let r2 = f2.solve()?;

    assert_close(
        r1.reaction(n0, Dof::Uy)?,
        r2.reaction(n1, Dof::Uy)?,
        1e-6,
        1e-9,
        "reversed triangle: R_left(q,0) = R_right(0,q)",
    );
    assert_close(
        r1.reaction(n1, Dof::Uy)?,
        r2.reaction(n0, Dof::Uy)?,
        1e-6,
        1e-9,
        "reversed triangle: R_right(q,0) = R_left(0,q)",
    );
    Ok(())
}

#[test]
fn trapezoidal_superposition() -> Result<(), FemError> {
    let (l, q_start, q_end) = (5.0, 4.0e3, 1.0e3);

    let mut f_combined = FrameModel::new();
    let n0 = f_combined.add_node(0.0, 0.0)?;
    let n1 = f_combined.add_node(l, 0.0)?;
    let m = f_combined.add_member(n0, n1, steel(), sec())?;
    f_combined.pin(n0)?;
    f_combined.pin(n1)?;
    f_combined.member_trapezoidal(m, 0.0, q_start, 0.0, q_end)?;
    let r_combined = f_combined.solve()?;

    let mut f_uniform = FrameModel::new();
    let n0 = f_uniform.add_node(0.0, 0.0)?;
    let n1 = f_uniform.add_node(l, 0.0)?;
    let m = f_uniform.add_member(n0, n1, steel(), sec())?;
    f_uniform.pin(n0)?;
    f_uniform.pin(n1)?;
    let q_avg = (q_start + q_end) / 2.0;
    f_uniform.member_udl(m, 0.0, q_avg)?;
    let r_uniform = f_uniform.solve()?;

    let mut f_triangular = FrameModel::new();
    let n0 = f_triangular.add_node(0.0, 0.0)?;
    let n1 = f_triangular.add_node(l, 0.0)?;
    let m = f_triangular.add_member(n0, n1, steel(), sec())?;
    f_triangular.pin(n0)?;
    f_triangular.pin(n1)?;
    let q_diff = (q_start - q_end) / 2.0;
    f_triangular.member_trapezoidal(m, 0.0, q_diff, 0.0, -q_diff)?;
    let r_triangular = f_triangular.solve()?;

    for &ni in &[n0, n1] {
        for dof in [Dof::Ux, Dof::Uy, Dof::Rz] {
            let sum = r_uniform.displacement(ni, dof)? + r_triangular.displacement(ni, dof)?;
            assert_close(
                r_combined.displacement(ni, dof)?,
                sum,
                1e-10,
                1e-9,
                "trapezoidal = uniform(avg) + triangular(diff)",
            );
        }
    }
    Ok(())
}

#[test]
fn trapezoidal_load_in_combination_preserves_variation() -> Result<(), FemError> {
    let l = 6.0;
    let q_start = -3000.0;
    let q_end = -1000.0;

    let mut frame = FrameModel::new();
    let n0 = frame.add_node(0.0, 0.0)?;
    let n1 = frame.add_node(l, 0.0)?;
    let m = frame.add_member(n0, n1, steel(), sec())?;
    frame.fix(n0)?;
    frame.fix(n1)?;

    let mut case = LoadCase::new("trap");
    case.member_trapezoidal(m, 0.0, q_start, 0.0, q_end)?;

    let mut combo = LoadCombination::new("1.0*trap");
    combo.add_case(&case, 1.0)?;
    let r_combo = frame.solve_combination(&combo)?;

    let mut frame_direct = FrameModel::new();
    let n0d = frame_direct.add_node(0.0, 0.0)?;
    let n1d = frame_direct.add_node(l, 0.0)?;
    let md = frame_direct.add_member(n0d, n1d, steel(), sec())?;
    frame_direct.fix(n0d)?;
    frame_direct.fix(n1d)?;
    frame_direct.member_trapezoidal(md, 0.0, q_start, 0.0, q_end)?;
    let r_direct = frame_direct.solve()?;

    for &xi in &[0.0, 0.25, 0.5, 0.75, 1.0] {
        let s_combo = r_combo.section_forces(m, xi)?;
        let s_direct = r_direct.section_forces(md, xi)?;
        assert_close(
            s_combo.axial,
            s_direct.axial,
            1e-6,
            1e-9,
            "combo vs direct axial",
        );
        assert_close(
            s_combo.shear,
            s_direct.shear,
            1e-6,
            1e-9,
            "combo vs direct shear",
        );
        assert_close(
            s_combo.moment,
            s_direct.moment,
            1e-6,
            1e-9,
            "combo vs direct moment",
        );
    }

    let eq = r_combo.equilibrium();
    assert!(
        eq.is_balanced(),
        "equilibrium should be balanced for trapezoidal in combination: {eq:?}"
    );
    Ok(())
}

#[test]
fn trapezoidal_load_equilibrium_balanced() -> Result<(), FemError> {
    let l = 5.0;

    let mut frame = FrameModel::new();
    let n0 = frame.add_node(0.0, 0.0)?;
    let n1 = frame.add_node(l, 0.0)?;
    let m = frame.add_member(n0, n1, steel(), sec())?;
    frame.fix(n0)?;
    frame.fix(n1)?;

    frame.member_trapezoidal(m, 0.0, -4000.0, 0.0, -1000.0)?;
    let result = frame.solve()?;
    let eq = result.equilibrium();
    assert!(
        eq.is_balanced(),
        "equilibrium should be balanced for trapezoidal load: {eq:?}"
    );
    Ok(())
}
