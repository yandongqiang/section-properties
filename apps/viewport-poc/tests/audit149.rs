//! Phase 149 audit — independent numerical cross-validation.
//!
//! Case C: 3D frame with non-default reference vector.

use structural_analysis::Dof3D;
use viewport_poc::{
    app_state::Project,
    edit_model::{EditModel, MemberId, NodalLoadSpec, SupportSpec},
    scene::ViewportScene,
};

/// Case A — Axial cantilever: u = FL/EA, R = -F.
#[test]
fn audit_case_a_axial_cantilever() {
    let e = 200e9;
    let g = 80e9;
    let area = 2e-3;
    let iy = 4e-6;
    let iz = 2e-6;
    let j = 6e-6;
    let l = 2.0;
    let f = 5000.0;

    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(l, 0.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    m.set_member_section(MemberId(0), e, g, area, iy, iz, j)
        .unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_nodal_load(
        b,
        NodalLoadSpec {
            fx: f,
            fy: 0.0,
            fz: 0.0,
            mx: 0.0,
            my: 0.0,
            mz: 0.0,
        },
    )
    .unwrap();

    let mut p = Project::empty();
    p.load_model("Axial".into(), m);
    p.solve().unwrap();

    let r = p.analysis_result().unwrap();
    let scene = ViewportScene::from_edit_model(p.model.as_ref().unwrap());
    let idx_b = scene.node_index(b).unwrap();
    let idx_a = scene.node_index(a).unwrap();

    let ux = r.displacement(idx_b, Dof3D::Ux).unwrap();
    let u_expected = f * l / (e * area);
    let rel_err = (ux - u_expected).abs() / u_expected.abs();
    assert!(
        rel_err < 1e-4,
        "Case A: ux={ux:.6e}, expected={u_expected:.6e}, rel_err={rel_err:.2e}"
    );

    let rx_a = r.reaction(idx_a, Dof3D::Ux).unwrap();
    assert!(
        (rx_a + f).abs() / f.abs() < 1e-4,
        "Case A: Rx_a={rx_a:.6e}, expected={:.6e}",
        -f
    );

    let f_local = r.member_end_forces(0).unwrap();
    let n_i = f_local[0];
    let n_j = f_local[6];
    assert!(
        (n_i + n_j).abs() < 1e-6,
        "Case A: axial equilibrium N_i+N_j={}",
        n_i + n_j
    );
    assert!(
        (n_j - f).abs() / f.abs() < 1e-4,
        "Case A: N_j={n_j:.6e}, expected={f:.6e}"
    );
}

/// Case B — Transverse cantilever: δ = FL³/3EI, R = -F, M = -FL.
#[test]
fn audit_case_b_transverse_cantilever() {
    let e = 200e9;
    let g = 80e9;
    let area = 2e-3;
    let iy = 4e-6;
    let iz = 2e-6;
    let j = 6e-6;
    let l = 2.0;
    let f = 5000.0;

    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(0.0, l, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    m.set_member_ref_vec(MemberId(0), [0.0, 0.0, 1.0]).unwrap();
    m.set_member_section(MemberId(0), e, g, area, iy, iz, j)
        .unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_nodal_load(
        b,
        NodalLoadSpec {
            fx: 0.0,
            fy: 0.0,
            fz: f,
            mx: 0.0,
            my: 0.0,
            mz: 0.0,
        },
    )
    .unwrap();

    let mut p = Project::empty();
    p.load_model("Transverse".into(), m);
    p.solve().unwrap();

    let r = p.analysis_result().unwrap();
    let scene = ViewportScene::from_edit_model(p.model.as_ref().unwrap());
    let idx_b = scene.node_index(b).unwrap();
    let idx_a = scene.node_index(a).unwrap();

    let uz = r.displacement(idx_b, Dof3D::Uz).unwrap();
    let delta_expected = f * l * l * l / (3.0 * e * iz);
    let rel_err = (uz - delta_expected).abs() / delta_expected.abs();
    assert!(
        rel_err < 1e-4,
        "Case B: uz={uz:.6e}, expected={delta_expected:.6e}, rel_err={rel_err:.2e}"
    );

    let rz_a = r.reaction(idx_a, Dof3D::Uz).unwrap();
    assert!(
        (rz_a + f).abs() / f.abs() < 1e-4,
        "Case B: Rz_a={rz_a:.6e}, expected={:.6e}",
        -f
    );

    let mx_a = r.reaction(idx_a, Dof3D::Rx).unwrap();
    let m_expected = -f * l;
    assert!(
        (mx_a - m_expected).abs() / m_expected.abs() < 1e-4,
        "Case B: Mx_a={mx_a:.6e}, expected={m_expected:.6e}"
    );
}

/// Case C — 3D frame with non-default reference vector.
///
/// Cantilever along X, ref_vec = [0, 1, 0] (swaps local y/z vs default).
/// Load in Z at free end. Verify:
/// 1. Local-axis orientation (local y = global Y, local z = global Z)
/// 2. Local/global end-force consistency via transformation
/// 3. Reaction equilibrium (ΣF = 0, ΣM = 0)
#[test]
fn audit_case_c_non_default_ref_vec() {
    let e = 200e9;
    let g = 80e9;
    let area = 2e-3;
    let iy = 4e-6;
    let iz = 2e-6;
    let j = 6e-6;
    let l = 1.5;
    let f = 3000.0;

    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(l, 0.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    m.set_member_ref_vec(MemberId(0), [0.0, 1.0, 0.0]).unwrap();
    m.set_member_section(MemberId(0), e, g, area, iy, iz, j)
        .unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_nodal_load(
        b,
        NodalLoadSpec {
            fx: 0.0,
            fy: 0.0,
            fz: f,
            mx: 0.0,
            my: 0.0,
            mz: 0.0,
        },
    )
    .unwrap();

    let mut p = Project::empty();
    p.load_model("RefVec".into(), m);
    p.solve().unwrap();

    let r = p.analysis_result().unwrap();
    let scene = ViewportScene::from_edit_model(p.model.as_ref().unwrap());
    let idx_a = scene.node_index(a).unwrap();
    let idx_b = scene.node_index(b).unwrap();

    // 1. Local-axis orientation
    let (lx, ly, lz) = r.member_local_axes(0).unwrap();
    // local x should be [1, 0, 0]
    assert!(
        (lx[0] - 1.0).abs() < 1e-10 && lx[1].abs() < 1e-10 && lx[2].abs() < 1e-10,
        "Case C: local x = {lx:?}, expected [1,0,0]"
    );
    // local y should be [0, 1, 0] (from ref_vec)
    assert!(
        (ly[0]).abs() < 1e-10 && (ly[1] - 1.0).abs() < 1e-10 && (ly[2]).abs() < 1e-10,
        "Case C: local y = {ly:?}, expected [0,1,0]"
    );
    // local z should be [0, 0, 1] (x × y)
    assert!(
        (lz[0]).abs() < 1e-10 && (lz[1]).abs() < 1e-10 && (lz[2] - 1.0).abs() < 1e-10,
        "Case C: local z = {lz:?}, expected [0,0,1]"
    );

    // 2. Local/global end-force consistency
    let f_local = r.member_end_forces(0).unwrap();
    let f_global = r.member_end_forces_global(0).unwrap();

    // Build rotation matrix [lx; ly; lz] (3x3)
    let rot = [
        [lx[0], lx[1], lx[2]],
        [ly[0], ly[1], ly[2]],
        [lz[0], lz[1], lz[2]],
    ];

    // For each end, transform local [N, Vy, Vz] to global [Fx, Fy, Fz]
    // and compare with f_global
    for end in 0..2 {
        let offset = end * 6;
        let local_force = [f_local[offset], f_local[offset + 1], f_local[offset + 2]];
        let global_expected = [f_global[offset], f_global[offset + 1], f_global[offset + 2]];

        // global = R^T * local (since R maps global to local)
        let global_computed = [
            rot[0][0] * local_force[0] + rot[1][0] * local_force[1] + rot[2][0] * local_force[2],
            rot[0][1] * local_force[0] + rot[1][1] * local_force[1] + rot[2][1] * local_force[2],
            rot[0][2] * local_force[0] + rot[1][2] * local_force[1] + rot[2][2] * local_force[2],
        ];

        for k in 0..3 {
            let diff = (global_computed[k] - global_expected[k]).abs();
            assert!(
                diff < 1e-6,
                "Case C: end {end}, dof {k}: transformed={:.6e}, global={:.6e}",
                global_computed[k],
                global_expected[k]
            );
        }
    }

    // 3. Reaction equilibrium: ΣR + ΣF_applied = 0
    let mut sum_fx = 0.0;
    let mut sum_fy = 0.0;
    let mut sum_fz = 0.0;
    for i in 0..scene.n_nodes() {
        sum_fx += r.reaction(i, Dof3D::Ux).unwrap_or(0.0);
        sum_fy += r.reaction(i, Dof3D::Uy).unwrap_or(0.0);
        sum_fz += r.reaction(i, Dof3D::Uz).unwrap_or(0.0);
    }
    // Applied load at node b: (0, 0, f)
    sum_fz += f;
    assert!(sum_fx.abs() < 1e-6, "Case C: ΣFx = {sum_fx:.6e}");
    assert!(sum_fy.abs() < 1e-6, "Case C: ΣFy = {sum_fy:.6e}");
    assert!(sum_fz.abs() < 1e-6, "Case C: ΣFz = {sum_fz:.6e}");

    // Moment equilibrium about node a: ΣM_a = 0
    // Reaction moment at a + moment from force at b (r_b × F_b)
    let mx_a = r.reaction(idx_a, Dof3D::Rx).unwrap();
    let my_a = r.reaction(idx_a, Dof3D::Ry).unwrap();
    let mz_a = r.reaction(idx_a, Dof3D::Rz).unwrap();
    // r_b = (l, 0, 0), F_b = (0, 0, f)
    // r_b × F_b = (0*f - 0*0, 0*0 - l*f, l*0 - 0*0) = (0, -l*f, 0)
    // M_reaction = -r_b × F_b = (0, l*f, 0)
    assert!((mx_a).abs() < 1e-6, "Case C: Mx_a = {mx_a:.6e}");
    assert!(
        (my_a - l * f).abs() < 1e-6,
        "Case C: My_a = {my_a:.6e}, expected={:.6e}",
        l * f
    );
    assert!((mz_a).abs() < 1e-6, "Case C: Mz_a = {mz_a:.6e}");

    // Also verify displacement is non-zero and in Z direction
    let uz = r.displacement(idx_b, Dof3D::Uz).unwrap();
    assert!(
        uz.abs() > 1e-10,
        "Case C: tip displacement should be non-zero, got {uz:.6e}"
    );
    let uy = r.displacement(idx_b, Dof3D::Uy).unwrap();
    assert!(
        uy.abs() < uz.abs() * 1e-6,
        "Case C: uy should be ~0, got {uy:.6e} vs uz={uz:.6e}"
    );
}

/// Case C extra — verify Iy/Iz usage with non-default ref vec.
///
/// With ref_vec = [0, 1, 0], local y = global Y, local z = global Z.
/// A Z-direction load bends about local y (Iy), NOT about local z (Iz).
/// δ = FL³ / (3 * E * Iy)
#[test]
fn audit_case_c_iy_iz_usage() {
    let e = 200e9;
    let g = 80e9;
    let area = 2e-3;
    let iy = 4e-6;
    let iz = 2e-6;
    let j = 6e-6;
    let l = 1.5;
    let f = 3000.0;

    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(l, 0.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    m.set_member_ref_vec(MemberId(0), [0.0, 1.0, 0.0]).unwrap();
    m.set_member_section(MemberId(0), e, g, area, iy, iz, j)
        .unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_nodal_load(
        b,
        NodalLoadSpec {
            fx: 0.0,
            fy: 0.0,
            fz: f,
            mx: 0.0,
            my: 0.0,
            mz: 0.0,
        },
    )
    .unwrap();

    let mut p = Project::empty();
    p.load_model("IyIz".into(), m);
    p.solve().unwrap();

    let r = p.analysis_result().unwrap();
    let scene = ViewportScene::from_edit_model(p.model.as_ref().unwrap());
    let idx_b = scene.node_index(b).unwrap();

    let uz = r.displacement(idx_b, Dof3D::Uz).unwrap();
    // With ref_vec=[0,1,0], local z = global Z. Load in Z = load in local z.
    // Bending about local y → uses Iy.
    let delta_expected = f * l * l * l / (3.0 * e * iy);
    let rel_err = (uz - delta_expected).abs() / delta_expected.abs();
    assert!(
        rel_err < 1e-4,
        "Case C Iy: uz={uz:.6e}, expected={delta_expected:.6e} (using Iy={iy}), rel_err={rel_err:.2e}"
    );

    // Verify it does NOT match Iz-based formula
    let delta_wrong = f * l * l * l / (3.0 * e * iz);
    let rel_err_wrong = (uz - delta_wrong).abs() / delta_wrong.abs();
    assert!(
        rel_err_wrong > 0.1,
        "Case C Iy: uz should NOT match Iz-based formula, but rel_err={rel_err_wrong:.2e}"
    );
}
