//! Phase 148 — Result visualization integration tests.
//!
//! Covers display-mode behavior, result validity (Policy A), stable-ID
//! mapping, and analytical verification of two classic cantilever cases.

use structural_analysis::Dof3D;
use viewport_poc::{
    app_state::{AnalysisStatus, Project},
    display::{
        ResultDisplayState, ResultViewMode, displacement_magnitude, displacement_magnitudes,
        fmt_eng, magnitude_color, max_abs, normalize,
    },
    edit_model::{EditModel, NodalLoadSpec, NodeId, SupportSpec},
    model,
    scene::ViewportScene,
};

// ── 1. Default model display ─────────────────────────────────────────

#[test]
fn default_mode_is_model() {
    let d = ResultDisplayState::default();
    assert_eq!(d.mode, ResultViewMode::Model);
    assert!(!d.mode.requires_result());
}

// ── 2. Mode switching ────────────────────────────────────────────────

#[test]
fn mode_switching() {
    let mut d = ResultDisplayState::default();
    assert_eq!(d.mode, ResultViewMode::Model);

    d.mode = ResultViewMode::Deformed;
    assert!(d.mode.requires_result());

    d.mode = ResultViewMode::Displacement;
    assert!(d.mode.requires_result());

    d.mode = ResultViewMode::Reaction;
    assert!(d.mode.requires_result());

    d.mode = ResultViewMode::MemberEndForces;
    assert!(d.mode.requires_result());

    d.mode = ResultViewMode::Model;
    assert!(!d.mode.requires_result());
}

// ── 3. Deformation amplification ────────────────────────────────────

#[test]
fn deformation_amplification_scales_linearly() {
    let scene = ViewportScene {
        node_positions: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
        node_ids: vec![NodeId(0), NodeId(1)],
        member_nodes: vec![(0, 1)],
        member_ids: vec![viewport_poc::edit_model::MemberId(0)],
        member_ref_vecs: vec![[0.0, 0.0, 1.0]],
        node_supports: vec![None, None],
        node_loads: vec![None, None],
        member_loads: vec![None],
        member_point_loads: vec![vec![]],
    };
    let disp = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0];

    let d0 = scene.deformed_positions(&disp, 0.0);
    let d1 = scene.deformed_positions(&disp, 1.0);
    let d2 = scene.deformed_positions(&disp, 2.0);

    assert_eq!(d0[1], [1.0, 0.0, 0.0]);
    assert_eq!(d1[1], [2.0, 0.0, 0.0]);
    assert_eq!(d2[1], [3.0, 0.0, 0.0]);
}

// ── 7. Reaction sign preservation ───────────────────────────────────

#[test]
fn reaction_sign_preserved() {
    let mut p = Project::empty();
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(1.0, 0.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_nodal_load(
        b,
        NodalLoadSpec {
            fx: 1000.0,
            fy: 0.0,
            fz: 0.0,
            mx: 0.0,
            my: 0.0,
            mz: 0.0,
        },
    )
    .unwrap();
    p.load_model("Cantilever".into(), m);
    p.solve().unwrap();

    let r = p.analysis_result().unwrap();
    let scene = ViewportScene::from_edit_model(p.model.as_ref().unwrap());
    let idx_a = scene.node_index(a).unwrap();

    let rx_a = r.reaction(idx_a, Dof3D::Ux).unwrap();
    assert!(
        rx_a < 0.0,
        "reaction at fixed end should be negative (opposing load), got {rx_a}"
    );
    assert!(
        (rx_a + 1000.0).abs() < 1e-6,
        "reaction should be -1000, got {rx_a}"
    );
}

// ── 8. Member local end-force ordering ───────────────────────────────

#[test]
fn member_end_force_ordering() {
    let mut p = Project::empty();
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(1.0, 0.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_nodal_load(
        b,
        NodalLoadSpec {
            fx: 1000.0,
            fy: 0.0,
            fz: 0.0,
            mx: 0.0,
            my: 0.0,
            mz: 0.0,
        },
    )
    .unwrap();
    p.load_model("Cantilever".into(), m);
    p.solve().unwrap();

    let r = p.analysis_result().unwrap();
    let scene = ViewportScene::from_edit_model(p.model.as_ref().unwrap());
    let mi = scene
        .member_index(viewport_poc::edit_model::MemberId(0))
        .unwrap();
    let f = r.member_end_forces(mi).unwrap();

    assert_eq!(f.len(), 12, "12 local end-force values");
    let n_i = f[0];
    let n_j = f[6];
    assert!((n_i + n_j).abs() < 1e-6, "axial equilibrium: N_i + N_j = 0");
}

// ── 9. Local/global force distinction ────────────────────────────────

#[test]
fn local_global_force_distinction() {
    let mut p = Project::empty();
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(1.0, 1.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_nodal_load(
        b,
        NodalLoadSpec {
            fx: 1000.0,
            fy: 0.0,
            fz: 0.0,
            mx: 0.0,
            my: 0.0,
            mz: 0.0,
        },
    )
    .unwrap();
    p.load_model("Inclined".into(), m);
    p.solve().unwrap();

    let r = p.analysis_result().unwrap();
    let scene = ViewportScene::from_edit_model(p.model.as_ref().unwrap());
    let mi = scene
        .member_index(viewport_poc::edit_model::MemberId(0))
        .unwrap();

    let f_local = r.member_end_forces(mi).unwrap();
    let f_global = r.member_end_forces_global(mi).unwrap();

    let any_differs = (0..12).any(|i| (f_local[i] - f_global[i]).abs() > 1e-10);
    assert!(
        any_differs,
        "local and global forces must differ for non-axis-aligned member"
    );
}

// ── 10. Stable-ID result mapping ─────────────────────────────────────

#[test]
fn stable_id_mapping_after_delete_and_reinsert() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(1.0, 0.0, 0.0).unwrap();
    let c = m.add_node(2.0, 0.0, 0.0).unwrap();
    let _mbc = m.add_member(b, c).unwrap();
    m.add_member(a, b).unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(c, SupportSpec::fixed()).unwrap();

    let scene_before = ViewportScene::from_edit_model(&m);
    assert_eq!(scene_before.node_index(c), Some(2));

    m.delete_member(_mbc).unwrap();
    m.delete_node(c).unwrap();
    let scene_after_delete = ViewportScene::from_edit_model(&m);
    assert_eq!(scene_after_delete.node_index(c), None);

    let c2 = m.add_node(2.0, 0.0, 0.0).unwrap();
    let scene_after_reinsert = ViewportScene::from_edit_model(&m);
    assert_eq!(scene_after_reinsert.node_index(c2), Some(2));
    assert_ne!(c2, c, "reinserted node must have a different stable ID");
}

#[test]
fn stable_id_mapping_non_contiguous() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(1.0, 0.0, 0.0).unwrap();
    let c = m.add_node(2.0, 0.0, 0.0).unwrap();
    m.delete_node(b).unwrap();

    let scene = ViewportScene::from_edit_model(&m);
    assert_eq!(scene.node_ids, vec![a, c]);
    assert_eq!(scene.node_index(a), Some(0));
    assert_eq!(scene.node_index(c), Some(1));
    assert_eq!(scene.node_index(b), None);
}

// ── 11. Stale-result hiding ──────────────────────────────────────────

#[test]
fn stale_result_hidden_by_policy_a() {
    let mut p = Project::empty();
    p.load_model("Portal".into(), model::build_portal_edit());
    p.solve().unwrap();
    assert!(p.analysis_result().is_some());

    let (id, x, y, z) = {
        let n = &p.model.as_ref().unwrap().nodes()[0];
        (n.id, n.x, n.y, n.z)
    };
    p.model
        .as_mut()
        .unwrap()
        .update_node(id, x + 0.5, y, z)
        .unwrap();
    assert_eq!(p.analysis_status(), AnalysisStatus::Stale);
    assert!(
        p.analysis_result().is_none(),
        "Policy A must hide stale results"
    );
}

// ── 12. Failed-result hiding ─────────────────────────────────────────

#[test]
fn failed_result_hidden_by_policy_a() {
    let mut p = Project::empty();
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(1.0, 0.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    p.load_model("Mechanism".into(), m);
    assert!(p.solve().is_err());
    assert_eq!(p.analysis_status(), AnalysisStatus::Failed);
    assert!(
        p.analysis_result().is_none(),
        "Policy A must hide failed results"
    );
}

// ── 13. Successful re-solve refresh ──────────────────────────────────

#[test]
fn re_solve_refreshes_result() {
    let mut p = Project::empty();
    p.load_model("Portal".into(), model::build_portal_edit());
    p.solve().unwrap();
    let r1 = p.analysis_result().unwrap().displacements.clone();

    let (id, x, y, z) = {
        let n = &p.model.as_ref().unwrap().nodes()[0];
        (n.id, n.x, n.y, n.z)
    };
    p.model
        .as_mut()
        .unwrap()
        .update_node(id, x + 0.001, y, z)
        .unwrap();
    assert!(p.analysis_result().is_none());
    p.solve().unwrap();
    let r2 = p.analysis_result().unwrap().displacements.clone();

    assert_ne!(r1, r2, "re-solve after edit must produce different results");
}

// ── 14. Preset compatibility ────────────────────────────────────────

#[test]
fn all_presets_produce_accessible_results() {
    for &preset in model::PresetModel::all() {
        let mut p = Project::empty();
        p.load_model(preset.name().into(), preset.build_edit());
        p.solve()
            .unwrap_or_else(|e| panic!("preset '{}' solve failed: {e}", preset.name()));
        assert_eq!(p.analysis_status(), AnalysisStatus::Current);
        let r = p.analysis_result().unwrap();
        assert!(
            !r.displacements.is_empty(),
            "preset '{}' has no displacements",
            preset.name()
        );
    }
}

// ── 15. Uniform-value legend normalization ──────────────────────────

#[test]
fn uniform_value_normalization() {
    let values = vec![5.0, 5.0, 5.0, 5.0];
    let max = max_abs(&values);
    assert_eq!(max, 5.0);
    for &v in &values {
        assert!((normalize(v, max) - 1.0).abs() < 1e-15);
    }
    let [r, g, _b] = magnitude_color(1.0);
    assert_eq!(r, 255);
    assert_eq!(g, 80);
}

// ── 16. Non-finite visualization input handling ─────────────────────

#[test]
fn non_finite_inputs_handled() {
    assert_eq!(displacement_magnitude(f64::NAN, 1.0, 1.0), 0.0);
    assert_eq!(displacement_magnitude(1.0, f64::INFINITY, 1.0), 0.0);
    assert_eq!(max_abs(&[f64::NAN, 1.0, f64::NEG_INFINITY]), 1.0);
    assert_eq!(normalize(f64::NAN, 1.0), 0.0);
    assert_eq!(fmt_eng(f64::NAN), "NaN");
}

// ── Analytical Case A: Axial cantilever u = FL/EA ───────────────────

#[test]
fn analytical_axial_cantilever() {
    let e = 200e9;
    let g = 80e9;
    let area = 1e-3;
    let iy = 2e-6;
    let iz = 1e-6;
    let j = 3e-6;
    let l = 1.0;
    let f = 1000.0;

    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(l, 0.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    m.set_member_section(viewport_poc::edit_model::MemberId(0), e, g, area, iy, iz, j)
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
    p.load_model("AxialCantilever".into(), m);
    p.solve().unwrap();

    let r = p.analysis_result().unwrap();
    let scene = ViewportScene::from_edit_model(p.model.as_ref().unwrap());
    let idx_b = scene.node_index(b).unwrap();

    let ux = r.displacement(idx_b, Dof3D::Ux).unwrap();
    let u_expected = f * l / (e * area);
    assert!(
        (ux - u_expected).abs() / u_expected.abs() < 1e-4,
        "axial: ux={ux:.6e}, expected={u_expected:.6e}"
    );

    let rx_a = r.reaction(scene.node_index(a).unwrap(), Dof3D::Ux).unwrap();
    assert!(
        (rx_a + f).abs() / f.abs() < 1e-4,
        "reaction: Rx_a={rx_a:.6e}, expected={:.6e}",
        -f
    );
}

// ── Analytical Case B: Transverse cantilever δ = FL³/3EI ────────────

#[test]
fn analytical_transverse_cantilever() {
    let e = 200e9;
    let g = 80e9;
    let area = 1e-3;
    let iy = 2e-6;
    let iz = 1e-6;
    let j = 3e-6;
    let l = 1.0;
    let f = 1000.0;

    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(0.0, l, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    m.set_member_ref_vec(viewport_poc::edit_model::MemberId(0), [0.0, 0.0, 1.0])
        .unwrap();
    m.set_member_section(viewport_poc::edit_model::MemberId(0), e, g, area, iy, iz, j)
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
    p.load_model("TransverseCantilever".into(), m);
    p.solve().unwrap();

    let r = p.analysis_result().unwrap();
    let scene = ViewportScene::from_edit_model(p.model.as_ref().unwrap());
    let idx_b = scene.node_index(b).unwrap();

    let uz = r.displacement(idx_b, Dof3D::Uz).unwrap();
    let delta_expected = f * l * l * l / (3.0 * e * iz);
    assert!(
        (uz - delta_expected).abs() / delta_expected.abs() < 1e-4,
        "transverse: uz={uz:.6e}, expected={delta_expected:.6e}"
    );

    let rz_a = r.reaction(scene.node_index(a).unwrap(), Dof3D::Uz).unwrap();
    assert!(
        (rz_a + f).abs() / f.abs() < 1e-4,
        "reaction: Rz_a={rz_a:.6e}, expected={:.6e}",
        -f
    );

    let mx_a = r.reaction(scene.node_index(a).unwrap(), Dof3D::Rx).unwrap();
    let m_expected = -f * l;
    assert!(
        (mx_a - m_expected).abs() / m_expected.abs() < 1e-4,
        "moment: Mx_a={mx_a:.6e}, expected={m_expected:.6e}"
    );
}

// ── Displacement magnitude from solved result ────────────────────────

#[test]
fn solved_displacement_magnitude_consistent() {
    let mut p = Project::empty();
    p.load_model("Portal".into(), model::build_portal_edit());
    p.solve().unwrap();
    let r = p.analysis_result().unwrap();
    let scene = ViewportScene::from_edit_model(p.model.as_ref().unwrap());

    let mags = displacement_magnitudes(&r.displacements, scene.n_nodes());
    assert_eq!(mags.len(), scene.n_nodes());

    for (i, &mag) in mags.iter().enumerate() {
        let ux = r.displacements[i * 6];
        let uy = r.displacements[i * 6 + 1];
        let uz = r.displacements[i * 6 + 2];
        let expected = (ux * ux + uy * uy + uz * uz).sqrt();
        assert!(
            (mag - expected).abs() < 1e-15,
            "magnitude mismatch at node {i}"
        );
    }
}
