//! Verify deformation display scaling does not modify the stored analysis results.

use structural_analysis::Dof3D;
use viewport_poc::{model, scene::ViewportScene};

#[test]
fn deformed_positions_match_formula() {
    let m = model::build_portal_frame();
    let result = m.solve().expect("portal frame solves");
    let scene = ViewportScene::from_result(&result);
    let scale = 50.0;

    let deformed = scene.deformed_positions(&result.displacements, scale);

    for i in 0..scene.n_nodes() {
        let orig = scene.node_positions[i];
        let ux = result.displacement(i, Dof3D::Ux).unwrap();
        let uy = result.displacement(i, Dof3D::Uy).unwrap();
        let uz = result.displacement(i, Dof3D::Uz).unwrap();

        assert!(
            (deformed[i][0] - (orig[0] + scale * ux)).abs() < 1e-10,
            "x at node {i}"
        );
        assert!(
            (deformed[i][1] - (orig[1] + scale * uy)).abs() < 1e-10,
            "y at node {i}"
        );
        assert!(
            (deformed[i][2] - (orig[2] + scale * uz)).abs() < 1e-10,
            "z at node {i}"
        );
    }
}

#[test]
fn zero_scale_gives_original() {
    let m = model::build_portal_frame();
    let result = m.solve().expect("portal frame solves");
    let scene = ViewportScene::from_result(&result);

    let deformed = scene.deformed_positions(&result.displacements, 0.0);
    assert_eq!(deformed, scene.node_positions);
}

#[test]
fn scaling_does_not_modify_displacements() {
    let m = model::build_portal_frame();
    let result = m.solve().expect("portal frame solves");
    let scene = ViewportScene::from_result(&result);

    let disp_before = result.displacements.clone();
    let _ = scene.deformed_positions(&result.displacements, 999.0);
    assert_eq!(
        result.displacements, disp_before,
        "displacements must not be modified"
    );
}

#[test]
fn scaling_does_not_modify_scene_positions() {
    let m = model::build_portal_frame();
    let result = m.solve().expect("portal frame solves");
    let scene = ViewportScene::from_result(&result);

    let pos_before = scene.node_positions.clone();
    let _ = scene.deformed_positions(&result.displacements, 999.0);
    assert_eq!(
        scene.node_positions, pos_before,
        "scene positions must not be modified"
    );
}

#[test]
fn different_scales_give_different_positions() {
    let m = model::build_portal_frame();
    let result = m.solve().expect("portal frame solves");
    let scene = ViewportScene::from_result(&result);

    let d1 = scene.deformed_positions(&result.displacements, 10.0);
    let d2 = scene.deformed_positions(&result.displacements, 100.0);

    // At least one node should have different positions (the loaded node)
    let mut any_diff = false;
    for i in 0..scene.n_nodes() {
        if d1[i] != d2[i] {
            any_diff = true;
            break;
        }
    }
    assert!(
        any_diff,
        "different scales should produce different deformed shapes"
    );
}

#[test]
fn deformed_positions_with_inclined_frame() {
    let m = model::build_inclined_frame();
    let result = m.solve().expect("inclined frame solves");
    let scene = ViewportScene::from_result(&result);

    let deformed = scene.deformed_positions(&result.displacements, 100.0);
    assert_eq!(deformed.len(), scene.n_nodes());

    // Verify formula for each node
    for i in 0..scene.n_nodes() {
        let orig = scene.node_positions[i];
        let ux = result.displacement(i, Dof3D::Ux).unwrap();
        let uy = result.displacement(i, Dof3D::Uy).unwrap();
        let uz = result.displacement(i, Dof3D::Uz).unwrap();
        assert!((deformed[i][0] - (orig[0] + 100.0 * ux)).abs() < 1e-10);
        assert!((deformed[i][1] - (orig[1] + 100.0 * uy)).abs() < 1e-10);
        assert!((deformed[i][2] - (orig[2] + 100.0 * uz)).abs() < 1e-10);
    }
}
