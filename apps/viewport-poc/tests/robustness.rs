//! Robustness: non-finite data, empty models, zero-length display bounds.

use viewport_poc::{camera::Camera3D, project, scene::ViewportScene, selection::SelectionState};

#[test]
fn empty_scene_bounding_box() {
    let scene = ViewportScene {
        node_positions: vec![],
        member_nodes: vec![],
        member_ref_vecs: vec![],
    };
    let (min, max) = scene.bounding_box();
    assert_eq!(min, [0.0; 3]);
    assert_eq!(max, [0.0; 3]);
}

#[test]
fn empty_scene_deformed_positions() {
    let scene = ViewportScene {
        node_positions: vec![],
        member_nodes: vec![],
        member_ref_vecs: vec![],
    };
    let deformed = scene.deformed_positions(&[], 100.0);
    assert!(deformed.is_empty());
}

#[test]
fn single_node_scene() {
    let scene = ViewportScene {
        node_positions: vec![[1.0, 2.0, 3.0]],
        member_nodes: vec![],
        member_ref_vecs: vec![],
    };
    let (min, max) = scene.bounding_box();
    assert_eq!(min, [1.0, 2.0, 3.0]);
    assert_eq!(max, [1.0, 2.0, 3.0]);

    let cam = Camera3D::fit(min, max);
    let p = cam.project([1.0, 2.0, 3.0], 800.0, 600.0);
    assert!(p.is_some(), "single node should be visible");
}

#[test]
fn coincident_nodes_bounding_box() {
    let scene = ViewportScene {
        node_positions: vec![[2.0, 2.0, 2.0], [2.0, 2.0, 2.0], [2.0, 2.0, 2.0]],
        member_nodes: vec![(0, 1), (1, 2)],
        member_ref_vecs: vec![[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]],
    };
    let (min, max) = scene.bounding_box();
    assert_eq!(min, [2.0, 2.0, 2.0]);
    assert_eq!(max, [2.0, 2.0, 2.0]);

    // Camera should still work with zero-size bounding box
    let cam = Camera3D::fit(min, max);
    let p = cam.project([2.0, 2.0, 2.0], 800.0, 600.0);
    assert!(p.is_some(), "coincident nodes should still be projectable");
}

#[test]
fn non_finite_displacement_returns_original() {
    let scene = ViewportScene {
        node_positions: vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
        member_nodes: vec![],
        member_ref_vecs: vec![],
    };
    let disp = vec![
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ];
    let deformed = scene.deformed_positions(&disp, 1.0);
    assert_eq!(
        deformed[0],
        [1.0, 2.0, 3.0],
        "non-finite disp should give original position"
    );
    assert_eq!(deformed[1], [4.0, 5.0, 6.0]);
}

#[test]
fn non_finite_node_position_projects_to_none() {
    let cam = Camera3D::default();
    assert!(cam.project([f64::NAN, 0.0, 0.0], 800.0, 600.0).is_none());
    assert!(
        cam.project([f64::INFINITY, 0.0, 0.0], 800.0, 600.0)
            .is_none()
    );
    assert!(cam.project([0.0, f64::NAN, 0.0], 800.0, 600.0).is_none());
}

#[test]
fn zero_length_member() {
    let scene = ViewportScene {
        node_positions: vec![[1.0, 1.0, 1.0], [1.0, 1.0, 1.0]],
        member_nodes: vec![(0, 1)],
        member_ref_vecs: vec![[0.0, 0.0, 1.0]],
    };
    let len = scene.member_length(0).unwrap();
    assert_eq!(len, 0.0);
    let mid = scene.member_midpoint(0).unwrap();
    assert_eq!(mid, [1.0, 1.0, 1.0]);
}

#[test]
fn camera_clamps_elevation() {
    let mut cam = Camera3D::default();
    cam.orbit(0.0, 10.0);
    assert!(cam.elevation < 1.6);
    cam.orbit(0.0, -20.0);
    assert!(cam.elevation > -1.6);
}

#[test]
fn camera_clamps_distance() {
    let mut cam = Camera3D::default();
    cam.zoom(1e-10);
    assert!(cam.distance >= 0.1);
    cam.zoom(1e10);
    assert!(cam.distance <= 1e5);
}

#[test]
fn selection_equality() {
    assert_eq!(SelectionState::Node(1), SelectionState::Node(1));
    assert_ne!(SelectionState::Node(1), SelectionState::Node(2));
    assert_ne!(SelectionState::Node(1), SelectionState::Member(1));
    assert_eq!(SelectionState::None, SelectionState::None);
}

#[test]
fn normalize_zero_and_non_finite() {
    assert_eq!(project::normalize([0.0, 0.0, 0.0]), [0.0; 3]);
    assert_eq!(project::normalize([f64::INFINITY, 0.0, 0.0]), [0.0; 3]);
}

#[test]
fn point_to_segment_degenerate() {
    let d = project::point_to_segment_dist_2d([1.0, 1.0], [0.0, 0.0], [0.0, 0.0]);
    assert!((d - std::f64::consts::SQRT_2).abs() < 1e-15);
}
