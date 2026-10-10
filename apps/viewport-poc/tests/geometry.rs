//! Verify geometry coordinates are mapped consistently to the displayed scene.

use structural_analysis::FrameModel3D;
use viewport_poc::{camera::Camera3D, model, scene::ViewportScene};

#[test]
fn scene_node_positions_match_model() {
    let m = model::build_portal_frame();
    let result = m.solve().expect("portal frame solves");
    let scene = ViewportScene::from_result(&result);

    assert_eq!(scene.n_nodes(), m.n_nodes());
    for i in 0..m.n_nodes() {
        let node = m.node(i).unwrap();
        let scene_pos = scene.node_positions[i];
        assert!(
            (scene_pos[0] - node.x()).abs() < 1e-15,
            "x mismatch at node {i}"
        );
        assert!(
            (scene_pos[1] - node.y()).abs() < 1e-15,
            "y mismatch at node {i}"
        );
        assert!(
            (scene_pos[2] - node.z()).abs() < 1e-15,
            "z mismatch at node {i}"
        );
    }
}

#[test]
fn scene_member_connectivity_match_model() {
    let m = model::build_portal_frame();
    let result = m.solve().expect("portal frame solves");
    let scene = ViewportScene::from_result(&result);

    assert_eq!(scene.n_members(), m.n_members());
    for i in 0..m.n_members() {
        let member = m.member(i).unwrap();
        let (ni, nj) = scene.member_nodes[i];
        assert_eq!(ni, member.node_i(), "node_i mismatch at member {i}");
        assert_eq!(nj, member.node_j(), "node_j mismatch at member {i}");
    }
}

#[test]
fn scene_ref_vecs_match_model() {
    let m = model::build_portal_frame();
    let result = m.solve().expect("portal frame solves");
    let scene = ViewportScene::from_result(&result);

    for i in 0..m.n_members() {
        let member = m.member(i).unwrap();
        let rv = scene.member_ref_vecs[i];
        let mrv = member.ref_vec();
        assert!(
            (rv[0] - mrv[0]).abs() < 1e-15,
            "ref_vec x mismatch at member {i}"
        );
        assert!(
            (rv[1] - mrv[1]).abs() < 1e-15,
            "ref_vec y mismatch at member {i}"
        );
        assert!(
            (rv[2] - mrv[2]).abs() < 1e-15,
            "ref_vec z mismatch at member {i}"
        );
    }
}

#[test]
fn projected_coordinates_within_screen_bounds() {
    let m = model::build_portal_frame();
    let result = m.solve().expect("portal frame solves");
    let scene = ViewportScene::from_result(&result);
    let (min, max) = scene.bounding_box();
    let cam = Camera3D::fit(min, max);

    let w = 800.0;
    let h = 600.0;
    for &pos in &scene.node_positions {
        let p = cam.project(pos, w, h).expect("node should be visible");
        assert!(p.x >= 0.0 && p.x <= w, "x out of bounds: {}", p.x);
        assert!(p.y >= 0.0 && p.y <= h, "y out of bounds: {}", p.y);
        assert!(p.depth > 0.0, "depth must be positive");
    }
}

#[test]
fn inclined_frame_scene_consistency() {
    let m = model::build_inclined_frame();
    let result = m.solve().expect("inclined frame solves");
    let scene = ViewportScene::from_result(&result);

    assert_eq!(scene.n_nodes(), 5);
    assert_eq!(scene.n_members(), 4);

    // Member 3 (n1 → n4) should have a non-trivial ref_vec
    let rv = scene.member_ref_vecs[3];
    assert!((rv[0] - 0.3).abs() < 1e-15);
    assert!((rv[1] - 0.7).abs() < 1e-15);
    assert!((rv[2] - 0.5).abs() < 1e-15);
}

#[test]
fn grid_frame_scene_grows_correctly() {
    for n in [1, 2, 3, 5] {
        let m = model::build_grid_frame(n, n);
        let result = m.solve().expect("grid frame solves");
        let scene = ViewportScene::from_result(&result);
        let expected_nodes = (n + 1) * (n + 1) * 2;
        assert_eq!(
            scene.n_nodes(),
            expected_nodes,
            "node count for {n}x{n} grid"
        );
    }
}
