//! Verify geometry coordinates are mapped consistently to the displayed scene.

use viewport_poc::{camera::Camera3D, model, scene::ViewportScene};

#[test]
fn scene_node_positions_match_edit_model() {
    let m = model::build_portal_edit();
    let scene = ViewportScene::from_edit_model(&m);

    assert_eq!(scene.n_nodes(), m.n_nodes());
    for (i, node) in m.nodes().iter().enumerate() {
        let scene_pos = scene.node_positions[i];
        assert!(
            (scene_pos[0] - node.x).abs() < 1e-15,
            "x mismatch at node {i}"
        );
        assert!(
            (scene_pos[1] - node.y).abs() < 1e-15,
            "y mismatch at node {i}"
        );
        assert!(
            (scene_pos[2] - node.z).abs() < 1e-15,
            "z mismatch at node {i}"
        );
    }
}

#[test]
fn scene_member_connectivity_match_edit_model() {
    let m = model::build_portal_edit();
    let scene = ViewportScene::from_edit_model(&m);

    assert_eq!(scene.n_members(), m.n_members());
    for (i, member) in m.members().iter().enumerate() {
        let (ni, nj) = scene.member_nodes[i];
        let expected_i = m.node_index(member.node_i).unwrap();
        let expected_j = m.node_index(member.node_j).unwrap();
        assert_eq!(ni, expected_i, "node_i mismatch at member {i}");
        assert_eq!(nj, expected_j, "node_j mismatch at member {i}");
    }
}

#[test]
fn scene_ref_vecs_match_edit_model() {
    let m = model::build_portal_edit();
    let scene = ViewportScene::from_edit_model(&m);

    for (i, member) in m.members().iter().enumerate() {
        let rv = scene.member_ref_vecs[i];
        assert!(
            (rv[0] - member.ref_vec[0]).abs() < 1e-15,
            "ref_vec x mismatch at member {i}"
        );
        assert!(
            (rv[1] - member.ref_vec[1]).abs() < 1e-15,
            "ref_vec y mismatch at member {i}"
        );
        assert!(
            (rv[2] - member.ref_vec[2]).abs() < 1e-15,
            "ref_vec z mismatch at member {i}"
        );
    }
}

#[test]
fn projected_coordinates_within_screen_bounds() {
    let m = model::build_portal_edit();
    let scene = ViewportScene::from_edit_model(&m);
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
    let m = model::build_inclined_edit();
    let scene = ViewportScene::from_edit_model(&m);

    assert_eq!(scene.n_nodes(), 5);
    assert_eq!(scene.n_members(), 4);

    let rv = scene.member_ref_vecs[3];
    assert!((rv[0] - 0.3).abs() < 1e-15);
    assert!((rv[1] - 0.7).abs() < 1e-15);
    assert!((rv[2] - 0.5).abs() < 1e-15);
}

#[test]
fn grid_frame_scene_grows_correctly() {
    for n in [1, 2, 3, 5] {
        let m = model::build_grid_edit(n, n);
        let scene = ViewportScene::from_edit_model(&m);
        let expected_nodes = (n + 1) * (n + 1) * 2;
        assert_eq!(
            scene.n_nodes(),
            expected_nodes,
            "node count for {n}x{n} grid"
        );
    }
}
