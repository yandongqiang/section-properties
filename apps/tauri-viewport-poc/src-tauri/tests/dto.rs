//! DTO generation and serialisation correctness tests.

use serde_json::Value;
use tauri_viewport_poc_lib::{
    build_inclined_frame, build_portal_frame, build_scene, build_scene_from_result,
};

#[test]
fn portal_scene_dto_counts() {
    let model = build_portal_frame();
    let scene = build_scene(&model).expect("portal scene should build");
    assert_eq!(scene.n_nodes, 4);
    assert_eq!(scene.n_members, 3);
    assert_eq!(scene.nodes.len(), 4);
    assert_eq!(scene.members.len(), 3);
}

#[test]
fn inclined_scene_dto_counts() {
    let model = build_inclined_frame();
    let scene = build_scene(&model).expect("inclined scene should build");
    assert_eq!(scene.n_nodes, 5);
    assert_eq!(scene.n_members, 4);
}

#[test]
fn portal_node_ids_sequential() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    for (i, node) in scene.nodes.iter().enumerate() {
        assert_eq!(node.id, i, "node id should match index");
    }
}

#[test]
fn portal_member_ids_sequential() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    for (j, member) in scene.members.iter().enumerate() {
        assert_eq!(member.id, j, "member id should match index");
    }
}

#[test]
fn portal_node_positions_match_model() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    assert_eq!(scene.nodes[0].position.x, 0.0);
    assert_eq!(scene.nodes[0].position.y, 0.0);
    assert_eq!(scene.nodes[0].position.z, 0.0);
    assert_eq!(scene.nodes[1].position.x, 4.0);
    assert_eq!(scene.nodes[2].position.y, 3.0);
    assert_eq!(scene.nodes[3].position.x, 4.0);
    assert_eq!(scene.nodes[3].position.y, 3.0);
}

#[test]
fn portal_member_connectivity() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    assert_eq!(scene.members[0].node_i, 0);
    assert_eq!(scene.members[0].node_j, 2);
    assert_eq!(scene.members[1].node_i, 2);
    assert_eq!(scene.members[1].node_j, 3);
    assert_eq!(scene.members[2].node_i, 1);
    assert_eq!(scene.members[2].node_j, 3);
}

#[test]
fn portal_member_ref_vecs() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    for member in &scene.members {
        assert_eq!(member.ref_vec.x, 0.0);
        assert_eq!(member.ref_vec.y, 0.0);
        assert_eq!(member.ref_vec.z, 1.0);
    }
}

#[test]
fn portal_local_axes_are_orthonormal() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    for member in &scene.members {
        let x = &member.local_x_axis;
        let y = &member.local_y_axis;
        let z = &member.local_z_axis;

        let x_dot_x = x.x * x.x + x.y * x.y + x.z * x.z;
        let y_dot_y = y.x * y.x + y.y * y.y + y.z * y.z;
        let z_dot_z = z.x * z.x + z.y * z.y + z.z * z.z;
        assert!((x_dot_x - 1.0).abs() < 1e-12, "x_hat not unit");
        assert!((y_dot_y - 1.0).abs() < 1e-12, "y_hat not unit");
        assert!((z_dot_z - 1.0).abs() < 1e-12, "z_hat not unit");

        let x_dot_y = x.x * y.x + x.y * y.y + x.z * y.z;
        let x_dot_z = x.x * z.x + x.y * z.y + x.z * z.z;
        let y_dot_z = y.x * z.x + y.y * z.y + y.z * z.z;
        assert!(x_dot_y.abs() < 1e-12, "x,y not orthogonal");
        assert!(x_dot_z.abs() < 1e-12, "x,z not orthogonal");
        assert!(y_dot_z.abs() < 1e-12, "y,z not orthogonal");
    }
}

#[test]
fn inclined_member_ref_vec_preserved() {
    let model = build_inclined_frame();
    let scene = build_scene(&model).unwrap();
    let inclined = &scene.members[3];
    assert!((inclined.ref_vec.x - 0.3).abs() < 1e-15);
    assert!((inclined.ref_vec.y - 0.7).abs() < 1e-15);
    assert!((inclined.ref_vec.z - 0.5).abs() < 1e-15);
}

#[test]
fn portal_displacements_nonzero() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    let n2 = &scene.nodes[2];
    let mag = n2.displacement.x.powi(2) + n2.displacement.y.powi(2) + n2.displacement.z.powi(2);
    assert!(mag > 0.0, "node N2 should have nonzero displacement");
}

#[test]
fn portal_reactions_at_fixed_nodes() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    let n0 = &scene.nodes[0];
    let n1 = &scene.nodes[1];
    let r0_mag = n0.reaction_force.x.powi(2) + n0.reaction_force.y.powi(2);
    let r1_mag = n1.reaction_force.x.powi(2) + n1.reaction_force.y.powi(2);
    assert!(r0_mag > 0.0, "fixed node N0 should have reactions");
    assert!(r1_mag > 0.0, "fixed node N1 should have reactions");
}

#[test]
fn portal_end_forces_local_has_12_components() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    for member in &scene.members {
        assert_eq!(member.end_forces_local.len(), 12);
        assert_eq!(member.end_forces_global.len(), 12);
    }
}

#[test]
fn portal_end_forces_not_all_zero() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    let any_nonzero = scene
        .members
        .iter()
        .flat_map(|m| m.end_forces_local.iter())
        .any(|&f| f.abs() > 1e-10);
    assert!(
        any_nonzero,
        "at least one member should have nonzero forces"
    );
}

#[test]
fn dto_serializes_to_json() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    let json = serde_json::to_string(&scene).expect("scene should serialize");
    let v: Value = serde_json::from_str(&json).expect("json should parse");
    assert_eq!(v["n_nodes"], 4);
    assert_eq!(v["n_members"], 3);
    assert_eq!(v["nodes"].as_array().unwrap().len(), 4);
    assert_eq!(v["members"].as_array().unwrap().len(), 3);
}

#[test]
fn dto_node_json_structure() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    let json = serde_json::to_string(&scene).unwrap();
    let v: Value = serde_json::from_str(&json).unwrap();
    let node0 = &v["nodes"][0];
    assert!(node0["id"].is_number());
    assert!(node0["position"]["x"].is_number());
    assert!(node0["position"]["y"].is_number());
    assert!(node0["position"]["z"].is_number());
    assert!(node0["displacement"]["x"].is_number());
    assert!(node0["rotation"]["x"].is_number());
    assert!(node0["reaction_force"]["x"].is_number());
    assert!(node0["reaction_moment"]["x"].is_number());
}

#[test]
fn dto_member_json_structure() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    let json = serde_json::to_string(&scene).unwrap();
    let v: Value = serde_json::from_str(&json).unwrap();
    let m0 = &v["members"][0];
    assert!(m0["id"].is_number());
    assert!(m0["node_i"].is_number());
    assert!(m0["node_j"].is_number());
    assert!(m0["ref_vec"]["x"].is_number());
    assert!(m0["local_x_axis"]["x"].is_number());
    assert!(m0["local_y_axis"]["x"].is_number());
    assert!(m0["local_z_axis"]["x"].is_number());
    assert_eq!(m0["end_forces_local"].as_array().unwrap().len(), 12);
    assert_eq!(m0["end_forces_global"].as_array().unwrap().len(), 12);
}

#[test]
fn build_scene_from_result_matches_build_scene() {
    let model = build_portal_frame();
    let result = model.solve().expect("solve should succeed");
    let scene1 = build_scene(&model).unwrap();
    let scene2 = build_scene_from_result(&result).unwrap();
    assert_eq!(scene1.n_nodes, scene2.n_nodes);
    assert_eq!(scene1.n_members, scene2.n_members);
    for (a, b) in scene1.nodes.iter().zip(scene2.nodes.iter()) {
        assert_eq!(a.id, b.id);
        assert!((a.position.x - b.position.x).abs() < 1e-15);
    }
}
