//! Robustness and correctness tests for the scene DTO.

use tauri_viewport_poc_lib::{
    build_grid_frame, build_inclined_frame, build_portal_frame, build_scene,
};

#[test]
fn portal_scene_is_finite() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    for node in &scene.nodes {
        assert!(
            node.position.x.is_finite(),
            "node {} position.x not finite",
            node.id
        );
        assert!(
            node.position.y.is_finite(),
            "node {} position.y not finite",
            node.id
        );
        assert!(
            node.position.z.is_finite(),
            "node {} position.z not finite",
            node.id
        );
        assert!(
            node.displacement.x.is_finite(),
            "node {} disp.x not finite",
            node.id
        );
        assert!(
            node.displacement.y.is_finite(),
            "node {} disp.y not finite",
            node.id
        );
        assert!(
            node.displacement.z.is_finite(),
            "node {} disp.z not finite",
            node.id
        );
        assert!(
            node.rotation.x.is_finite(),
            "node {} rot.x not finite",
            node.id
        );
        assert!(
            node.rotation.y.is_finite(),
            "node {} rot.y not finite",
            node.id
        );
        assert!(
            node.rotation.z.is_finite(),
            "node {} rot.z not finite",
            node.id
        );
        assert!(
            node.reaction_force.x.is_finite(),
            "node {} rf.x not finite",
            node.id
        );
        assert!(
            node.reaction_force.y.is_finite(),
            "node {} rf.y not finite",
            node.id
        );
        assert!(
            node.reaction_force.z.is_finite(),
            "node {} rf.z not finite",
            node.id
        );
        assert!(
            node.reaction_moment.x.is_finite(),
            "node {} rm.x not finite",
            node.id
        );
        assert!(
            node.reaction_moment.y.is_finite(),
            "node {} rm.y not finite",
            node.id
        );
        assert!(
            node.reaction_moment.z.is_finite(),
            "node {} rm.z not finite",
            node.id
        );
    }
    for member in &scene.members {
        for f in &member.end_forces_local {
            assert!(f.is_finite(), "member {} local force not finite", member.id);
        }
        for f in &member.end_forces_global {
            assert!(
                f.is_finite(),
                "member {} global force not finite",
                member.id
            );
        }
    }
}

#[test]
fn inclined_scene_is_finite() {
    let model = build_inclined_frame();
    let scene = build_scene(&model).unwrap();
    for node in &scene.nodes {
        assert!(node.position.x.is_finite());
        assert!(node.displacement.x.is_finite());
        assert!(node.reaction_force.x.is_finite());
    }
    for member in &scene.members {
        for f in &member.end_forces_local {
            assert!(f.is_finite());
        }
        for f in &member.end_forces_global {
            assert!(f.is_finite());
        }
    }
}

#[test]
fn portal_member_node_indices_in_bounds() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    for member in &scene.members {
        assert!(member.node_i < scene.n_nodes, "node_i out of bounds");
        assert!(member.node_j < scene.n_nodes, "node_j out of bounds");
    }
}

#[test]
fn inclined_member_node_indices_in_bounds() {
    let model = build_inclined_frame();
    let scene = build_scene(&model).unwrap();
    for member in &scene.members {
        assert!(member.node_i < scene.n_nodes, "node_i out of bounds");
        assert!(member.node_j < scene.n_nodes, "node_j out of bounds");
    }
}

#[test]
fn portal_no_duplicate_nodes() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    let mut ids: Vec<usize> = scene.nodes.iter().map(|n| n.id).collect();
    ids.sort();
    ids.dedup();
    assert_eq!(ids.len(), scene.n_nodes, "duplicate node ids");
}

#[test]
fn portal_no_duplicate_members() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    let mut ids: Vec<usize> = scene.members.iter().map(|m| m.id).collect();
    ids.sort();
    ids.dedup();
    assert_eq!(ids.len(), scene.n_members, "duplicate member ids");
}

#[test]
fn portal_local_axes_right_handed() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    for member in &scene.members {
        let x = &member.local_x_axis;
        let y = &member.local_y_axis;
        let z = &member.local_z_axis;
        let cross_x = y.y * z.z - y.z * z.y;
        let cross_y = y.z * z.x - y.x * z.z;
        let cross_z = y.x * z.y - y.y * z.x;
        assert!((cross_x - x.x).abs() < 1e-12, "not right-handed: x");
        assert!((cross_y - x.y).abs() < 1e-12, "not right-handed: y");
        assert!((cross_z - x.z).abs() < 1e-12, "not right-handed: z");
    }
}

#[test]
fn inclined_local_axes_right_handed() {
    let model = build_inclined_frame();
    let scene = build_scene(&model).unwrap();
    for member in &scene.members {
        let x = &member.local_x_axis;
        let y = &member.local_y_axis;
        let z = &member.local_z_axis;
        let cross_x = y.y * z.z - y.z * z.y;
        let cross_y = y.z * z.x - y.x * z.z;
        let cross_z = y.x * z.y - y.y * z.x;
        assert!((cross_x - x.x).abs() < 1e-12);
        assert!((cross_y - x.y).abs() < 1e-12);
        assert!((cross_z - x.z).abs() < 1e-12);
    }
}

#[test]
fn portal_x_axis_along_member() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    for member in &scene.members {
        let pi = &scene.nodes[member.node_i].position;
        let pj = &scene.nodes[member.node_j].position;
        let dx = pj.x - pi.x;
        let dy = pj.y - pi.y;
        let dz = pj.z - pi.z;
        let len = dx.hypot(dy).hypot(dz);
        assert!(len > 0.0, "zero-length member");
        let ux = dx / len;
        let uy = dy / len;
        let uz = dz / len;
        assert!(
            (member.local_x_axis.x - ux).abs() < 1e-12,
            "x_hat doesn't align with member"
        );
        assert!((member.local_x_axis.y - uy).abs() < 1e-12);
        assert!((member.local_x_axis.z - uz).abs() < 1e-12);
    }
}

#[test]
fn grid_1x1_scene_builds() {
    let model = build_grid_frame(1, 1);
    let scene = build_scene(&model).expect("1x1 grid should build");
    assert!(scene.n_nodes > 0);
    assert!(scene.n_members > 0);
}

#[test]
fn grid_3x3_scene_builds() {
    let model = build_grid_frame(3, 3);
    let scene = build_scene(&model).expect("3x3 grid should build");
    assert!(scene.n_nodes > 0);
    assert!(scene.n_members > 0);
    for node in &scene.nodes {
        assert!(node.displacement.x.is_finite());
    }
}

#[test]
fn repeated_scene_build_produces_same_result() {
    let model = build_portal_frame();
    let scene1 = build_scene(&model).unwrap();
    let scene2 = build_scene(&model).unwrap();
    assert_eq!(scene1.n_nodes, scene2.n_nodes);
    assert_eq!(scene1.n_members, scene2.n_members);
    for (a, b) in scene1.nodes.iter().zip(scene2.nodes.iter()) {
        assert!((a.displacement.x - b.displacement.x).abs() < 1e-15);
        assert!((a.displacement.y - b.displacement.y).abs() < 1e-15);
    }
    for (a, b) in scene1.members.iter().zip(scene2.members.iter()) {
        for i in 0..12 {
            assert!((a.end_forces_local[i] - b.end_forces_local[i]).abs() < 1e-15);
        }
    }
}

#[test]
fn portal_global_force_equilibrium() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    let mut sum_fx = 0.0;
    let mut sum_fy = 0.0;
    let mut sum_fz = 0.0;
    for node in &scene.nodes {
        sum_fx += node.reaction_force.x;
        sum_fy += node.reaction_force.y;
        sum_fz += node.reaction_force.z;
    }
    // Reactions equilibrate applied load: sum(R) = -sum(F_applied)
    // Applied load: 5kN in X at N2, so sum(Rx) = -5000
    assert!(
        (sum_fx + 5e3).abs() < 1.0,
        "sum Rx should equal -5kN, got {sum_fx}"
    );
    assert!(sum_fy.abs() < 1.0, "sum Ry should be ~0, got {sum_fy}");
    assert!(sum_fz.abs() < 1.0, "sum Rz should be ~0, got {sum_fz}");
}

#[test]
fn portal_global_moment_equilibrium() {
    let model = build_portal_frame();
    let scene = build_scene(&model).unwrap();
    let mut sum_mx = 0.0;
    let mut sum_my = 0.0;
    let mut sum_mz = 0.0;
    for node in &scene.nodes {
        let r = &node.reaction_force;
        let m = &node.reaction_moment;
        let p = &node.position;
        sum_mx += m.x + p.y * r.z - p.z * r.y;
        sum_my += m.y + p.z * r.x - p.x * r.z;
        sum_mz += m.z + p.x * r.y - p.y * r.x;
    }
    // Reactions equilibrate applied load moment: sum(M_R) = -M_applied
    // Applied load 5kN at N2 (0,3,0) in X: M = r×F = (0,0,-15000)
    // So sum(Mz) = +15000
    assert!(sum_mx.abs() < 10.0, "sum Mx should be ~0, got {sum_mx}");
    assert!(sum_my.abs() < 10.0, "sum My should be ~0, got {sum_my}");
    assert!(
        (sum_mz - 15e3).abs() < 10.0,
        "sum Mz should equal +15kN·m, got {sum_mz}"
    );
}
