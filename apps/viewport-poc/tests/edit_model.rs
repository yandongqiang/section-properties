//! Integration tests for the editable structural model.

use viewport_poc::{
    edit_model::{EditError, EditModel, MemberId, NodeId},
    model,
    scene::ViewportScene,
};

// ── Node CRUD ─────────────────────────────────────────────────────────

#[test]
fn add_update_delete_node() {
    let mut m = EditModel::new();
    let id = m.add_node(1.0, 2.0, 3.0).unwrap();
    assert_eq!(m.n_nodes(), 1);

    m.update_node(id, 4.0, 5.0, 6.0).unwrap();
    let n = m.node(id).unwrap();
    assert_eq!(n.x, 4.0);
    assert_eq!(n.y, 5.0);
    assert_eq!(n.z, 6.0);

    m.delete_node(id).unwrap();
    assert_eq!(m.n_nodes(), 0);
}

#[test]
fn node_ids_survive_deletion() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(1.0, 0.0, 0.0).unwrap();
    let c = m.add_node(2.0, 0.0, 0.0).unwrap();

    m.delete_node(b).unwrap();
    assert_eq!(m.n_nodes(), 2);

    assert!(m.node(a).is_some());
    assert!(m.node(b).is_none());
    assert!(m.node(c).is_some());

    let d = m.add_node(3.0, 0.0, 0.0).unwrap();
    assert_ne!(d, a);
    assert_ne!(d, b);
    assert_ne!(d, c);
}

#[test]
fn delete_node_blocked_by_member() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    let _mid = m.add_member(a, b).unwrap();

    let err = m.delete_node(a).unwrap_err();
    assert!(matches!(err, EditError::NodeInUse(ref members) if members.len() == 1));
    assert_eq!(m.n_nodes(), 2);
}

#[test]
fn delete_node_then_member_then_node() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();

    assert!(m.delete_node(a).is_err());
    m.delete_member(mid).unwrap();
    m.delete_node(a).unwrap();
    m.delete_node(b).unwrap();
    assert!(m.is_empty());
}

// ── Member CRUD ───────────────────────────────────────────────────────

#[test]
fn add_delete_member() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();
    assert_eq!(m.n_members(), 1);

    m.delete_member(mid).unwrap();
    assert_eq!(m.n_members(), 0);
    assert_eq!(m.n_nodes(), 2);
}

#[test]
fn member_validations() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();

    assert_eq!(m.add_member(a, a), Err(EditError::SelfConnection));
    assert_eq!(
        m.add_member(a, NodeId(99)),
        Err(EditError::NodeNotFound(NodeId(99)))
    );
    assert_eq!(
        m.add_member(NodeId(99), a),
        Err(EditError::NodeNotFound(NodeId(99)))
    );

    m.add_member(a, b).unwrap();
    assert_eq!(m.add_member(a, b), Err(EditError::DuplicateMember));
    assert_eq!(m.add_member(b, a), Err(EditError::DuplicateMember));
}

#[test]
fn zero_length_member_rejected() {
    let mut m = EditModel::new();
    let a = m.add_node(1.0, 2.0, 3.0).unwrap();
    let b = m.add_node(1.0, 2.0, 3.0).unwrap();
    assert_eq!(m.add_member(a, b), Err(EditError::ZeroLengthMember));
}

// ── Preset compatibility ─────────────────────────────────────────────

#[test]
fn all_presets_build_edit_and_solve() {
    for &preset in model::PresetModel::all() {
        let m = preset.build_edit();
        let fm = m
            .to_frame_model()
            .unwrap_or_else(|e| panic!("preset '{}' to_frame_model failed: {e:?}", preset.name()));
        let result = fm
            .solve()
            .unwrap_or_else(|e| panic!("preset '{}' solve failed: {e:?}", preset.name()));
        assert!(
            result.displacements.len() > 0,
            "preset '{}' has no displacements",
            preset.name()
        );
    }
}

#[test]
fn portal_edit_has_correct_structure() {
    let m = model::build_portal_edit();
    assert_eq!(m.n_nodes(), 4);
    assert_eq!(m.n_members(), 3);
}

#[test]
fn inclined_edit_has_correct_structure() {
    let m = model::build_inclined_edit();
    assert_eq!(m.n_nodes(), 5);
    assert_eq!(m.n_members(), 4);
}

#[test]
fn grid_edit_node_count() {
    let m = model::build_grid_edit(3, 3);
    let expected = (3 + 1) * (3 + 1) * 2;
    assert_eq!(m.n_nodes(), expected);
}

// ── Scene from edit model ────────────────────────────────────────────

#[test]
fn scene_from_edit_model_consistency() {
    let m = model::build_portal_edit();
    let scene = ViewportScene::from_edit_model(&m);
    assert_eq!(scene.n_nodes(), m.n_nodes());
    assert_eq!(scene.n_members(), m.n_members());

    for (i, node) in m.nodes().iter().enumerate() {
        assert_eq!(scene.node_ids[i], node.id);
        assert_eq!(scene.node_positions[i], [node.x, node.y, node.z]);
    }

    for (i, member) in m.members().iter().enumerate() {
        assert_eq!(scene.member_ids[i], member.id);
    }
}

#[test]
fn scene_node_index_lookup() {
    let m = model::build_portal_edit();
    let scene = ViewportScene::from_edit_model(&m);

    for node in m.nodes() {
        assert!(scene.node_index(node.id).is_some());
    }
    assert!(scene.node_index(NodeId(999)).is_none());
}

#[test]
fn scene_member_index_lookup() {
    let m = model::build_portal_edit();
    let scene = ViewportScene::from_edit_model(&m);

    for member in m.members() {
        assert!(scene.member_index(member.id).is_some());
    }
    assert!(scene.member_index(MemberId(999)).is_none());
}

// ── Edit then solve ──────────────────────────────────────────────────

#[test]
fn edit_node_then_solve() {
    let mut m = model::build_portal_edit();
    let nodes: Vec<_> = m.nodes().iter().map(|n| n.id).collect();
    m.update_node(nodes[0], 0.0, 0.0, 0.0).unwrap();

    let fm = m.to_frame_model().unwrap();
    assert!(fm.solve().is_ok());
}

#[test]
fn add_node_and_member_then_solve() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(4.0, 0.0, 0.0).unwrap();
    let c = m.add_node(0.0, 3.0, 0.0).unwrap();
    let d = m.add_node(4.0, 3.0, 0.0).unwrap();

    m.add_member(a, c).unwrap();
    m.add_member(c, d).unwrap();
    m.add_member(b, d).unwrap();

    m.fix_node(a).unwrap();
    m.fix_node(b).unwrap();
    m.add_nodal_load(c, [5e3, 0.0, 0.0, 0.0, 0.0, 0.0]).unwrap();

    let fm = m.to_frame_model().unwrap();
    let result = fm.solve().expect("portal frame should solve");
    assert!(result.displacements.len() > 0);
}

#[test]
fn delete_member_then_solve() {
    let mut m = model::build_portal_edit();
    let members: Vec<_> = m.members().iter().map(|mb| mb.id).collect();
    m.delete_member(members[0]).unwrap();

    let fm = m.to_frame_model().unwrap();
    assert!(fm.solve().is_ok());
}

// ── Atomicity ────────────────────────────────────────────────────────

#[test]
fn failed_delete_preserves_model() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();

    let err = m.delete_node(a).unwrap_err();
    assert!(matches!(err, EditError::NodeInUse(_)));
    assert_eq!(m.n_nodes(), 2);
    assert_eq!(m.n_members(), 1);
}

#[test]
fn failed_add_member_preserves_model() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();

    let err = m.add_member(a, b).unwrap_err();
    assert_eq!(err, EditError::DuplicateMember);
    assert_eq!(m.n_members(), 1);
}

// ── ref_vec support ──────────────────────────────────────────────────

#[test]
fn set_member_ref_vec() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();

    m.set_member_ref_vec(mid, [1.0, 0.0, 0.0]).unwrap();
    assert_eq!(m.member(mid).unwrap().ref_vec, [1.0, 0.0, 0.0]);

    let fm = m.to_frame_model().unwrap();
    let member = fm.member(0).unwrap();
    let rv = member.ref_vec();
    assert!((rv[0] - 1.0).abs() < 1e-15);
}

#[test]
fn default_ref_vec_is_z_axis() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();
    assert_eq!(m.member(mid).unwrap().ref_vec, [0.0, 0.0, 1.0]);
}
