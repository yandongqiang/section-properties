//! Integration tests for the editable structural model.

use structural_analysis::Dof3D;
use viewport_poc::{
    edit_model::{EditError, EditModel, MemberId, NodalLoadSpec, NodeId, SupportSpec},
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
    m.set_nodal_load(c, NodalLoadSpec::new(5e3, 0.0, 0.0, 0.0, 0.0, 0.0))
        .unwrap();

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

    m.set_member_ref_vec(mid, [0.0, 1.0, 0.0]).unwrap();
    assert_eq!(m.member(mid).unwrap().ref_vec, [0.0, 1.0, 0.0]);

    let fm = m.to_frame_model().unwrap();
    let member = fm.member(0).unwrap();
    let rv = member.ref_vec();
    assert!((rv[1] - 1.0).abs() < 1e-15);
}

#[test]
fn default_ref_vec_is_z_axis() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();
    assert_eq!(m.member(mid).unwrap().ref_vec, [0.0, 0.0, 1.0]);
}

// ── Support editing integration tests ───────────────────────────────

#[test]
fn set_support_fixed_then_solve() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(4.0, 0.0, 0.0).unwrap();
    let c = m.add_node(0.0, 3.0, 0.0).unwrap();
    let d = m.add_node(4.0, 3.0, 0.0).unwrap();
    m.add_member(a, c).unwrap();
    m.add_member(c, d).unwrap();
    m.add_member(b, d).unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(b, SupportSpec::fixed()).unwrap();
    m.set_nodal_load(c, NodalLoadSpec::new(5e3, 0.0, 0.0, 0.0, 0.0, 0.0))
        .unwrap();
    let fm = m.to_frame_model().unwrap();
    assert!(fm.solve().is_ok());
}

#[test]
fn set_support_pinned_then_solve() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(4.0, 0.0, 0.0).unwrap();
    let c = m.add_node(0.0, 3.0, 0.0).unwrap();
    let d = m.add_node(4.0, 3.0, 0.0).unwrap();
    let e = m.add_node(2.0, 3.0, 0.0).unwrap();
    m.add_member(a, c).unwrap();
    m.add_member(c, e).unwrap();
    m.add_member(e, d).unwrap();
    m.add_member(b, d).unwrap();
    m.add_member(c, d).unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(b, SupportSpec::fixed()).unwrap();
    m.set_nodal_load(e, NodalLoadSpec::new(5e3, 0.0, 0.0, 0.0, 0.0, 0.0))
        .unwrap();
    let fm = m.to_frame_model().unwrap();
    assert!(fm.solve().is_ok());
}

#[test]
fn clear_support_makes_node_free() {
    let mut m = EditModel::new();
    let id = m.add_node(0.0, 0.0, 0.0).unwrap();
    m.set_support(id, SupportSpec::fixed()).unwrap();
    assert!(m.has_support(id));
    m.clear_support(id).unwrap();
    assert!(!m.has_support(id));
    assert!(m.support(id).is_none());
}

#[test]
fn fix_dof_merges_into_pinned() {
    let mut m = EditModel::new();
    let id = m.add_node(0.0, 0.0, 0.0).unwrap();
    m.set_support(id, SupportSpec::pinned()).unwrap();
    m.fix_dof(id, Dof3D::Rz).unwrap();
    let s = m.support(id).unwrap();
    assert_eq!(s.n_constrained(), 4);
    assert!(!s.is_pinned());
    assert!(!s.is_fixed());
}

#[test]
fn support_preserved_after_node_coordinate_update() {
    let mut m = EditModel::new();
    let id = m.add_node(0.0, 0.0, 0.0).unwrap();
    m.set_support(id, SupportSpec::fixed()).unwrap();
    m.update_node(id, 1.0, 2.0, 3.0).unwrap();
    assert!(m.support(id).unwrap().is_fixed());
}

#[test]
fn preset_inclined_supports_correct() {
    let m = model::build_inclined_edit();
    let nodes: Vec<_> = m.nodes().iter().map(|n| n.id).collect();
    assert!(m.support(nodes[0]).unwrap().is_fixed());
    assert!(m.support(nodes[1]).unwrap().is_fixed());
    let s4 = m.support(nodes[4]).unwrap();
    assert_eq!(s4.n_constrained(), 3);
    assert!(s4.dof(Dof3D::Ux).is_some());
    assert!(s4.dof(Dof3D::Uy).is_some());
    assert!(s4.dof(Dof3D::Uz).is_some());
    assert!(s4.dof(Dof3D::Rx).is_none());
    assert!(s4.dof(Dof3D::Ry).is_none());
    assert!(s4.dof(Dof3D::Rz).is_none());
}

#[test]
fn all_presets_supports_solve() {
    for &preset in model::PresetModel::all() {
        let m = preset.build_edit();
        let fm = m
            .to_frame_model()
            .unwrap_or_else(|e| panic!("preset '{}' to_frame_model failed: {e:?}", preset.name()));
        assert!(
            fm.solve().is_ok(),
            "preset '{}' with supports should solve",
            preset.name()
        );
    }
}

// ── Scene node_supports tests ───────────────────────────────────────

#[test]
fn scene_reflects_supports() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(b, SupportSpec::pinned()).unwrap();
    let scene = ViewportScene::from_edit_model(&m);
    assert_eq!(scene.node_supports.len(), 2);
    assert!(scene.node_supports[0].unwrap().is_fixed());
    assert!(scene.node_supports[1].unwrap().is_pinned());
}

#[test]
fn scene_no_supports_when_none_set() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    let scene = ViewportScene::from_edit_model(&m);
    assert!(scene.node_supports[0].is_none());
    assert!(scene.node_supports[1].is_none());
}

#[test]
fn scene_supports_match_model_after_clear() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.clear_support(a).unwrap();
    let scene = ViewportScene::from_edit_model(&m);
    assert!(scene.node_supports[0].is_none());
}

// ── Nodal load editing integration tests ─────────────────────────────

#[test]
fn scene_reflects_loads() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    let load = NodalLoadSpec::new(1e3, 2e3, 3e3, 4e3, 5e3, 6e3);
    m.set_nodal_load(b, load).unwrap();
    let scene = ViewportScene::from_edit_model(&m);
    assert_eq!(scene.node_loads.len(), 2);
    assert!(scene.node_loads[0].is_none());
    let s = scene.node_loads[1].unwrap();
    assert_eq!(s.fx, 1e3);
    assert_eq!(s.fy, 2e3);
    assert_eq!(s.fz, 3e3);
    assert_eq!(s.mx, 4e3);
    assert_eq!(s.my, 5e3);
    assert_eq!(s.mz, 6e3);
}

#[test]
fn scene_no_loads_when_none_set() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    let scene = ViewportScene::from_edit_model(&m);
    assert!(scene.node_loads[0].is_none());
    assert!(scene.node_loads[1].is_none());
}

#[test]
fn scene_loads_match_model_after_clear() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    m.set_nodal_load(b, NodalLoadSpec::new(1e3, 0.0, 0.0, 0.0, 0.0, 0.0))
        .unwrap();
    m.clear_nodal_load(b).unwrap();
    let scene = ViewportScene::from_edit_model(&m);
    assert!(scene.node_loads[1].is_none());
}

#[test]
fn solver_load_transfer() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(4.0, 0.0, 0.0).unwrap();
    let c = m.add_node(0.0, 3.0, 0.0).unwrap();
    let d = m.add_node(4.0, 3.0, 0.0).unwrap();
    m.add_member(a, c).unwrap();
    m.add_member(c, d).unwrap();
    m.add_member(b, d).unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(b, SupportSpec::fixed()).unwrap();
    let load = NodalLoadSpec::new(0.0, -5e3, 0.0, 0.0, 0.0, 0.0);
    m.set_nodal_load(c, load).unwrap();
    let fm = m.to_frame_model().unwrap();
    let result = fm.solve().expect("portal frame with load should solve");
    assert!(result.displacements.len() > 0);
    let c_idx = m.node_index(c).unwrap();
    let uy = result.displacements[c_idx * 6 + 1];
    assert!(
        uy < 0.0,
        "vertical displacement at loaded node should be negative (downward), got {uy}"
    );
}

#[test]
fn preset_load_preservation() {
    for &preset in model::PresetModel::all() {
        let m = preset.build_edit();
        let loads_before: Vec<_> = m.nodal_loads().to_vec();
        let fm = m
            .to_frame_model()
            .unwrap_or_else(|e| panic!("preset '{}' to_frame_model failed: {e:?}", preset.name()));
        assert!(
            fm.solve().is_ok(),
            "preset '{}' with loads should solve",
            preset.name()
        );
        let m2 = preset.build_edit();
        let loads_after: Vec<_> = m2.nodal_loads().to_vec();
        assert_eq!(
            loads_before.len(),
            loads_after.len(),
            "preset '{}' load count changed",
            preset.name()
        );
    }
}

#[test]
fn edited_load_produces_expected_response() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(6.0, 0.0, 0.0).unwrap();
    let c = m.add_node(3.0, 4.0, 0.0).unwrap();
    m.add_member(a, c).unwrap();
    m.add_member(b, c).unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_support(b, SupportSpec::fixed()).unwrap();
    m.set_nodal_load(c, NodalLoadSpec::new(0.0, -10e3, 0.0, 0.0, 0.0, 0.0))
        .unwrap();
    let fm = m.to_frame_model().unwrap();
    let result = fm.solve().expect("triangle frame should solve");
    let c_idx = m.node_index(c).unwrap();
    let uy = result.displacements[c_idx * 6 + 1];
    assert!(
        uy < 0.0,
        "apex vertical displacement should be negative, got {uy}"
    );
    assert!(
        uy.abs() > 1e-12,
        "apex vertical displacement should be non-trivial, got {uy}"
    );
    let a_idx = m.node_index(a).unwrap();
    let b_idx = m.node_index(b).unwrap();
    let ua = result.displacements[a_idx * 6 + 0];
    let ub = result.displacements[b_idx * 6 + 0];
    assert!(
        ua.abs() < 1e-6,
        "fixed node a should have ~zero Ux, got {ua}"
    );
    assert!(
        ub.abs() < 1e-6,
        "fixed node b should have ~zero Ux, got {ub}"
    );
}

#[test]
fn load_preserved_after_node_coordinate_update() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    let load = NodalLoadSpec::new(100.0, 200.0, 300.0, 0.0, 0.0, 0.0);
    m.set_nodal_load(b, load).unwrap();
    m.update_node(b, 5.0, 0.0, 0.0).unwrap();
    let s = m.nodal_load(b).unwrap();
    assert_eq!(s.fx, 100.0);
    assert_eq!(s.fy, 200.0);
    assert_eq!(s.fz, 300.0);
}

#[test]
fn delete_node_removes_load_from_scene() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    let c = m.add_node(6.0, 0.0, 0.0).unwrap();
    let _m_ab = m.add_member(a, b).unwrap();
    let m_bc = m.add_member(b, c).unwrap();
    m.set_nodal_load(b, NodalLoadSpec::new(1e3, 0.0, 0.0, 0.0, 0.0, 0.0))
        .unwrap();
    m.set_nodal_load(c, NodalLoadSpec::new(2e3, 0.0, 0.0, 0.0, 0.0, 0.0))
        .unwrap();
    m.delete_member(m_bc).unwrap();
    m.delete_node(c).unwrap();
    assert!(!m.has_nodal_load(c));
    assert!(m.has_nodal_load(b));
    let scene = ViewportScene::from_edit_model(&m);
    assert_eq!(scene.node_loads.len(), 2);
    assert!(scene.node_loads[1].is_some());
}

#[test]
fn replace_load_updates_scene() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    m.set_nodal_load(b, NodalLoadSpec::new(1e3, 0.0, 0.0, 0.0, 0.0, 0.0))
        .unwrap();
    m.set_nodal_load(b, NodalLoadSpec::new(5e3, 0.0, 0.0, 0.0, 0.0, 0.0))
        .unwrap();
    let scene = ViewportScene::from_edit_model(&m);
    assert_eq!(scene.node_loads[1].unwrap().fx, 5e3);
    assert_eq!(m.nodal_loads().len(), 1);
}

#[test]
fn support_and_load_coexist_in_scene() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    m.add_member(a, b).unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_nodal_load(b, NodalLoadSpec::new(0.0, -1e3, 0.0, 0.0, 0.0, 0.0))
        .unwrap();
    let scene = ViewportScene::from_edit_model(&m);
    assert!(scene.node_supports[0].unwrap().is_fixed());
    assert!(scene.node_loads[1].is_some());
    assert!(scene.node_loads[0].is_none());
}

// ── Member property editing integration tests ───────────────────────

#[test]
fn preset_section_preservation() {
    use structural_analysis::FrameSection3D;
    for &preset in model::PresetModel::all() {
        let m = preset.build_edit();
        let fm = m
            .to_frame_model()
            .unwrap_or_else(|e| panic!("preset '{}' to_frame_model failed: {e:?}", preset.name()));
        for member in m.members() {
            let idx = m.member_index(member.id).unwrap();
            let sec_edit = &member.section;
            let sec_fm = fm.member(idx).unwrap().section();
            assert!(
                (sec_edit.E() - sec_fm.E()).abs() < 1e-3,
                "preset '{}' M{}: E mismatch",
                preset.name(),
                member.id.0
            );
            assert!(
                (sec_edit.area() - sec_fm.area()).abs() < 1e-15,
                "preset '{}' M{}: A mismatch",
                preset.name(),
                member.id.0
            );
        }
        let _ = FrameSection3D::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
    }
}

#[test]
fn preset_ref_vec_preservation() {
    for &preset in model::PresetModel::all() {
        let m = preset.build_edit();
        let fm = m
            .to_frame_model()
            .unwrap_or_else(|e| panic!("preset '{}' to_frame_model failed: {e:?}", preset.name()));
        for member in m.members() {
            let idx = m.member_index(member.id).unwrap();
            let rv_edit = member.ref_vec;
            let rv_fm = fm.member(idx).unwrap().ref_vec();
            for k in 0..3 {
                assert!(
                    (rv_edit[k] - rv_fm[k]).abs() < 1e-15,
                    "preset '{}' M{}: ref_vec[{}] mismatch",
                    preset.name(),
                    member.id.0,
                    k
                );
            }
        }
    }
}

#[test]
fn member_id_stable_after_unrelated_deletion() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    let c = m.add_node(6.0, 0.0, 0.0).unwrap();
    let _m_ab = m.add_member(a, b).unwrap();
    let m_bc = m.add_member(b, c).unwrap();
    m.set_member_section(m_bc, 210e9, 81e9, 2e-3, 4e-6, 3e-6, 5e-6)
        .unwrap();
    let other_node = m.add_node(10.0, 10.0, 10.0).unwrap();
    m.delete_node(other_node).unwrap();
    let s = m.member_section(m_bc).unwrap();
    assert!((s.E() - 210e9).abs() < 1e-3);
    assert!((s.area() - 2e-3).abs() < 1e-15);
}

#[test]
fn section_edit_marks_stale() {
    use viewport_poc::app_state::Project;
    let mut p = Project::empty();
    p.load_model("Portal".into(), model::build_portal_edit());
    p.solve().unwrap();
    assert!(!p.results_are_stale());
    let model = p.model.as_mut().unwrap();
    let mid = model.members()[0].id;
    model
        .set_member_section(mid, 210e9, 81e9, 2e-3, 4e-6, 3e-6, 5e-6)
        .unwrap();
    assert!(p.results_are_stale());
    p.solve().unwrap();
    assert!(!p.results_are_stale());
}

#[test]
fn ref_vec_edit_marks_stale() {
    use viewport_poc::app_state::Project;
    let mut p = Project::empty();
    p.load_model("Portal".into(), model::build_portal_edit());
    p.solve().unwrap();
    assert!(!p.results_are_stale());
    let model = p.model.as_mut().unwrap();
    let mid = model.members()[0].id;
    model.set_member_ref_vec(mid, [1.0, 0.0, 0.0]).unwrap();
    assert!(p.results_are_stale());
}

// ── Numerical integration: axial deformation u = FL/EA ─────────────

#[test]
fn axial_deformation_matches_formula() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();

    let e = 200e9;
    let area = 1e-3;
    let iy = 2e-6;
    let iz = 1e-6;
    let j = 3e-6;
    let g = 80e9;
    m.set_member_section(mid, e, g, area, iy, iz, j).unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();

    let f = 1e3;
    m.set_nodal_load(b, NodalLoadSpec::new(f, 0.0, 0.0, 0.0, 0.0, 0.0))
        .unwrap();

    let fm = m.to_frame_model().unwrap();
    let result = fm.solve().expect("axial cantilever should solve");
    let b_idx = m.node_index(b).unwrap();
    let ux = result.displacements[b_idx * 6 + 0];
    let l = 3.0_f64;
    let expected = f * l / (e * area);
    assert!(
        (ux - expected).abs() / expected.abs() < 1e-4,
        "axial: ux={ux:.6e}, expected={expected:.6e}"
    );
}

#[test]
fn axial_deformation_doubles_when_area_halves() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();

    let e = 200e9;
    let area = 1e-3;
    m.set_member_section(mid, e, 80e9, area, 2e-6, 1e-6, 3e-6)
        .unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_nodal_load(b, NodalLoadSpec::new(1e3, 0.0, 0.0, 0.0, 0.0, 0.0))
        .unwrap();

    let fm = m.to_frame_model().unwrap();
    let result = fm.solve().unwrap();
    let b_idx = m.node_index(b).unwrap();
    let ux1 = result.displacements[b_idx * 6 + 0];

    m.set_member_section(mid, e, 80e9, area / 2.0, 2e-6, 1e-6, 3e-6)
        .unwrap();
    let fm2 = m.to_frame_model().unwrap();
    let result2 = fm2.solve().unwrap();
    let ux2 = result2.displacements[b_idx * 6 + 0];

    assert!(
        (ux2 / ux1 - 2.0).abs() < 1e-4,
        "halving A should double u: ux1={ux1:.6e}, ux2={ux2:.6e}"
    );
}

// ── Numerical integration: cantilever bending δ = FL³/(3EI) ─────────

#[test]
fn cantilever_bending_matches_formula() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();

    let e = 200e9;
    let iz = 1e-6;
    m.set_member_section(mid, e, 80e9, 1e-3, 2e-6, iz, 3e-6)
        .unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();

    let f = 1e3;
    m.set_nodal_load(b, NodalLoadSpec::new(0.0, 0.0, f, 0.0, 0.0, 0.0))
        .unwrap();

    let fm = m.to_frame_model().unwrap();
    let result = fm.solve().expect("bending cantilever should solve");
    let b_idx = m.node_index(b).unwrap();
    let uz = result.displacements[b_idx * 6 + 2];
    let l = 3.0_f64;
    let expected = f * l * l * l / (3.0 * e * iz);
    assert!(
        (uz - expected).abs() / expected.abs() < 1e-4,
        "bending: uz={uz:.6e}, expected={expected:.6e}"
    );
}

#[test]
fn cantilever_bending_doubles_when_i_halves() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(3.0, 0.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();

    let e = 200e9;
    let iz = 1e-6;
    m.set_member_section(mid, e, 80e9, 1e-3, 2e-6, iz, 3e-6)
        .unwrap();
    m.set_support(a, SupportSpec::fixed()).unwrap();
    m.set_nodal_load(b, NodalLoadSpec::new(0.0, 0.0, 1e3, 0.0, 0.0, 0.0))
        .unwrap();

    let fm = m.to_frame_model().unwrap();
    let result = fm.solve().unwrap();
    let b_idx = m.node_index(b).unwrap();
    let uz1 = result.displacements[b_idx * 6 + 2];

    m.set_member_section(mid, e, 80e9, 1e-3, 2e-6, iz / 2.0, 3e-6)
        .unwrap();
    let fm2 = m.to_frame_model().unwrap();
    let result2 = fm2.solve().unwrap();
    let uz2 = result2.displacements[b_idx * 6 + 2];

    assert!(
        (uz2 / uz1 - 2.0).abs() < 1e-4,
        "halving I should double δ: uz1={uz1:.6e}, uz2={uz2:.6e}"
    );
}

#[test]
fn edited_section_solve_remains_stable() {
    let mut m = model::build_portal_edit();
    let mid = m.members()[0].id;
    m.set_member_section(mid, 210e9, 81e9, 2e-3, 4e-6, 3e-6, 5e-6)
        .unwrap();
    let fm = m.to_frame_model().unwrap();
    assert!(fm.solve().is_ok());
}

#[test]
fn edited_ref_vec_solve_remains_stable() {
    let mut m = model::build_portal_edit();
    let mid = m.members()[0].id;
    m.set_member_ref_vec(mid, [0.3, 0.7, 0.5]).unwrap();
    let fm = m.to_frame_model().unwrap();
    assert!(fm.solve().is_ok());
}

// ── Revision tracking regression tests ───────────────────────────────

#[test]
fn revision_starts_at_zero() {
    let m = EditModel::new();
    assert_eq!(m.revision(), 0);
}

#[test]
fn add_node_bumps_revision() {
    let mut m = EditModel::new();
    m.add_node(0.0, 0.0, 0.0).unwrap();
    assert_eq!(m.revision(), 1);
    m.add_node(1.0, 0.0, 0.0).unwrap();
    assert_eq!(m.revision(), 2);
}

#[test]
fn update_node_bumps_revision() {
    let mut m = EditModel::new();
    let id = m.add_node(0.0, 0.0, 0.0).unwrap();
    assert_eq!(m.revision(), 1);
    m.update_node(id, 1.0, 2.0, 3.0).unwrap();
    assert_eq!(m.revision(), 2);
}

#[test]
fn update_node_no_op_preserves_revision() {
    let mut m = EditModel::new();
    let id = m.add_node(1.0, 2.0, 3.0).unwrap();
    let rev = m.revision();
    m.update_node(id, 1.0, 2.0, 3.0).unwrap();
    assert_eq!(m.revision(), rev);
}

#[test]
fn delete_node_bumps_revision() {
    let mut m = EditModel::new();
    let id = m.add_node(0.0, 0.0, 0.0).unwrap();
    let rev = m.revision();
    m.delete_node(id).unwrap();
    assert_eq!(m.revision(), rev + 1);
}

#[test]
fn add_member_bumps_revision() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(1.0, 0.0, 0.0).unwrap();
    let rev = m.revision();
    m.add_member(a, b).unwrap();
    assert_eq!(m.revision(), rev + 1);
}

#[test]
fn delete_member_bumps_revision() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(1.0, 0.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();
    let rev = m.revision();
    m.delete_member(mid).unwrap();
    assert_eq!(m.revision(), rev + 1);
}

#[test]
fn set_member_ref_vec_bumps_revision() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(1.0, 0.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();
    let rev = m.revision();
    m.set_member_ref_vec(mid, [0.0, 1.0, 0.0]).unwrap();
    assert_eq!(m.revision(), rev + 1);
}

#[test]
fn set_member_ref_vec_no_op_preserves_revision() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(1.0, 0.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();
    let rv = m.members()[0].ref_vec;
    let rev = m.revision();
    m.set_member_ref_vec(mid, rv).unwrap();
    assert_eq!(m.revision(), rev);
}

#[test]
fn set_member_section_bumps_revision() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(1.0, 0.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();
    let rev = m.revision();
    m.set_member_section(mid, 210e9, 81e9, 2e-3, 4e-6, 3e-6, 5e-6)
        .unwrap();
    assert_eq!(m.revision(), rev + 1);
}

#[test]
fn set_member_section_no_op_preserves_revision() {
    let mut m = EditModel::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(1.0, 0.0, 0.0).unwrap();
    let mid = m.add_member(a, b).unwrap();
    let s = &m.members()[0].section;
    let rev = m.revision();
    m.set_member_section(mid, s.E(), s.G(), s.area(), s.iy(), s.iz(), s.j())
        .unwrap();
    assert_eq!(m.revision(), rev);
}

#[test]
fn fix_dof_bumps_revision() {
    let mut m = EditModel::new();
    let id = m.add_node(0.0, 0.0, 0.0).unwrap();
    let rev = m.revision();
    m.fix_dof(id, Dof3D::Ux).unwrap();
    assert_eq!(m.revision(), rev + 1);
}

#[test]
fn fix_dof_no_op_preserves_revision() {
    let mut m = EditModel::new();
    let id = m.add_node(0.0, 0.0, 0.0).unwrap();
    m.fix_dof(id, Dof3D::Ux).unwrap();
    let rev = m.revision();
    m.fix_dof(id, Dof3D::Ux).unwrap();
    assert_eq!(m.revision(), rev);
}

#[test]
fn set_support_bumps_revision() {
    let mut m = EditModel::new();
    let id = m.add_node(0.0, 0.0, 0.0).unwrap();
    let rev = m.revision();
    m.set_support(id, SupportSpec::fixed()).unwrap();
    assert_eq!(m.revision(), rev + 1);
}

#[test]
fn set_support_no_op_preserves_revision() {
    let mut m = EditModel::new();
    let id = m.add_node(0.0, 0.0, 0.0).unwrap();
    m.set_support(id, SupportSpec::pinned()).unwrap();
    let rev = m.revision();
    m.set_support(id, SupportSpec::pinned()).unwrap();
    assert_eq!(m.revision(), rev);
}

#[test]
fn clear_support_bumps_revision() {
    let mut m = EditModel::new();
    let id = m.add_node(0.0, 0.0, 0.0).unwrap();
    m.set_support(id, SupportSpec::fixed()).unwrap();
    let rev = m.revision();
    m.clear_support(id).unwrap();
    assert_eq!(m.revision(), rev + 1);
}

#[test]
fn clear_support_no_op_preserves_revision() {
    let mut m = EditModel::new();
    let id = m.add_node(0.0, 0.0, 0.0).unwrap();
    let rev = m.revision();
    m.clear_support(id).unwrap();
    assert_eq!(m.revision(), rev);
}

#[test]
fn set_nodal_load_bumps_revision() {
    let mut m = EditModel::new();
    let id = m.add_node(0.0, 0.0, 0.0).unwrap();
    let rev = m.revision();
    m.set_nodal_load(
        id,
        NodalLoadSpec {
            fx: 1.0,
            fy: 0.0,
            fz: 0.0,
            mx: 0.0,
            my: 0.0,
            mz: 0.0,
        },
    )
    .unwrap();
    assert_eq!(m.revision(), rev + 1);
}

#[test]
fn set_nodal_load_no_op_preserves_revision() {
    let mut m = EditModel::new();
    let id = m.add_node(0.0, 0.0, 0.0).unwrap();
    let load = NodalLoadSpec {
        fx: 1.0,
        fy: 2.0,
        fz: 3.0,
        mx: 0.0,
        my: 0.0,
        mz: 0.0,
    };
    m.set_nodal_load(id, load).unwrap();
    let rev = m.revision();
    m.set_nodal_load(id, load).unwrap();
    assert_eq!(m.revision(), rev);
}

#[test]
fn clear_nodal_load_bumps_and_no_op() {
    let mut m = EditModel::new();
    let id = m.add_node(0.0, 0.0, 0.0).unwrap();

    // Clearing when no load exists — no-op.
    let rev0 = m.revision();
    m.clear_nodal_load(id).unwrap();
    assert_eq!(m.revision(), rev0);

    // Add a load, then clear it — bumps.
    m.set_nodal_load(
        id,
        NodalLoadSpec {
            fx: 1.0,
            fy: 0.0,
            fz: 0.0,
            mx: 0.0,
            my: 0.0,
            mz: 0.0,
        },
    )
    .unwrap();
    let rev1 = m.revision();
    m.clear_nodal_load(id).unwrap();
    assert_eq!(m.revision(), rev1 + 1);
}
