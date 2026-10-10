//! Integration tests for application state behavior.

use viewport_poc::{
    app_state::{AppMessage, MessageLevel, Project},
    edit_model::NodeId,
    model::{self, PresetModel},
    scene::ViewportScene,
    selection::SelectionState,
};

#[test]
fn empty_project_has_no_scene() {
    let p = Project::empty();
    assert!(p.is_empty());
    assert!(!p.has_results());
}

#[test]
fn load_and_solve_produces_result() {
    let mut p = Project::empty();
    p.load_model("Portal".into(), model::build_portal_edit());
    p.solve().unwrap();
    assert!(p.has_results());
    assert!(p.analysis.as_ref().unwrap().result.displacements.len() > 0);
}

#[test]
fn switching_model_marks_results_stale() {
    let mut p = Project::empty();
    p.load_model("Portal".into(), model::build_portal_edit());
    p.solve().unwrap();
    assert!(!p.results_are_stale());

    p.load_model("Inclined".into(), model::build_inclined_edit());
    assert!(p.results_are_stale());
}

#[test]
fn stale_results_still_accessible() {
    let mut p = Project::empty();
    p.load_model("Portal".into(), model::build_portal_edit());
    p.solve().unwrap();
    p.load_model("Inclined".into(), model::build_inclined_edit());

    assert!(p.results_are_stale());
    let analysis = p.analysis.as_ref().unwrap();
    assert!(analysis.result.displacements.len() > 0);
}

#[test]
fn re_solve_clears_staleness() {
    let mut p = Project::empty();
    p.load_model("Portal".into(), model::build_portal_edit());
    p.solve().unwrap();
    p.load_model("Inclined".into(), model::build_inclined_edit());
    assert!(p.results_are_stale());

    p.solve().unwrap();
    assert!(!p.results_are_stale());
}

#[test]
fn selection_validation_clears_invalid_node() {
    let mut p = Project::empty();
    let m = model::build_portal_edit();
    p.load_model("Portal".into(), m);
    p.solve().unwrap();

    let model = p.model.as_ref().unwrap();
    let scene = ViewportScene::from_edit_model(model);
    let n = scene.n_nodes();

    let sel = SelectionState::Node(NodeId(n as u32 + 100));
    assert!(scene.node_index(sel.node_id().unwrap()).is_none());
}

#[test]
fn selection_validation_clears_invalid_member() {
    let mut p = Project::empty();
    let m = model::build_portal_edit();
    p.load_model("Portal".into(), m);
    p.solve().unwrap();

    let model = p.model.as_ref().unwrap();
    let scene = ViewportScene::from_edit_model(model);
    let m_count = scene.n_members();

    let sel = SelectionState::Member(viewport_poc::edit_model::MemberId(m_count as u32 + 100));
    assert!(scene.member_index(sel.member_id().unwrap()).is_none());
}

#[test]
fn all_presets_solve_and_produce_results() {
    for &preset in PresetModel::all() {
        let mut p = Project::empty();
        p.load_model(preset.name().into(), preset.build_edit());
        p.solve()
            .unwrap_or_else(|e| panic!("preset '{}' solve failed: {e}", preset.name()));

        let result = p.analysis.as_ref().unwrap();
        assert!(
            result.result.displacements.len() > 0,
            "preset '{}' has no displacements",
            preset.name()
        );
    }
}

#[test]
fn messages_accumulate() {
    let mut msgs = vec![AppMessage::info("start")];
    msgs.push(AppMessage::warning("careful"));
    msgs.push(AppMessage::error("broken"));
    assert_eq!(msgs.len(), 3);
    assert_eq!(msgs[0].level, MessageLevel::Info);
    assert_eq!(msgs[1].level, MessageLevel::Warning);
    assert_eq!(msgs[2].level, MessageLevel::Error);
}

#[test]
fn project_clear_removes_everything() {
    let mut p = Project::empty();
    p.load_model("Portal".into(), model::build_portal_edit());
    p.solve().unwrap();
    assert!(!p.is_empty());
    assert!(p.has_results());

    p.clear();
    assert!(p.is_empty());
    assert!(!p.has_results());
    assert!(!p.results_are_stale());
}

#[test]
fn deformation_scale_does_not_trigger_solve() {
    let mut p = Project::empty();
    p.load_model("Portal".into(), model::build_portal_edit());
    p.solve().unwrap();
    let result_ref = p.analysis.as_ref().unwrap();
    let original_displacements = result_ref.result.displacements.clone();

    let model = p.model.as_ref().unwrap();
    let scene = ViewportScene::from_edit_model(model);
    let _deformed = scene.deformed_positions(&original_displacements, 500.0);

    let result_after = p.analysis.as_ref().unwrap();
    assert_eq!(
        result_after.result.displacements, original_displacements,
        "deformation scale change must not modify analysis results"
    );
    assert!(!p.results_are_stale());
}

#[test]
fn mark_analysis_stale_works() {
    let mut p = Project::empty();
    p.load_model("Portal".into(), model::build_portal_edit());
    p.solve().unwrap();
    assert!(!p.results_are_stale());

    p.mark_analysis_stale();
    assert!(p.results_are_stale());
}

#[test]
fn mark_analysis_stale_no_op_without_results() {
    let mut p = Project::empty();
    p.load_model("Portal".into(), model::build_portal_edit());
    assert!(!p.has_results());

    p.mark_analysis_stale();
    assert!(!p.results_are_stale());
}
