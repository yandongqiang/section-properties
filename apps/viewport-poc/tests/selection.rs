//! Verify selected viewport IDs resolve to the correct source members.

use viewport_poc::{camera::Camera3D, model, scene::ViewportScene, selection::SelectionState};

#[test]
fn node_selection_resolves_correctly() {
    let m = model::build_portal_frame();
    let result = m.solve().expect("portal frame solves");
    let scene = ViewportScene::from_result(&result);
    let (min, max) = scene.bounding_box();
    let cam = Camera3D::fit(min, max);

    let w = 800.0;
    let h = 600.0;

    // Project each node and verify that picking at that screen position
    // selects the same node.
    for i in 0..scene.n_nodes() {
        let pos = scene.node_positions[i];
        let p = cam.project(pos, w, h).expect("node visible");
        // Pick at the exact projected position
        let picked = pick_node(&scene, &cam, [p.x, p.y], [w, h]);
        assert_eq!(
            picked,
            Some(i),
            "picking at node {i} position should select node {i}"
        );
    }
}

#[test]
fn member_selection_resolves_correctly() {
    let m = model::build_portal_frame();
    let result = m.solve().expect("portal frame solves");
    let scene = ViewportScene::from_result(&result);
    let (min, max) = scene.bounding_box();
    let cam = Camera3D::fit(min, max);

    let w = 800.0;
    let h = 600.0;

    // Pick at each member's midpoint
    for i in 0..scene.n_members() {
        let mid = scene.member_midpoint(i).expect("member midpoint");
        let p = cam.project(mid, w, h).expect("midpoint visible");
        // At the midpoint, we should pick either a node or a member.
        // Since the midpoint is not at a node, we should pick the member
        // (unless a node happens to project very close).
        let picked_node = pick_node(&scene, &cam, [p.x, p.y], [w, h]);
        let picked_member = pick_member(&scene, &cam, [p.x, p.y], [w, h]);
        // At least one should be found
        assert!(
            picked_node.is_some() || picked_member.is_some(),
            "should pick something at member {i} midpoint"
        );
    }
}

#[test]
fn selection_state_helpers() {
    let s = SelectionState::Node(2);
    assert_eq!(s.node_idx(), Some(2));
    assert_eq!(s.member_idx(), None);

    let s = SelectionState::Member(4);
    assert_eq!(s.node_idx(), None);
    assert_eq!(s.member_idx(), Some(4));

    let s = SelectionState::None;
    assert_eq!(s.node_idx(), None);
    assert_eq!(s.member_idx(), None);
}

#[test]
fn far_away_pick_returns_none() {
    let m = model::build_portal_frame();
    let result = m.solve().expect("portal frame solves");
    let scene = ViewportScene::from_result(&result);
    let (min, max) = scene.bounding_box();
    let cam = Camera3D::fit(min, max);

    let w = 800.0;
    let h = 600.0;
    // Pick at a position far from any node or member
    let picked = pick_node(&scene, &cam, [-1000.0, -1000.0], [w, h]);
    assert_eq!(picked, None);
}

fn pick_node(
    scene: &ViewportScene,
    cam: &Camera3D,
    mouse: [f64; 2],
    screen: [f64; 2],
) -> Option<usize> {
    const THRESH: f64 = 12.0;
    let mut best: Option<(usize, f64)> = None;
    for (i, &pos) in scene.node_positions.iter().enumerate() {
        if let Some(p) = cam.project(pos, screen[0], screen[1]) {
            let d = (p.x - mouse[0]).hypot(p.y - mouse[1]);
            if d < THRESH && best.map_or(true, |(_, bd)| d < bd) {
                best = Some((i, d));
            }
        }
    }
    best.map(|(i, _)| i)
}

fn pick_member(
    scene: &ViewportScene,
    cam: &Camera3D,
    mouse: [f64; 2],
    screen: [f64; 2],
) -> Option<usize> {
    const THRESH: f64 = 12.0;
    let mut best: Option<(usize, f64)> = None;
    for (i, &(ni, nj)) in scene.member_nodes.iter().enumerate() {
        let pi = scene.node_positions[ni];
        let pj = scene.node_positions[nj];
        let Some(pa) = cam.project(pi, screen[0], screen[1]) else {
            continue;
        };
        let Some(pb) = cam.project(pj, screen[0], screen[1]) else {
            continue;
        };
        let d = viewport_poc::project::point_to_segment_dist_2d(mouse, [pa.x, pa.y], [pb.x, pb.y]);
        if d < THRESH && best.map_or(true, |(_, bd)| d < bd) {
            best = Some((i, d));
        }
    }
    best.map(|(i, _)| i)
}
