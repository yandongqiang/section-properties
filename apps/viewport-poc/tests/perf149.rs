//! Phase 149 audit — performance measurements.

use std::time::Instant;

use viewport_poc::{
    app_state::Project,
    edit_model::{EditModel, MemberId, NodalLoadSpec, SupportSpec},
    model,
    scene::ViewportScene,
};

fn median(times: &mut [f64]) -> f64 {
    times.sort_by(|a, b| a.partial_cmp(b).unwrap());
    times[times.len() / 2]
}

#[test]
fn perf_pipeline() {
    let n_reps = 20;

    // ── Preset: Portal ──
    let mut t_edit = Vec::with_capacity(n_reps);
    let mut t_convert = Vec::with_capacity(n_reps);
    let mut t_solve = Vec::with_capacity(n_reps);
    let mut t_scene = Vec::with_capacity(n_reps);

    for _ in 0..n_reps {
        let t0 = Instant::now();
        let edit = model::build_portal_edit();
        let t1 = Instant::now();
        t_edit.push(t1.duration_since(t0).as_secs_f64() * 1e6);

        let t0 = Instant::now();
        let _frame = edit.to_frame_model().unwrap();
        let t1 = Instant::now();
        t_convert.push(t1.duration_since(t0).as_secs_f64() * 1e6);

        let t0 = Instant::now();
        let mut p = Project::empty();
        p.load_model("Portal".into(), edit);
        p.solve().unwrap();
        let t1 = Instant::now();
        t_solve.push(t1.duration_since(t0).as_secs_f64() * 1e6);

        let t0 = Instant::now();
        let _scene = ViewportScene::from_edit_model(p.model.as_ref().unwrap());
        let t1 = Instant::now();
        t_scene.push(t1.duration_since(t0).as_secs_f64() * 1e6);
    }

    let portal_nodes = model::build_portal_edit().nodes().len();
    let portal_members = model::build_portal_edit().members().len();

    eprintln!("=== Portal ({portal_nodes} nodes, {portal_members} members, {n_reps} reps) ===");
    eprintln!(
        "  EditModel construction: {:.1} us (median)",
        median(&mut t_edit)
    );
    eprintln!(
        "  to_frame_model:         {:.1} us (median)",
        median(&mut t_convert)
    );
    eprintln!(
        "  solve:                  {:.1} us (median)",
        median(&mut t_solve)
    );
    eprintln!(
        "  scene generation:       {:.1} us (median)",
        median(&mut t_scene)
    );

    // ── Generated: 10x10 grid ──
    let mut t_edit_g = Vec::with_capacity(n_reps);
    let mut t_solve_g = Vec::with_capacity(n_reps);
    let mut t_scene_g = Vec::with_capacity(n_reps);

    for _ in 0..n_reps {
        let t0 = Instant::now();
        let edit = model::build_grid_edit(10, 10);
        let t1 = Instant::now();
        t_edit_g.push(t1.duration_since(t0).as_secs_f64() * 1e6);

        let t0 = Instant::now();
        let mut p = Project::empty();
        p.load_model("Grid".into(), edit);
        p.solve().unwrap();
        let t1 = Instant::now();
        t_solve_g.push(t1.duration_since(t0).as_secs_f64() * 1e6);

        let t0 = Instant::now();
        let _scene = ViewportScene::from_edit_model(p.model.as_ref().unwrap());
        let t1 = Instant::now();
        t_scene_g.push(t1.duration_since(t0).as_secs_f64() * 1e6);
    }

    let grid_edit = model::build_grid_edit(10, 10);
    let grid_nodes = grid_edit.nodes().len();
    let grid_members = grid_edit.members().len();
    let grid_dofs = grid_nodes * 6;

    eprintln!(
        "=== Grid10x10 ({grid_nodes} nodes, {grid_members} members, {grid_dofs} DOFs, {n_reps} reps) ==="
    );
    eprintln!(
        "  EditModel construction: {:.1} us (median)",
        median(&mut t_edit_g)
    );
    eprintln!(
        "  solve:                  {:.1} us (median)",
        median(&mut t_solve_g)
    );
    eprintln!(
        "  scene generation:       {:.1} us (median)",
        median(&mut t_scene_g)
    );

    // ── Generated: 20x20 grid ──
    let mut t_edit_20 = Vec::with_capacity(n_reps);
    let mut t_solve_20 = Vec::with_capacity(n_reps);
    let mut t_scene_20 = Vec::with_capacity(n_reps);

    for _ in 0..n_reps {
        let t0 = Instant::now();
        let edit = build_grid(20, 20);
        let t1 = Instant::now();
        t_edit_20.push(t1.duration_since(t0).as_secs_f64() * 1e6);

        let t0 = Instant::now();
        let mut p = Project::empty();
        p.load_model("Grid20".into(), edit);
        p.solve().unwrap();
        let t1 = Instant::now();
        t_solve_20.push(t1.duration_since(t0).as_secs_f64() * 1e6);

        let t0 = Instant::now();
        let _scene = ViewportScene::from_edit_model(p.model.as_ref().unwrap());
        let t1 = Instant::now();
        t_scene_20.push(t1.duration_since(t0).as_secs_f64() * 1e6);
    }

    let grid20_edit = build_grid(20, 20);
    let g20_nodes = grid20_edit.nodes().len();
    let g20_members = grid20_edit.members().len();
    let g20_dofs = g20_nodes * 6;

    eprintln!(
        "=== Grid20x20 ({g20_nodes} nodes, {g20_members} members, {g20_dofs} DOFs, {n_reps} reps) ==="
    );
    eprintln!(
        "  EditModel construction: {:.1} us (median)",
        median(&mut t_edit_20)
    );
    eprintln!(
        "  solve:                  {:.1} us (median)",
        median(&mut t_solve_20)
    );
    eprintln!(
        "  scene generation:       {:.1} us (median)",
        median(&mut t_scene_20)
    );
}

fn build_grid(nx: usize, ny: usize) -> EditModel {
    let mut m = EditModel::new();
    let mut nodes = Vec::with_capacity((nx + 1) * (ny + 1));
    for i in 0..=nx {
        for j in 0..=ny {
            nodes.push(m.add_node(i as f64, j as f64, 0.0).unwrap());
        }
    }
    for i in 0..nx {
        for j in 0..=ny {
            let a = nodes[i * (ny + 1) + j];
            let b = nodes[(i + 1) * (ny + 1) + j];
            m.add_member(a, b).unwrap();
        }
    }
    for i in 0..=nx {
        for j in 0..ny {
            let a = nodes[i * (ny + 1) + j];
            let b = nodes[i * (ny + 1) + j + 1];
            m.add_member(a, b).unwrap();
        }
    }
    let n0 = nodes[0];
    let n_last = nodes[nx * (ny + 1) + ny];
    m.set_support(n0, SupportSpec::fixed()).unwrap();
    m.set_support(n_last, SupportSpec::fixed()).unwrap();
    let mid = nodes[nx / 2 * (ny + 1) + ny / 2];
    m.set_nodal_load(
        mid,
        NodalLoadSpec {
            fx: 0.0,
            fy: 0.0,
            fz: -1000.0,
            mx: 0.0,
            my: 0.0,
            mz: 0.0,
        },
    )
    .unwrap();
    m
}
