//! Performance: measure projection cost for small and medium synthetic models.

use std::time::Instant;
use viewport_poc::{camera::Camera3D, model, scene::ViewportScene};

#[test]
fn projection_performance() {
    println!("\n=== Performance: projection cost (1000-frame average) ===");
    println!("  Environment: Windows, Rust 1.97.1, release mode");
    println!("  Methodology: measure 3D→2D projection of all nodes + member endpoints");
    println!("  Rendering mode: software projection via egui 2D painter (no custom GPU shaders)\n");

    for n in [1, 2, 3, 5, 8, 10] {
        let m = model::build_grid_edit(n, n);
        let scene = ViewportScene::from_edit_model(&m);
        let (min, max) = scene.bounding_box();
        let cam = Camera3D::fit(min, max);
        let w = 1200.0;
        let h = 800.0;

        let nodes = scene.n_nodes();
        let members = scene.n_members();

        let start = Instant::now();
        for _ in 0..1000 {
            for &pos in &scene.node_positions {
                let _ = cam.project(pos, w, h);
            }
            for &(ni, nj) in &scene.member_nodes {
                let _ = cam.project(scene.node_positions[ni], w, h);
                let _ = cam.project(scene.node_positions[nj], w, h);
            }
        }
        let elapsed = start.elapsed();
        let per_frame_us = elapsed.as_micros() as f64 / 1000.0;
        let fps = if per_frame_us > 0.0 {
            1e6 / per_frame_us
        } else {
            f64::INFINITY
        };
        println!(
            "  {n}x{n} grid: {nodes} nodes, {members} members | {per_frame_us:.1} µs/frame | {fps:.0} fps (projection only)"
        );
    }
    println!("\n  Note: Full frame time includes egui layout, painting, and compositing.");
    println!("  These numbers measure only the viewport projection cost.\n");
}
