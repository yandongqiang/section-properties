//! Performance tests — scene DTO build time for increasing model sizes.

use std::time::Instant;
use tauri_viewport_poc_lib::{build_grid_frame, build_scene};

#[test]
fn portal_scene_build_time() {
    let model = build_grid_frame(1, 1);
    let start = Instant::now();
    let scene = build_scene(&model).expect("1x1 grid should build");
    let elapsed = start.elapsed();
    assert!(scene.n_nodes > 0);
    println!(
        "1x1 grid ({} nodes, {} members): {:?}",
        scene.n_nodes, scene.n_members, elapsed
    );
}

#[test]
fn grid_5x5_scene_build_time() {
    let model = build_grid_frame(5, 5);
    let start = Instant::now();
    let scene = build_scene(&model).expect("5x5 grid should build");
    let elapsed = start.elapsed();
    println!(
        "5x5 grid ({} nodes, {} members): {:?}",
        scene.n_nodes, scene.n_members, elapsed
    );
}

#[test]
fn grid_10x10_scene_build_time() {
    let model = build_grid_frame(10, 10);
    let start = Instant::now();
    let scene = build_scene(&model).expect("10x10 grid should build");
    let elapsed = start.elapsed();
    println!(
        "10x10 grid ({} nodes, {} members): {:?}",
        scene.n_nodes, scene.n_members, elapsed
    );
}

#[test]
fn grid_10x10_dto_serialisation_time() {
    let model = build_grid_frame(10, 10);
    let scene = build_scene(&model).unwrap();
    let start = Instant::now();
    let json = serde_json::to_string(&scene).expect("should serialize");
    let elapsed = start.elapsed();
    println!(
        "10x10 grid serialisation: {} bytes, {:?}",
        json.len(),
        elapsed
    );
    assert!(!json.is_empty());
}

#[test]
fn grid_10x10_repeated_build_no_degradation() {
    let model = build_grid_frame(10, 10);
    let mut times = Vec::new();
    for _ in 0..5 {
        let start = Instant::now();
        let _scene = build_scene(&model).unwrap();
        times.push(start.elapsed());
    }
    let first = times[0].as_micros();
    let last = times[4].as_micros();
    println!(
        "Repeated build times (µs): {:?}",
        times.iter().map(|t| t.as_micros()).collect::<Vec<_>>()
    );
    // Allow some variance but last build shouldn't be 10x slower than first
    assert!(
        last < first * 10,
        "degradation: first={first}µs last={last}µs"
    );
}
