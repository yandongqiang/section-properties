//! FEM Lagrange Matrix Benchmark
//!
//! Benchmarks the largest real FEM Lagrange matrix (1182x1182 or similar)
//! Records: matrix size, nnz, factorization time, solve time, memory, residual, fallback reason
//!
//! Run with: cargo test --test fem_lagrange_benchmark -- --nocapture

use section_properties::mesh::MeshControl;
use section_properties::plastic::warping_fem::{
    FemWarpingSolution, compute_fem_warping_solution, diagnose_warping_fem,
};
use section_properties::section::Section;
use section_properties::section_library::ParametricSection;
use section_properties::section_library::steel::{AngleSection, ChannelSection, ISection};
use section_properties::section_properties::SectionProperties;
use std::time::Instant;

fn benchmark_section(name: &str, section: Section, nu: f64) {
    println!("\n{}", "=".repeat(80));
    println!("BENCHMARK: {}", name);
    println!("{}", "=".repeat(80));

    let props = SectionProperties::from_section(&section);

    // Run diagnostics first to get matrix size
    println!("\n--- Matrix Diagnostics ---");
    let diag = match diagnose_warping_fem(&section, name, nu) {
        Ok(d) => d,
        Err(e) => {
            println!("Diagnostics failed: {:?}", e);
            return;
        }
    };

    println!("n_dof:           {}", diag.n_dof);
    println!("n_elements:      {}", diag.n_elements);
    println!("K nnz (est):     {}", diag.n_dof * 7); // approximate for Tri6

    // Time the full FEM solution
    println!("\n--- Timing Full FEM Solution ---");
    let start = Instant::now();
    let fem_result: FemWarpingSolution =
        match compute_fem_warping_solution(&section, &props, nu, MeshControl::Fine) {
            Ok(r) => r,
            Err(e) => {
                println!("FEM failed: {:?}", e);
                return;
            }
        };
    let elapsed = start.elapsed();

    println!("Total time:      {:.3?}", elapsed);
    println!("J:               {:.6e}", fem_result.j);
    println!("Iw:              {:.6e}", fem_result.iw);
    println!(
        "Shear center:    ({:.6}, {:.6})",
        fem_result.shear_center.x, fem_result.shear_center.y
    );
    println!("omega_max:       {:.6e}", fem_result.omega_max);
    println!("Used exact:      {}", fem_result.used_exact_solver);
    println!("Used reg:        {}", fem_result.used_regularization);
    println!("Exact residual:  {:.2e}", fem_result.exact_residual);
    println!("Reg residual:    {:.2e}", fem_result.regularized_residual);
    println!(
        "Analytical fallback: {}",
        fem_result.used_analytical_fallback
    );

    // Record fallback reason
    let fallback_reason = if fem_result.used_analytical_fallback {
        "Negative J"
    } else if fem_result.used_regularization {
        if fem_result.exact_residual > 0.0 {
            "Exact solver residual check failed"
        } else {
            "Exact solver unavailable"
        }
    } else {
        "None (exact solver used)"
    };
    println!("Fallback reason: {}", fallback_reason);

    // Memory estimate
    let nnz_estimate = diag.n_dof * 7; // Tri6 ~7 entries per row
    let memory_mb = (nnz_estimate as f64 * 8.0 * 3.0) / (1024.0 * 1024.0); // 3 matrices * 8 bytes
    println!("Memory estimate: {:.2} MB (3 x nnz x 8 bytes)", memory_mb);
}

#[test]
fn benchmark_channel_200x75() {
    let channel = ChannelSection::new(200.0, 75.0, 8.0, 10.0, 12.0, 0.0);
    let section = channel.build();
    benchmark_section("Channel 200x75x8x10", section, 0.3);
}

#[test]
fn benchmark_channel_300x100_thin() {
    let channel = ChannelSection::new(300.0, 100.0, 3.0, 6.0, 8.0, 0.0);
    let section = channel.build();
    benchmark_section("Channel 300x100x3x6 (thin)", section, 0.3);
}

#[test]
fn benchmark_angle_100x100() {
    let angle = AngleSection::equal_leg(100.0, 8.0);
    let section = angle.build();
    benchmark_section("Angle 100x100x8", section, 0.3);
}

#[test]
fn benchmark_angle_150x75() {
    let angle = AngleSection::new(150.0, 75.0, 10.0, 12.0, 15.0);
    let section = angle.build();
    benchmark_section("Angle 150x75x10", section, 0.3);
}

#[test]
fn benchmark_i_section_300x150() {
    let i_section = ISection::new(300.0, 150.0, 8.0, 12.0, 15.0);
    let section = i_section.build();
    benchmark_section("I-section 300x150", section, 0.3);
}

#[test]
fn benchmark_large_channel() {
    // Large channel to approach 1182x1182
    let channel = ChannelSection::new(400.0, 150.0, 10.0, 15.0, 20.0, 0.0);
    let section = channel.build();
    benchmark_section("Channel 400x150x10x15 (large)", section, 0.3);
}

#[test]
fn benchmark_large_angle() {
    let angle = AngleSection::new(200.0, 200.0, 16.0, 18.0, 24.0);
    let section = angle.build();
    benchmark_section("Angle 200x200x16", section, 0.3);
}

#[test]
fn benchmark_large_i_section() {
    let i_section = ISection::new(600.0, 300.0, 16.0, 20.0, 25.0);
    let section = i_section.build();
    benchmark_section("I-section 600x300 (large)", section, 0.3);
}

#[test]
fn benchmark_mesh_convergence_study() {
    // Test mesh convergence for a channel section
    println!("\n{}", "=".repeat(80));
    println!("MESH CONVERGENCE STUDY: Channel 200x75");
    println!("{}", "=".repeat(80));

    let channel = ChannelSection::new(200.0, 75.0, 8.0, 10.0, 12.0, 0.0);
    let section = channel.build();
    let props = SectionProperties::from_section(&section);
    let nu = 0.3;

    let controls = [
        ("Coarse", MeshControl::Coarse),
        ("Normal", MeshControl::Normal),
        ("Fine", MeshControl::Fine),
    ];

    let mut prev_j = 0.0;

    for (name, control) in &controls {
        println!("\n--- Mesh: {} ---", name);

        let start = Instant::now();
        let fem_result = compute_fem_warping_solution(&section, &props, nu, *control);
        let elapsed = start.elapsed();

        match fem_result {
            Ok(r) => {
                println!("J: {:.6e}", r.j);
                println!("Time: {:.3?}", elapsed);
                println!("n_dof: {}", r.omega.len());
                if prev_j > 0.0 {
                    let conv = (r.j - prev_j).abs() / prev_j;
                    println!(
                        "Convergence: {:.4}% {}",
                        conv * 100.0,
                        if conv < 0.01 { "✓" } else { "✗" }
                    );
                }
                prev_j = r.j;

                if r.j <= 0.0 {
                    println!("!!! NEGATIVE J: {:.6e} !!!", r.j);
                }
            }
            Err(e) => {
                println!("FAILED: {:?}", e);
            }
        }
    }
}
