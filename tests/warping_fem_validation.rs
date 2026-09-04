//! Warping FEM Validation
//!
//! Validates complete warping FEM pipeline for thin-wall sections.
//! Checks for negative J issue and verifies FEM accuracy.
//!
//! Run with: cargo test --test warping_fem_validation -- --nocapture

use section_properties::section::Section;
use section_properties::section_library::steel::{ChannelSection, ISection, AngleSection};
use section_properties::section_library::ParametricSection;
use section_properties::section_properties::SectionProperties;
use section_properties::plastic::warping_fem::{compute_fem_warping_solution, FemWarpingSolution};
use section_properties::mesh::MeshControl;

fn print_section_header(name: &str) {
    println!("\n{}", "=".repeat(70));
    println!("  WARPING FEM VALIDATION: {}", name);
    println!("{}", "=".repeat(70));
}

fn run_warping_fem_test(name: &str, section: Section) {
    print_section_header(name);

    let nu = 0.3;
    let props = SectionProperties::from_section(&section);

    println!("\n--- FEM Warping Solution ---");
    let fem_result: FemWarpingSolution = match compute_fem_warping_solution(&section, &props, nu, MeshControl::Fine) {
        Ok(r) => {
            println!("✓ FEM calculation SUCCEEDED");
            r
        }
        Err(e) => {
            println!("✗ FEM calculation FAILED: {:?}", e);
            println!("  → Would use analytical fallback");
            return;
        }
    };

    // Print FEM results
    println!("\n--- FEM Results ---");
    println!("J (FEM):            {:.6e}", fem_result.j);
    println!("Iw (FEM):           {:.6e}", fem_result.iw);
    println!("Shear center (FEM): ({:.6}, {:.6})", fem_result.shear_center.x, fem_result.shear_center.y);
    println!("βx:                 {:.6e}", fem_result.beta_x_plus);
    println!("βy:                 {:.6e}", fem_result.beta_y_plus);
    println!("β11:                {:.6e}", fem_result.beta_11_plus);
    println!("β22:                {:.6e}", fem_result.beta_22_plus);
    println!("Asx:                {:.6e}", fem_result.a_sx);
    println!("Asy:                {:.6e}", fem_result.a_sy);
    println!("δs:                 {:.6e}", fem_result.delta_s);
    println!("ω_max:              {:.6e}", fem_result.omega_max);
    println!("τ_sv_max:           {:.6e}", fem_result.tau_sv_max);

    // Check for negative J (critical issue)
    if fem_result.j <= 0.0 {
        println!("\n!!! CRITICAL: NEGATIVE J DETECTED: {:.6e} !!!", fem_result.j);
    }

    // Verify basic sanity
    let passed = fem_result.j > 0.0 && fem_result.iw >= 0.0 && fem_result.omega_max >= 0.0;
    println!("\n--- Summary ---");
    if passed {
        println!("✓ {}: FEM warping solution VALIDATED", name);
    } else {
        println!("✗ {}: FEM warping solution HAS ISSUES", name);
    }
}

#[test]
fn warping_fem_channel() {
    // Standard channel section - thin wall
    let channel = ChannelSection::new(200.0, 75.0, 8.0, 10.0, 12.0, 0.0);
    let section = channel.build();
    run_warping_fem_test("Channel 200x75x8x10 (UPN200)", section);
}

#[test]
fn warping_fem_channel_thin() {
    // Thin-wall channel - prone to negative J
    let channel = ChannelSection::new(150.0, 50.0, 4.0, 5.0, 8.0, 0.0);
    let section = channel.build();
    run_warping_fem_test("Channel 150x50x4x5 (thin)", section);
}

#[test]
fn warping_fem_angle_equal() {
    // Equal leg angle
    let angle = AngleSection::new(100.0, 100.0, 8.0, 10.0, 12.0);
    let section = angle.build();
    run_warping_fem_test("Angle 100x100x8 (equal)", section);
}

#[test]
fn warping_fem_angle_unequal() {
    // Unequal leg angle
    let angle = AngleSection::new(150.0, 75.0, 10.0, 12.0, 15.0);
    let section = angle.build();
    run_warping_fem_test("Angle 150x75x10 (unequal)", section);
}

#[test]
fn warping_fem_thin_wall_channel() {
    // Very thin wall channel
    let channel = ChannelSection::new(300.0, 100.0, 3.0, 6.0, 8.0, 0.0);
    let section = channel.build();
    run_warping_fem_test("Channel 300x100x3x6 (very thin)", section);
}

#[test]
fn warping_fem_i_section() {
    // I-section for comparison (not thin-wall)
    let i_section = ISection::new(300.0, 150.0, 8.0, 12.0, 15.0);
    let section = i_section.build();
    run_warping_fem_test("I-section 300x150", section);
}

#[test]
fn warping_fem_mesh_convergence() {
    // Test mesh convergence for thin-wall channel
    print_section_header("Mesh Convergence: Channel 200x75");

    let channel = ChannelSection::new(200.0, 75.0, 8.0, 10.0, 12.0, 0.0);
    let section = channel.build();
    let props = SectionProperties::from_section(&section);

    let controls = [
        ("Coarse", MeshControl::Coarse),
        ("Normal", MeshControl::Normal),
        ("Fine", MeshControl::Fine),
    ];

    let mut prev_j = 0.0;

    for (name, control) in &controls {
        println!("\n--- Mesh: {} ---", name);

        let fem_result = compute_fem_warping_solution(&section, &props, 0.3, *control);

        match fem_result {
            Ok(r) => {
                println!("J: {:.6e}", r.j);
                if prev_j > 0.0 {
                    let conv = (r.j - prev_j).abs() / prev_j;
                    println!("Convergence: {:.4}% {}", conv * 100.0, if conv < 0.01 { "✓" } else { "✗" });
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