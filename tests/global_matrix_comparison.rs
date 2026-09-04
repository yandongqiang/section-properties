//! Global Warping Matrix Cross-Validation
//!
//! Exports global K, F, C matrices from Rust and compares with Python assembly.
//!
//! Run with: cargo test --test global_matrix_comparison -- --nocapture

use section_properties::section::Section;
use section_properties::section_library::steel::{ChannelSection, ISection, AngleSection};
use section_properties::section_library::ParametricSection;
use section_properties::plastic::warping_fem::export_global_warping_matrices;
use std::fs;

fn test_section(section_name: &str, section: Section, output_dir: &str) {
    let output_path = format!("{}/{}.json", output_dir, section_name.replace(' ', "_"));
    println!("\nExporting {} to {}", section_name, output_path);
    export_global_warping_matrices(&section, section_name, &output_path).expect("Export failed");
    println!("✓ Exported successfully");
}

#[test]
fn export_global_matrices() {
    // Create output directory
    let output_dir = "global_matrix_exports";
    fs::create_dir_all(output_dir).expect("Failed to create output directory");
    
    // Test 1: Channel 200x75x8x10
    let channel = ChannelSection::new(200.0, 75.0, 8.0, 10.0, 12.0, 0.0);
    let section = channel.build();
    test_section("Channel_200x75", section, "global_matrix_exports");
    
    // Test 2: I-section 300x150
    let i_section = ISection::new(300.0, 150.0, 8.0, 12.0, 15.0);
    let section = i_section.build();
    test_section("I_section_300x150", section, "global_matrix_exports");
    
    // Test 3: Angle 100x100x8
    let angle = AngleSection::new(100.0, 100.0, 8.0, 10.0, 5.0);
    let section = angle.build();
    test_section("Angle_100x100", section, "global_matrix_exports");
    
    // Test 4: Thin Channel 300x100x3
    let channel = ChannelSection::new(300.0, 100.0, 3.0, 6.0, 8.0, 0.0);
    let section = channel.build();
    test_section("Thin_Channel_300x100x3", section, "global_matrix_exports");
}