use section_properties::section_library::steel::{ChannelSection, ISection, AngleSection};
use section_properties::section_library::ParametricSection;
use section_properties::plastic::warping_fem::export_exact_augmented_system_from_python_mesh;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Channel 200x75
    export_exact_augmented_system_from_python_mesh(
        "python_global_Channel_200x75.json",
        "Channel_200x75",
        "rust_augmented_Channel_200x75.json",
    )?;
    
    // I-section 300x150
    export_exact_augmented_system_from_python_mesh(
        "python_global_I_section_300x150.json",
        "I_section_300x150",
        "rust_augmented_I_section_300x150.json",
    )?;
    
    // Angle 100x100
    export_exact_augmented_system_from_python_mesh(
        "python_global_Angle_100x100.json",
        "Angle_100x100",
        "rust_augmented_Angle_100x100.json",
    )?;
    
    // Thin Channel 300x100x3
    export_exact_augmented_system_from_python_mesh(
        "python_global_Thin_Channel_300x100x3.json",
        "Thin_Channel_300x100x3",
        "rust_augmented_Thin_Channel_300x100x3.json",
    )?;
    
    println!("All augmented systems exported successfully!");
    Ok(())
}