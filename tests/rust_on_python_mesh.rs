//! Run FEM on Python-exported meshes for cross-validation

use section_properties::plastic::warping_fem::run_fem_on_python_mesh;

#[test]
fn run_fem_on_python_meshes() {
    let meshes = [
        "Channel_200x75",
        "I_section_300x150",
        "Angle_100x100",
        "Thin_Channel_300x100x3",
    ];
    
    for name in meshes {
        let mesh_path = format!("python_mesh_{}.json", name);
        let output_path = format!("rust_on_python_mesh_{}.json", name);
        
        println!("\nRunning FEM on Python mesh: {}", name);
        let result = run_fem_on_python_mesh(&format!("python_mesh_{}.json", name), name, &output_path);
        
        match result {
            Ok(diag) => {
                println!("  ✓ FEM succeeded");
                println!("  n_dof: {}", diag.n_dof);
                println!("  n_elements: {}", diag.n_elements);
                println!("  J_raw: {:.6e}", diag.j_raw);
                println!("  ωᵀF: {:.6e}", diag.omega_dot_f);
                println!("  Residual: {:.2e}", diag.residual_norm);
            }
            Err(e) => {
                println!("  ✗ FEM failed: {}", e);
            }
        }
    }
}