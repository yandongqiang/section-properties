//! Run FEM on Python-exported meshes for cross-validation

use section_properties::plastic::warping_fem::run_fem_on_python_mesh;

fn workspace_root() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf()
}

#[test]
fn run_fem_on_python_meshes() {
    let ws = workspace_root();
    let meshes = [
        "Channel_200x75",
        "I_section_300x150",
        "Angle_100x100",
        "Thin_Channel_300x100x3",
    ];

    for name in meshes {
        let mesh_path = ws.join(format!("python_mesh_{}.json", name));
        let output_path = format!("rust_on_python_mesh_{}.json", name);

        println!("\nRunning FEM on Python mesh: {}", name);
        let result = run_fem_on_python_mesh(mesh_path.to_str().unwrap(), name, &output_path);

        let diag = result.unwrap_or_else(|e| panic!("FEM failed on {}: {}", name, e));
        println!("  ✓ FEM succeeded");
        println!("  n_dof: {}", diag.n_dof);
        println!("  n_elements: {}", diag.n_elements);
        println!("  J_raw: {:.6e}", diag.j_raw);
        println!("  ωᵀF: {:.6e}", diag.omega_dot_f);
        println!("  Residual: {:.2e}", diag.residual_norm);
        assert!(diag.n_dof > 0, "n_dof must be positive");
        assert!(diag.n_elements > 0, "n_elements must be positive");
        assert!(diag.j_raw.is_finite(), "J_raw must be finite");
        assert!(diag.residual_norm.is_finite(), "residual must be finite");
    }
}
