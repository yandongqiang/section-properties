use section_properties::plastic::warping_fem::run_fem_on_python_tri6_mesh;
fn main() {
    run_fem_on_python_tri6_mesh(
        "python_global_Channel_200x75.json",
        "Channel_200x75",
        "C:/Users/HP/AppData/Local/Temp/keep_rust_k.json",
    )
    .unwrap();
}
