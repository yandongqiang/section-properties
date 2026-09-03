use section_properties::mesh::{MeshParams, mesh_section};
use section_properties::section_library::steel::ISection;
use section_properties::section_properties::SectionProperties;
use section_properties::{ParametricSection, Section};

fn kappa_for(section: &Section, max_iterations: usize) -> (f64, f64) {
    // Reuse the internal mesh params by rebuilding a mesh with the given cap.
    let params = MeshParams {
        target_size: 0.01,
        max_size: 0.02,
        min_size: 0.003,
        quality_threshold: 0.3,
        use_delaunay: true,
        max_iterations,
        max_nodes: 20000,
    };
    let mesh = mesh_section(section, params);
    (mesh.nodes.len() as f64, mesh.elements.len() as f64)
}

fn main() {
    let i = ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
    let section = i.build();
    let props = SectionProperties::from_section(&section);
    println!("area={} ixx={} iyy={}", props.area, props.ix, props.iy);
    for k in 1..=6 {
        let (nn, ne) = kappa_for(&section, k);
        println!("max_iter={}: tri3-ish nodes={} elements={}", k, nn, ne);
    }
    println!("expected web az = {}", 0.28 * 0.007);
    println!("expected flange ay = {}", 2.0 * 0.15 * 0.01);
}
