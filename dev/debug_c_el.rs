use section_properties::ParametricSection;
use section_properties::fea::Tri6;
use section_properties::geometry::Point;

fn main() {
    let points = [
        Point::new(0.0, 0.0), // node 0
        Point::new(1.0, 0.0), // node 1
        Point::new(0.0, 1.0), // node 2
        Point::new(0.5, 0.0), // node 3 (mid 0-1)
        Point::new(0.5, 0.5), // node 4 (mid 1-2)
        Point::new(0.0, 0.5), // node 5 (mid 2-0)
    ];

    let tri6 = Tri6::from_points(0, points, [0, 1, 2, 3, 4, 5], 1.0, 1.0, 1.0).unwrap();

    let (k_el, f_el, c_el) = tri6.torsion_properties();

    println!("=== Tri6 Element Matrices on Unit Triangle (C_e with 7-point rule) ===");
    println!("\nC_e (6x1):");
    for i in 0..6 {
        println!("  C[{}] = {:.10}", i, c_el[i]);
    }
    let c_sum: f64 = c_el.iter().sum();
    println!("Sum C_e = {:.10} (expected 0.5)", c_sum);

    // Expected with correct 7-point rule:
    // C_corner = A/12 = 0.5/12 = 0.041666...
    // C_mid = A/6 = 0.5/6 = 0.08333...
    println!(
        "\nExpected: corner = {:.10}, mid = {:.10}",
        0.5 / 12.0,
        0.5 / 6.0
    );
}
