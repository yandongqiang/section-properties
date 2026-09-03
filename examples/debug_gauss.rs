use section_properties::fea::gauss_points;

fn main() {
    println!("Testing gauss_points:");
    for n in [1, 3, 4, 6, 7] {
        let pts = gauss_points(n);
        let sum: f64 = pts.iter().map(|(w, _, _, _)| w).sum();
        println!("n={}: {} points, sum weights = {:.10}", n, pts.len(), sum);
        for (i, (w, e, x, z)) in pts.iter().enumerate() {
            println!("  {}: w={:.10} e={:.6} x={:.6} z={:.6}", i, w, e, x, z);
        }
    }
}
