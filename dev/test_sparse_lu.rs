use section_properties::fea::SparseMatrix;
use section_properties::fea::solvers::SparseLu;

fn main() {
    // Test 1: Simple 2x2 matrix
    println!("Test 1: 2x2 matrix [[0, 1], [1, 0]]");
    let mut a = SparseMatrix::new(2);
    a.add(0, 1, 1.0);
    a.add(1, 0, 1.0);
    a.compress();
    let lu = SparseLu::factor(&a).unwrap();
    let b = vec![1.0, 2.0];
    let x = lu.solve(&b);
    println!("  Solution: {:?}", x);
    println!("  Expected: [2, 1]");

    // Test 2: Diagonally dominant
    println!("\nTest 2: Diagonally dominant");
    let mut a2 = SparseMatrix::new(3);
    a2.add(0, 0, 4.0);
    a2.add(0, 1, 1.0);
    a2.add(1, 0, 1.0);
    a2.add(1, 1, 4.0);
    a2.add(1, 2, 1.0);
    a2.add(2, 1, 1.0);
    a2.add(2, 2, 4.0);
    a2.compress();
    let lu2 = SparseLu::factor(&a2).unwrap();
    let b2 = vec![1.0, 2.0, 3.0];
    let x2 = lu2.solve(&b2);
    println!("  Solution: {:?}", x2);

    // Verify
    let mut a2_dense = vec![vec![0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            // Reconstruct from factors
        }
    }
}
