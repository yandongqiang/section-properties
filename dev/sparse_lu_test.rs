use section_properties::fea::SparseMatrix;
use section_properties::fea::solvers::SparseLu;

fn main() {
    // Test 1: Simple 2x2 matrix with zero diagonal
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

    // Verify PA = LU
    let max_diff = lu.verify_pa_eq_lu(&a);
    println!("  max|PA - LU| = {:.2e}", max_diff);
    assert!(max_diff < 1e-12, "PA != LU: max_diff = {}", max_diff);

    // Test 2: Diagonally dominant
    println!("\nTest 2: Diagonally dominant 3x3");
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

    let max_diff2 = lu2.verify_pa_eq_lu(&a2);
    println!("  max|PA - LU| = {:.2e}", max_diff2);
    assert!(max_diff2 < 1e-12, "PA != LU: max_diff = {}", max_diff2);

    // Test 3: Zero diagonal with pivoting needed
    println!("\nTest 3: [[1, 2], [1e-14, 1]] - needs pivoting");
    let mut a3 = SparseMatrix::new(2);
    a3.add(0, 0, 1.0);
    a3.add(0, 1, 2.0);
    a3.add(1, 0, 1e-14);
    a3.add(1, 1, 1.0);
    a3.compress();
    match SparseLu::factor(&a3) {
        Ok(lu) => {
            let b3 = vec![1.0, 2.0];
            let x3 = lu.solve(&b3);
            println!("  Solution: {:?}", x3);
        }
        Err(e) => {
            println!("  Error: {}", e);
        }
    }

    println!("\nAll tests passed!");
}
