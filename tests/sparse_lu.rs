//! SparseLU Integration Tests
//!
//! Tests for the SparseLU solver with true row partial pivoting (PA = LU).

use section_properties::fea::SparseMatrix;
use section_properties::fea::solvers::SparseLu;

fn build_dense(n: usize, data: &[f64]) -> SparseMatrix {
    let mut a = SparseMatrix::new(n);
    for i in 0..n {
        for j in 0..n {
            let v = data[i * n + j];
            if v.abs() > 1e-15 {
                a.add(i, j, v);
            }
        }
    }
    a.compress();
    a
}

#[test]
fn test_zero_diagonal() {
    // A = [[0, 1], [1, 0]] - requires pivoting
    let a = build_dense(2, &[0.0, 1.0, 1.0, 0.0]);
    let lu = SparseLu::factor(&a).unwrap();

    // Check that pivoting occurred (perm != identity)
    assert_ne!(lu.perm(), &[0, 1], "Should have pivoted");

    // Verify PA = LU
    let max_diff = lu.verify_pa_eq_lu(&build_dense(2, &[0.0, 1.0, 1.0, 0.0]));
    assert!(max_diff < 1e-10, "PA != LU: max_diff = {}", max_diff);

    // Solve A x = b
    let b = vec![1.0, 2.0];
    let x = lu.solve(&b).unwrap();
    assert!((x[0] - 2.0).abs() < 1e-12);
    assert!((x[1] - 1.0).abs() < 1e-12);
}

#[test]
fn test_must_pivot() {
    // A = [[1e-14, 1], [1, 1]] - first pivot is tiny, must pivot to row 1
    let a = build_dense(2, &[1e-14, 1.0, 1.0, 1.0]);
    let lu = SparseLu::factor(&a).unwrap();

    // Must have pivoted
    assert_ne!(lu.perm(), &[0, 1], "Should have pivoted to row 1");

    // Verify PA = LU
    let max_diff = lu.verify_pa_eq_lu(&build_dense(2, &[1e-14, 1.0, 1.0, 1.0]));
    assert!(max_diff < 1e-10, "PA != LU: max_diff = {}", max_diff);

    // Solve
    let b = vec![1.0, 2.0];
    let x = lu.solve(&b).unwrap();
    println!("Solution: {:?}", x);
}

#[test]
fn test_multiple_pivots() {
    // 3x3 matrix requiring multiple pivots
    let a = build_dense(3, &[1e-14, 1.0, 2.0, 1.0, 1.0, 3.0, 0.0, 1.0, 1.0]);
    let lu = SparseLu::factor(&a).unwrap();

    // Should have non-identity permutation (multiple pivots)
    assert_ne!(lu.perm(), &[0, 1, 2], "Should have multiple pivots");

    // Verify PA = LU
    let max_diff = lu.verify_pa_eq_lu(&build_dense(
        3,
        &[1e-14, 1.0, 2.0, 1.0, 1.0, 3.0, 0.0, 1.0, 1.0],
    ));
    assert!(max_diff < 1e-10, "PA != LU: max_diff = {}", max_diff);

    // Solve
    let b = vec![1.0, 2.0, 3.0];
    let x = lu.solve(&b).unwrap();
    println!("Solution: {:?}", x);
}

#[test]
fn test_singular() {
    // A = [[1, 2], [2, 4]] - singular
    let a = build_dense(2, &[1.0, 2.0, 2.0, 4.0]);
    let result = SparseLu::factor(&a);

    // Must return error, not silently succeed
    assert!(result.is_err(), "Should fail on singular matrix");
    let err = match result {
        Err(e) => e,
        Ok(_) => panic!("Expected error"),
    };
    assert!(
        err.contains("singular") || err.contains("Singular"),
        "Error should mention singular: {}",
        err
    );
}

#[test]
fn test_pa_lu_random() {
    // Test random small matrices
    for n in [3, 5, 10] {
        for seed in 0..5 {
            let mut a = SparseMatrix::new(n);
            let mut data = vec![0.0f64; n * n];
            use std::collections::hash_map::DefaultHasher;
            use std::hash::{Hash, Hasher};

            let mut hasher = DefaultHasher::new();
            (seed as u64).hash(&mut hasher);
            let mut seed = hasher.finish();

            for i in 0..n * n {
                seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                let val = ((seed as f64) / (u64::MAX as f64)) * 20.0 - 10.0;
                data[i] = val;
            }

            let a = build_dense(n, &data);

            match SparseLu::factor(&a) {
                Ok(lu) => {
                    let max_diff = lu.verify_pa_eq_lu(&build_dense(n, &data));
                    assert!(
                        max_diff < 1e-10,
                        "PA != LU for n={}, seed={}: max_diff = {}",
                        n,
                        seed,
                        max_diff
                    );

                    // Test solve
                    let b: Vec<f64> = (0..n).map(|i| (i as f64) + 1.0).collect();
                    let x = lu.solve(&b).unwrap();

                    // Verify A*x ≈ b
                    let mut a_sparse = SparseMatrix::new(n);
                    let a_ref = build_dense(n, &data);
                    for i in 0..n {
                        for j in 0..n {
                            let v = a_ref.csr_vals()[a_ref.row_ptr()[i]..a_ref.row_ptr()[i + 1]]
                                .iter()
                                .enumerate()
                                .find(|(idx, _)| a_ref.csr_cols()[a_ref.row_ptr()[i] + *idx] == j)
                                .map(|(_, v)| *v)
                                .unwrap_or(0.0);
                            if v.abs() > 1e-15 {
                                a_sparse.add(i, j, v);
                            }
                        }
                    }
                    a_sparse.compress();

                    let mut res = vec![0.0; n];
                    for i in 0..n {
                        for k in a_sparse.row_ptr()[i]..a_sparse.row_ptr()[i + 1] {
                            let j = a_sparse.csr_cols()[k];
                            res[i] += a_sparse.csr_vals()[k] * x[j];
                        }
                    }

                    let b: Vec<f64> = (0..n).map(|i| (i as f64) + 1.0).collect();
                    let max_res = res
                        .iter()
                        .zip(b.iter())
                        .map(|(&r, &b)| (r - b).abs())
                        .fold(0.0, f64::max);
                    assert!(max_res < 1e-10, "Residual too large: {}", max_res);
                }
                Err(_) => {
                    // Singular matrix - that's fine
                }
            }
        }
    }
}

#[test]
fn test_indefinite() {
    // Symmetric indefinite matrix
    let a = build_dense(2, &[0.0, 1.0, 1.0, 0.0]);
    let lu = SparseLu::factor(&a).unwrap();

    // Must succeed for indefinite matrix
    assert_ne!(lu.perm(), &[0, 1]);

    let max_diff = lu.verify_pa_eq_lu(&build_dense(2, &[0.0, 1.0, 1.0, 0.0]));
    assert!(max_diff < 1e-10);

    let b = vec![1.0, 2.0];
    let x = lu.solve(&b).unwrap();
    assert!((x[0] - 2.0).abs() < 1e-12);
    assert!((x[1] - 1.0).abs() < 1e-12);
}

#[test]
fn test_scale_invariance() {
    // True scale invariance test: same well-conditioned matrix at different scales
    // Scales: 1e-12, 1e-9, 1e-6, 1, 1e6, 1e9, 1e12
    let base_data = &[4.0, 1.0, 0.0, 1.0, 4.0, 1.0, 0.0, 1.0, 4.0];
    let b = vec![1.0, 2.0, 3.0];
    let scales = [1e-12, 1e-9, 1e-6, 1.0, 1e6, 1e9, 1e12];

    // Compute reference solution at scale 1.0
    let a_ref = build_dense(3, base_data);
    let lu_ref = SparseLu::factor(&a_ref).unwrap();
    let x_ref = lu_ref.solve(&b).unwrap();

    for &scale in &scales {
        println!("Testing scale = {:.0e}", scale);
        let scaled_data: Vec<f64> = base_data.iter().map(|v| v * scale).collect();
        let a = build_dense(3, &scaled_data);

        // Factorization should succeed
        let lu = SparseLu::factor(&a)
            .unwrap_or_else(|e| panic!("Factorization failed at scale {:.0e}: {}", scale, e));

        // Verify PA = LU
        let max_diff = lu.verify_pa_eq_lu(&build_dense(3, &scaled_data));
        assert!(
            max_diff < 1e-10,
            "PA != LU at scale {:.0e}: max_diff = {}",
            scale,
            max_diff
        );

        // Solve
        let x = lu
            .solve(&b)
            .unwrap_or_else(|e| panic!("Solve failed at scale {:.0e}: {}", scale, e));

        // Verify A*x ≈ b (relative residual)
        let mut res = vec![0.0; 3];
        for i in 0..3 {
            for j in 0..3 {
                let v = match (i, j) {
                    (0, 0) => 4.0 * scale,
                    (0, 1) => 1.0 * scale,
                    (0, 2) => 0.0,
                    (1, 0) => 1.0 * scale,
                    (1, 1) => 4.0 * scale,
                    (1, 2) => 1.0 * scale,
                    (2, 0) => 0.0,
                    (2, 1) => 1.0 * scale,
                    (2, 2) => 4.0 * scale,
                    _ => 0.0,
                };
                res[i] += v * x[j];
            }
        }

        let b_norm = b.iter().map(|v| v * v).sum::<f64>().sqrt();
        let max_res = res
            .iter()
            .zip(b.iter())
            .map(|(&r, &b)| (r - b).abs())
            .fold(0.0, f64::max);
        let rel_res = max_res / b_norm.max(1e-300);
        assert!(
            rel_res < 1e-9,
            "Relative residual too large at scale {:.0e}: {:.2e}",
            scale,
            rel_res
        );

        // Compare solution with reference (scaled appropriately)
        // At scale s, A*s*x = b => x = x_ref / s (solution scales inversely with matrix)
        for i in 0..3 {
            let expected = x_ref[i] / scale;
            let rel_diff = (x[i] - expected).abs() / expected.abs().max(1e-300);
            assert!(
                rel_diff < 1e-9,
                "Solution differs from reference at scale {:.0e}: x[{}] = {:.2e} vs {:.2e} (rel_diff = {:.2e})",
                scale,
                i,
                x[i],
                expected,
                rel_diff
            );
        }
    }
}
