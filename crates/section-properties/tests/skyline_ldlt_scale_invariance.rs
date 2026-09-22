//! SkylineLdlt Scale Invariance Tests
//!
//! Verifies that the SkylineLdlt solver produces results that scale correctly
//! with matrix and RHS scaling.

use section_properties::fea::{SkylineLdlt, SparseMatrix};

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

fn rel_err(a: f64, b: f64) -> f64 {
    if b.abs() < 1e-15 {
        return (a - b).abs();
    }
    (a - b).abs() / b.abs()
}

#[test]
fn test_skyline_ldlt_scale_invariance() {
    // Test matrix: SPD tridiagonal
    let base_data = &[
        4.0, 1.0, 0.0, 1.0, 1.0, 4.0, 1.0, 0.0, 0.0, 1.0, 4.0, 1.0, 1.0, 0.0, 1.0, 4.0,
    ];
    let base_b = vec![1.0, 2.0, 3.0, 4.0];
    let scales = [1e-6, 1e-3, 1.0, 1e3, 1e6];

    // Reference solution at scale 1.0
    let a_ref = build_dense(4, base_data);
    let solver_ref = SkylineLdlt::factor(&a_ref).unwrap();
    let x_ref = solver_ref.solve(&base_b).unwrap();

    for &scale in &scales {
        println!("\n--- Scale = {:.0e} ---", scale);

        let scaled_a_data: Vec<f64> = base_data.iter().map(|v| v * scale).collect();
        let scaled_b: Vec<f64> = base_b.iter().map(|v| v * scale).collect();
        let a = build_dense(4, &scaled_a_data);

        let solver = SkylineLdlt::factor(&a)
            .unwrap_or_else(|e| panic!("Factorization failed at scale {:.0e}: {:?}", scale, e));

        let x = solver
            .solve(&scaled_b)
            .unwrap_or_else(|e| panic!("Solve failed at scale {:.0e}: {:?}", scale, e));

        // Expected: x(αA, αb) = x(A, b) for SPD systems
        for i in 0..4 {
            let rel_diff = rel_err(x[i], x_ref[i]);
            println!(
                "  x[{}] = {:.6e} vs {:.6e} (rel_err = {:.2e})",
                i, x[i], x_ref[i], rel_diff
            );
            assert!(
                rel_diff < 1e-10,
                "Solution differs from reference at scale {:.0e}: x[{}] rel_err = {:.2e}",
                scale,
                i,
                rel_diff
            );
        }

        // Verify A*x ≈ b (relative residual)
        let mut res = vec![0.0; 4];
        for i in 0..4 {
            for j in 0..4 {
                let v = match (i, j) {
                    (0, 0) => 4.0 * scale,
                    (0, 1) => 1.0 * scale,
                    (0, 2) => 0.0,
                    (0, 3) => 1.0 * scale,
                    (1, 0) => 1.0 * scale,
                    (1, 1) => 4.0 * scale,
                    (1, 2) => 1.0 * scale,
                    (1, 3) => 0.0,
                    (2, 0) => 0.0,
                    (2, 1) => 1.0 * scale,
                    (2, 2) => 4.0 * scale,
                    (2, 3) => 1.0 * scale,
                    (3, 0) => 1.0 * scale,
                    (3, 1) => 0.0,
                    (3, 2) => 1.0 * scale,
                    (3, 3) => 4.0 * scale,
                    _ => 0.0,
                };
                res[i] += v * x[j];
            }
        }

        let b_norm = scaled_b.iter().map(|v| v * v).sum::<f64>().sqrt();
        let max_res = res
            .iter()
            .zip(scaled_b.iter())
            .map(|(&r, &b)| (r - b).abs())
            .fold(0.0, f64::max);
        let rel_res = max_res / b_norm.max(1e-300);
        println!("  Relative residual: {:.2e}", rel_res);
        assert!(
            rel_res < 1e-9,
            "Relative residual too large at scale {:.0e}: {:.2e}",
            scale,
            rel_res
        );
    }
}

#[test]
fn test_skyline_ldlt_rhs_scaling() {
    let base_data = &[
        4.0, 1.0, 0.0, 1.0, 1.0, 4.0, 1.0, 0.0, 0.0, 1.0, 4.0, 1.0, 1.0, 0.0, 1.0, 4.0,
    ];
    let base_b = vec![1.0, 2.0, 3.0, 4.0];
    let alphas = [1e-6, 1e-3, 1.0, 1e3, 1e6];
    let betas = [1e-6, 1e-3, 1.0, 1e3, 1e6];

    let a_ref = build_dense(4, base_data);
    let solver_ref = SkylineLdlt::factor(&a_ref).unwrap();
    let x_ref = solver_ref.solve(&base_b).unwrap();

    for &alpha in &alphas {
        for &beta in &betas {
            let scaled_a_data: Vec<f64> = base_data.iter().map(|v| v * alpha).collect();
            let scaled_a = build_dense(4, &scaled_a_data);
            let scaled_b: Vec<f64> = base_b.iter().map(|v| v * beta).collect();

            let solver = SkylineLdlt::factor(&scaled_a).unwrap();
            let x = solver.solve(&scaled_b).unwrap();

            // Expected: x(αA, βb) = (β/α) * x(A, b)
            let expected: Vec<f64> = x_ref.iter().map(|v| v * beta / alpha).collect();

            for i in 0..4 {
                let rel_diff = rel_err(x[i], expected[i]);
                assert!(
                    rel_diff < 1e-10,
                    "RHS scaling failed: alpha={:.0e}, beta={:.0e}, x[{}] rel_err = {:.2e}",
                    alpha,
                    beta,
                    i,
                    rel_diff
                );
            }

            // Verify residual
            let mut res = vec![0.0; 4];
            for i in 0..4 {
                for j in 0..4 {
                    let v = match (i, j) {
                        (0, 0) => 4.0 * alpha,
                        (0, 1) => 1.0 * alpha,
                        (0, 2) => 0.0,
                        (0, 3) => 1.0 * alpha,
                        (1, 0) => 1.0 * alpha,
                        (1, 1) => 4.0 * alpha,
                        (1, 2) => 1.0 * alpha,
                        (1, 3) => 0.0,
                        (2, 0) => 0.0,
                        (2, 1) => 1.0 * alpha,
                        (2, 2) => 4.0 * alpha,
                        (2, 3) => 1.0 * alpha,
                        (3, 0) => 1.0 * alpha,
                        (3, 1) => 0.0,
                        (3, 2) => 1.0 * alpha,
                        (3, 3) => 4.0 * alpha,
                        _ => 0.0,
                    };
                    res[i] += v * x[j];
                }
            }

            let b_norm = scaled_b.iter().map(|v| v * v).sum::<f64>().sqrt();
            let max_res = res
                .iter()
                .zip(scaled_b.iter())
                .map(|(&r, &b)| (r - b).abs())
                .fold(0.0, f64::max);
            let rel_res = max_res / b_norm.max(1e-300);
            assert!(
                rel_res < 1e-9,
                "Residual check failed: alpha={:.0e}, beta={:.0e}, rel_res={:.2e}",
                alpha,
                beta,
                rel_res
            );
        }
    }
}

#[test]
fn test_skyline_ldlt_singular_detection() {
    // Singular matrix should be detected at all scales
    let base_singular = &[
        1.0, 2.0, 2.0, 4.0, 2.0, 4.0, 4.0, 8.0, 3.0, 6.0, 5.0, 10.0, 4.0, 8.0, 6.0, 12.0,
    ];
    let scales = [1e-6, 1e-3, 1.0, 1e3, 1e6];

    for &scale in &scales {
        let scaled_data: Vec<f64> = base_singular.iter().map(|v| v * scale).collect();
        let a = build_dense(4, &scaled_data);
        let result = SkylineLdlt::factor(&a);
        assert!(
            result.is_err(),
            "Singular matrix should fail at scale {:.0e}",
            scale
        );
    }
}

#[test]
fn test_skyline_ldlt_near_singular() {
    // Near-singular matrix: [1 1; 1 1+ε]
    // For ε >= 1e-12, should factorize successfully
    // For ε = 1e-14, should fail or be very inaccurate
    let epsilons = [1e-8, 1e-10, 1e-12, 1e-14];

    for &eps in &epsilons {
        let a = build_dense(2, &[1.0, 1.0, 1.0, 1.0 + eps]);
        let result = SkylineLdlt::factor(&a);

        if eps >= 1e-12 {
            assert!(
                result.is_ok(),
                "Near-singular matrix with eps={:.0e} should factorize",
                eps
            );
            let solver = result.unwrap();
            let b = vec![1.0, 2.0];
            let x = solver.solve(&b).unwrap();
            // Verify A*x ≈ b
            let mut res = vec![0.0; 2];
            for i in 0..2 {
                for j in 0..2 {
                    let v = match (i, j) {
                        (0, 0) => 1.0,
                        (0, 1) => 1.0,
                        (1, 0) => 1.0,
                        (1, 1) => 1.0 + eps,
                        _ => 0.0,
                    };
                    res[i] += v * x[j];
                }
            }
            let max_res = res
                .iter()
                .zip(b.iter())
                .map(|(&r, &b)| (r - b).abs())
                .fold(0.0, f64::max);
            assert!(
                max_res < 1e-8,
                "Residual too large for eps={:.0e}: {:.2e}",
                eps,
                max_res
            );
        } else {
            // For very small eps, behavior may vary
            // Just ensure it doesn't panic
            let _ = result;
        }
    }
}

#[test]
fn test_skyline_ldlt_lagrange_scale_invariance() {
    // Test the Lagrange multiplier solver scale invariance
    // When K, c, f are all scaled by α: u and λ should be INVARIANT
    // [αK  αc; αc^T 0] [u; λ] = [αf; 0]  =>  divide by α: [K c; c^T 0] [u; λ] = [f; 0]
    let base_k = &[
        4.0, 1.0, 0.0, 1.0, 1.0, 4.0, 1.0, 0.0, 0.0, 1.0, 4.0, 1.0, 1.0, 0.0, 1.0, 4.0,
    ];
    let base_c = vec![1.0, 0.0, 0.0, 0.0];
    let base_f = vec![1.0, 2.0, 3.0, 4.0];
    let scales = [1e-3, 1.0, 1e3];

    let k_ref = build_dense(4, base_k);
    let solver_ref = SkylineLdlt::factor(&k_ref).unwrap();
    let u_ref = solver_ref.solve_lagrange(&base_c, &base_f).unwrap();
    println!("u_ref = {:?}", u_ref);

    for &scale in &scales {
        let k_data: Vec<f64> = base_k.iter().map(|v| v * scale).collect();
        let c_data: Vec<f64> = base_c.iter().map(|v| v * scale).collect();
        let f_data: Vec<f64> = base_f.iter().map(|v| v * scale).collect();

        let k = build_dense(4, &k_data);
        let solver = SkylineLdlt::factor(&k).unwrap();
        let u = solver.solve_lagrange(&c_data, &f_data).unwrap();
        println!("scale = {:.0e}, u = {:?}", scale, u);

        // For Lagrange system with scaled K, c, f by α: u should be INVARIANT
        for i in 0..4 {
            let expected = u_ref[i];
            let abs_diff = (u[i] - expected).abs();
            let tol = if expected.abs() < 1e-12 {
                1e-12 // absolute tolerance
            } else {
                expected.abs() * 1e-10 // relative tolerance
            };
            println!(
                "  u[{}] = {:.6e} vs expected {:.6e} (abs_diff = {:.2e}, tol = {:.2e})",
                i, u[i], expected, abs_diff, tol
            );
            assert!(
                abs_diff < tol,
                "Lagrange solver scale invariance failed at scale {:.0e}: u[{}] = {:.6e} vs expected {:.6e} (abs_diff = {:.2e} > tol {:.2e})",
                scale,
                i,
                u[i],
                expected,
                abs_diff,
                tol
            );
        }
    }
}
