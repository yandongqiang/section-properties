//! Final Scale Gate Test - Verifies true mathematical scale invariance
//!
//! Tests solver behavior at extreme scales to catch scale.max(1.0) issues

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
fn test_dense_solver_extreme_scales() {
    // SPD tridiagonal matrix
    let base_k = &[
        4.0, 1.0, 0.0, 1.0, 1.0, 4.0, 1.0, 0.0, 0.0, 1.0, 4.0, 1.0, 1.0, 0.0, 1.0, 4.0,
    ];
    let base_f = vec![1.0, 2.0, 3.0, 4.0];

    // Test extreme scales including those where max(1.0) would clamp
    let scales = [
        1e-12, 1e-9, 1e-6, 1e-3, 1e-2, 1e-1, 1.0, 1e1, 1e2, 1e3, 1e6, 1e9, 1e12,
    ];

    // Reference at scale 1.0
    let k_ref = build_dense(4, base_k);
    let solver_ref = SkylineLdlt::factor(&k_ref).unwrap();
    let x_ref = solver_ref.solve(&base_f).unwrap();

    println!("\n=== DENSE SOLVER EXTREME SCALE TEST ===");
    println!("x_ref = {:?}", x_ref);

    for &alpha in &scales {
        let k_data: Vec<f64> = base_k.iter().map(|v| v * alpha).collect();
        let f_data: Vec<f64> = base_f.iter().map(|v| v * alpha).collect();

        let k = build_dense(4, &k_data);
        let solver =
            SkylineLdlt::factor(&k).expect(&format!("Factorization failed at α={:.0e}", alpha));

        // Check scale factor stored
        println!("\nα = {:.0e}: stored scale = {:.2e}", alpha, solver.scale());

        let x = solver
            .solve(&f_data)
            .expect(&format!("Solve failed at α={:.0e}", alpha));

        // For αA x = αb -> x should be invariant
        let mut max_err = 0.0f64;
        for i in 0..4 {
            let err = rel_err(x[i], x_ref[i]);
            max_err = max_err.max(err);
        }

        let pivot_tol = 1e-15 * solver.scale().max(1.0);
        println!("  pivot_tol = {:.2e}, max_err = {:.2e}", pivot_tol, max_err);

        // TRUE scale invariance: error should be ~1e-15 at ALL scales
        assert!(
            max_err < 1e-10,
            "Scale invariance violated at α={:.0e}: max_err={:.2e}",
            alpha,
            max_err
        );
    }
}

#[test]
fn test_skyline_ldlt_extreme_scales() {
    let base_k = &[
        4.0, 1.0, 0.0, 1.0, 1.0, 4.0, 1.0, 0.0, 0.0, 1.0, 4.0, 1.0, 1.0, 0.0, 1.0, 4.0,
    ];
    let base_f = vec![1.0, 2.0, 3.0, 4.0];
    let scales = [
        1e-12, 1e-9, 1e-6, 1e-3, 1e-2, 1e-1, 1.0, 1e1, 1e2, 1e3, 1e6, 1e9, 1e12,
    ];

    let k_ref = build_dense(4, base_k);
    let solver_ref = SkylineLdlt::factor(&k_ref).unwrap();
    let x_ref = solver_ref.solve(&base_f).unwrap();

    println!("\n=== SKYLINE LDLT EXTREME SCALE TEST ===");

    for &alpha in &scales {
        let k_data: Vec<f64> = base_k.iter().map(|v| v * alpha).collect();
        let f_data: Vec<f64> = base_f.iter().map(|v| v * alpha).collect();

        let k = build_dense(4, &k_data);
        let solver =
            SkylineLdlt::factor(&k).expect(&format!("Factorization failed at α={:.0e}", alpha));

        let x = solver
            .solve(&f_data)
            .expect(&format!("Solve failed at α={:.0e}", alpha));

        let mut max_err = 0.0f64;
        for i in 0..4 {
            let err = rel_err(x[i], x_ref[i]);
            max_err = max_err.max(err);
        }

        let pivot_tol = 1e-15 * solver.scale().max(1.0);
        println!(
            "α = {:.0e}: scale={:.2e}, pivot_tol={:.2e}, max_err={:.2e}",
            alpha,
            solver.scale(),
            pivot_tol,
            max_err
        );

        assert!(
            max_err < 1e-10,
            "SkylineLdlt scale invariance violated at α={:.0e}: max_err={:.2e}",
            alpha,
            max_err
        );
    }
}

#[test]
fn test_lagrange_extreme_scales() {
    let base_k = &[
        4.0, 1.0, 0.0, 1.0, 1.0, 4.0, 1.0, 0.0, 0.0, 1.0, 4.0, 1.0, 1.0, 0.0, 1.0, 4.0,
    ];
    let base_c = vec![1.0, 0.0, 0.0, 0.0];
    let base_f = vec![1.0, 2.0, 3.0, 4.0];
    let scales = [
        1e-12, 1e-9, 1e-6, 1e-3, 1e-2, 1e-1, 1.0, 1e1, 1e2, 1e3, 1e6, 1e9, 1e12,
    ];

    let k_ref = build_dense(4, base_k);
    let solver_ref = SkylineLdlt::factor(&k_ref).unwrap();
    let u_ref = solver_ref.solve_lagrange(&base_c, &base_f).unwrap();

    println!("\n=== LAGRANGE SOLVER EXTREME SCALE TEST ===");
    println!("u_ref = {:?}", u_ref);

    for &alpha in &scales {
        let k_data: Vec<f64> = base_k.iter().map(|v| v * alpha).collect();
        let c_data: Vec<f64> = base_c.iter().map(|v| v * alpha).collect();
        let f_data: Vec<f64> = base_f.iter().map(|v| v * alpha).collect();

        let k = build_dense(4, &k_data);
        let solver =
            SkylineLdlt::factor(&k).expect(&format!("Factorization failed at α={:.0e}", alpha));

        let u = solver
            .solve_lagrange(&c_data, &f_data)
            .expect(&format!("Lagrange solve failed at α={:.0e}", alpha));

        // Lagrange system with scaled K,c,f: u should be INVARIANT
        let mut max_err = 0.0f64;
        for i in 0..4 {
            let err = rel_err(u[i], u_ref[i]);
            max_err = max_err.max(err);
        }

        let near_zero_tol = 1e-15 * solver.scale().max(1.0);
        println!(
            "α = {:.0e}: scale={:.2e}, near_zero_tol={:.2e}, max_err={:.2e}",
            alpha,
            solver.scale(),
            near_zero_tol,
            max_err
        );

        assert!(
            max_err < 1e-10,
            "Lagrange scale invariance violated at α={:.0e}: max_err={:.2e}",
            alpha,
            max_err
        );
    }
}

#[test]
fn test_sparse_lu_extreme_scales() {
    use section_properties::fea::solvers::SparseLu;

    let base_data = &[
        4.0, 1.0, 0.0, 1.0, 1.0, 4.0, 1.0, 0.0, 0.0, 1.0, 4.0, 1.0, 1.0, 0.0, 1.0, 4.0,
    ];
    let base_b = vec![1.0, 2.0, 3.0, 4.0];
    let scales = [
        1e-12, 1e-9, 1e-6, 1e-3, 1e-2, 1e-1, 1.0, 1e1, 1e2, 1e3, 1e6, 1e9, 1e12,
    ];

    let a_ref = build_dense(4, base_data);
    let lu_ref = SparseLu::factor(&a_ref).unwrap();
    let x_ref = lu_ref.solve(&base_b).unwrap();

    println!("\n=== SPARSE LU EXTREME SCALE TEST ===");

    for &alpha in &scales {
        let k_data: Vec<f64> = base_data.iter().map(|v| v * alpha).collect();
        let b_data: Vec<f64> = base_b.iter().map(|v| v * alpha).collect();

        let a = build_dense(4, &k_data);
        let lu = SparseLu::factor(&a)
            .expect(&format!("SparseLU factorization failed at α={:.0e}", alpha));

        let x = lu
            .solve(&b_data)
            .expect(&format!("SparseLU solve failed at α={:.0e}", alpha));

        let mut max_err = 0.0f64;
        for i in 0..4 {
            let err = rel_err(x[i], x_ref[i]);
            max_err = max_err.max(err);
        }

        println!("α = {:.0e}: max_err={:.2e}", alpha, max_err);

        assert!(
            max_err < 1e-10,
            "SparseLU scale invariance violated at α={:.0e}: max_err={:.2e}",
            alpha,
            max_err
        );
    }
}
