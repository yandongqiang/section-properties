//! Phase 59: CG/ICCG scale-aware convergence regression tests.
//!
//! Verifies that the LinearSolver adapter (CgSolver / IccgSolver) uses
//! relative residual convergence (||r|| / ||b|| < tol), matching the raw
//! cg_solve / iccg_solve functions.  Also verifies that the ICCG adapter's
//! IC(0) factorization and backward substitution are correct.

use section_properties::fea::solver::LinearSolver;
use section_properties::fea::solver::impls::cg::CgSolver;
use section_properties::fea::solver::impls::iccg::IccgSolver;
use section_properties::fea::solvers::iccg_solve;
use section_properties::fea::{SparseMatrix, cg_solve};

fn spd_matrix_3x3() -> SparseMatrix {
    let mut a = SparseMatrix::new(3);
    a.add(0, 0, 4.0);
    a.add(0, 1, 1.0);
    a.add(0, 2, 1.0);
    a.add(1, 0, 1.0);
    a.add(1, 1, 3.0);
    a.add(1, 2, 1.0);
    a.add(2, 0, 1.0);
    a.add(2, 1, 1.0);
    a.add(2, 2, 2.0);
    a.compress();
    a
}

fn b_vector() -> Vec<f64> {
    vec![1.0, 2.0, 3.0]
}

fn matvec(a: &SparseMatrix, x: &[f64]) -> Vec<f64> {
    let n = a.n;
    let mut y = vec![0.0; n];
    a.matvec_into(x, &mut y);
    y
}

fn residual_norm(a: &SparseMatrix, x: &[f64], b: &[f64]) -> f64 {
    let ax = matvec(a, x);
    let n = a.n;
    (0..n).map(|i| (ax[i] - b[i]).powi(2)).sum::<f64>().sqrt()
}

fn solution_error(x: &[f64], x_ref: &[f64]) -> f64 {
    let n = x.len();
    let diff: f64 = (0..n)
        .map(|i| (x[i] - x_ref[i]).powi(2))
        .sum::<f64>()
        .sqrt();
    let ref_norm: f64 = (0..n).map(|i| x_ref[i].powi(2)).sum::<f64>().sqrt();
    if ref_norm > 0.0 {
        diff / ref_norm
    } else {
        diff
    }
}

const SCALES: [f64; 7] = [1e-12, 1e-9, 1e-6, 1.0, 1e6, 1e9, 1e12];

// --- 7.1 CG scale sweep ---

#[test]
fn cg_adapter_scale_sweep_relative_residual() {
    let a = spd_matrix_3x3();
    let b = b_vector();
    let tol = 1e-10;
    let max_iter = 1000;

    for &s in &SCALES {
        let bs: Vec<f64> = b.iter().map(|&v| s * v).collect();
        let b_norm = bs.iter().map(|v| v * v).sum::<f64>().sqrt();

        let mut adapter = CgSolver::new().with_params(max_iter, tol);
        adapter.factor(&a).unwrap();
        let x = adapter.solve(&bs).unwrap();
        let rel_res = residual_norm(&a, &x, &bs) / b_norm;

        assert!(
            rel_res < 1e-6,
            "CG scale {:.0e}: relative residual {:.2e} should be < 1e-6",
            s,
            rel_res
        );
    }
}

// --- 7.2 ICCG scale sweep ---

#[test]
fn iccg_adapter_scale_sweep_relative_residual() {
    let a = spd_matrix_3x3();
    let b = b_vector();
    let tol = 1e-10;
    let max_iter = 1000;

    for &s in &SCALES {
        let bs: Vec<f64> = b.iter().map(|&v| s * v).collect();
        let b_norm = bs.iter().map(|v| v * v).sum::<f64>().sqrt();

        let mut adapter = IccgSolver::new().with_params(max_iter, tol);
        adapter.factor(&a).unwrap();
        let x = adapter.solve(&bs).unwrap();
        let rel_res = residual_norm(&a, &x, &bs) / b_norm;

        assert!(
            rel_res < 1e-6,
            "ICCG scale {:.0e}: relative residual {:.2e} should be < 1e-6",
            s,
            rel_res
        );
    }
}

// --- 7.3 RHS scaling ---

#[test]
fn cg_adapter_rhs_scaling_invariance() {
    let a = spd_matrix_3x3();
    let b = b_vector();
    let tol = 1e-10;
    let max_iter = 1000;

    let mut adapter = CgSolver::new().with_params(max_iter, tol);
    adapter.factor(&a).unwrap();
    let x_base = adapter.solve(&b).unwrap();

    for &s in &[1e-6, 1.0, 1e6] {
        let bs: Vec<f64> = b.iter().map(|&v| s * v).collect();
        let xs: Vec<f64> = x_base.iter().map(|&v| s * v).collect();

        let x = adapter.solve(&bs).unwrap();
        let err = solution_error(&x, &xs);

        assert!(
            err < 1e-10,
            "CG RHS scale {:.0e}: solution error {:.2e} should be < 1e-10",
            s,
            err
        );
    }
}

#[test]
fn iccg_adapter_rhs_scaling_invariance() {
    let a = spd_matrix_3x3();
    let b = b_vector();
    let tol = 1e-10;
    let max_iter = 1000;

    let mut adapter = IccgSolver::new().with_params(max_iter, tol);
    adapter.factor(&a).unwrap();
    let x_base = adapter.solve(&b).unwrap();

    for &s in &[1e-6, 1.0, 1e6] {
        let bs: Vec<f64> = b.iter().map(|&v| s * v).collect();
        let xs: Vec<f64> = x_base.iter().map(|&v| s * v).collect();

        let x = adapter.solve(&bs).unwrap();
        let err = solution_error(&x, &xs);

        assert!(
            err < 1e-10,
            "ICCG RHS scale {:.0e}: solution error {:.2e} should be < 1e-10",
            s,
            err
        );
    }
}

// --- 7.4 Matrix scaling ---

#[test]
fn cg_adapter_matrix_scaling_invariance() {
    let a = spd_matrix_3x3();
    let b = b_vector();
    let tol = 1e-10;
    let max_iter = 1000;

    let mut adapter = CgSolver::new().with_params(max_iter, tol);
    adapter.factor(&a).unwrap();
    let x_base = adapter.solve(&b).unwrap();

    for &s in &[1e-6, 1.0, 1e6] {
        let mut sa = SparseMatrix::new(3);
        sa.add(0, 0, s * 4.0);
        sa.add(0, 1, s * 1.0);
        sa.add(0, 2, s * 1.0);
        sa.add(1, 0, s * 1.0);
        sa.add(1, 1, s * 3.0);
        sa.add(1, 2, s * 1.0);
        sa.add(2, 0, s * 1.0);
        sa.add(2, 1, s * 1.0);
        sa.add(2, 2, s * 2.0);
        sa.compress();

        let sb: Vec<f64> = b.iter().map(|&v| s * v).collect();

        let mut adapter2 = CgSolver::new().with_params(max_iter, tol);
        adapter2.factor(&sa).unwrap();
        let x = adapter2.solve(&sb).unwrap();
        let err = solution_error(&x, &x_base);

        assert!(
            err < 1e-10,
            "CG matrix scale {:.0e}: solution error {:.2e} should be < 1e-10",
            s,
            err
        );
    }
}

// --- 7.5 Zero RHS ---

#[test]
fn cg_adapter_zero_rhs() {
    let a = spd_matrix_3x3();
    let tol = 1e-10;
    let max_iter = 1000;

    let mut adapter = CgSolver::new().with_params(max_iter, tol);
    adapter.factor(&a).unwrap();

    let b_zero = vec![0.0, 0.0, 0.0];
    let x = adapter.solve(&b_zero).unwrap();
    assert!(
        x.iter().all(|&v| v.abs() < 1e-15),
        "CG zero RHS: x should be zero, got {:?}",
        x
    );
}

#[test]
fn iccg_adapter_zero_rhs() {
    let a = spd_matrix_3x3();
    let tol = 1e-10;
    let max_iter = 1000;

    let mut adapter = IccgSolver::new().with_params(max_iter, tol);
    adapter.factor(&a).unwrap();

    let b_zero = vec![0.0, 0.0, 0.0];
    let x = adapter.solve(&b_zero).unwrap();
    assert!(
        x.iter().all(|&v| v.abs() < 1e-15),
        "ICCG zero RHS: x should be zero, got {:?}",
        x
    );
}

// --- 7.6 Non-finite RHS ---

#[test]
fn cg_adapter_non_finite_rhs() {
    let a = spd_matrix_3x3();
    let tol = 1e-10;
    let max_iter = 1000;

    let mut adapter = CgSolver::new().with_params(max_iter, tol);
    adapter.factor(&a).unwrap();

    let b_nan = vec![1.0, f64::NAN, 3.0];
    assert!(adapter.solve(&b_nan).is_err());

    let b_inf = vec![1.0, f64::INFINITY, 3.0];
    assert!(adapter.solve(&b_inf).is_err());
}

#[test]
fn iccg_adapter_non_finite_rhs() {
    let a = spd_matrix_3x3();
    let tol = 1e-10;
    let max_iter = 1000;

    let mut adapter = IccgSolver::new().with_params(max_iter, tol);
    adapter.factor(&a).unwrap();

    let b_nan = vec![1.0, f64::NAN, 3.0];
    assert!(adapter.solve(&b_nan).is_err());

    let b_inf = vec![1.0, f64::INFINITY, 3.0];
    assert!(adapter.solve(&b_inf).is_err());
}

// --- 8. Cross-validation with raw cg_solve / iccg_solve ---

#[test]
fn cg_adapter_matches_raw_cg_solve() {
    let a = spd_matrix_3x3();
    let b = b_vector();
    let tol = 1e-10;
    let max_iter = 1000;

    for &s in &SCALES {
        let bs: Vec<f64> = b.iter().map(|&v| s * v).collect();

        let raw = cg_solve(&a, &bs, max_iter, tol);

        let mut adapter = CgSolver::new().with_params(max_iter, tol);
        adapter.factor(&a).unwrap();
        let adapter_x = adapter.solve(&bs).unwrap();

        let err = solution_error(&raw.x, &adapter_x);
        assert!(
            err < 1e-10,
            "CG scale {:.0e}: adapter vs raw solution error {:.2e}",
            s,
            err
        );
    }
}

#[test]
fn iccg_adapter_matches_raw_iccg_solve() {
    let a = spd_matrix_3x3();
    let b = b_vector();
    let tol = 1e-10;
    let max_iter = 1000;

    for &s in &SCALES {
        let bs: Vec<f64> = b.iter().map(|&v| s * v).collect();

        let raw = iccg_solve(&a, &bs, max_iter, tol);

        let mut adapter = IccgSolver::new().with_params(max_iter, tol);
        adapter.factor(&a).unwrap();
        let adapter_x = adapter.solve(&bs).unwrap();

        let err = solution_error(&raw.x, &adapter_x);
        assert!(
            err < 1e-8,
            "ICCG scale {:.0e}: adapter vs raw solution error {:.2e}",
            s,
            err
        );
    }
}
