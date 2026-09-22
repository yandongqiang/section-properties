//! Phase 62: ICCG empty-row / missing-diagonal regression tests.
//!
//! Verifies that IC(0) factorization handles rows with no lower-triangle
//! entries (e.g. zero diagonal skipped by compress()) without panicking,
//! returning structured errors or correct results as appropriate.

use section_properties::fea::SparseMatrix;
use section_properties::fea::solver::LinearSolver;
use section_properties::fea::solver::impls::iccg::IccgSolver;

fn matvec(a: &SparseMatrix, x: &[f64]) -> Vec<f64> {
    let mut y = vec![0.0; a.n];
    a.matvec_into(x, &mut y);
    y
}

fn residual_norm(a: &SparseMatrix, x: &[f64], b: &[f64]) -> f64 {
    let ax = matvec(a, x);
    (0..a.n).map(|i| (ax[i] - b[i]).powi(2)).sum::<f64>().sqrt()
}

// --- Case A: diagonal-only SPD matrix ---

#[test]
fn iccg_diagonal_matrix() {
    let mut a = SparseMatrix::new(3);
    a.add(0, 0, 4.0);
    a.add(1, 1, 3.0);
    a.add(2, 2, 2.0);
    a.compress();

    let b = vec![1.0, 2.0, 3.0];
    let b_norm = b.iter().map(|v| v * v).sum::<f64>().sqrt();

    let mut solver = IccgSolver::new().with_params(1000, 1e-12);
    solver.factor(&a).unwrap();
    let x = solver.solve(&b).unwrap();

    let rel_res = residual_norm(&a, &x, &b) / b_norm;
    assert!(rel_res < 1e-10, "rel_res {:.2e}", rel_res);

    // Exact: x = [0.25, 0.667, 1.5]
    assert!((x[0] - 0.25).abs() < 1e-12, "x[0] = {}", x[0]);
    assert!((x[1] - 2.0 / 3.0).abs() < 1e-12, "x[1] = {}", x[1]);
    assert!((x[2] - 1.5).abs() < 1e-12, "x[2] = {}", x[2]);
}

// --- Case B: mixed sparse matrix with empty lower rows ---

#[test]
fn iccg_empty_lower_row() {
    // Row 2 and 3 have no lower off-diagonal entries
    let mut a = SparseMatrix::new(4);
    a.add(0, 0, 4.0);
    a.add(0, 1, 1.0);
    a.add(1, 0, 1.0);
    a.add(1, 1, 4.0);
    a.add(2, 2, 3.0);
    a.add(3, 3, 2.0);
    a.compress();

    let b = vec![1.0, 2.0, 3.0, 4.0];
    let b_norm = b.iter().map(|v| v * v).sum::<f64>().sqrt();

    let mut solver = IccgSolver::new().with_params(1000, 1e-12);
    solver.factor(&a).unwrap();
    let x = solver.solve(&b).unwrap();

    let rel_res = residual_norm(&a, &x, &b) / b_norm;
    assert!(rel_res < 1e-10, "rel_res {:.2e}", rel_res);

    // Verify no NaN/Inf
    assert!(x.iter().all(|v| v.is_finite()), "non-finite: {:?}", x);
}

// --- Case C: zero diagonal returns Err, not panic ---

#[test]
fn iccg_zero_diagonal_returns_error() {
    let mut a = SparseMatrix::new(2);
    a.add(0, 0, 0.0);
    a.add(1, 1, 2.0);
    a.compress();

    let mut solver = IccgSolver::new().with_params(1000, 1e-10);
    let result = solver.factor(&a);
    assert!(result.is_err(), "zero diagonal should return Err, got Ok");
}

// --- Case C2: zero diagonal with off-diagonal entries ---

#[test]
fn iccg_zero_diagonal_with_coupling_returns_error() {
    let mut a = SparseMatrix::new(3);
    a.add(0, 0, 0.0);
    a.add(0, 1, 1.0);
    a.add(1, 0, 1.0);
    a.add(1, 1, 3.0);
    a.add(1, 2, 1.0);
    a.add(2, 1, 1.0);
    a.add(2, 2, 2.0);
    a.compress();

    let mut solver = IccgSolver::new().with_params(1000, 1e-10);
    let result = solver.factor(&a);
    assert!(
        result.is_err(),
        "zero diagonal with coupling should return Err"
    );
}

// --- Case D: negative diagonal returns Err ---

#[test]
fn iccg_negative_diagonal_returns_error() {
    let mut a = SparseMatrix::new(2);
    a.add(0, 0, -1.0);
    a.add(1, 1, 2.0);
    a.compress();

    let mut solver = IccgSolver::new().with_params(1000, 1e-10);
    let result = solver.factor(&a);
    assert!(result.is_err(), "negative diagonal should return Err");

    // Verify no NaN in error message
    if let Err(e) = &result {
        let msg = format!("{e}");
        assert!(!msg.contains("NaN"), "error message contains NaN: {}", msg);
    }
}
