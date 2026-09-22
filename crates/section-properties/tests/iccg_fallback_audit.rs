//! Phase 60: ICCG fallback diagonal scale-aware regression tests.
//!
//! Verifies that the ICCG adapter's IC(0) factorization returns a structured
//! error on breakdown instead of silently regularizing with a fixed 1e-12
//! diagonal.  This eliminates the scale-dependent behavior of the previous
//! absolute fallback.

use section_properties::fea::SparseMatrix;
use section_properties::fea::solver::LinearSolver;
use section_properties::fea::solver::impls::iccg::IccgSolver;
use section_properties::fea::solvers::iccg_solve;

fn scaled_spd(s: f64) -> SparseMatrix {
    let mut a = SparseMatrix::new(3);
    a.add(0, 0, s * 4.0);
    a.add(0, 1, s * 1.0);
    a.add(0, 2, s * 1.0);
    a.add(1, 0, s * 1.0);
    a.add(1, 1, s * 3.0);
    a.add(1, 2, s * 1.0);
    a.add(2, 0, s * 1.0);
    a.add(2, 1, s * 1.0);
    a.add(2, 2, s * 2.0);
    a.compress();
    a
}

fn scaled_breakdown(s: f64) -> SparseMatrix {
    let mut a = SparseMatrix::new(3);
    a.add(0, 0, s * 1.0);
    a.add(0, 2, s * 1.0);
    a.add(1, 1, s * 1.0);
    a.add(1, 2, s * 1.0);
    a.add(2, 0, s * 1.0);
    a.add(2, 1, s * 1.0);
    a.add(2, 2, s * 1.0);
    a.compress();
    a
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

const SCALES: [f64; 7] = [1e-12, 1e-9, 1e-6, 1.0, 1e6, 1e9, 1e12];

// --- 9.1 Tiny-scale SPD ---

#[test]
fn spd_tiny_scale_no_breakdown() {
    for &s in &[1e-18, 1e-15, 1e-12, 1e-9, 1e-6] {
        let a = scaled_spd(s);
        let b = vec![s * 1.0, s * 2.0, s * 3.0];
        let b_norm = b.iter().map(|v| v * v).sum::<f64>().sqrt();

        let mut adapter = IccgSolver::new().with_params(1000, 1e-10);
        adapter.factor(&a).unwrap();
        let x = adapter.solve(&b).unwrap();
        let rel_res = residual_norm(&a, &x, &b) / b_norm;

        assert!(
            rel_res < 1e-6,
            "SPD tiny scale {:.0e}: rel_res {:.2e}",
            s,
            rel_res
        );
    }
}

// --- 9.2 Large-scale SPD ---

#[test]
fn spd_large_scale_no_breakdown() {
    for &s in &[1e6, 1e9, 1e12, 1e15, 1e18] {
        let a = scaled_spd(s);
        let b = vec![s * 1.0, s * 2.0, s * 3.0];
        let b_norm = b.iter().map(|v| v * v).sum::<f64>().sqrt();

        let mut adapter = IccgSolver::new().with_params(1000, 1e-10);
        adapter.factor(&a).unwrap();
        let x = adapter.solve(&b).unwrap();
        let rel_res = residual_norm(&a, &x, &b) / b_norm;

        assert!(
            rel_res < 1e-6,
            "SPD large scale {:.0e}: rel_res {:.2e}",
            s,
            rel_res
        );
    }
}

// --- 9.3 Factorization scaling consistency ---

#[test]
fn spd_factorization_scale_invariant() {
    let b = vec![1.0, 2.0, 3.0];
    let tol = 1e-10;
    let max_iter = 1000;

    let a_base = scaled_spd(1.0);
    let mut adapter_base = IccgSolver::new().with_params(max_iter, tol);
    adapter_base.factor(&a_base).unwrap();
    let x_base = adapter_base.solve(&b).unwrap();

    for &s in &[1e-6, 1.0, 1e6] {
        let a = scaled_spd(s);
        let bs: Vec<f64> = b.iter().map(|&v| s * v).collect();

        let mut adapter = IccgSolver::new().with_params(max_iter, tol);
        adapter.factor(&a).unwrap();
        let x = adapter.solve(&bs).unwrap();

        let err: f64 = x
            .iter()
            .zip(x_base.iter())
            .map(|(&xi, &xb)| (xi - xb).powi(2))
            .sum::<f64>()
            .sqrt();
        let scale_norm = x_base.iter().map(|&v| v * v).sum::<f64>().sqrt();

        assert!(
            err / scale_norm < 1e-10,
            "Scale {:.0e}: solution not scale-consistent, err={:.2e}",
            s,
            err / scale_norm
        );
    }
}

// --- 9.4 Breakdown matrix returns error, not silent regularization ---

#[test]
fn breakdown_returns_error_not_silent_regularization() {
    for &s in &SCALES {
        let a = scaled_breakdown(s);

        let mut adapter = IccgSolver::new().with_params(1000, 1e-10);
        let result = adapter.factor(&a);

        assert!(
            result.is_err(),
            "Breakdown matrix at scale {:.0e}: factor should return Err, not silently regularize",
            s
        );
    }
}

// --- 9.5 Breakdown behavior is scale-consistent ---

#[test]
fn breakdown_scale_consistent_error() {
    let mut all_error = true;
    for &s in &SCALES {
        let a = scaled_breakdown(s);
        let mut adapter = IccgSolver::new().with_params(1000, 1e-10);
        if adapter.factor(&a).is_ok() {
            all_error = false;
        }
    }
    assert!(
        all_error,
        "ICCG adapter should consistently reject breakdown matrices at all scales"
    );
}

// --- 9.6 Raw iccg_solve fallback behavior on breakdown ---

#[test]
fn breakdown_raw_iccg_fallback_to_cg() {
    let a = scaled_breakdown(1.0);
    let b = vec![1.0, 2.0, 3.0];

    let raw = iccg_solve(&a, &b, 1000, 1e-10);
    assert!(
        raw.status == section_properties::fea::CgStatus::Converged
            || raw.status == section_properties::fea::CgStatus::NotPositiveDefinite
            || raw.status == section_properties::fea::CgStatus::Breakdown,
        "Raw iccg_solve on breakdown matrix should handle gracefully, got {:?}",
        raw.status
    );
}

// --- 9.7 Existing ICCG regression (Phase 59) still passes ---

#[test]
fn spd_normal_scale_converges() {
    let a = scaled_spd(1.0);
    let b = vec![1.0, 2.0, 3.0];
    let b_norm = b.iter().map(|v| v * v).sum::<f64>().sqrt();

    let mut adapter = IccgSolver::new().with_params(1000, 1e-10);
    adapter.factor(&a).unwrap();
    let x = adapter.solve(&b).unwrap();
    let rel_res = residual_norm(&a, &x, &b) / b_norm;

    assert!(rel_res < 1e-10, "Normal SPD: rel_res {:.2e}", rel_res);
}
