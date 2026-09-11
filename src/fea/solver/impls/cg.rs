//! Conjugate Gradient iterative solver
use crate::fea::SparseMatrix;
use crate::fea::solver::{LinearSolver, SolverCapabilities, SolverError};

/// Conjugate Gradient iterative solver
pub struct CgSolver {
    n: usize,
    max_iter: usize,
    tol: f64,
    matrix: Option<SparseMatrix>,
}

impl CgSolver {
    pub fn new() -> Self {
        Self {
            n: 0,
            max_iter: 10000,
            tol: 1e-10,
            matrix: None,
        }
    }

    pub fn with_params(mut self, max_iter: usize, tol: f64) -> Self {
        self.max_iter = max_iter;
        self.tol = tol;
        self
    }
}

impl LinearSolver for CgSolver {
    fn name(&self) -> &'static str {
        "cg"
    }

    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::cg()
    }

    fn factor(&mut self, matrix: &SparseMatrix) -> Result<(), SolverError> {
        // Invalidate any previous factorization first, so a failed factor()
        // cannot leave stale state usable by a later solve().
        self.matrix = None;
        self.n = 0;

        let n = matrix.n;
        if n == 0 {
            return Err(SolverError::invalid_input("Empty matrix"));
        }

        // Check symmetry
        if !matrix.is_symmetric(1e-12) {
            return Err(SolverError::not_symmetric());
        }

        // Check positive definiteness (diagonal dominance heuristic)
        for i in 0..n {
            let mut diag = 0.0;
            let mut off_diag_sum = 0.0;
            for k in matrix.row_ptr[i]..matrix.row_ptr[i + 1] {
                let j = matrix.csr_cols[k];
                let val = matrix.csr_vals[k];
                if i == j {
                    diag = val;
                } else {
                    off_diag_sum += val.abs();
                }
            }
            if diag <= off_diag_sum {
                // Not strictly diagonally dominant - might not be SPD
                // But CG can still work, just warn via capabilities
            }
        }

        self.n = n;
        self.matrix = Some(matrix.clone());

        Ok(())
    }

    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolverError> {
        let matrix = self
            .matrix
            .as_ref()
            .ok_or_else(|| SolverError::not_factorized())?;
        let n = self.n;

        if rhs.len() != n {
            return Err(SolverError::dimension_mismatch(n, rhs.len()));
        }

        // Check for NaN/Inf
        for (i, &val) in rhs.iter().enumerate() {
            if !val.is_finite() {
                return Err(SolverError::non_finite(i));
            }
        }

        let mut x = vec![0.0f64; n];
        let mut r = rhs.to_vec();
        let mut p = r.clone();
        let mut rsold = r.iter().map(|v| v * v).sum::<f64>();

        if rsold.sqrt() < self.tol {
            return Ok(x);
        }

        for _iter in 0..self.max_iter {
            // A * p
            let mut ap = vec![0.0f64; n];
            for i in 0..n {
                let mut sum = 0.0;
                for k in matrix.row_ptr[i]..matrix.row_ptr[i + 1] {
                    let j = matrix.csr_cols[k];
                    sum += matrix.csr_vals[k] * p[j];
                }
                ap[i] = sum;
            }

            let p_ap: f64 = p.iter().zip(ap.iter()).map(|(pi, api)| pi * api).sum();
            if p_ap <= 0.0 {
                return Err(SolverError::singular(
                    "Matrix not positive definite (p^T A p <= 0)".to_string(),
                ));
            }

            let alpha = rsold / p_ap;

            for i in 0..n {
                x[i] += alpha * p[i];
                r[i] -= alpha * ap[i];
            }

            let rsnew = r.iter().map(|v| v * v).sum::<f64>();

            if rsnew.sqrt() < self.tol {
                return Ok(x);
            }

            let beta = rsnew / rsold;
            for i in 0..n {
                p[i] = r[i] + beta * p[i];
            }

            rsold = rsnew;
        }

        Err(SolverError::not_converged())
    }
}
