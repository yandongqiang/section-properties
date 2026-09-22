//! Incomplete Cholesky CG solver
use crate::fea::SparseMatrix;
use crate::fea::solver::{LinearSolver, SolverCapabilities, SolverError};

/// Incomplete Cholesky CG solver
pub struct IccgSolver {
    n: usize,
    max_iter: usize,
    tol: f64,
    l_diag: Vec<f64>,
    l_lower: Vec<f64>,
    l_row_ptr: Vec<usize>,
    l_col_idx: Vec<usize>,
    matrix: Option<SparseMatrix>,
}

impl IccgSolver {
    pub fn new() -> Self {
        Self {
            n: 0,
            max_iter: 10000,
            tol: 1e-10,
            l_diag: Vec::new(),
            l_lower: Vec::new(),
            l_row_ptr: Vec::new(),
            l_col_idx: Vec::new(),
            matrix: None,
        }
    }

    pub fn with_params(mut self, max_iter: usize, tol: f64) -> Self {
        self.max_iter = max_iter;
        self.tol = tol;
        self
    }
}

impl LinearSolver for IccgSolver {
    fn name(&self) -> &'static str {
        "iccg"
    }

    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::iccg()
    }

    fn factor(&mut self, matrix: &SparseMatrix) -> Result<(), SolverError> {
        // Invalidate any previous factorization first, so a failed factor()
        // cannot leave stale preconditioner/factors usable by a later solve().
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

        // Incomplete Cholesky factorization (IC(0))
        let mut l_diag = vec![0.0f64; n];
        let mut l_lower = Vec::new();
        let mut l_col_idx = Vec::new();
        let mut l_row_ptr = vec![0usize; n + 1];

        // Count nonzeros in lower triangle
        for i in 0..n {
            for k in matrix.row_ptr[i]..matrix.row_ptr[i + 1] {
                let j = matrix.csr_cols[k];
                if j <= i {
                    l_row_ptr[i + 1] += 1;
                }
            }
        }

        for i in 1..=n {
            l_row_ptr[i] += l_row_ptr[i - 1];
        }

        l_lower.resize(l_row_ptr[n], 0.0);
        l_col_idx.resize(l_row_ptr[n], 0);

        // Fill column indices
        let mut pos = l_row_ptr.clone();
        for i in 0..n {
            for k in matrix.row_ptr[i]..matrix.row_ptr[i + 1] {
                let j = matrix.csr_cols[k];
                if j <= i {
                    let idx = pos[i];
                    l_col_idx[idx] = j;
                    pos[i] += 1;
                }
            }
        }

        // IC(0) factorization (Cholesky form: L L^T ≈ A, L_ii = sqrt(d_i))
        for i in 0..n {
            // Compute off-diagonals FIRST (diagonal depends on them).
            // Guard against empty rows (e.g. zero diagonal skipped by compress()).
            if l_row_ptr[i + 1] > l_row_ptr[i] {
                for idx in l_row_ptr[i]..l_row_ptr[i + 1] - 1 {
                    let j = l_col_idx[idx];

                    // Find A_ij
                    let mut a_ij = 0.0;
                    for k in matrix.row_ptr[i]..matrix.row_ptr[i + 1] {
                        if matrix.csr_cols[k] == j {
                            a_ij = matrix.csr_vals[k];
                            break;
                        }
                    }
                    let mut sum = a_ij;

                    // Subtract contributions: sum -= Σ_{k<j} L_ik * L_jk
                    for idx2 in l_row_ptr[j]..l_row_ptr[j + 1] {
                        let k = l_col_idx[idx2];
                        if k < j {
                            // Find L_ik
                            for idx3 in l_row_ptr[i]..l_row_ptr[i + 1] {
                                if l_col_idx[idx3] == k {
                                    sum -= l_lower[idx3] * l_lower[idx2];
                                    break;
                                }
                            }
                        }
                    }

                    l_lower[idx] = sum / l_diag[j];
                }
            }

            // Compute diagonal AFTER off-diagonals.
            let mut sum = 0.0;
            for k in matrix.row_ptr[i]..matrix.row_ptr[i + 1] {
                let j = matrix.csr_cols[k];
                let val = matrix.csr_vals[k];
                if j == i {
                    sum += val;
                } else if j < i {
                    // Find L_ij in our structure
                    for idx in l_row_ptr[i]..l_row_ptr[i + 1] {
                        if l_col_idx[idx] == j {
                            sum -= l_lower[idx] * l_lower[idx];
                            break;
                        }
                    }
                }
            }

            if sum <= 0.0 || !sum.is_finite() {
                return Err(SolverError::singular(format!(
                    "IC(0) breakdown at row {i}: non-positive diagonal ({:.2e})",
                    sum
                )));
            } else {
                l_diag[i] = sum.sqrt();
            }
        }

        self.n = n;
        self.l_diag = l_diag;
        self.l_lower = l_lower;
        self.l_row_ptr = l_row_ptr;
        self.l_col_idx = l_col_idx;
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

        // Scale-aware convergence: use relative residual ||r|| / ||b|| < tol,
        // matching the raw iccg_solve() semantics.  The previous code used the
        // absolute preconditioned residual sqrt(r^T M^{-1} r) < tol, which is
        // scale-dependent and uses a preconditioned quantity rather than the
        // true residual.
        let b_norm = rhs.iter().map(|v| v * v).sum::<f64>().sqrt();
        if b_norm == 0.0 || !b_norm.is_finite() {
            return Ok(vec![0.0f64; n]);
        }
        let conv_tol = self.tol * b_norm;

        // Preconditioned CG with IC(0)
        let mut x = vec![0.0f64; n];
        let mut r = rhs.to_vec();

        // Apply preconditioner: M = L L^T
        let mut z = self.apply_preconditioner(&r)?;
        let mut p = z.clone();

        let mut rz_old = r.iter().zip(z.iter()).map(|(ri, zi)| ri * zi).sum::<f64>();

        // Check true residual for initial convergence.
        let r_norm_sq = r.iter().map(|v| v * v).sum::<f64>();
        if r_norm_sq.sqrt() < conv_tol {
            return Ok(x);
        }

        // Scale of initial preconditioned residual for relative convergence.
        let rz_scale = rz_old.abs().sqrt();

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

            let alpha = rz_old / p_ap;

            for i in 0..n {
                x[i] += alpha * p[i];
                r[i] -= alpha * ap[i];
            }

            z = self.apply_preconditioner(&r)?;
            let rz_new = r.iter().zip(z.iter()).map(|(ri, zi)| ri * zi).sum::<f64>();

            // Check true residual for convergence (not preconditioned residual).
            // Dual criterion: (1) true relative residual ||r|| < tol * ||b||,
            // (2) relative preconditioned residual sqrt(r^T z) < tol * sqrt(rz₀).
            // The secondary criterion catches early convergence when the
            // preconditioned residual is small but the true residual is
            // temporarily inflated by cancellation — avoiding unnecessary
            // extra iterations.  Both criteria are scale-aware (Phase 59).
            let r_norm_sq = r.iter().map(|v| v * v).sum::<f64>();
            if r_norm_sq.sqrt() < conv_tol || rz_new.abs().sqrt() < self.tol * rz_scale {
                return Ok(x);
            }

            let beta = rz_new / rz_old;
            for i in 0..n {
                p[i] = z[i] + beta * p[i];
            }

            rz_old = rz_new;
        }

        Err(SolverError::not_converged())
    }
}

impl IccgSolver {
    fn apply_preconditioner(&self, r: &[f64]) -> Result<Vec<f64>, SolverError> {
        let n = self.n;
        let mut y = vec![0.0f64; n];

        // Forward: L y = r
        for i in 0..n {
            let mut sum = r[i];
            for idx in self.l_row_ptr[i]..self.l_row_ptr[i + 1] - 1 {
                let j = self.l_col_idx[idx];
                sum -= self.l_lower[idx] * y[j];
            }
            y[i] = sum / self.l_diag[i];
        }

        // Backward: L^T z = y via scattered updates.
        let mut z = y;
        for i in (0..n).rev() {
            let zi = z[i] / self.l_diag[i];
            z[i] = zi;
            for idx in self.l_row_ptr[i]..self.l_row_ptr[i + 1] - 1 {
                let j = self.l_col_idx[idx];
                z[j] -= self.l_lower[idx] * zi;
            }
        }

        Ok(z)
    }
}
