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

        // IC(0) factorization
        for i in 0..n {
            let _diag_pos = l_row_ptr[i + 1] - 1; // Diagonal is last in row

            // Compute diagonal
            let mut sum = 0.0;
            for k in matrix.row_ptr[i]..matrix.row_ptr[i + 1] {
                let j = matrix.csr_cols[k];
                let val = matrix.csr_vals[k];
                if j == i {
                    sum = val;
                } else if j < i {
                    // Find L_ij in our structure
                    for idx in l_row_ptr[j]..l_row_ptr[j + 1] {
                        if l_col_idx[idx] == i {
                            sum -= l_lower[idx] * l_lower[idx] * l_diag[j];
                            break;
                        }
                    }
                }
            }

            if sum <= 0.0 {
                // Modified IC: add small positive value
                l_diag[i] = 1e-12;
            } else {
                l_diag[i] = sum.sqrt();
            }

            // Compute off-diagonals
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

                // Subtract contributions
                for idx2 in l_row_ptr[j]..l_row_ptr[j + 1] {
                    let k = l_col_idx[idx2];
                    if k < j {
                        // Find L_ik
                        for idx3 in l_row_ptr[i]..l_row_ptr[i + 1] {
                            if l_col_idx[idx3] == k {
                                sum -= l_lower[idx3] * l_lower[idx2] * l_diag[k];
                                break;
                            }
                        }
                    }
                }

                l_lower[idx] = sum / l_diag[j];
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

        // Preconditioned CG with IC(0)
        let mut x = vec![0.0f64; n];
        let mut r = rhs.to_vec();

        // Apply preconditioner: M = L L^T
        let mut z = self.apply_preconditioner(&r)?;
        let mut p = z.clone();

        let mut rz_old = r.iter().zip(z.iter()).map(|(ri, zi)| ri * zi).sum::<f64>();

        if rz_old.sqrt() < self.tol {
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

            let alpha = rz_old / p_ap;

            for i in 0..n {
                x[i] += alpha * p[i];
                r[i] -= alpha * ap[i];
            }

            z = self.apply_preconditioner(&r)?;
            let rz_new = r.iter().zip(z.iter()).map(|(ri, zi)| ri * zi).sum::<f64>();

            if rz_new.sqrt() < self.tol {
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
        let mut z = vec![0.0f64; n];

        // Forward: L y = r
        for i in 0..n {
            let mut sum = r[i];
            for idx in self.l_row_ptr[i]..self.l_row_ptr[i + 1] - 1 {
                let j = self.l_col_idx[idx];
                sum -= self.l_lower[idx] * y[j];
            }
            y[i] = sum / self.l_diag[i];
        }

        // Backward: L^T z = y
        for i in (0..n).rev() {
            let mut sum = y[i];
            for idx in self.l_row_ptr[i]..self.l_row_ptr[i + 1] - 1 {
                let j = self.l_col_idx[idx];
                sum -= self.l_lower[idx] * z[j];
            }
            z[i] = sum / self.l_diag[i];
        }

        Ok(z)
    }
}
