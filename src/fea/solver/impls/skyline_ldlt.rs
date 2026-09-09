//! Skyline LDL^T factorization solver
use crate::fea::SparseMatrix;
use crate::fea::solver::{LinearSolver, SolverCapabilities, SolverError};

/// Skyline LDL^T factorization solver
pub struct SkylineLdltSolver {
    n: usize,
    diag: Vec<f64>,
    upper: Vec<f64>,
    row_start: Vec<usize>,
    col_start: Vec<usize>,
    scale: f64,
    symmetric: bool,
}

impl SkylineLdltSolver {
    pub fn new() -> Self {
        Self {
            n: 0,
            diag: Vec::new(),
            upper: Vec::new(),
            row_start: Vec::new(),
            col_start: Vec::new(),
            scale: 1.0,
            symmetric: true,
        }
    }
}

impl LinearSolver for SkylineLdltSolver {
    fn name(&self) -> &'static str {
        "skyline_ldlt"
    }

    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::skyline_ldlt()
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

        // Compute skyline profile
        let mut row_start = vec![0usize; n + 1];
        let mut col_start = vec![0usize; n + 1];

        // Find first non-zero in each row (from diagonal leftwards)
        for i in 0..n {
            let mut first = i;
            for k in matrix.row_ptr[i]..matrix.row_ptr[i + 1] {
                let j = matrix.csr_cols[k];
                if j <= i {
                    first = first.min(j);
                }
            }
            row_start[i + 1] = i - first + 1;
        }

        // Find first non-zero in each column (from diagonal upwards)
        for j in 0..n {
            let mut first = j;
            for i in 0..n {
                for k in matrix.row_ptr[i]..matrix.row_ptr[i + 1] {
                    if matrix.csr_cols[k] == j && i <= j {
                        first = first.min(i);
                    }
                }
            }
            col_start[j + 1] = j - first + 1;
        }

        // Cumulative sum
        for i in 1..=n {
            row_start[i] += row_start[i - 1];
            col_start[i] += col_start[i - 1];
        }

        let nnz_upper = row_start[n];
        let mut diag = vec![0.0f64; n];
        let mut upper = vec![0.0f64; nnz_upper];

        // Fill skyline matrix
        for i in 0..n {
            let diag_idx = row_start[i + 1] - 1;
            for k in matrix.row_ptr[i]..matrix.row_ptr[i + 1] {
                let j = matrix.csr_cols[k];
                let val = matrix.csr_vals[k];
                if j == i {
                    diag[i] = val;
                } else if j < i {
                    // Lower triangle - store in upper by symmetry
                    let idx = row_start[j] + (i - j) - 1;
                    upper[idx] = val;
                } else {
                    // Upper triangle
                    let idx = row_start[i] + (j - i) - 1;
                    upper[idx] = val;
                }
            }
        }

        // Compute scale for pivot tolerance
        let mut scale = 0.0f64;
        for i in 0..n {
            scale = scale.max(diag[i].abs());
            for idx in row_start[i]..row_start[i + 1] - 1 {
                scale = scale.max(upper[idx].abs());
            }
        }
        self.scale = scale.max(1.0);

        // LDL^T factorization
        let pivot_tol = crate::fea::PIVOT_TOL_BASE * self.scale.max(1.0);

        for k in 0..n {
            // Check pivot
            if diag[k].abs() <= pivot_tol {
                return Err(SolverError::singular(format!(
                    "Skyline pivot {} near zero: {:.2e} <= {:.2e}",
                    k,
                    diag[k].abs(),
                    pivot_tol
                )));
            }

            // L_ik = A_ik / D_kk
            for i in (row_start[k]..row_start[k + 1] - 1).rev() {
                let row = k - (row_start[k + 1] - 1 - i);
                let l_ik = upper[i] / diag[k];
                upper[i] = l_ik;

                // Update trailing submatrix
                for j in (row_start[row]..i).rev() {
                    let col = row - (i - j);
                    let idx_j = row_start[row] + (col - row) - 1;
                    upper[idx_j] -= l_ik * upper[i];
                }
            }
        }

        self.n = n;
        self.diag = diag;
        self.upper = upper;
        self.row_start = row_start;
        self.col_start = col_start;

        Ok(())
    }

    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolverError> {
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

        let mut x = rhs.to_vec();

        // Forward substitution: L y = b
        for i in 0..n {
            let mut sum = 0.0;
            for idx in self.row_start[i]..self.row_start[i + 1] - 1 {
                let j = i - (self.row_start[i + 1] - 1 - idx);
                sum += self.upper[idx] * x[j];
            }
            x[i] -= sum;
        }

        // Diagonal scaling: D z = y
        for i in 0..n {
            if self.diag[i] == 0.0 || !self.diag[i].is_finite() {
                return Err(SolverError::singular(format!(
                    "Zero or invalid diagonal at row {}: {:.2e}",
                    i, self.diag[i]
                )));
            }
            x[i] /= self.diag[i];
        }

        // Backward substitution: L^T x = z
        for i in (0..n).rev() {
            let mut sum = 0.0;
            for idx in self.row_start[i]..self.row_start[i + 1] - 1 {
                let j = i - (self.row_start[i + 1] - 1 - idx);
                sum += self.upper[idx] * x[j];
            }
            x[i] -= sum;
        }

        Ok(x)
    }
}
