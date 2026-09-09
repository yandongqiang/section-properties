//! Dense Gaussian elimination solver (for small systems)
use crate::fea::SparseMatrix;
use crate::fea::solver::{LinearSolver, SolverCapabilities, SolverError};

/// Dense Gaussian elimination solver (for small systems)
pub struct DenseGaussianSolver {
    n: usize,
    a: Vec<Vec<f64>>,
    pivot: Vec<usize>,
    scale: f64,
}

impl DenseGaussianSolver {
    pub fn new() -> Self {
        Self {
            n: 0,
            a: Vec::new(),
            pivot: Vec::new(),
            scale: 1.0,
        }
    }
}

impl LinearSolver for DenseGaussianSolver {
    fn name(&self) -> &'static str {
        "dense"
    }

    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::dense()
    }

    fn factor(&mut self, matrix: &SparseMatrix) -> Result<(), SolverError> {
        let n = matrix.n;
        if n == 0 {
            return Err(SolverError::invalid_input("Empty matrix"));
        }

        // Convert to dense
        let mut a = vec![vec![0.0f64; n]; n];
        for i in 0..n {
            for k in matrix.row_ptr[i]..matrix.row_ptr[i + 1] {
                let j = matrix.csr_cols[k];
                a[i][j] = matrix.csr_vals[k];
            }
        }

        // Compute scale for pivot tolerance
        let mut scale = 0.0f64;
        for i in 0..n {
            for j in 0..n {
                scale = scale.max(a[i][j].abs());
            }
        }
        self.scale = scale.max(1.0);

        // Gaussian elimination with partial pivoting
        let mut pivot = (0..n).collect::<Vec<usize>>();

        for k in 0..n {
            // Partial pivoting
            let mut max_row = k;
            let mut max_val = a[k][k].abs();
            for i in (k + 1)..n {
                let v = a[i][k].abs();
                if v > max_val {
                    max_val = v;
                    max_row = i;
                }
            }

            let pivot_tol = crate::fea::PIVOT_TOL_BASE * self.scale.max(1.0);
            if max_val <= pivot_tol {
                return Err(SolverError::singular(format!(
                    "Singular or near-singular matrix at column {}: max pivot = {:.2e}, tolerance = {:.2e}",
                    k, max_val, pivot_tol
                )));
            }

            if max_row != k {
                a.swap(k, max_row);
                pivot.swap(k, max_row);
            }

            // Eliminate
            for i in (k + 1)..n {
                let factor = a[i][k] / a[k][k];
                for j in k..n {
                    a[i][j] -= factor * a[k][j];
                }
            }
        }

        self.n = n;
        self.a = a;
        self.pivot = pivot;

        Ok(())
    }

    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolverError> {
        let n = self.n;
        if n == 0 {
            return Err(SolverError::not_factorized());
        }
        if rhs.len() != n {
            return Err(SolverError::dimension_mismatch(n, rhs.len()));
        }

        // Check for NaN/Inf
        for (i, &val) in rhs.iter().enumerate() {
            if !val.is_finite() {
                return Err(SolverError::non_finite(i));
            }
        }

        // Apply permutation: Pb
        let mut x = vec![0.0f64; n];
        for i in 0..n {
            x[i] = rhs[self.pivot[i]];
        }

        // Forward substitution
        for i in 0..n {
            let mut sum = 0.0;
            for j in 0..i {
                sum += self.a[i][j] * x[j];
            }
            x[i] -= sum;
        }

        // Backward substitution
        for i in (0..n).rev() {
            let mut sum = 0.0;
            for j in (i + 1)..n {
                sum += self.a[i][j] * x[j];
            }
            let pivot_val = self.a[i][i];
            if !pivot_val.is_finite() || pivot_val == 0.0 {
                return Err(SolverError::singular(format!(
                    "Zero or invalid diagonal at row {}: {:.2e}",
                    i, pivot_val
                )));
            }
            x[i] = (x[i] - sum) / pivot_val;
        }

        // Inverse permutation
        let mut out = vec![0.0; n];
        for (new_idx, &old_idx) in self.pivot.iter().enumerate() {
            out[old_idx] = x[new_idx];
        }

        Ok(out)
    }
}
