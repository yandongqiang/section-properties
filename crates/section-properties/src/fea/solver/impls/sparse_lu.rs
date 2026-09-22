//! Sparse LU solver wrapper
use crate::fea::SparseMatrix;
use crate::fea::solver::{LinearSolver, SolverCapabilities, SolverError};
use crate::fea::solvers::SparseLu;

/// Sparse LU solver wrapper
pub struct SparseLuSolver {
    lu: Option<SparseLu>,
    n: usize,
    scale: f64,
}

impl SparseLuSolver {
    pub fn new() -> Self {
        Self {
            lu: None,
            n: 0,
            scale: 1.0,
        }
    }
}

impl LinearSolver for SparseLuSolver {
    fn name(&self) -> &'static str {
        "sparse_lu"
    }

    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::sparse_lu()
    }

    fn factor(&mut self, matrix: &SparseMatrix) -> Result<(), SolverError> {
        // Invalidate any previous factorization first, so a failed factor()
        // cannot leave stale factors usable by a later solve().
        self.lu = None;
        self.n = 0;

        let n = matrix.n;
        if n == 0 {
            return Err(SolverError::invalid_input("Empty matrix"));
        }

        let lu = SparseLu::factor(matrix).map_err(|e| SolverError::factorization_failed(e))?;

        // Compute scale
        let mut scale = 0.0f64;
        for &val in &matrix.csr_vals {
            scale = scale.max(val.abs());
        }
        self.scale = scale.max(1.0);

        self.lu = Some(lu);
        self.n = n;

        Ok(())
    }

    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolverError> {
        let lu = self
            .lu
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

        lu.solve(rhs).map_err(|e| SolverError::solve_failed(e))
    }
}
