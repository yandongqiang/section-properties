//! Skyline (profile) LDL^T factorization solver — unified `LinearSolver`
//! backend.
//!
//! This backend is a thin adapter over the mature, tested
//! [`crate::fea::SkylineLdlt`] implementation (RCM-ordered profile LDL^T).
//! Delegating keeps a **single** skyline factorization in the codebase: the
//! registry backend and the direct `SkylineLdlt` API now share the same
//! numerical implementation.
//!
//! SPD only: the skyline LDL^T requires a symmetric positive-definite matrix.
//! Symmetry is checked here; a non-positive pivot is reported by the
//! underlying factorization as a singular/near-singular error.
use crate::fea::SparseMatrix;
use crate::fea::solver::{LinearSolver, SolverCapabilities, SolverError};

/// Skyline LDL^T factorization solver
pub struct SkylineLdltSolver {
    inner: Option<crate::fea::SkylineLdlt>,
    n: usize,
}

impl SkylineLdltSolver {
    pub fn new() -> Self {
        Self { inner: None, n: 0 }
    }
}

impl Default for SkylineLdltSolver {
    fn default() -> Self {
        Self::new()
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

        // Skyline LDL^T requires a symmetric matrix.
        if !matrix.is_symmetric(1e-12) {
            return Err(SolverError::not_symmetric());
        }

        let ldlt = crate::fea::SkylineLdlt::factor(matrix).map_err(|e| match e {
            // Preserve the "singular / not SPD" contract for callers that
            // distinguish it from a generic factorisation failure.
            crate::mesh::fem::FemError::SingularMatrix => SolverError::singular(e.to_string()),
            _ => SolverError::factorization_failed(e.to_string()),
        })?;

        self.inner = Some(ldlt);
        self.n = n;
        Ok(())
    }

    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolverError> {
        let ldlt = self
            .inner
            .as_ref()
            .ok_or_else(SolverError::not_factorized)?;

        if rhs.len() != self.n {
            return Err(SolverError::dimension_mismatch(self.n, rhs.len()));
        }
        for (i, &val) in rhs.iter().enumerate() {
            if !val.is_finite() {
                return Err(SolverError::non_finite(i));
            }
        }

        ldlt.solve(rhs)
            .map_err(|e| SolverError::solve_failed(e.to_string()))
    }
}
