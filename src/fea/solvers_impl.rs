//! Solver implementations for the unified LinearSolver trait.
//!
//! Provides concrete implementations of the LinearSolver trait for
//! various solver backends.

use crate::fea::{SparseMatrix, SkylineLdlt, cg_solve, iccg_solve, CgResult, CgStatus, SOLVER_TOL};
use crate::fea::solver::{LinearSolver, LinearSolverWithConstraints, FactoredSolver, SolverCapabilities, SolverError};
use crate::fea::SparseMatrix;

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
    
    fn pivot_tolerance(&self) -> f64 {
        crate::fea::PIVOT_TOL_BASE * self.scale.max(1.0)
    }
}

impl crate::fea::solver::LinearSolver for DenseGaussianSolver {
    fn name(&self) -> &'static str {
        "dense"
    }
    
    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::dense()
    }
    
    fn factor(&mut self, matrix: &SparseMatrix) -> Result<(), SolverError> {
        let n = matrix.n;
        if n == 0 {
            return Err(SolverError::InvalidInput("Empty matrix".to_string()));
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
        let pivot_tol = crate::fea::PIVOT_TOL_BASE * self.scale.max(1.0);
        
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
            
            if max_val <= self.pivot_tolerance() {
                return Err(SolverError::singular(format!(
                    "Singular or near-singular matrix at column {}: max pivot = {:.2e}, tolerance = {:.2e}",
                    k, max_val, self.pivot_tolerance()
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
                    "Zero or invalid diagonal at row {}: {:.2e}", i, pivot_val
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
    
    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::dense()
    }
    
    fn name(&self) -> &'static str {
        "dense"
    }
}

/// Wrapper for SkylineLdlt
pub struct SkylineLdltSolver {
    inner: Option<SkylineLdlt>,
}

impl SkylineLdltSolver {
    pub fn new() -> Self {
        Self { inner: None }
    }
}

impl crate::fea::solver::LinearSolver for SkylineLdltSolver {
    fn name(&self) -> &'static str {
        "skyline_ldlt"
    }
    
    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::skyline_ldlt()
    }
    
    fn factor(&mut self, matrix: &SparseMatrix) -> Result<(), SolverError> {
        let ldlt = SkylineLdlt::factor(matrix)
            .map_err(|e| SolverError::factorization_failed(e.to_string()))?;
        self.inner = Some(ldlt);
        Ok(())
    }
    
    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolverError> {
        let ldlt = self.inner.as_ref()
            .ok_or_else(|| SolverError::solve_failed("Not factorized".to_string()))?;
        
        let n = ldlt.n();
        if rhs.len() != n {
            return Err(SolverError::dimension_mismatch(n, rhs.len()));
        }
        
        // Check for NaN/Inf
        for (i, &val) in rhs.iter().enumerate() {
            if !val.is_finite() {
                return Err(SolverError::non_finite(i));
            }
        }
        
        ldlt.solve(rhs)
            .map_err(|e| SolverError::solve_failed(e.to_string()))
    }
    
    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::skyline_ldlt()
    }
    
    fn name(&self) -> &'static str {
        "skyline_ldlt"
    }
}

impl crate::fea::solver::LinearSolverWithConstraints for SkylineLdltSolver {
    fn solve_lagrange(
        &self,
        c: &[f64],
        f: &[f64],
    ) -> Result<(Vec<f64>, f64), SolverError> {
        let ldlt = self.inner.as_ref()
            .ok_or_else(|| SolverError::solve_failed("Not factorized".to_string()))?;
        
        let n = ldlt.n();
        if c.len() != n || f.len() != n {
            return Err(SolverError::dimension_mismatch(n, c.len().max(f.len())));
        }
        
        let (u, lambda) = ldlt.solve_lagrange(c, f)
            .map_err(|e| SolverError::solve_failed(e.to_string()))?;
        
        Ok((u, lambda))
    }
}

/// Wrapper for SparseLU
pub struct SparseLuSolver {
    inner: Option<crate::fea::solvers::SparseLu>,
}

impl SparseLuSolver {
    pub fn new() -> Self {
        Self { inner: None }
    }
}

impl crate::fea::solver::LinearSolver for SparseLuSolver {
    fn name(&self) -> &'static str {
        "sparse_lu"
    }
    
    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::sparse_lu()
    }
    
    fn factor(&mut self, matrix: &SparseMatrix) -> Result<(), SolverError> {
        let lu = crate::fea::solvers::SparseLu::factor(matrix)
            .map_err(|e| SolverError::factorization_failed(e))?;
        self.inner = Some(lu);
        Ok(())
    }
    
    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolverError> {
        let lu = self.inner.as_ref()
            .ok_or_else(|| SolverError::solve_failed("Not factorized".to_string()))?;
        
        let n = lu.n();
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
    
    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::sparse_lu()
    }
    
    fn name(&self) -> &'static str {
        "sparse_lu"
    }
}

/// CG solver
pub struct CgSolver {
    matrix: Option<SparseMatrix>,
    max_iter: usize,
    tol: f64,
}

impl CgSolver {
    pub fn new() -> Self {
        Self {
            matrix: None,
            max_iter: 1000,
            tol: 1e-12,
        }
    }
    
    pub fn with_params(max_iter: usize, tol: f64) -> Self {
        Self {
            matrix: None,
            max_iter,
            tol,
        }
    }
}

impl crate::fea::solver::LinearSolver for CgSolver {
    fn name(&self) -> &'static str {
        "cg"
    }
    
    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::cg()
    }
    
    fn factor(&mut self, matrix: &SparseMatrix) -> Result<(), SolverError> {
        if !matrix.is_symmetric(1e-12).unwrap_or(false) {
            return Err(SolverError::Unsupported("CG requires symmetric matrix".to_string()));
        }
        self.matrix = Some(matrix.clone());
        Ok(())
    }
    
    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolverError> {
        let matrix = self.matrix.as_ref()
            .ok_or_else(|| SolverError::solve_failed("Not factorized".to_string()))?;
        
        let n = matrix.n;
        if rhs.len() != n {
            return Err(SolverError::dimension_mismatch(n, rhs.len()));
        }
        
        // Check for NaN/Inf
        for (i, &val) in rhs.iter().enumerate() {
            if !val.is_finite() {
                return Err(SolverError::non_finite(i));
            }
        }
        
        let max_iter = (n * 10).clamp(1000, 100000);
        let result = cg_solve(matrix, rhs, max_iter, self.tol);
        
        match result.status {
            crate::fea::CgStatus::Converged => Ok(result.x),
            crate::fea::CgStatus::NotPositiveDefinite => 
                Err(SolverError::singular("Matrix not positive definite".to_string())),
            crate::fea::CgStatus::Breakdown => 
                Err(SolverError::solve_failed("CG breakdown".to_string())),
            crate::fea::CgStatus::InvalidInput => 
                Err(SolverError::InvalidInput("Invalid input to CG".to_string())),
            crate::fea::CgStatus::MaxIterations => 
                Err(SolverError::convergence_failed("CG max iterations reached".to_string())),
        }
    }
    
    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::cg()
    }
    
    fn name(&self) -> &'static str {
        "cg"
    }
}

/// ICCG solver
pub struct IccgSolver {
    matrix: Option<SparseMatrix>,
    max_iter: usize,
    tol: f64,
}

impl IccgSolver {
    pub fn new() -> Self {
        Self {
            matrix: None,
            max_iter: 1000,
            tol: 1e-12,
        }
    }
}

impl crate::fea::solver::LinearSolver for IccgSolver {
    fn name(&self) -> &'static str {
        "iccg"
    }
    
    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::iccg()
    }
    
    fn factor(&mut self, matrix: &SparseMatrix) -> Result<(), SolverError> {
        if !matrix.is_symmetric(1e-12).unwrap_or(false) {
            return Err(SolverError::Unsupported("ICCG requires symmetric matrix".to_string()));
        }
        self.matrix = Some(matrix.clone());
        Ok(())
    }
    
    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolverError> {
        let matrix = self.matrix.as_ref()
            .ok_or_else(|| SolverError::solve_failed("Not factorized".to_string()))?;
        
        let n = matrix.n;
        if rhs.len() != n {
            return Err(SolverError::dimension_mismatch(n, rhs.len()));
        }
        
        // Check for NaN/Inf
        for (i, &val) in rhs.iter().enumerate() {
            if !val.is_finite() {
                return Err(SolverError::non_finite(i));
            }
        }
        
        let max_iter = (n * 4).clamp(1000, 60000);
        let result = iccg_solve(matrix, rhs, max_iter, self.tol);
        
        match result.status {
            crate::fea::CgStatus::Converged => Ok(result.x),
            crate::fea::CgStatus::NotPositiveDefinite => 
                Err(SolverError::singular("Matrix not positive definite".to_string())),
            crate::fea::CgStatus::Breakdown => 
                Err(SolverError::solve_failed("ICCG breakdown".to_string())),
            crate::fea::CgStatus::InvalidInput => 
                Err(SolverError::InvalidInput("Invalid input to ICCG".to_string())),
            crate::fea::CgStatus::MaxIterations => 
                Err(SolverError::convergence_failed("ICCG max iterations reached".to_string())),
        }
    }
    
    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::iccg()
    }
    
    fn name(&self) -> &'static str {
        "iccg"
    }
}

#[cfg(feature = "pardiso")]
mod pardiso_wrapper {
    use super::*;
    use crate::fea::solvers::pardiso::PardisoSolver;
    use crate::fea::SparseMatrix;
    use std::sync::Mutex;
    
    pub struct PardisoSolverWrapper {
        inner: Mutex<Option<crate::fea::solvers::pardiso::PardisoSolver>>,
    }
    
    impl PardisoSolverWrapper {
        pub fn new() -> Self {
            Self {
                inner: Mutex::new(None),
            }
        }
    }
    
    impl crate::fea::solver::LinearSolver for PardisoSolverWrapper {
        fn name(&self) -> &'static str {
            "pardiso"
        }
        
        fn capabilities(&self) -> SolverCapabilities {
            SolverCapabilities::pardiso()
        }
        
        fn factor(&mut self, matrix: &SparseMatrix) -> Result<(), SolverError> {
            // PARDISO requires constraint vector at construction
            // For now, return not implemented for standard factorization
            Err(SolverError::Unsupported("PARDISO requires constraint vector at construction. Use DirectLagrangeSolver with PARDISO kernel.".to_string()))
        }
        
        fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolverError> {
            Err(SolverError::Unsupported("PARDISO requires constraint vector. Use DirectLagrangeSolver with PARDISO kernel.".to_string()))
        }
        
        fn capabilities(&self) -> SolverCapabilities {
            SolverCapabilities::pardiso()
        }
        
        fn name(&self) -> &'static str {
            "pardiso"
        }
    }
    
    impl crate::fea::solver::LinearSolverWithConstraints for PardisoSolverWrapper {
        fn solve_lagrange(
            &self,
            c: &[f64],
            f: &[f64],
        ) -> Result<(Vec<f64>, f64), SolverError> {
            // This is handled by DirectLagrangeSolver with PARDISO kernel
            Err(SolverError::Unsupported("Use DirectLagrangeSolver with PARDISO kernel".to_string()))
        }
    }
}