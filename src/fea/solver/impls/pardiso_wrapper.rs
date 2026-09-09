//! PARDISO wrapper solver (stub - actual implementation in solvers.rs for Lagrange systems)
use crate::fea::SparseMatrix;
use crate::fea::solver::{LinearSolver, SolverCapabilities, SolverError};

/// PARDISO wrapper solver
///
/// Note: The full PARDISO integration is available via the `solvers` module
/// for augmented Lagrangian systems. This wrapper is a placeholder for the
/// unified LinearSolver interface.
pub struct PardisoSolverWrapper {
    n: usize,
}

impl PardisoSolverWrapper {
    pub fn new() -> Self {
        Self { n: 0 }
    }
}

impl LinearSolver for PardisoSolverWrapper {
    fn name(&self) -> &'static str {
        "pardiso"
    }
    
    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities {
            symmetric: true,
            spd: true,
            symmetric_indefinite: true,
            general: true,
            multiple_rhs: true,
            complex: false,
            max_size: None,
        }
    }
    
    fn factor(&mut self, matrix: &SparseMatrix) -> Result<(), SolverError> {
        let n = matrix.n;
        if n == 0 {
            return Err(SolverError::invalid_input("Empty matrix"));
        }
        
        #[cfg(feature = "pardiso")]
        {
            // PARDISO is available but requires specialized setup for
            // augmented Lagrangian systems. Use the solvers module directly.
            return Err(SolverError::unsupported(
                "PARDISO general solver not yet implemented in unified interface. \
                 Use crate::fea::solvers::PardisoSolver for Lagrange systems."
            ));
        }
        
        #[cfg(not(feature = "pardiso"))]
        {
            Err(SolverError::backend_not_available("PARDISO feature not enabled"))
        }
    }
    
    fn solve(&self, _rhs: &[f64]) -> Result<Vec<f64>, SolverError> {
        Err(SolverError::not_factorized())
    }
}