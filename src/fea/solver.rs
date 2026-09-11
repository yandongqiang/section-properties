//! Unified linear solver abstraction for the FEM framework.
//!
//! Provides a common interface for different linear solvers (direct, iterative, external)
//! enabling a plugin-based architecture for the FEM framework.

use crate::fea::SparseMatrix;
use crate::fea::solver::impls::cg::CgSolver;
use crate::fea::solver::impls::dense_gaussian::DenseGaussianSolver;
use crate::fea::solver::impls::iccg::IccgSolver;
use crate::fea::solver::impls::skyline_ldlt::SkylineLdltSolver;
use crate::fea::solver::impls::sparse_lu::SparseLuSolver;

#[cfg(feature = "pardiso")]
use crate::fea::solver::impls::pardiso_wrapper::PardisoSolverWrapper;

pub mod impls;

/// Capabilities of a linear solver
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SolverCapabilities {
    /// Supports symmetric matrices
    pub symmetric: bool,
    /// Supports symmetric positive definite matrices
    pub spd: bool,
    /// Supports indefinite symmetric matrices
    pub symmetric_indefinite: bool,
    /// Supports general non-symmetric matrices
    pub general: bool,
    /// Supports multiple right-hand sides efficiently
    pub multiple_rhs: bool,
    /// Supports complex numbers
    pub complex: bool,
    /// Maximum problem size (None = unlimited)
    pub max_size: Option<usize>,
}

impl SolverCapabilities {
    /// Capabilities for dense Gaussian elimination
    pub fn dense() -> Self {
        Self {
            symmetric: true,
            spd: true,
            symmetric_indefinite: true,
            general: true,
            multiple_rhs: true,
            complex: false,
            max_size: Some(500),
        }
    }

    /// Capabilities for Skyline LDL^T
    pub fn skyline_ldlt() -> Self {
        Self {
            symmetric: true,
            spd: true,
            symmetric_indefinite: false,
            general: false,
            multiple_rhs: true,
            complex: false,
            max_size: Some(10000),
        }
    }

    /// Capabilities for SparseLU
    pub fn sparse_lu() -> Self {
        Self {
            symmetric: true,
            spd: true,
            symmetric_indefinite: true,
            general: true,
            multiple_rhs: true,
            complex: false,
            max_size: Some(3000),
        }
    }

    /// Capabilities for PARDISO
    #[cfg(feature = "pardiso")]
    pub fn pardiso() -> Self {
        Self {
            symmetric: true,
            spd: true,
            symmetric_indefinite: true,
            general: true,
            multiple_rhs: true,
            complex: false,
            max_size: None,
        }
    }

    /// Capabilities for CG
    pub fn cg() -> Self {
        Self {
            symmetric: true,
            spd: true,
            symmetric_indefinite: false,
            general: false,
            multiple_rhs: false,
            complex: false,
            max_size: None,
        }
    }

    /// Capabilities for ICCG
    pub fn iccg() -> Self {
        Self {
            symmetric: true,
            spd: true,
            symmetric_indefinite: false,
            general: false,
            multiple_rhs: false,
            complex: false,
            max_size: None,
        }
    }
}

/// Unified error type for all linear solvers
#[derive(Debug, Clone, thiserror::Error)]
pub enum SolverError {
    #[error("invalid input: {0}")]
    InvalidInput(String),

    #[error("non-finite input at index {index}")]
    NonFiniteInput { index: usize },

    #[error("dimension mismatch: expected {expected}, got {actual}")]
    DimensionMismatch { expected: usize, actual: usize },

    #[error("singular matrix: {0}")]
    SingularMatrix(String),

    #[error("near-singular matrix: {0}")]
    NearSingularMatrix(String),

    #[error("factorization failed: {0}")]
    FactorizationFailed(String),

    #[error("solve failed: {0}")]
    SolveFailed(String),

    #[error("backend unavailable: {0}")]
    BackendUnavailable(String),

    #[error("backend error: {0}")]
    BackendError(String),

    #[error("unsupported operation: {0}")]
    Unsupported(String),

    #[error("convergence failed: {0}")]
    ConvergenceFailed(String),

    #[error("backend not available: {0}")]
    BackendNotAvailable(String),

    #[error("not implemented: {0}")]
    NotImplemented(String),

    #[error("matrix is not symmetric")]
    NotSymmetric,

    #[error("solver not factorized")]
    NotFactorized,

    #[error("solver did not converge")]
    NotConverged,
}

impl SolverError {
    pub fn invalid_input(msg: impl Into<String>) -> Self {
        Self::InvalidInput(msg.into())
    }

    pub fn non_finite(index: usize) -> Self {
        Self::NonFiniteInput { index }
    }

    pub fn dimension_mismatch(expected: usize, actual: usize) -> Self {
        Self::DimensionMismatch { expected, actual }
    }

    pub fn singular(msg: impl Into<String>) -> Self {
        Self::SingularMatrix(msg.into())
    }

    pub fn near_singular(msg: impl Into<String>) -> Self {
        Self::NearSingularMatrix(msg.into())
    }

    pub fn factorization_failed(msg: impl Into<String>) -> Self {
        Self::FactorizationFailed(msg.into())
    }

    pub fn solve_failed(msg: impl Into<String>) -> Self {
        Self::SolveFailed(msg.into())
    }

    pub fn backend_unavailable(msg: impl Into<String>) -> Self {
        Self::BackendUnavailable(msg.into())
    }

    pub fn backend_error(msg: impl Into<String>) -> Self {
        Self::BackendError(msg.into())
    }

    pub fn unsupported(msg: impl Into<String>) -> Self {
        Self::Unsupported(msg.into())
    }

    pub fn convergence_failed(msg: impl Into<String>) -> Self {
        Self::ConvergenceFailed(msg.into())
    }

    pub fn backend_not_available(msg: impl Into<String>) -> Self {
        Self::BackendNotAvailable(msg.into())
    }

    pub fn not_implemented(msg: impl Into<String>) -> Self {
        Self::NotImplemented(msg.into())
    }

    pub fn not_symmetric() -> Self {
        Self::NotSymmetric
    }

    pub fn not_factorized() -> Self {
        Self::NotFactorized
    }

    pub fn not_converged() -> Self {
        Self::NotConverged
    }
}

/// Trait for a factored solver that can solve multiple RHS
pub trait FactoredSolver: Send + Sync {
    /// Solve for a single RHS
    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolverError>;

    /// Solve for multiple RHS efficiently
    fn solve_many(&self, rhs: &[Vec<f64>]) -> Result<Vec<Vec<f64>>, SolverError> {
        rhs.iter().map(|rhs| self.solve(rhs)).collect()
    }

    /// Get solver capabilities
    fn capabilities(&self) -> SolverCapabilities;

    /// Get solver name
    fn name(&self) -> &'static str;
}

/// Trait for linear solvers that can factorize and solve
pub trait LinearSolver: Send + Sync {
    /// Get solver name
    fn name(&self) -> &'static str;

    /// Get solver capabilities
    fn capabilities(&self) -> SolverCapabilities;

    /// Factorize the matrix
    fn factor(&mut self, matrix: &SparseMatrix) -> Result<(), SolverError>;

    /// Solve for a single RHS (requires prior factorization)
    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, SolverError>;

    /// Solve for multiple RHS (requires prior factorization)
    fn solve_many(&self, rhs: &[Vec<f64>]) -> Result<Vec<Vec<f64>>, SolverError> {
        rhs.iter().map(|rhs| self.solve(rhs)).collect()
    }
}

/// Combined trait for solvers that support both factor+solve and Lagrange systems
pub trait LinearSolverWithConstraints: LinearSolver {
    /// Solve a Lagrange system [K C^T; C 0] [u; lambda] = [f; 0]
    fn solve_lagrange(&self, c: &[f64], f: &[f64]) -> Result<(Vec<f64>, f64), SolverError>;
}

/// Solver backend type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SolverBackend {
    Dense,
    SkylineLdlt,
    SparseLu,
    #[cfg(feature = "pardiso")]
    Pardiso,
    Cg,
    Iccg,
}

/// Registry for solver backends
pub struct SolverRegistry {
    backends: std::collections::HashMap<String, Box<dyn LinearSolverFactory>>,
}

pub trait LinearSolverFactory: Send + Sync {
    fn create(&self) -> Box<dyn LinearSolver>;
    fn name(&self) -> &'static str;
    fn capabilities(&self) -> SolverCapabilities;
}

impl SolverRegistry {
    pub fn new() -> Self {
        Self {
            backends: std::collections::HashMap::new(),
        }
    }

    pub fn register(&mut self, factory: Box<dyn LinearSolverFactory>) {
        self.backends.insert(factory.name().to_string(), factory);
    }

    pub fn get(&self, name: &str) -> Option<&dyn LinearSolverFactory> {
        self.backends.get(name).map(|b| b.as_ref())
    }

    pub fn list(&self) -> Vec<String> {
        self.backends.keys().cloned().collect()
    }

    pub fn create(&self, name: &str) -> Option<Box<dyn LinearSolver>> {
        self.backends.get(name).map(|f| f.create())
    }

    /// Auto-select best solver based on matrix properties
    ///
    /// Selection logic:
    /// - Small matrices (n <= 500): Dense Gaussian (handles all types)
    /// - Symmetric matrices: Skyline LDL^T (requires SPD, verified during factor)
    /// - Non-symmetric matrices (n <= 3000): SparseLU
    /// - Large non-symmetric: Falls back to Skyline if symmetric, else SparseLU
    ///
    /// Note: CG/ICCG are NOT auto-selected because they require SPD.
    /// Users must explicitly choose CG/ICCG when they know the matrix is SPD.
    pub fn auto_select(&self, matrix: &SparseMatrix) -> Option<Box<dyn LinearSolver>> {
        let n = matrix.n;
        let nnz = if matrix.compressed {
            matrix.csr_vals.len()
        } else {
            matrix.vals.len()
        };
        let _density = nnz as f64 / (n * n) as f64;

        // Small matrices: dense handles everything
        if n <= 500 {
            return self.create("dense");
        }

        let is_symmetric = matrix.is_symmetric(1e-12);

        if is_symmetric {
            // Symmetric: Skyline LDL^T (requires SPD)
            // User must verify SPD if using CG/ICCG explicitly
            if n <= 10000 {
                self.create("skyline_ldlt")
            } else {
                // Large symmetric: still prefer direct solver over CG
                // CG requires SPD which we cannot verify reliably here
                self.create("skyline_ldlt")
            }
        } else if n <= 3000 {
            // Non-symmetric small/medium: SparseLU
            self.create("sparse_lu")
        } else {
            // Large non-symmetric: no good built-in option
            // Fall back to SparseLU (may be slow) or return None
            #[cfg(feature = "pardiso")]
            return self.create("pardiso");
            #[cfg(not(feature = "pardiso"))]
            self.create("sparse_lu")
        }
    }
}

impl Default for SolverRegistry {
    fn default() -> Self {
        let mut registry = Self::new();

        // Register built-in backends
        registry.register(Box::new(DenseSolverFactory));
        registry.register(Box::new(SkylineLdltFactory));
        registry.register(Box::new(SparseLuFactory));
        registry.register(Box::new(CgSolverFactory));
        registry.register(Box::new(IccgSolverFactory));

        // PARDISO is NOT registered by default - it's a stub for the unified interface.
        // The real PARDISO implementation in solvers.rs is for augmented Lagrange systems.
        // Users must explicitly create it if needed.

        registry
    }
}

/// Factory for Dense Gaussian elimination
struct DenseSolverFactory;
impl LinearSolverFactory for DenseSolverFactory {
    fn create(&self) -> Box<dyn LinearSolver> {
        Box::new(DenseGaussianSolver::new())
    }
    fn name(&self) -> &'static str {
        "dense"
    }
    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::dense()
    }
}

/// Factory for Skyline LDL^T
struct SkylineLdltFactory;
impl LinearSolverFactory for SkylineLdltFactory {
    fn create(&self) -> Box<dyn LinearSolver> {
        Box::new(SkylineLdltSolver::new())
    }
    fn name(&self) -> &'static str {
        "skyline_ldlt"
    }
    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::skyline_ldlt()
    }
}

/// Factory for SparseLU
struct SparseLuFactory;
impl LinearSolverFactory for SparseLuFactory {
    fn create(&self) -> Box<dyn LinearSolver> {
        Box::new(SparseLuSolver::new())
    }
    fn name(&self) -> &'static str {
        "sparse_lu"
    }
    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::sparse_lu()
    }
}

/// Factory for CG
struct CgSolverFactory;
impl LinearSolverFactory for CgSolverFactory {
    fn create(&self) -> Box<dyn LinearSolver> {
        Box::new(CgSolver::new())
    }
    fn name(&self) -> &'static str {
        "cg"
    }
    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::cg()
    }
}

/// Factory for ICCG
struct IccgSolverFactory;
impl LinearSolverFactory for IccgSolverFactory {
    fn create(&self) -> Box<dyn LinearSolver> {
        Box::new(IccgSolver::new())
    }
    fn name(&self) -> &'static str {
        "iccg"
    }
    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::iccg()
    }
}

#[cfg(feature = "pardiso")]
struct PardisoSolverFactory;
#[cfg(feature = "pardiso")]
impl LinearSolverFactory for PardisoSolverFactory {
    fn create(&self) -> Box<dyn LinearSolver> {
        Box::new(PardisoSolverWrapper::new())
    }
    fn name(&self) -> &'static str {
        "pardiso"
    }
    fn capabilities(&self) -> SolverCapabilities {
        SolverCapabilities::pardiso()
    }
}

// Re-export for public API
pub use self::LinearSolver as Solver;
pub use self::SolverCapabilities as Capabilities;
pub use self::SolverError as Error;
pub use self::SolverRegistry as Registry;
