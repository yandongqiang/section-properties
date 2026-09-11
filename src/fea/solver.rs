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

/// How a linear solver is chosen for a system.
///
/// # Canonical solver names
///
/// `Named` carries one of the registered backend names below. Unknown names are
/// rejected with a structured [`SolverError`] — they never fall back.
///
/// | name             | symmetry        | definiteness        | intended size |
/// |------------------|-----------------|---------------------|---------------|
/// | `dense`          | any (general)   | any                 | small         |
/// | `skyline_ldlt`   | symmetric only  | SPD (verified)      | medium/large  |
/// | `sparse_lu`      | any (general)   | any                 | medium/large  |
/// | `cg`             | symmetric only  | SPD (assumed)       | large         |
/// | `iccg`           | symmetric only  | SPD (assumed)       | large         |
/// | `pardiso`        | any (general)   | any                 | very large (feature) |
///
/// # Semantics
///
/// - [`Auto`](Self::Auto): the registry analyses the matrix and picks a backend.
///   It may substitute a different backend than any the caller had in mind.
/// - [`Named`](Self::Named): the request is **authoritative**. The registry
///   validates capabilities and returns an error if the backend cannot legally
///   handle the matrix. It never substitutes another backend.
///
/// `skyline_ldlt`, `cg` and `iccg` require positive definiteness but perform
/// their own definiteness check during [`LinearSolver::factor`] (skyline rejects
/// a non-positive pivot; CG/ICCG report failure). They are therefore safe to
/// select on a symmetric matrix, but are **never** chosen by `Auto` for a matrix
/// whose SPD property is not established.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum SolverSelection {
    /// Registry-based automatic selection (default).
    #[default]
    Auto,
    /// Explicit backend request (canonical name, see type docs).
    Named(String),
}

impl SolverSelection {
    /// Explicit selection by canonical name.
    pub fn named(name: impl Into<String>) -> Self {
        Self::Named(name.into())
    }
    /// Select the dense Gaussian backend.
    pub fn dense() -> Self {
        Self::Named("dense".to_string())
    }
    /// Select the skyline LDL^T backend (symmetric SPD).
    pub fn skyline_ldlt() -> Self {
        Self::Named("skyline_ldlt".to_string())
    }
    /// Select the sparse LU backend.
    pub fn sparse_lu() -> Self {
        Self::Named("sparse_lu".to_string())
    }
    /// Select the conjugate-gradient backend (symmetric SPD).
    pub fn cg() -> Self {
        Self::Named("cg".to_string())
    }
    /// Select the incomplete-Cholesky CG backend (symmetric SPD).
    pub fn iccg() -> Self {
        Self::Named("iccg".to_string())
    }
    /// Select the PARDISO backend (feature-gated).
    pub fn pardiso() -> Self {
        Self::Named("pardiso".to_string())
    }
    /// Whether this is automatic selection.
    pub fn is_auto(&self) -> bool {
        matches!(self, Self::Auto)
    }
    /// The explicitly requested name, or `None` for [`SolverSelection::Auto`].
    pub fn requested_name(&self) -> Option<&str> {
        match self {
            Self::Auto => None,
            Self::Named(n) => Some(n.as_str()),
        }
    }
}

/// Why a particular solver was selected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SelectionReason {
    /// The caller requested this backend explicitly.
    ExplicitlyRequested,
    /// Small system: dense is used for any matrix class.
    SmallSystem,
    /// Symmetric matrix whose diagonal is positive (necessary condition for
    /// SPD): skyline is chosen; it verifies definiteness during factorization.
    SymmetricPositiveDiagonal,
    /// General (possibly non-symmetric) system: sparse LU.
    GeneralSystem,
    /// Very large system with PARDISO available.
    LargeSystem,
}

/// The outcome of a solver-selection query. No solver is constructed and no
/// factorization is performed, so this is cheap and side-effect free.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SolverSelectionInfo {
    /// Canonical name of the chosen backend.
    pub solver_name: String,
    /// Why it was chosen.
    pub reason: SelectionReason,
}

/// Validate that a solver with `caps` may legally handle `matrix`.
///
/// This is a **class** check (symmetry / generality). It deliberately does not
/// attempt to prove positive definiteness: a symmetric indefinite matrix is a
/// legal input to every backend that claims `general` support, and backends that
/// require SPD (`skyline_ldlt`, `cg`, `iccg`) verify that themselves during
/// [`LinearSolver::factor`], returning an error rather than a wrong answer.
fn validate_solver_for_matrix(
    name: &str,
    caps: &SolverCapabilities,
    matrix: &SparseMatrix,
) -> Result<(), SolverError> {
    if matrix.n == 0 {
        return Err(SolverError::invalid_input("Empty matrix"));
    }
    let symmetric = matrix.is_symmetric(1e-12);
    if !symmetric && !caps.general {
        return Err(SolverError::unsupported(format!(
            "solver '{}' requires a symmetric matrix, but the matrix is not symmetric; \
             use 'dense' or 'sparse_lu'",
            name
        )));
    }
    Ok(())
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

    /// Registered solver names, sorted (deterministic).
    pub fn list_sorted(&self) -> Vec<String> {
        let mut v = self.list();
        v.sort();
        v
    }

    pub fn create(&self, name: &str) -> Option<Box<dyn LinearSolver>> {
        self.backends.get(name).map(|f| f.create())
    }

    /// Validate that the named backend can legally handle `matrix`, returning
    /// its capabilities on success. Performs **no** factorization.
    ///
    /// # Errors
    ///
    /// - [`SolverError::Unsupported`] if the name is unknown or the matrix class
    ///   is incompatible with the backend.
    /// - [`SolverError::InvalidInput`] for an empty matrix.
    pub fn validate_selection(
        &self,
        name: &str,
        matrix: &SparseMatrix,
    ) -> Result<SolverCapabilities, SolverError> {
        let factory = self
            .get(name)
            .ok_or_else(|| SolverError::unsupported(format!("unknown solver '{}'", name)))?;
        let caps = factory.capabilities();
        validate_solver_for_matrix(name, &caps, matrix)?;
        Ok(caps)
    }

    /// Decide which backend would be used, **without** constructing or
    /// factorizing anything. Answers "which solver, and why?".
    ///
    /// # Errors
    ///
    /// Returns an error for an unknown explicit name, an incompatible matrix
    /// class, an empty matrix, or (for `Auto`) when no suitable backend exists.
    pub fn select(
        &self,
        matrix: &SparseMatrix,
        selection: &SolverSelection,
    ) -> Result<SolverSelectionInfo, SolverError> {
        match selection {
            SolverSelection::Named(name) => {
                self.validate_selection(name, matrix)?;
                Ok(SolverSelectionInfo {
                    solver_name: name.clone(),
                    reason: SelectionReason::ExplicitlyRequested,
                })
            }
            SolverSelection::Auto => self.auto_select_info(matrix),
        }
    }

    /// The auto-selection policy, exposed as a decision (name + reason).
    ///
    /// Policy (conservative; never selects a solver that cannot handle the
    /// detected matrix class):
    ///
    /// 1. `n <= 500` → `dense` (general, any class).
    /// 2. symmetric with a positive diagonal (necessary condition for SPD) →
    ///    `skyline_ldlt` (it verifies definiteness at factorization).
    /// 3. `pardiso` if registered/available (very large).
    /// 4. otherwise → `sparse_lu` (general; handles symmetric or not).
    ///
    /// `cg`/`iccg` are never auto-selected: their SPD requirement cannot be
    /// established reliably from the matrix alone.
    pub fn auto_select_info(
        &self,
        matrix: &SparseMatrix,
    ) -> Result<SolverSelectionInfo, SolverError> {
        if matrix.n == 0 {
            return Err(SolverError::invalid_input("Empty matrix"));
        }

        if matrix.n <= 500 && self.get("dense").is_some() {
            return Ok(SolverSelectionInfo {
                solver_name: "dense".to_string(),
                reason: SelectionReason::SmallSystem,
            });
        }

        if matrix.is_symmetric(1e-12)
            && has_positive_diagonal(matrix)
            && self.get("skyline_ldlt").is_some()
        {
            return Ok(SolverSelectionInfo {
                solver_name: "skyline_ldlt".to_string(),
                reason: SelectionReason::SymmetricPositiveDiagonal,
            });
        }

        #[cfg(feature = "pardiso")]
        if self.get("pardiso").is_some() {
            return Ok(SolverSelectionInfo {
                solver_name: "pardiso".to_string(),
                reason: SelectionReason::LargeSystem,
            });
        }

        if self.get("sparse_lu").is_some() {
            return Ok(SolverSelectionInfo {
                solver_name: "sparse_lu".to_string(),
                reason: SelectionReason::GeneralSystem,
            });
        }

        Err(SolverError::unsupported(
            "no suitable solver is registered for this matrix",
        ))
    }

    /// Construct a solver for `selection`, after capability validation.
    ///
    /// Explicit selections are authoritative: if the requested backend cannot
    /// legally handle the matrix (or is unknown / uncreatable), an error is
    /// returned and **no** other backend is substituted. `Auto` may pick any
    /// backend that can handle the matrix class.
    ///
    /// The solver is returned **unfactored** — the caller calls
    /// [`LinearSolver::factor`] then [`LinearSolver::solve`].
    pub fn create_selected(
        &self,
        matrix: &SparseMatrix,
        selection: &SolverSelection,
    ) -> Result<Box<dyn LinearSolver>, SolverError> {
        let info = self.select(matrix, selection)?;
        self.create(&info.solver_name).ok_or_else(|| {
            SolverError::backend_unavailable(format!(
                "solver '{}' was selected but cannot be constructed",
                info.solver_name
            ))
        })
    }

    /// Backwards-compatible automatic selection: returns the chosen solver, or
    /// `None` if none is registered. Prefer [`Self::select`] /
    /// [`Self::create_selected`] for new code.
    pub fn auto_select(&self, matrix: &SparseMatrix) -> Option<Box<dyn LinearSolver>> {
        let info = self.auto_select_info(matrix).ok()?;
        self.create(&info.solver_name)
    }
}

/// True if every diagonal entry of `matrix` is strictly positive.
///
/// This is a *necessary* (not sufficient) condition for SPD and is used only as
/// a conservative screen for auto-selection. It never asserts definiteness.
fn has_positive_diagonal(matrix: &SparseMatrix) -> bool {
    let n = matrix.n;
    let mut diag = vec![0.0f64; n];
    if matrix.compressed {
        for i in 0..n {
            for k in matrix.row_ptr[i]..matrix.row_ptr[i + 1] {
                if matrix.csr_cols[k] == i {
                    diag[i] = matrix.csr_vals[k];
                }
            }
        }
    } else {
        for k in 0..matrix.rows.len() {
            if matrix.rows[k] == matrix.cols[k] {
                diag[matrix.rows[k]] = matrix.vals[k];
            }
        }
    }
    diag.iter().all(|&d| d > 0.0)
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
