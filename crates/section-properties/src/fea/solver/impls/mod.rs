//! Implementation module for LinearSolver trait implementations.
pub mod cg;
pub mod dense_gaussian;
pub mod iccg;
pub mod skyline_ldlt;
pub mod sparse_lu;

#[cfg(feature = "pardiso")]
pub mod pardiso_wrapper;
