//! # structural-analysis
//!
//! 2D structural analysis: beam and frame finite element analysis,
//! solver selection, and mechanism diagnostics.
//!
//! Depends on [`section-properties`](section_properties) for cross-section
//! properties, materials, and numerical infrastructure.
//!
//! # Quick start — frame analysis
//!
//! ```rust
//! use structural_analysis::{FrameModel, BeamSection, Dof};
//! use section_properties::Material;
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! let mut frame = FrameModel::new();
//! let base = frame.add_node(0.0, 0.0)?;
//! let tip = frame.add_node(2.0, 0.0)?;
//! frame.add_member(base, tip, Material::new(200e9, 0.3, 7850.0, "Steel"),
//!                   BeamSection::new(5e-3, 2e-5))?;
//! frame.fix(base)?;
//! frame.nodal_load(tip, 0.0, -1.0e4)?;
//!
//! let result = frame.solve()?;
//! let uy = result.displacement(tip, Dof::Uy)?;
//! assert!((uy + 1.0e4 * 2.0f64.powi(3) / (3.0 * 200e9 * 2e-5)).abs() < 1e-12);
//! assert!(result.equilibrium().is_balanced());
//! # Ok(())
//! # }
//! ```

pub mod beam_fem;
pub mod frame;
pub mod mechanism;

pub use crate::beam_fem::{
    BeamAnalysis, BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver, Dof, FemError,
};
pub use crate::frame::{
    EquilibriumReport, FrameAnalysisResult, FrameModel, FrameSolver, MemberHandle, NodeHandle,
    StructuralDiagnostic,
};
