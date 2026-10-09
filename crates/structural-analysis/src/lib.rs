//! # structural-analysis
//!
//! 2D structural analysis: beam and frame finite element analysis,
//! solver selection, and mechanism diagnostics.
//!
//! ## Purpose
//!
//! Provides linear static analysis of 2D Euler–Bernoulli beam and frame
//! structures. Built on top of [`section-properties`](section_properties) for
//! cross-section properties, materials, and numerical infrastructure (sparse
//! matrix assembly, linear solvers).
//!
//! ## Current scope
//!
//! - **Elements**: 2D Euler–Bernoulli beam/frame element (3 DOF per node:
//!   `ux`, `uy`, `rz`) and 2D pin-jointed truss element (2 DOF per node:
//!   `ux`, `uy`)
//! - **Analysis**: linear static
//! - **Supports**: fixed, pinned, roller (x/y), inclined roller, spring,
//!   arbitrary DOF restraint
//! - **End releases**: hinge (rotation release) at either or both member ends
//! - **Loads**: nodal forces, nodal moments, uniform distributed loads,
//!   trapezoidal distributed loads, member point loads, applied moments
//! - **Load cases & combinations**: named load cases with linear combination
//!   factors
//! - **Results**: displacements, reactions, member end forces (local & global),
//!   section forces `N(x)`, `V(x)`, `M(x)`, force diagrams, multi-case envelopes
//! - **Diagnostics**: mechanism detection, rigid-body mode classification,
//!   ill-conditioning reporting, equilibrium verification (frame/beam/truss)
//! - **Solvers**: dense Gaussian, skyline LDLᵀ, sparse LU, CG, ICCG (via
//!   `section-properties`)
//!
//! **Not supported** (and not claimed): plate/shell/solid elements,
//! nonlinear analysis, dynamic analysis, buckling, design code
//! combinations.
//!
//! ## Architecture
//!
//! ```text
//! section-properties          ← cross-section properties, materials, solvers
//!         ↑
//!         │
//! structural-analysis         ← structural model, elements, loads, solve
//! ```
//!
//! `structural-analysis` depends on `section-properties` (one-way). The solver
//! infrastructure (`LinearSolver`, `SparseMatrix`, etc.) lives in
//! `section-properties::fea`; this crate provides the structural model and
//! delegates the linear algebra.
//!
//! ## Basic workflow
//!
//! ```text
//! create model → add nodes → add members → define supports
//!              → apply loads → solve → inspect results
//! ```
//!
//! Two API levels are available:
//!
//! - **`BeamModel` / `BeamSolver`** — lower-level API with raw node/element
//!   indices. See [`beam_fem`].
//! - **`FrameModel`** — higher-level façade with typed `NodeHandle` /
//!   `MemberHandle`, support vocabulary, load cases, and equilibrium
//!   reporting. See [`frame`].
//! - **`TrussModel`** — 2D truss analysis with pin-jointed elements (axial
//!   only, 2 DOF per node). See [`truss`].
//!
//! ## Quick start — frame analysis
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
//!
//! ## Conventions
//!
//! - **Global DOF**: `[ux, uy, rz]` per node; `rz` is counter-clockwise
//!   positive. Index mapping: `dof(node, d) = 3*node + d`.
//! - **Nodal loads**: in **global** coordinates.
//! - **Member loads** (distributed, point): in the member's **local**
//!   coordinate system (local x from `node_i` to `node_j`, local y
//!   transverse).
//! - **Applied moments**: in **global** coordinates (counter-clockwise
//!   positive).
//! - **Reactions**: `R = K_original · u - f_global` (global).
//! - **Member end forces**: `f_end = f_equiv - K_e · u_e` (element-on-node,
//!   local axes by default; global variant available).
//! - **Section forces**: `N` (tension positive), `V` (dM/dx), `M` (sagging
//!   positive). See [`beam_fem::SectionForces`].
//! - **Springs**: `reaction = -k · displacement`; stiffness added to the
//!   global diagonal, DOF remains free.
//! - **Inclined rollers**: `(nx, ny)` is the constrained direction
//!   (auto-normalized); the orthogonal direction is free.
//! - **End releases**: remove the moment transfer between member end and node;
//!   the node's rotational DOF is **not** removed from the global system.
//!
//! ## Limitations
//!
//! - Only 2D (planar) analysis.
//! - Only Euler–Bernoulli beam kinematics (no shear deformation).
//! - Parallel members between the same node pair are rejected.
//! - A single connected structural system is required.
//! - `publish = false` — this crate is workspace-only, not on crates.io.

pub mod beam_fem;
pub mod frame;
pub mod load;
pub mod mechanism;
pub mod postprocessing;
pub mod truss;
pub mod truss3d;

pub use crate::beam_fem::{
    BeamAnalysis, BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver, Dof, EndRelease,
    FemError,
};
pub use crate::frame::{
    EquilibriumReport, FrameAnalysisResult, FrameModel, FrameSolver, MemberHandle, NodeHandle,
    PreparedFrameAnalysis, StructuralDiagnostic,
};
pub use crate::load::{LoadCase, LoadCombination, LoadCombinationTerm, LoadSource};
pub use crate::postprocessing::{
    Envelope, EnvelopeSample, Extremum, NodeDisplacementSample, NodeReactionSample,
    SectionForceSources, Truss3DAxialForceSample, Truss3DEnvelope, Truss3DNodeDisplacementSample,
    Truss3DNodeReactionSample,
};
pub use crate::truss::{
    TrussAnalysisResult, TrussDof, TrussElement, TrussEquilibriumReport, TrussModel, TrussNode,
    TrussSolver,
};
pub use crate::truss3d::{
    TrussAnalysisResult3D, TrussDof3D, TrussElement3D, TrussEquilibriumReport3D, TrussModel3D,
    TrussNode3D, TrussSolver3D,
};
