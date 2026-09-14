//! 2D frame façade over the verified Beam FEM core (Phase 13).
//!
//! This module adds **no mechanics**. Every element stiffness, coordinate
//! transformation, load vector, static-condensation step, end-force recovery
//! and solver call is delegated to [`BeamModel`] / [`BeamSolver`]. What it adds
//! is a frame-shaped surface:
//!
//! * typed [`NodeHandle`] / [`MemberHandle`] addressing instead of positional
//!   indices;
//! * connectivity validation (orphan nodes, disconnected components, duplicate
//!   and zero-length members) with *targeted* diagnostics that replace the
//!   opaque "singular matrix" error;
//! * a support vocabulary ([`FrameModel::fix`], [`FrameModel::pin`],
//!   [`FrameModel::roller_y`], [`FrameModel::roller_x`],
//!   [`FrameModel::restrain`]);
//! * frame-level result access and a global equilibrium report.
//!
//! # Conventions (frozen - identical to `docs/beam_fem.md`)
//!
//! ```text
//! DOFs              [ux, uy, rz] per node (global), rz counter-clockwise positive
//! nodal loads       GLOBAL   (add via FrameModel::nodal_load / nodal_moment)
//! member loads      LOCAL    (add via FrameModel::member_udl / member_point_load)
//! member end forces f_end = f_equiv - K_e u_e   (element-on-node, LOCAL axes)
//! reactions         R = K_original u - f_global (global)
//! constraints       static condensation K_ff u_f = f_f - K_fc u_c (no penalties)
//! ```
//!
//! A member point load at `xi = 0` or `xi = 1` acts at a node and is **not**
//! double counted.
//!
//! # Current limitations (documented, not defects)
//!
//! * Parallel members between the same node pair are **not** supported: the
//!   second one is rejected as a duplicate. Rigid offsets / releases are not
//!   modelled.
//! * A single connected structural system is required; multiple independent
//!   substructures are rejected as disconnected.
//! * Member loads are uniform (single `(qx, qy)` per member per call; repeated
//!   calls accumulate, matching the core behaviour).
//!
//! # Example
//!
//! ```rust
//! use section_properties::frame::FrameModel;
//! use section_properties::{BeamSection, Dof, Material};
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
//! // v = -P L^3 / 3EI
//! let uy = result.displacement(tip, Dof::Uy)?;
//! assert!((uy + 1.0e4 * 2.0f64.powi(3) / (3.0 * 200e9 * 2e-5)).abs() < 1e-12);
//! assert!(result.equilibrium().is_balanced());
//! # Ok(())
//! # }
//! ```

use crate::SolverSelection;
use crate::beam_fem::{
    BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver, Dof, FemError, ReducedSystem,
};
use crate::fea::mechanism::diagnose_reduced;
use crate::material::Material;

use std::collections::VecDeque;

pub use crate::fea::mechanism::StructuralDiagnostic;

// ---------------------------------------------------------------------------
// Handles
// ---------------------------------------------------------------------------

/// Handle to a node of a [`FrameModel`].
///
/// Handles are produced by [`FrameModel::add_node`] and are only valid for the
/// model that created them. They are deliberately distinct from raw DOF
/// indices so a node reference cannot be confused with a degree of freedom.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct NodeHandle(usize);

impl NodeHandle {
    /// The zero-based index of the node inside the model.
    pub fn index(self) -> usize {
        self.0
    }

    /// Build a handle from a raw index.
    ///
    /// The handle is **not** validated here; validity is checked when it is
    /// used against a model (returning [`FemError::InvalidNode`]). Prefer
    /// [`FrameModel::add_node`] in normal use.
    pub fn from_index(index: usize) -> Self {
        Self(index)
    }
}

/// Handle to a member (two-node frame element) of a [`FrameModel`].
///
/// Produced by [`FrameModel::add_member`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct MemberHandle(usize);

impl MemberHandle {
    /// The zero-based index of the member inside the model.
    pub fn index(self) -> usize {
        self.0
    }

    /// Build a handle from a raw index.
    ///
    /// The handle is **not** validated here; validity is checked when it is
    /// used against a model (returning [`FemError::InvalidMember`]). Prefer
    /// [`FrameModel::add_member`] in normal use.
    pub fn from_index(index: usize) -> Self {
        Self(index)
    }
}

// ---------------------------------------------------------------------------
// Equilibrium report
// ---------------------------------------------------------------------------

/// Result of the global equilibrium check, summed **about the global origin
/// `(0, 0)`**.
///
/// Residuals are `applied + reaction` and should be zero for a solved,
/// properly restrained frame. Both the residuals and the contributing totals
/// are exposed so a caller can diagnose which component is out of balance
/// instead of receiving a single boolean.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EquilibriumReport {
    /// Residual of `ΣFx` (applied + reaction).
    pub fx_residual: f64,
    /// Residual of `ΣFy` (applied + reaction).
    pub fy_residual: f64,
    /// Residual of `ΣMz` about the global origin (applied + reaction).
    pub mz_residual: f64,
    /// Total applied horizontal load (global nodal, plus member load resultants).
    pub applied_fx: f64,
    /// Total applied vertical load.
    pub applied_fy: f64,
    /// Total applied moment about the origin, including force moments `x·Fy − y·Fx`.
    pub applied_mz: f64,
    /// Sum of support reaction forces in X.
    pub reaction_fx: f64,
    /// Sum of support reaction forces in Y.
    pub reaction_fy: f64,
    /// Sum of reaction moments about the origin, including `x·Ry − y·Rx`.
    pub reaction_mz: f64,
    /// Tolerance applied by [`Self::is_balanced`] for the force components.
    pub tolerance: f64,
}

impl EquilibriumReport {
    /// Whether all three residuals are within the report's tolerance.
    ///
    /// The force tolerance is `1e-6 · max(1, |ΣFx| + |ΣFy|)` relative to the
    /// applied load magnitude (the same order of tolerance used by the Beam FEM
    /// equilibrium tests); the moment tolerance uses the same relative factor on
    /// the applied moment magnitude.
    pub fn is_balanced(&self) -> bool {
        let m_scale = self.applied_mz.abs().max(self.tolerance * 10.0).max(1.0);
        self.fx_residual.abs() <= self.tolerance
            && self.fy_residual.abs() <= self.tolerance
            && self.mz_residual.abs() <= 1e-6 * m_scale
    }
}

// ---------------------------------------------------------------------------
// FrameModel
// ---------------------------------------------------------------------------

/// A 2D frame: nodes, members, supports and loads.
///
/// Wraps the existing [`BeamModel`] and delegates all mechanics to it; supports
/// arbitrary planar connectivity (not just a straight beam), per-DOF
/// constraints and prescribed displacements.
#[derive(Debug, Clone)]
pub struct FrameModel {
    inner: BeamModel,
}

impl Default for FrameModel {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameModel {
    /// Create an empty frame.
    pub fn new() -> Self {
        Self {
            inner: BeamModel::new(),
        }
    }

    /// Number of nodes.
    pub fn n_nodes(&self) -> usize {
        self.inner.nodes.len()
    }

    /// Number of members.
    pub fn n_members(&self) -> usize {
        self.inner.elements.len()
    }

    /// Add a node at global `(x, y)` and return its handle.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if either coordinate is not finite.
    pub fn add_node(&mut self, x: f64, y: f64) -> Result<NodeHandle, FemError> {
        if !x.is_finite() || !y.is_finite() {
            return Err(FemError::InvalidInput(format!(
                "node coordinates must be finite, got ({x}, {y})"
            )));
        }
        let idx = self.inner.nodes.len();
        self.inner.add_node(BeamNode::new(idx, x, y));
        Ok(NodeHandle(idx))
    }

    /// Add a member between two existing nodes.
    ///
    /// The member stiffness uses `EA` (axial) and `EI` (bending) from the given
    /// material and section, exactly as the Beam FEM core does.
    ///
    /// # Errors
    ///
    /// * [`FemError::InvalidNode`] if either handle is not valid for this model;
    /// * [`FemError::ZeroLengthMember`] if the two nodes coincide;
    /// * [`FemError::DuplicateMember`] if a member already connects the same
    ///   node pair (in either order) - parallel members are not supported;
    /// * [`FemError::InvalidInput`] if `E`, `A` or `I` is non-finite or
    ///   non-positive (the same predicate the solver applies at assembly time).
    pub fn add_member(
        &mut self,
        a: NodeHandle,
        b: NodeHandle,
        material: Material,
        section: BeamSection,
    ) -> Result<MemberHandle, FemError> {
        let ia = self.check_node(a)?;
        let ib = self.check_node(b)?;
        if ia == ib {
            return Err(FemError::ZeroLengthMember(format!(
                "member connects node {ia} to itself"
            )));
        }
        let (pa, pb) = (self.inner.nodes[ia].point(), self.inner.nodes[ib].point());
        let (dx, dy) = (pb.x - pa.x, pb.y - pa.y);
        if !dx.is_finite() || !dy.is_finite() || dx * dx + dy * dy <= 0.0 {
            return Err(FemError::ZeroLengthMember(format!(
                "nodes {ia} and {ib} coincide at ({}, {})",
                pa.x, pa.y
            )));
        }

        let (e, area, inertia) = (material.youngs_modulus, section.area, section.second_moment);
        if !e.is_finite() || e <= 0.0 {
            return Err(FemError::InvalidInput(format!(
                "Young's modulus must be finite and positive, got {e}"
            )));
        }
        if !area.is_finite() || area <= 0.0 {
            return Err(FemError::InvalidInput(format!(
                "cross-section area must be finite and positive, got {area}"
            )));
        }
        if !inertia.is_finite() || inertia <= 0.0 {
            return Err(FemError::InvalidInput(format!(
                "second moment of area must be finite and positive, got {inertia}"
            )));
        }

        let (lo, hi) = if ia < ib { (ia, ib) } else { (ib, ia) };
        for existing in &self.inner.elements {
            let (elo, ehi) = if existing.node_i < existing.node_j {
                (existing.node_i, existing.node_j)
            } else {
                (existing.node_j, existing.node_i)
            };
            if elo == lo && ehi == hi {
                return Err(FemError::DuplicateMember(format!(
                    "a member already connects nodes {lo} and {hi}; parallel members are not supported"
                )));
            }
        }

        let element = BeamElement::new(ia, ib, material, section)?;
        self.inner.add_element(element);
        Ok(MemberHandle(self.inner.elements.len() - 1))
    }

    /// Fully fix a node (`ux = uy = rz = 0`).
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if the handle is not valid.
    pub fn fix(&mut self, node: NodeHandle) -> Result<(), FemError> {
        let i = self.check_node(node)?;
        self.inner.try_fix_node(i)
    }

    /// Pin a node: restrain `ux` and `uy`, leave the rotation free.
    pub fn pin(&mut self, node: NodeHandle) -> Result<(), FemError> {
        let i = self.check_node(node)?;
        self.inner.try_fix_dof(i, Dof::Ux.index(), 0.0)?;
        self.inner.try_fix_dof(i, Dof::Uy.index(), 0.0)
    }

    /// Roller restraining the vertical translation (`uy = 0`), horizontal free.
    pub fn roller_y(&mut self, node: NodeHandle) -> Result<(), FemError> {
        let i = self.check_node(node)?;
        self.inner.try_fix_dof(i, Dof::Uy.index(), 0.0)
    }

    /// Roller restraining the horizontal translation (`ux = 0`), vertical free.
    pub fn roller_x(&mut self, node: NodeHandle) -> Result<(), FemError> {
        let i = self.check_node(node)?;
        self.inner.try_fix_dof(i, Dof::Ux.index(), 0.0)
    }

    /// Prescribe a DOF to an arbitrary finite value (e.g. a support settlement).
    ///
    /// Applied through the existing static condensation
    /// (`K_ff u_f = f_f - K_fc u_c`); no penalty constraints are used.
    pub fn restrain(&mut self, node: NodeHandle, dof: Dof, value: f64) -> Result<(), FemError> {
        let i = self.check_node(node)?;
        self.inner.try_fix(i, dof, value)
    }

    /// Apply a **global** nodal force `(fx, fy)`.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if the handle is not valid, or
    /// [`FemError::InvalidInput`] if either component is non-finite. Both
    /// components are validated **before** either is recorded, so a rejected
    /// call leaves the model unchanged (no half-applied load).
    pub fn nodal_load(&mut self, node: NodeHandle, fx: f64, fy: f64) -> Result<(), FemError> {
        let i = self.check_node(node)?;
        if !fx.is_finite() {
            return Err(FemError::InvalidInput(format!(
                "nodal force Fx must be finite, got {fx}"
            )));
        }
        if !fy.is_finite() {
            return Err(FemError::InvalidInput(format!(
                "nodal force Fy must be finite, got {fy}"
            )));
        }
        self.inner.try_add_nodal_force(i, Dof::Ux.index(), fx)?;
        self.inner.try_add_nodal_force(i, Dof::Uy.index(), fy)
    }

    /// Apply a **global** nodal moment `mz` (counter-clockwise positive).
    pub fn nodal_moment(&mut self, node: NodeHandle, mz: f64) -> Result<(), FemError> {
        let i = self.check_node(node)?;
        self.inner.add_applied_moment(i, mz)
    }

    /// Apply a uniform distributed load in the member's **local** axes.
    ///
    /// `qy > 0` is upward in local +y; `qx > 0` is tensile (towards `node_j`).
    /// Repeated calls accumulate on the same member.
    pub fn member_udl(&mut self, member: MemberHandle, qx: f64, qy: f64) -> Result<(), FemError> {
        let i = self.check_member(member)?;
        self.inner.add_distributed_load(i, qx, qy)
    }

    /// Apply a point load in the member's **local** axes at normalised position
    /// `xi in [0, 1]`.
    ///
    /// `xi = 0` / `xi = 1` places the load at a node; it is applied exactly once
    /// (no double counting).
    pub fn member_point_load(
        &mut self,
        member: MemberHandle,
        xi: f64,
        fx: f64,
        fy: f64,
        mz: f64,
    ) -> Result<(), FemError> {
        let i = self.check_member(member)?;
        self.inner.add_point_load(i, xi, fx, fy, mz)
    }

    /// Solve with automatic solver selection.
    pub fn solve(&self) -> Result<FrameAnalysisResult, FemError> {
        self.solve_with(SolverSelection::Auto)
    }

    /// Solve with an explicit backend selection.
    ///
    /// Validates the model (see [`Self::validate`]) before assembling, so
    /// structural modelling errors are reported as targeted diagnostics rather
    /// than as a matrix singularity.
    pub fn solve_with(&self, selection: SolverSelection) -> Result<FrameAnalysisResult, FemError> {
        FrameSolver::new(self).with_selection(selection).solve()
    }

    /// A solver builder for this frame (configure a backend, then solve).
    pub fn solver(&self) -> FrameSolver<'_> {
        FrameSolver::new(self)
    }

    /// Validate the structural model.
    ///
    /// Checks, in order:
    ///
    /// 1. the model is not empty (at least one node and one member);
    /// 2. no **orphan nodes** - every node is used by at least one member;
    /// 3. the members form a **single connected component**.
    ///
    /// Per-element checks (zero length, duplicate connectivity, invalid
    /// material/section) are enforced when the member is added.
    ///
    /// Insufficient restraint is *not* diagnosed here: [`Self::diagnostic`]
    /// classifies the boundary-conditioned system separately, without
    /// rejecting the model, and [`FrameSolver::solve`] attaches that
    /// classification to the solver error when the solve fails.
    pub fn validate(&self) -> Result<(), FemError> {
        let inner = &self.inner;
        if inner.nodes.is_empty() {
            return Err(FemError::InvalidModel(
                "frame has no nodes; add at least one node and one member".to_string(),
            ));
        }
        if inner.elements.is_empty() {
            return Err(FemError::InvalidModel(
                "frame has no members; add at least one member".to_string(),
            ));
        }

        let mut used = vec![false; inner.nodes.len()];
        for e in &inner.elements {
            used[e.node_i] = true;
            used[e.node_j] = true;
        }
        if let Some(i) = used.iter().position(|u| !u) {
            let p = inner.nodes[i].point();
            return Err(FemError::OrphanNode(format!(
                "node {i} at ({}, {}) is not connected to any member",
                p.x, p.y
            )));
        }

        let components = count_components(inner.nodes.len(), &inner.elements);
        if components > 1 {
            return Err(FemError::DisconnectedStructure(format!(
                "frame has {components} disconnected components; a single connected \
                 structural system is required"
            )));
        }
        Ok(())
    }

    /// Diagnose the **boundary-conditioned** (reduced) system without solving
    /// it: is this structure stable, under-restrained (rigid-body mechanism),
    /// internally rank deficient (mechanism), or merely numerically
    /// ill-conditioned?
    ///
    /// The diagnosis reads the reduced free-free stiffness block `K_ff` that
    /// the **actual solve path** condenses (`BeamSolver::condense`) — the exact
    /// matrix a solve factorises, not a second assembly — and classifies it with
    /// relative, scale-invariant probes (see [`crate::fea::mechanism`] for the
    /// criterion and its limits). It never assembles with penalties, never
    /// perturbs the matrix and never fabricates displacements, and it does not
    /// run as part of a successful [`Self::solve`].
    ///
    /// The verdict is a property of the **structure**, not of the loads: the
    /// load vector is not consulted, so scaling or omitting the loads cannot
    /// change the result.
    ///
    /// # Errors
    ///
    /// The same model errors as [`Self::solve`]: [`Self::validate`] plus the
    /// construction checks [`BeamSolver::from_model`] performs (its error
    /// surface is slightly wider than `validate` alone).
    pub fn diagnostic(&self) -> Result<StructuralDiagnostic, FemError> {
        self.validate()?;
        let mut beam = BeamSolver::from_model(&self.inner)?;
        Ok(self.classify(beam.condense()))
    }

    /// Classify a reduced (boundary-conditioned) stiffness system using this
    /// model's geometry: the global rigid-body motions are built in *reduced*
    /// order from the solver's own free-DOF map, so no independent reassembly
    /// of `K_ff` is involved.
    fn classify(&self, reduced: &ReducedSystem) -> StructuralDiagnostic {
        let rigid = self.rigid_candidates(&reduced.free_to_global);
        diagnose_reduced(&reduced.k_ff, &rigid)
    }

    /// The three global rigid-body motions (Tx, Ty, Rz) restricted to the free
    /// DOFs, laid out in reduced order.
    ///
    /// `free_to_global[r]` is the global DOF (`3 * node + dof`) of reduced index
    /// `r`; constrained entries are dropped, not displaced. Rotation by `theta`
    /// about the global origin is `ux = -theta*y`, `uy = theta*x`, `rz = theta`.
    fn rigid_candidates(&self, free_to_global: &[usize]) -> Vec<Vec<f64>> {
        let n_free = free_to_global.len();
        let mut tx = vec![0.0; n_free];
        let mut ty = vec![0.0; n_free];
        let mut rz = vec![0.0; n_free];
        for (r, &global) in free_to_global.iter().enumerate() {
            let point = self.inner.nodes[global / 3].point();
            match global % 3 {
                0 => {
                    tx[r] = 1.0;
                    rz[r] = -point.y;
                }
                1 => {
                    ty[r] = 1.0;
                    rz[r] = point.x;
                }
                _ => rz[r] = 1.0,
            }
        }
        vec![tx, ty, rz]
    }

    /// Handle of the node with the given index.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if the index is out of range.
    pub fn node_handle(&self, index: usize) -> Result<NodeHandle, FemError> {
        if index >= self.inner.nodes.len() {
            return Err(FemError::InvalidNode(format!(
                "node index {index} is out of range (max: {})",
                self.inner.nodes.len().saturating_sub(1)
            )));
        }
        Ok(NodeHandle(index))
    }

    /// Handle of the member with the given index.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidMember`] if the index is out of range.
    pub fn member_handle(&self, index: usize) -> Result<MemberHandle, FemError> {
        if index >= self.inner.elements.len() {
            return Err(FemError::InvalidMember(format!(
                "member index {index} is out of range (max: {})",
                self.inner.elements.len().saturating_sub(1)
            )));
        }
        Ok(MemberHandle(index))
    }

    fn check_node(&self, node: NodeHandle) -> Result<usize, FemError> {
        if node.0 >= self.inner.nodes.len() {
            return Err(FemError::InvalidNode(format!(
                "node handle {} is not valid for this frame (max: {})",
                node.0,
                self.inner.nodes.len().saturating_sub(1)
            )));
        }
        Ok(node.0)
    }

    fn check_member(&self, member: MemberHandle) -> Result<usize, FemError> {
        if member.0 >= self.inner.elements.len() {
            return Err(FemError::InvalidMember(format!(
                "member handle {} is not valid for this frame (max: {})",
                member.0,
                self.inner.elements.len().saturating_sub(1)
            )));
        }
        Ok(member.0)
    }
}

/// Number of connected components of the member connectivity graph.
fn count_components(n_nodes: usize, elements: &[BeamElement]) -> usize {
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n_nodes];
    for e in elements {
        adj[e.node_i].push(e.node_j);
        adj[e.node_j].push(e.node_i);
    }
    let mut seen = vec![false; n_nodes];
    let mut components = 0;
    for start in 0..n_nodes {
        if seen[start] || adj[start].is_empty() {
            continue;
        }
        components += 1;
        seen[start] = true;
        let mut queue = VecDeque::from([start]);
        while let Some(u) = queue.pop_front() {
            for &v in &adj[u] {
                if !seen[v] {
                    seen[v] = true;
                    queue.push_back(v);
                }
            }
        }
    }
    components
}

// ---------------------------------------------------------------------------
// FrameSolver
// ---------------------------------------------------------------------------

/// Solves a [`FrameModel`]; configure the backend, then [`FrameSolver::solve`].
pub struct FrameSolver<'a> {
    model: &'a FrameModel,
    selection: SolverSelection,
}

impl<'a> FrameSolver<'a> {
    /// Create a solver for `model` using automatic backend selection.
    pub fn new(model: &'a FrameModel) -> Self {
        Self {
            model,
            selection: SolverSelection::Auto,
        }
    }

    /// Choose an explicit backend (or [`SolverSelection::Auto`]).
    pub fn with_selection(mut self, selection: SolverSelection) -> Self {
        self.selection = selection;
        self
    }

    /// Set the backend (builder-style alternative to [`Self::with_selection`]).
    pub fn set_solver(&mut self, selection: SolverSelection) {
        self.selection = selection;
    }

    /// Validate, assemble and solve the frame.
    ///
    /// A successful solve runs **no** diagnostic and its numerical path is
    /// unchanged (see [`FrameModel::diagnostic`] for the on-demand API). Only a
    /// *failed* solve additionally classifies the reduced system, and appends
    /// that classification to the error message so an under-restrained or
    /// rank-deficient frame is not reported as an opaque "singular matrix".
    ///
    /// # Errors
    ///
    /// Model-structure errors from [`FrameModel::validate`] plus any
    /// [`FemError::SolverError`] from the linear solver (e.g. insufficient
    /// restraint, or a conditioning limit at extreme scales). The variant is
    /// unchanged; the message carries the diagnosis.
    pub fn solve(&self) -> Result<FrameAnalysisResult, FemError> {
        self.model.validate()?;
        let mut beam = BeamSolver::from_model(&self.model.inner)?;
        beam.set_solver(self.selection.clone());
        if let Err(e) = beam.solve_configured() {
            // The solver condensed the boundary conditions before attempting any
            // backend, so its retained reduced system is the exact matrix the
            // failed factorisation saw - no second assembly.
            let diagnosis = beam.reduced_system().map(|rs| self.model.classify(rs));
            return Err(with_structural_diagnosis(e, diagnosis));
        }
        Ok(FrameAnalysisResult {
            beam,
            model: self.model.clone(),
        })
    }
}

/// Record the structural diagnosis of a failed solve in the error message.
///
/// The error **variant** is deliberately preserved (`SolverError` stays
/// `SolverError`): the diagnosis is extra information, not a re-typing of the
/// failure, so no existing caller or exhaustive match changes meaning. A
/// `Stable` diagnosis adds nothing - the failure was not structural - and a
/// missing diagnosis (`None`, i.e. no reduced system was available) leaves the
/// error untouched.
fn with_structural_diagnosis(err: FemError, diagnosis: Option<StructuralDiagnostic>) -> FemError {
    match (err, diagnosis) {
        (FemError::SolverError(msg), Some(d)) if !matches!(d, StructuralDiagnostic::Stable) => {
            FemError::SolverError(format!("{msg}; structural diagnosis: {d}"))
        }
        (other, _) => other,
    }
}

// ---------------------------------------------------------------------------
// FrameAnalysisResult
// ---------------------------------------------------------------------------

/// Solved frame: frame-level access to displacements, reactions, member end
/// forces and global equilibrium.
///
/// All values come from the existing [`BeamSolver`]; nothing is recomputed with
/// a second formulation.
pub struct FrameAnalysisResult {
    beam: BeamSolver,
    model: FrameModel,
}

impl std::fmt::Debug for FrameAnalysisResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FrameAnalysisResult")
            .field("nodes", &self.model.n_nodes())
            .field("members", &self.model.n_members())
            .field("solver", &self.beam.solver_name())
            .finish_non_exhaustive()
    }
}

impl FrameAnalysisResult {
    /// Name of the backend used by the successful solve.
    pub fn solver_name(&self) -> Option<&str> {
        self.beam.solver_name()
    }

    /// Number of nodes in the solved frame.
    pub fn n_nodes(&self) -> usize {
        self.model.n_nodes()
    }

    /// Number of members in the solved frame.
    pub fn n_members(&self) -> usize {
        self.model.n_members()
    }

    /// Full global displacement vector, laid out `[ux, uy, rz]` per node.
    pub fn displacements(&self) -> &[f64] {
        self.beam.displacements()
    }

    /// Global support reactions, laid out `[Rx, Ry, Mz]` per node.
    pub fn reactions(&self) -> Vec<f64> {
        self.beam.reactions()
    }

    /// Displacement of a DOF at a node (global).
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if the handle is not valid for this frame.
    pub fn displacement(&self, node: NodeHandle, dof: Dof) -> Result<f64, FemError> {
        self.check_node(node)?;
        self.beam.displacement_dof(node.index(), dof)
    }

    /// Support reaction of a DOF at a node (global).
    ///
    /// Free DOFs report the raw (round-off) residual, exactly as
    /// [`BeamSolver::reaction_dof`] does.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if the handle is not valid for this frame.
    pub fn reaction(&self, node: NodeHandle, dof: Dof) -> Result<f64, FemError> {
        self.check_node(node)?;
        self.beam.reaction_dof(node.index(), dof)
    }

    /// Member end forces in **local** axes:
    /// `[N_i, V_i, M_i, N_j, V_j, M_j]`.
    ///
    /// These are element-on-node forces, `f_end = f_equiv - K_e u_e`, exactly as
    /// documented in `docs/beam_fem.md`; the sign convention is **not** inverted
    /// for the frame API.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidMember`] if the handle is not valid for this frame.
    pub fn member_end_forces(&self, member: MemberHandle) -> Result<[f64; 6], FemError> {
        let i = self.check_member(member)?;
        let all = self.beam.element_end_forces()?;
        all.get(i)
            .copied()
            .ok_or_else(|| FemError::InvalidMember(format!("member handle {} is out of range", i)))
    }

    /// Member end forces transformed to **global** axes
    /// (`f_global = Tᵀ f_local`).
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidMember`] if the handle is not valid for this frame.
    pub fn member_end_forces_global(&self, member: MemberHandle) -> Result<[f64; 6], FemError> {
        let i = self.check_member(member)?;
        let all = self.beam.element_end_forces_global()?;
        all.get(i)
            .copied()
            .ok_or_else(|| FemError::InvalidMember(format!("member handle {} is out of range", i)))
    }

    /// Global equilibrium check about the origin `(0, 0)`.
    ///
    /// Combines support reactions, global nodal loads and moments, and the
    /// global resultants of member distributed and point loads (transformed from
    /// member-local axes and applied at their true global locations).
    /// See [`EquilibriumReport`].
    pub fn equilibrium(&self) -> EquilibriumReport {
        let m = &self.model.inner;
        let mut applied_fx = 0.0;
        let mut applied_fy = 0.0;
        let mut applied_mz = 0.0;

        // Global nodal loads and moments.
        for (node, dof, v) in &m.nodal_forces {
            let p = m.nodes[*node].point();
            match *dof {
                0 => {
                    applied_fx += v;
                    applied_mz += -p.y * v;
                }
                1 => {
                    applied_fy += v;
                    applied_mz += p.x * v;
                }
                _ => applied_mz += v,
            }
        }
        for am in &m.applied_moments {
            applied_mz += am.value;
        }

        // Member load resultants, rotated from local to global.
        for (elem_idx, dl) in m.distributed_loads.iter().enumerate() {
            let _ = elem_idx;
            let Some((pi, pj)) = member_points(m, dl.element_idx) else {
                continue;
            };
            let (dx, dy) = (pj.x - pi.x, pj.y - pi.y);
            let len = (dx * dx + dy * dy).sqrt();
            let (c, s) = (dx / len, dy / len);
            let (lfx, lfy) = (dl.qx * len, dl.qy * len);
            let (gx, gy) = (c * lfx - s * lfy, s * lfx + c * lfy);
            let (xm, ym) = (pi.x + 0.5 * dx, pi.y + 0.5 * dy);
            applied_fx += gx;
            applied_fy += gy;
            applied_mz += xm * gy - ym * gx;
        }
        for pl in &m.point_loads {
            let Some((pi, _)) = member_points(m, pl.element_idx) else {
                continue;
            };
            let el = &m.elements[pl.element_idx];
            let pj = m.nodes[el.node_j].point();
            let (dx, dy) = (pj.x - pi.x, pj.y - pi.y);
            let len = (dx * dx + dy * dy).sqrt();
            let (c, s) = (dx / len, dy / len);
            let (gx, gy) = (c * pl.fx - s * pl.fy, s * pl.fx + c * pl.fy);
            let (xp, yp) = (pi.x + pl.position * dx, pi.y + pl.position * dy);
            applied_fx += gx;
            applied_fy += gy;
            applied_mz += xp * gy - yp * gx + pl.mz;
        }

        // Support reactions, per node.
        let reactions = self.beam.reactions();
        let mut reaction_fx = 0.0;
        let mut reaction_fy = 0.0;
        let mut reaction_mz = 0.0;
        for (idx, node) in m.nodes.iter().enumerate() {
            let p = node.point();
            let rx = reactions.get(3 * idx).copied().unwrap_or(0.0);
            let ry = reactions.get(3 * idx + 1).copied().unwrap_or(0.0);
            let rz = reactions.get(3 * idx + 2).copied().unwrap_or(0.0);
            reaction_fx += rx;
            reaction_fy += ry;
            reaction_mz += p.x * ry - p.y * rx + rz;
        }

        let tolerance = 1e-6
            * (applied_fx.abs() + applied_fy.abs() + reaction_fx.abs() + reaction_fy.abs())
                .max(1.0);
        EquilibriumReport {
            fx_residual: applied_fx + reaction_fx,
            fy_residual: applied_fy + reaction_fy,
            mz_residual: applied_mz + reaction_mz,
            applied_fx,
            applied_fy,
            applied_mz,
            reaction_fx,
            reaction_fy,
            reaction_mz,
            tolerance,
        }
    }

    fn check_node(&self, node: NodeHandle) -> Result<(), FemError> {
        if node.index() >= self.model.n_nodes() {
            return Err(FemError::InvalidNode(format!(
                "node handle {} is not valid for this result (max: {})",
                node.index(),
                self.model.n_nodes().saturating_sub(1)
            )));
        }
        Ok(())
    }

    fn check_member(&self, member: MemberHandle) -> Result<usize, FemError> {
        if member.index() >= self.model.n_members() {
            return Err(FemError::InvalidMember(format!(
                "member handle {} is not valid for this result (max: {})",
                member.index(),
                self.model.n_members().saturating_sub(1)
            )));
        }
        Ok(member.index())
    }
}

/// End-point coordinates of a member, or `None` if the index is out of range.
fn member_points(
    model: &BeamModel,
    member: usize,
) -> Option<(crate::geometry::Point, crate::geometry::Point)> {
    let el = model.elements.get(member)?;
    Some((
        model.nodes[el.node_i].point(),
        model.nodes[el.node_j].point(),
    ))
}

// ---------------------------------------------------------------------------
// Single-source guarantee: the diagnostic reads the solver's condensed system
// ---------------------------------------------------------------------------
//
// These are in-crate tests on purpose: `tests/*.rs` are integration tests and
// cannot see `pub(crate)` items such as `BeamSolver::condense` /
// `FrameModel::classify`, so this guarantee is not testable from there.
#[cfg(test)]
mod reduced_system_single_source_tests {
    use super::*;
    use crate::fea::SparseMatrix;

    fn steel() -> Material {
        Material::new(200e9, 0.3, 7850.0, "Steel")
    }

    fn sec() -> BeamSection {
        BeamSection::new(5e-3, 2e-5)
    }

    /// The diagnostic must classify exactly the system the solver's own
    /// condensation produced: reading the live solver and the public API must
    /// agree on every structure in the battery.
    fn assert_agrees(f: &FrameModel, label: &str) -> Result<(), FemError> {
        let mut beam = BeamSolver::from_model(&f.inner)?;
        let from_solver = f.classify(beam.condense());
        assert_eq!(
            from_solver,
            f.diagnostic()?,
            "{label}: diagnostic disagrees with the solver's condensed system"
        );
        Ok(())
    }

    /// Two-column portal: `base1 - top1 - top2 - base2`, with the given supports
    /// on the two base nodes.
    fn portal(base_1: &[Dof], base_2: &[Dof]) -> Result<FrameModel, FemError> {
        let mut f = FrameModel::new();
        let b1 = f.add_node(0.0, 0.0)?;
        let t1 = f.add_node(0.0, 3.0)?;
        let t2 = f.add_node(4.0, 3.0)?;
        let b2 = f.add_node(4.0, 0.0)?;
        f.add_member(b1, t1, steel(), sec())?;
        f.add_member(t1, t2, steel(), sec())?;
        f.add_member(b2, t2, steel(), sec())?;
        for dof in base_1 {
            f.restrain(b1, *dof, 0.0)?;
        }
        for dof in base_2 {
            f.restrain(b2, *dof, 0.0)?;
        }
        Ok(f)
    }

    #[test]
    fn diagnostic_classifies_the_solver_condensed_system() -> Result<(), FemError> {
        // Completely free beam - three rigid-body modes.
        let mut free = FrameModel::new();
        let a = free.add_node(0.0, 0.0)?;
        let b = free.add_node(2.0, 0.0)?;
        free.add_member(a, b, steel(), sec())?;
        assert_agrees(&free, "free beam")?;

        // One DOF restrained - still under-constrained.
        let mut one = FrameModel::new();
        let a = one.add_node(0.0, 0.0)?;
        let b = one.add_node(2.0, 0.0)?;
        one.add_member(a, b, steel(), sec())?;
        one.restrain(a, Dof::Ux, 0.0)?;
        assert_agrees(&one, "one-DOF-restrained beam")?;

        // Stable cantilever.
        let mut cant = FrameModel::new();
        let base = cant.add_node(0.0, 0.0)?;
        let tip = cant.add_node(2.0, 0.0)?;
        cant.add_member(base, tip, steel(), sec())?;
        cant.fix(base)?;
        assert_agrees(&cant, "stable cantilever")?;

        // Portal with several support sets, stable and deficient.
        let fixed = portal(&[Dof::Ux, Dof::Uy, Dof::Rz], &[Dof::Ux, Dof::Uy, Dof::Rz])?;
        assert_agrees(&fixed, "portal, both bases fixed")?;
        let pinned = portal(&[Dof::Ux, Dof::Uy], &[Dof::Ux, Dof::Uy])?;
        assert_agrees(&pinned, "portal, both bases pinned")?;
        let roller = portal(&[Dof::Ux, Dof::Uy, Dof::Rz], &[Dof::Uy])?;
        assert_agrees(&roller, "portal, fixed base + roller")?;
        let mechanism = portal(&[Dof::Ux, Dof::Uy], &[])?;
        assert_agrees(&mechanism, "portal, single pin (mechanism)")?;

        Ok(())
    }

    #[test]
    fn diagnostic_uses_the_supplied_matrix_not_a_copy() -> Result<(), FemError> {
        let mut f = FrameModel::new();
        let base = f.add_node(0.0, 0.0)?;
        let tip = f.add_node(2.0, 0.0)?;
        f.add_member(base, tip, steel(), sec())?;
        f.fix(base)?;

        let mut beam = BeamSolver::from_model(&f.inner)?;
        let real = beam.condense().clone();
        let stable = f.classify(&real);
        assert_eq!(
            stable,
            StructuralDiagnostic::Stable,
            "control must be stable"
        );

        // Forge a reduced system with the first free DOF (tip `ux`) removed. If
        // `classify` reassembled its own matrix instead of reading its argument,
        // this forgery would have no effect at all.
        let n = real.k_ff.n;
        let mut forged_k = SparseMatrix::new(n);
        for i in 0..n {
            for j in 0..n {
                let v = real.k_ff.get(i, j);
                if i != 0 && j != 0 && v != 0.0 {
                    forged_k.add(i, j, v);
                }
            }
        }
        forged_k.compress();
        let forged = ReducedSystem {
            k_ff: forged_k,
            free_to_global: real.free_to_global.clone(),
        };

        let verdict = f.classify(&forged);
        assert_ne!(
            verdict, stable,
            "classify must read the supplied matrix: zeroing a free DOF changed nothing"
        );
        // The public diagnostic still reads the untouched, real system.
        assert_eq!(f.diagnostic()?, stable);
        Ok(())
    }
}
