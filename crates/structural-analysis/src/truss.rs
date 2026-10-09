//! 2D truss finite element analysis.
//!
//! Pin-jointed 2D truss elements with 2 DOF per node: `[ux, uy]`.
//! Each element carries only axial force (no bending, no shear).
//!
//! # Mathematical model
//!
//! For an element of length `L`, direction cosines `c = dx/L`, `s = dy/L`,
//! axial stiffness `EA/L`, the 4×4 global stiffness matrix is:
//!
//! ```text
//! EA/L * [
//!   c²   cs  -c²  -cs
//!   cs   s²  -cs  -s²
//!  -c²  -cs   c²   cs
//!  -cs  -s²   cs   s²
//! ]
//! ```
//!
//! # Sign convention
//!
//! - **Axial force**: tension positive (element pulled apart along `node_i → node_j`).
//! - **Nodal loads**: global coordinates.
//! - **Reactions**: `R = K·u - f` (global).
//!
//! # Reused infrastructure
//!
//! This module reuses [`section_properties::fea::SparseMatrix`],
//! [`section_properties::fea::solver::LinearSolver`], and
//! [`crate::mechanism::diagnose_reduced`] — all of which are DOF-agnostic.
//! The truss-specific parts are the 2-DOF/node mapping, the 4×4 element
//! stiffness, and the 2-DOF rigid-body candidates.

#![allow(non_snake_case)]

use section_properties::fea::{
    SparseMatrix,
    solver::{LinearSolver, SolverRegistry, SolverSelection},
};
use section_properties::geometry::Point;
use section_properties::material::Material;

use crate::beam_fem::FemError;
use crate::frame::{load_case_source, load_combination_source};
use crate::load::{LoadCase, LoadCombination, LoadSource};
use crate::mechanism::{StructuralDiagnostic, diagnose_reduced};

// ---------------------------------------------------------------------------
// DOF
// ---------------------------------------------------------------------------

/// A translational degree of freedom of a truss node.
///
/// Truss nodes have 2 DOF: `Ux` and `Uy` (no rotation).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TrussDof {
    /// Horizontal displacement.
    Ux,
    /// Vertical displacement.
    Uy,
}

impl TrussDof {
    /// All DOFs in canonical order `[Ux, Uy]`.
    pub const ALL: [TrussDof; 2] = [TrussDof::Ux, TrussDof::Uy];

    /// Raw index: `Ux → 0`, `Uy → 1`.
    pub const fn index(self) -> usize {
        match self {
            TrussDof::Ux => 0,
            TrussDof::Uy => 1,
        }
    }

    /// Human-readable name (`"ux"`, `"uy"`).
    pub const fn name(self) -> &'static str {
        match self {
            TrussDof::Ux => "ux",
            TrussDof::Uy => "uy",
        }
    }
}

// ---------------------------------------------------------------------------
// Node
// ---------------------------------------------------------------------------

/// Truss node with 2 DOF: `[ux, uy]`.
#[derive(Debug, Clone, Copy)]
pub struct TrussNode {
    /// Node index (position in the model's node list).
    pub id: usize,
    /// X coordinate `m`.
    pub x: f64,
    /// Y coordinate `m`.
    pub y: f64,
}

impl TrussNode {
    /// Create a node at `(x, y)` with the given `id`.
    pub fn new(id: usize, x: f64, y: f64) -> Self {
        Self { id, x, y }
    }

    /// Return coordinates as a [`Point`].
    pub fn point(&self) -> Point {
        Point::new(self.x, self.y)
    }
}

// ---------------------------------------------------------------------------
// Element
// ---------------------------------------------------------------------------

/// 2D pin-jointed truss element (axial-only, 4 DOF).
///
/// # Sign convention
///
/// Axial force is **tension positive**: a positive value means the element is
/// being pulled apart along the `node_i → node_j` direction.
#[derive(Debug, Clone)]
pub struct TrussElement {
    /// Index of the first node.
    pub node_i: usize,
    /// Index of the second node.
    pub node_j: usize,
    /// Young's modulus `Pa`.
    pub E: f64,
    /// Cross-sectional area `m²`.
    pub A: f64,
}

impl TrussElement {
    /// Create a truss element connecting `node_i` and `node_j`.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if `E` or `A` is non-finite or non-positive.
    pub fn new(
        node_i: usize,
        node_j: usize,
        material: &Material,
        area: f64,
    ) -> Result<Self, FemError> {
        let E = material.youngs_modulus;
        if !E.is_finite() || E <= 0.0 {
            return Err(FemError::InvalidInput(format!(
                "Young's modulus must be finite and positive, got {E}"
            )));
        }
        if !area.is_finite() || area <= 0.0 {
            return Err(FemError::InvalidInput(format!(
                "cross-sectional area must be finite and positive, got {area}"
            )));
        }
        Ok(Self {
            node_i,
            node_j,
            E,
            A: area,
        })
    }

    /// Element length from node coordinates.
    pub fn length(&self, pi: Point, pj: Point) -> f64 {
        let dx = pj.x - pi.x;
        let dy = pj.y - pi.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Direction cosines `(c, s)` where `c = dx/L`, `s = dy/L`.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if the element has zero length.
    pub fn direction(&self, pi: Point, pj: Point) -> Result<(f64, f64), FemError> {
        let L = self.length(pi, pj);
        if L <= 0.0 {
            return Err(FemError::InvalidInput(
                "truss element has zero length".to_string(),
            ));
        }
        Ok(((pj.x - pi.x) / L, (pj.y - pi.y) / L))
    }

    /// 4×4 global stiffness matrix.
    ///
    /// DOF ordering: `[u_i, v_i, u_j, v_j]` in **global** coordinates.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if the element has zero length.
    pub fn global_stiffness(&self, pi: Point, pj: Point) -> Result<[[f64; 4]; 4], FemError> {
        let L = self.length(pi, pj);
        if L <= 0.0 {
            return Err(FemError::InvalidInput(
                "truss element has zero length for stiffness".to_string(),
            ));
        }
        let (c, s) = self.direction(pi, pj)?;
        let k = self.E * self.A / L;
        let cc = c * c;
        let cs = c * s;
        let ss = s * s;
        Ok([
            [k * cc, k * cs, -k * cc, -k * cs],
            [k * cs, k * ss, -k * cs, -k * ss],
            [-k * cc, -k * cs, k * cc, k * cs],
            [-k * cs, -k * ss, k * cs, k * ss],
        ])
    }

    /// Axial force from the 4 global nodal displacements `[u_i, v_i, u_j, v_j]`.
    ///
    /// **Tension positive**: `N = EA/L · (-c·u_i - s·v_i + c·u_j + s·v_j)`.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if the element has zero length.
    pub fn axial_force(&self, pi: Point, pj: Point, u: [f64; 4]) -> Result<f64, FemError> {
        let L = self.length(pi, pj);
        if L <= 0.0 {
            return Err(FemError::InvalidInput(
                "truss element has zero length for axial force".to_string(),
            ));
        }
        let (c, s) = self.direction(pi, pj)?;
        let delta = -c * u[0] - s * u[1] + c * u[2] + s * u[3];
        Ok(self.E * self.A / L * delta)
    }
}

// ---------------------------------------------------------------------------
// Model
// ---------------------------------------------------------------------------

/// Truss model: nodes, elements, loads, and boundary conditions.
///
/// # Conventions
///
/// - Each node has 2 **global** DOFs `[ux, uy]`.
/// - DOF mapping: `dof(node, d) = 2*node + d`.
/// - Nodal loads are in **global** coordinates.
/// - Boundary conditions are enforced by static condensation (no penalties).
///
/// # Snapshot semantics
///
/// [`TrussSolver::from_model`] clones the model. Mutating a `TrussModel`
/// after building a solver does not affect that solver.
#[derive(Debug, Clone)]
pub struct TrussModel {
    /// Nodes.
    pub nodes: Vec<TrussNode>,
    /// Elements.
    pub elements: Vec<TrussElement>,
    /// Nodal forces: `(node_idx, dof, value)`.
    pub nodal_forces: Vec<(usize, usize, f64)>,
    /// Fixed DOFs: `(node_idx, dof, prescribed_value)`.
    pub fixed_dofs: Vec<(usize, usize, f64)>,
}

impl Default for TrussModel {
    fn default() -> Self {
        Self::new()
    }
}

impl TrussModel {
    /// Create an empty truss model.
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            elements: Vec::new(),
            nodal_forces: Vec::new(),
            fixed_dofs: Vec::new(),
        }
    }

    /// Add a node to the model.
    pub fn add_node(&mut self, node: TrussNode) {
        self.nodes.push(node);
    }

    /// Add a truss element to the model.
    pub fn add_element(&mut self, element: TrussElement) {
        self.elements.push(element);
    }

    /// Apply a nodal force `(fx, fy)` at `node_idx` in **global** coordinates.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if `node_idx` is out of bounds.
    /// [`FemError::InvalidInput`] if `fx` or `fy` is non-finite.
    pub fn add_nodal_force(&mut self, node_idx: usize, fx: f64, fy: f64) -> Result<(), FemError> {
        if node_idx >= self.nodes.len() {
            return Err(FemError::InvalidNode(format!(
                "node index {node_idx} out of bounds (max {})",
                self.nodes.len().saturating_sub(1)
            )));
        }
        if !fx.is_finite() || !fy.is_finite() {
            return Err(FemError::InvalidInput(format!(
                "nodal force must be finite, got fx = {fx}, fy = {fy}"
            )));
        }
        self.nodal_forces.push((node_idx, 0, fx));
        self.nodal_forces.push((node_idx, 1, fy));
        Ok(())
    }

    /// Restrain a single DOF of a node to a prescribed `value` (usually 0.0).
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if `node_idx` is out of bounds.
    /// [`FemError::InvalidInput`] if `value` is non-finite.
    pub fn fix_dof(&mut self, node_idx: usize, dof: TrussDof, value: f64) -> Result<(), FemError> {
        if node_idx >= self.nodes.len() {
            return Err(FemError::InvalidNode(format!(
                "node index {node_idx} out of bounds (max {})",
                self.nodes.len().saturating_sub(1)
            )));
        }
        if !value.is_finite() {
            return Err(FemError::InvalidInput(format!(
                "prescribed value must be finite, got {value}"
            )));
        }
        self.fixed_dofs.push((node_idx, dof.index(), value));
        Ok(())
    }

    /// Fix a node: restrain both `ux` and `uy` to zero.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if `node_idx` is out of bounds.
    pub fn fix_node(&mut self, node_idx: usize) -> Result<(), FemError> {
        self.fix_dof(node_idx, TrussDof::Ux, 0.0)?;
        self.fix_dof(node_idx, TrussDof::Uy, 0.0)
    }

    /// Total number of DOFs.
    pub fn n_dof(&self) -> usize {
        self.nodes.len() * 2
    }

    /// Global DOF index: `2*node + dof`.
    pub(crate) fn dof_index(&self, node_idx: usize, dof: usize) -> usize {
        node_idx * 2 + dof
    }

    /// Solve the truss under a single [`LoadCase`].
    ///
    /// The stiffness matrix and boundary conditions come from `self`; the load
    /// vector is assembled entirely from `case`. Any loads added directly to
    /// the model (via [`add_nodal_force`](Self::add_nodal_force)) are
    /// **ignored** — only the load case's loads are used.
    ///
    /// Prescribed displacements in the case override the model's fixed DOFs
    /// for the same `(node, dof)`. A prescription at a DOF that is not already
    /// constrained adds a new constraint.
    ///
    /// The result's [`load_source`](TrussAnalysisResult::load_source) is
    /// `LoadSource::LoadCase`.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if a load or prescribed displacement in the
    /// case references a node that does not exist.
    /// [`FemError::InvalidInput`] if a load or prescribed displacement in the
    /// case references a DOF index ≥ 2 (2D truss only has DOF 0 and 1).
    /// [`FemError::InvalidModel`] if the model is invalid (see
    /// [`TrussSolver::from_model`]).
    /// [`FemError::SolverError`] if the linear solve fails.
    pub fn solve_case(&self, case: &LoadCase) -> Result<TrussAnalysisResult, FemError> {
        for &(node_idx, dof, _) in case.nodal_forces() {
            if node_idx >= self.nodes.len() {
                return Err(FemError::InvalidNode(format!(
                    "load case '{}' references node {node_idx} out of bounds (max {})",
                    case.name(),
                    self.nodes.len().saturating_sub(1)
                )));
            }
            if dof >= 2 {
                return Err(FemError::InvalidInput(format!(
                    "load case '{}' has nodal force with DOF {dof} at node {node_idx}; \
                     2D truss only has DOF 0 (ux) and 1 (uy)",
                    case.name()
                )));
            }
        }
        for &(node_idx, dof, _) in case.prescribed_displacements() {
            if node_idx >= self.nodes.len() {
                return Err(FemError::InvalidNode(format!(
                    "load case '{}' prescribes displacement at node {node_idx} out of bounds (max {})",
                    case.name(),
                    self.nodes.len().saturating_sub(1)
                )));
            }
            if dof >= 2 {
                return Err(FemError::InvalidInput(format!(
                    "load case '{}' prescribes displacement with DOF {dof} at node {node_idx}; \
                     2D truss only has DOF 0 (ux) and 1 (uy)",
                    case.name()
                )));
            }
        }

        let mut model = self.clone();
        model.nodal_forces.clear();
        model.nodal_forces.extend_from_slice(case.nodal_forces());

        for &(node_idx, dof, value) in case.prescribed_displacements() {
            let mut found = false;
            for entry in &mut model.fixed_dofs {
                if entry.0 == node_idx && entry.1 == dof {
                    entry.2 = value;
                    found = true;
                    break;
                }
            }
            if !found {
                model.fixed_dofs.push((node_idx, dof, value));
            }
        }

        let mut solver = TrussSolver::from_model(&model)?;
        solver.solve_configured()?;
        let mut result = solver.results()?;
        result.load_source = load_case_source(case);
        Ok(result)
    }

    /// Solve the truss under a [`LoadCombination`].
    ///
    /// The right-hand side is merged: `f = Σ factor_i × f_case_i`, then
    /// `K u = f` is solved **once**. This is mathematically equivalent to
    /// solving each case separately and superposing the results (by linearity
    /// of the solve), but more efficient (a single factorisation).
    ///
    /// The result's [`load_source`](TrussAnalysisResult::load_source) is
    /// `LoadSource::LoadCombination` and records every scaled term.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if any case in the combination contains
    /// prescribed displacements (load combinations do not support prescribed
    /// displacement combination).
    /// [`FemError::InvalidNode`] if a load in any case references a node that
    /// does not exist.
    /// [`FemError::InvalidInput`] if a load in any case references a DOF
    /// index ≥ 2 (2D truss only has DOF 0 and 1).
    /// [`FemError::InvalidModel`] if the model is invalid.
    /// [`FemError::SolverError`] if the linear solve fails.
    pub fn solve_combination(
        &self,
        combo: &LoadCombination,
    ) -> Result<TrussAnalysisResult, FemError> {
        for (case, _) in combo.terms() {
            if case.has_prescribed_displacements() {
                return Err(FemError::InvalidInput(format!(
                    "load case '{}' in combination '{}' contains prescribed displacements; \
                     LoadCombination does not support prescribed displacement combination",
                    case.name(),
                    combo.name()
                )));
            }
        }

        let mut model = self.clone();
        model.nodal_forces.clear();
        for (case, factor) in combo.terms() {
            for &(node_idx, dof, value) in case.nodal_forces() {
                if node_idx >= self.nodes.len() {
                    return Err(FemError::InvalidNode(format!(
                        "load case '{}' in combination '{}' references node {node_idx} out of bounds (max {})",
                        case.name(),
                        combo.name(),
                        self.nodes.len().saturating_sub(1)
                    )));
                }
                if dof >= 2 {
                    return Err(FemError::InvalidInput(format!(
                        "load case '{}' in combination '{}' has nodal force with DOF {dof} at node {node_idx}; \
                         2D truss only has DOF 0 (ux) and 1 (uy)",
                        case.name(),
                        combo.name()
                    )));
                }
                model.nodal_forces.push((node_idx, dof, factor * value));
            }
        }

        let mut solver = TrussSolver::from_model(&model)?;
        solver.solve_configured()?;
        let mut result = solver.results()?;
        result.load_source = load_combination_source(combo);
        Ok(result)
    }
}

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

/// Condensed free-free system produced by static condensation.
struct ReducedSystem {
    k_ff: SparseMatrix,
    free_to_global: Vec<usize>,
}

/// Truss solver: assembles the global system, applies boundary conditions,
/// and delegates the linear solve to a `LinearSolver` backend.
///
/// # Workflow
///
/// ```rust
/// use structural_analysis::truss::{TrussElement, TrussModel, TrussNode, TrussSolver, TrussDof};
/// use section_properties::Material;
///
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// let mut model = TrussModel::new();
/// model.add_node(TrussNode::new(0, 0.0, 0.0));
/// model.add_node(TrussNode::new(1, 1.0, 0.0));
/// model.add_element(TrussElement::new(0, 1, &Material::new(1.0, 0.3, 1.0, "unit"), 1.0)?);
/// model.fix_node(0)?;
/// model.fix_dof(1, TrussDof::Uy, 0.0)?; // no lateral stiffness in a truss
/// model.add_nodal_force(1, 10.0, 0.0)?;
///
/// let mut solver = TrussSolver::from_model(&model)?;
/// solver.solve_configured()?;
/// let ux = solver.displacement(1, TrussDof::Ux)?;
/// assert!((ux - 10.0).abs() < 1e-9); // delta = FL/EA = 10
/// # Ok(())
/// # }
/// ```
pub struct TrussSolver {
    k_global: SparseMatrix,
    f_global: Vec<f64>,
    u_global: Vec<f64>,
    fixed_dofs: Vec<bool>,
    prescribed_values: Vec<Option<f64>>,
    k_original: SparseMatrix,
    n_dof: usize,
    model: TrussModel,
    solver_selection: SolverSelection,
    solver_name: Option<String>,
    reduced: Option<ReducedSystem>,
}

impl TrussSolver {
    /// Create a solver from a `TrussModel`.
    ///
    /// Clones the model, assembles the global stiffness matrix, and applies
    /// boundary conditions (static condensation).
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidModel`] if the model has no nodes, or if any element
    /// references an out-of-bounds node or has zero length.
    pub fn from_model(model: &TrussModel) -> Result<Self, FemError> {
        let n_dof = model.n_dof();
        if n_dof == 0 {
            return Err(FemError::InvalidModel("model has no nodes".to_string()));
        }

        for (idx, el) in model.elements.iter().enumerate() {
            if el.node_i >= model.nodes.len() {
                return Err(FemError::InvalidModel(format!(
                    "element {idx}: node_i {} out of bounds (max {})",
                    el.node_i,
                    model.nodes.len().saturating_sub(1)
                )));
            }
            if el.node_j >= model.nodes.len() {
                return Err(FemError::InvalidModel(format!(
                    "element {idx}: node_j {} out of bounds (max {})",
                    el.node_j,
                    model.nodes.len().saturating_sub(1)
                )));
            }
            let pi = model.nodes[el.node_i].point();
            let pj = model.nodes[el.node_j].point();
            if el.length(pi, pj) <= 0.0 {
                return Err(FemError::InvalidModel(format!(
                    "element {idx}: zero length (nodes {} and {} coincide)",
                    el.node_i, el.node_j
                )));
            }
        }

        let mut k_global = SparseMatrix::new(n_dof);
        for el in &model.elements {
            let pi = model.nodes[el.node_i].point();
            let pj = model.nodes[el.node_j].point();
            let k_e = el.global_stiffness(pi, pj)?;
            let dof_map = [
                model.dof_index(el.node_i, 0),
                model.dof_index(el.node_i, 1),
                model.dof_index(el.node_j, 0),
                model.dof_index(el.node_j, 1),
            ];
            for a in 0..4 {
                for b in 0..4 {
                    if k_e[a][b] != 0.0 {
                        k_global.add(dof_map[a], dof_map[b], k_e[a][b]);
                    }
                }
            }
        }
        k_global.compress();

        let mut f_global = vec![0.0; n_dof];
        for &(node_idx, dof, value) in &model.nodal_forces {
            let idx = model.dof_index(node_idx, dof);
            f_global[idx] += value;
        }

        let mut fixed_dofs = vec![false; n_dof];
        let mut prescribed_values = vec![None; n_dof];
        for &(node_idx, dof, value) in &model.fixed_dofs {
            let idx = model.dof_index(node_idx, dof);
            fixed_dofs[idx] = true;
            prescribed_values[idx] = Some(value);
        }

        let k_original = k_global.clone();

        Ok(Self {
            k_global,
            f_global,
            u_global: vec![0.0; n_dof],
            fixed_dofs,
            prescribed_values,
            k_original,
            n_dof,
            model: model.clone(),
            solver_selection: SolverSelection::Auto,
            solver_name: None,
            reduced: None,
        })
    }

    /// Configure the solver backend.
    pub fn set_solver(&mut self, selection: SolverSelection) {
        self.solver_selection = selection;
    }

    /// The configured solver selection.
    pub fn solver_selection(&self) -> &SolverSelection {
        &self.solver_selection
    }

    /// Name of the backend used by the most recent successful solve.
    pub fn solver_name(&self) -> Option<&str> {
        self.solver_name.as_deref()
    }

    fn apply_boundary_conditions(&mut self) -> Result<(ReducedSystem, Vec<f64>), FemError> {
        let n = self.n_dof;
        let mut free_to_global = Vec::new();
        let mut global_to_free = vec![None; n];
        for i in 0..n {
            if !self.fixed_dofs[i] {
                global_to_free[i] = Some(free_to_global.len());
                free_to_global.push(i);
            }
        }
        let n_free = free_to_global.len();
        if n_free == 0 {
            return Err(FemError::InvalidModel(
                "all DOFs are constrained — no free DOFs".to_string(),
            ));
        }

        let mut k_ff = SparseMatrix::new(n_free);
        let mut f_reduced = vec![0.0; n_free];

        let mut k_orig = self.k_global.clone();
        k_orig.compress();
        let row_ptr = k_orig.row_ptr();
        let csr_cols = k_orig.csr_cols();
        let csr_vals = k_orig.csr_vals();

        for (free_idx, &global_i) in free_to_global.iter().enumerate() {
            f_reduced[free_idx] = self.f_global[global_i];
            for idx in row_ptr[global_i]..row_ptr[global_i + 1] {
                let global_j = csr_cols[idx];
                let val = csr_vals[idx];
                if let Some(free_j) = global_to_free[global_j] {
                    k_ff.add(free_idx, free_j, val);
                } else {
                    let u_c = self.prescribed_values[global_j].unwrap_or(0.0);
                    f_reduced[free_idx] -= val * u_c;
                }
            }
        }
        k_ff.compress();

        Ok((
            ReducedSystem {
                k_ff,
                free_to_global,
            },
            f_reduced,
        ))
    }

    /// Solve with a custom `LinearSolver` backend.
    ///
    /// # Errors
    ///
    /// [`FemError::SolverError`] if the linear solver fails (singular matrix, etc.).
    pub fn solve(&mut self, solver: &mut dyn LinearSolver) -> Result<(), FemError> {
        let (reduced, f_reduced) = self.apply_boundary_conditions()?;
        let free_to_global = reduced.free_to_global.clone();

        solver.factor(&reduced.k_ff)?;
        let u_free = solver.solve(&f_reduced)?;

        self.u_global.fill(0.0);
        for (free_idx, &global_idx) in free_to_global.iter().enumerate() {
            self.u_global[global_idx] = u_free[free_idx];
        }
        for i in 0..self.n_dof {
            if self.fixed_dofs[i] {
                self.u_global[i] = self.prescribed_values[i].unwrap_or(0.0);
            }
        }

        self.reduced = Some(reduced);
        Ok(())
    }

    /// Solve using the configured (or auto-selected) backend.
    ///
    /// # Errors
    ///
    /// [`FemError::SolverError`] if the linear solver fails.
    pub fn solve_configured(&mut self) -> Result<(), FemError> {
        self.solver_name = None;
        let (reduced, f_reduced) = self.apply_boundary_conditions()?;
        let n_free = reduced.k_ff.n;
        let free_to_global = reduced.free_to_global.clone();

        if n_free == 0 {
            self.u_global.fill(0.0);
            for i in 0..self.n_dof {
                if self.fixed_dofs[i] {
                    self.u_global[i] = self.prescribed_values[i].unwrap_or(0.0);
                }
            }
            return Ok(());
        }

        let registry = SolverRegistry::default();
        let mut solver = registry
            .create_selected(&reduced.k_ff, &self.solver_selection)
            .map_err(FemError::from)?;
        let name = solver.name().to_string();

        solver.factor(&reduced.k_ff)?;
        let u_free = solver.solve(&f_reduced)?;

        self.u_global.fill(0.0);
        for (free_idx, &global_idx) in free_to_global.iter().enumerate() {
            self.u_global[global_idx] = u_free[free_idx];
        }
        for i in 0..self.n_dof {
            if self.fixed_dofs[i] {
                self.u_global[i] = self.prescribed_values[i].unwrap_or(0.0);
            }
        }

        self.reduced = Some(reduced);
        self.solver_name = Some(name);
        Ok(())
    }

    /// Full displacement vector `[ux, uy]` per node.
    pub fn displacements(&self) -> &[f64] {
        &self.u_global
    }

    /// Displacement of a single DOF.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if `node_idx` is out of bounds.
    pub fn displacement(&self, node_idx: usize, dof: TrussDof) -> Result<f64, FemError> {
        if node_idx >= self.model.nodes.len() {
            return Err(FemError::InvalidNode(format!(
                "node index {node_idx} out of bounds (max {})",
                self.model.nodes.len().saturating_sub(1)
            )));
        }
        Ok(self.u_global[self.model.dof_index(node_idx, dof.index())])
    }

    /// Global reactions `R = K·u - f`, laid out `[Rx, Ry]` per node.
    pub fn reactions(&self) -> Vec<f64> {
        let mut reactions = vec![0.0; self.n_dof];
        let ku = self.k_original.matvec(&self.u_global);
        for i in 0..self.n_dof {
            reactions[i] = ku[i] - self.f_global[i];
        }
        reactions
    }

    /// Reaction at a single DOF.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if `node_idx` is out of bounds.
    pub fn reaction(&self, node_idx: usize, dof: TrussDof) -> Result<f64, FemError> {
        if node_idx >= self.model.nodes.len() {
            return Err(FemError::InvalidNode(format!(
                "node index {node_idx} out of bounds (max {})",
                self.model.nodes.len().saturating_sub(1)
            )));
        }
        Ok(self.reactions()[self.model.dof_index(node_idx, dof.index())])
    }

    /// Axial force in element `element_idx`.
    ///
    /// **Tension positive**: a positive value means the element is in tension.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidMember`] if `element_idx` is out of bounds.
    pub fn axial_force(&self, element_idx: usize) -> Result<f64, FemError> {
        if element_idx >= self.model.elements.len() {
            return Err(FemError::InvalidMember(format!(
                "element index {element_idx} out of bounds (max {})",
                self.model.elements.len().saturating_sub(1)
            )));
        }
        let el = &self.model.elements[element_idx];
        let pi = self.model.nodes[el.node_i].point();
        let pj = self.model.nodes[el.node_j].point();
        let u = [
            self.u_global[self.model.dof_index(el.node_i, 0)],
            self.u_global[self.model.dof_index(el.node_i, 1)],
            self.u_global[self.model.dof_index(el.node_j, 0)],
            self.u_global[self.model.dof_index(el.node_j, 1)],
        ];
        el.axial_force(pi, pj, u)
    }

    /// Axial forces for all elements.
    pub fn axial_forces(&self) -> Result<Vec<f64>, FemError> {
        (0..self.model.elements.len())
            .map(|i| self.axial_force(i))
            .collect()
    }

    /// Run mechanism diagnostics on the condensed free-free system.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidModel`] if the system has not been solved yet.
    pub fn diagnostic(&self) -> Result<StructuralDiagnostic, FemError> {
        let reduced = self.reduced.as_ref().ok_or_else(|| {
            FemError::InvalidModel("no reduced system — call solve first".to_string())
        })?;

        let candidates = self.rigid_candidates(&reduced.free_to_global);
        Ok(diagnose_reduced(&reduced.k_ff, &candidates))
    }

    /// Build 2D rigid-body mode candidates restricted to the free DOFs.
    ///
    /// For 2 DOF/node: `global / 2` gives the node index, `global % 2` gives
    /// `0 = ux`, `1 = uy`. Three modes: Tx, Ty, Rz (rotation about origin).
    fn rigid_candidates(&self, free_to_global: &[usize]) -> Vec<Vec<f64>> {
        let n_free = free_to_global.len();
        let mut tx = vec![0.0; n_free];
        let mut ty = vec![0.0; n_free];
        let mut rz = vec![0.0; n_free];
        for (r, &global) in free_to_global.iter().enumerate() {
            let node = global / 2;
            let p = self.model.nodes[node].point();
            match global % 2 {
                0 => {
                    tx[r] = 1.0;
                    rz[r] = -p.y;
                }
                1 => {
                    ty[r] = 1.0;
                    rz[r] = p.x;
                }
                _ => {}
            }
        }
        vec![tx, ty, rz]
    }

    /// Borrow the underlying model.
    pub fn model(&self) -> &TrussModel {
        &self.model
    }
}

// ---------------------------------------------------------------------------
// Analysis result
// ---------------------------------------------------------------------------

/// Result of a truss analysis: displacements, reactions, axial forces, and
/// geometry snapshot for post-processing.
#[derive(Debug, Clone)]
pub struct TrussAnalysisResult {
    /// Full displacement vector `[ux, uy]` per node.
    pub displacements: Vec<f64>,
    /// Global reactions `[Rx, Ry]` per node.
    pub reactions: Vec<f64>,
    /// Axial force per element (tension positive).
    pub axial_forces: Vec<f64>,
    /// Name of the solver backend used.
    pub solver_name: Option<String>,
    /// Node coordinates `(x, y)`, same order as the model.
    pub node_coords: Vec<(f64, f64)>,
    /// Element connectivity `(node_i, node_j)`, same order as the model.
    pub element_nodes: Vec<(usize, usize)>,
    /// Nodal forces `(node_idx, dof, value)` that were applied.
    pub nodal_forces: Vec<(usize, usize, f64)>,
    /// Provenance of the load that produced this result.
    load_source: LoadSource,
    n_nodes: usize,
    n_elements: usize,
}

impl TrussAnalysisResult {
    /// Number of nodes.
    pub fn n_nodes(&self) -> usize {
        self.n_nodes
    }

    /// Number of elements.
    pub fn n_elements(&self) -> usize {
        self.n_elements
    }

    /// Solver backend name.
    pub fn solver_name(&self) -> Option<&str> {
        self.solver_name.as_deref()
    }

    /// Provenance of the load that produced this result.
    pub fn load_source(&self) -> &LoadSource {
        &self.load_source
    }

    /// Coordinates of a node `(x, y)`, or `None` if out of range.
    pub fn node_position(&self, node_index: usize) -> Option<(f64, f64)> {
        self.node_coords.get(node_index).copied()
    }

    /// End-node indices `(node_i, node_j)` of an element, or `None` if out
    /// of range.
    pub fn element_endpoints(&self, element_index: usize) -> Option<(usize, usize)> {
        self.element_nodes.get(element_index).copied()
    }

    /// Global equilibrium check: `ΣFx`, `ΣFy`, `ΣMz` about the origin.
    ///
    /// Combines applied nodal forces and support reactions. Truss elements
    /// carry only axial force, so there are no member-level distributed or
    /// point loads to include.
    ///
    /// Returns a [`TrussEquilibriumReport`] with force residuals and a
    /// balanced verdict. Moment equilibrium (`ΣMz`) uses the node coordinates
    /// stored in the result snapshot.
    pub fn equilibrium(&self) -> TrussEquilibriumReport {
        let mut applied_fx = 0.0;
        let mut applied_fy = 0.0;
        let mut applied_mz = 0.0;
        let mut applied_f_mag = 0.0;

        for &(node, dof, v) in &self.nodal_forces {
            let (x, y) = self.node_coords.get(node).copied().unwrap_or((0.0, 0.0));
            match dof {
                0 => {
                    applied_fx += v;
                    applied_mz += -y * v;
                    applied_f_mag += v.abs();
                }
                1 => {
                    applied_fy += v;
                    applied_mz += x * v;
                    applied_f_mag += v.abs();
                }
                _ => {}
            }
        }

        let mut reaction_fx = 0.0;
        let mut reaction_fy = 0.0;
        let mut reaction_mz = 0.0;
        let mut reaction_f_mag = 0.0;
        for (idx, &(x, y)) in self.node_coords.iter().enumerate() {
            let rx = self.reactions.get(2 * idx).copied().unwrap_or(0.0);
            let ry = self.reactions.get(2 * idx + 1).copied().unwrap_or(0.0);
            reaction_fx += rx;
            reaction_fy += ry;
            reaction_mz += x * ry - y * rx;
            reaction_f_mag += rx.abs() + ry.abs();
        }

        let f_mag = applied_f_mag + reaction_f_mag;
        let l_char: f64 = self
            .node_coords
            .iter()
            .map(|&(x, y)| x.abs().max(y.abs()))
            .fold(0.0_f64, f64::max);
        let rel = 1e-6;
        let f_scale = if l_char > 0.0 { f_mag } else { f_mag };

        TrussEquilibriumReport {
            fx_residual: applied_fx + reaction_fx,
            fy_residual: applied_fy + reaction_fy,
            mz_residual: applied_mz + reaction_mz,
            applied_fx,
            applied_fy,
            applied_mz,
            reaction_fx,
            reaction_fy,
            reaction_mz,
            tolerance: rel * f_scale,
        }
    }
}

/// Equilibrium report for a truss analysis.
///
/// Truss nodes have 2 DOF (`ux`, `uy`), so there are no reaction moments.
/// Moment equilibrium is checked about the global origin using node
/// coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrussEquilibriumReport {
    /// Residual of `ΣFx` (applied + reaction).
    pub fx_residual: f64,
    /// Residual of `ΣFy` (applied + reaction).
    pub fy_residual: f64,
    /// Residual of `ΣMz` about the origin (applied + reaction).
    pub mz_residual: f64,
    /// Total applied horizontal load.
    pub applied_fx: f64,
    /// Total applied vertical load.
    pub applied_fy: f64,
    /// Total applied moment about the origin.
    pub applied_mz: f64,
    /// Sum of support reaction forces in X.
    pub reaction_fx: f64,
    /// Sum of support reaction forces in Y.
    pub reaction_fy: f64,
    /// Sum of reaction moments about the origin.
    pub reaction_mz: f64,
    /// Force tolerance: `1e-6 * Σ|F|`.
    pub tolerance: f64,
}

impl TrussEquilibriumReport {
    /// Whether all three residuals are within relative tolerance.
    pub fn is_balanced(&self) -> bool {
        let rel = 1e-6;
        self.fx_residual.abs() <= self.tolerance
            && self.fy_residual.abs() <= self.tolerance
            && self.mz_residual.abs() <= self.tolerance.max(rel * self.tolerance)
    }

    /// Force tolerance used by [`Self::is_balanced`].
    pub fn force_tolerance(&self) -> f64 {
        self.tolerance
    }
}

impl TrussSolver {
    /// Collect all results into a [`TrussAnalysisResult`].
    ///
    /// # Errors
    ///
    /// Propagates errors from [`Self::axial_forces`].
    pub fn results(&self) -> Result<TrussAnalysisResult, FemError> {
        Ok(TrussAnalysisResult {
            displacements: self.u_global.clone(),
            reactions: self.reactions(),
            axial_forces: self.axial_forces()?,
            solver_name: self.solver_name.clone(),
            node_coords: self.model.nodes.iter().map(|n| (n.x, n.y)).collect(),
            element_nodes: self
                .model
                .elements
                .iter()
                .map(|e| (e.node_i, e.node_j))
                .collect(),
            nodal_forces: self.model.nodal_forces.clone(),
            load_source: LoadSource::ModelLoads,
            n_nodes: self.model.nodes.len(),
            n_elements: self.model.elements.len(),
        })
    }
}
