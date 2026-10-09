//! 3D truss finite element analysis.
//!
//! Pin-jointed 3D truss elements with 3 DOF per node: `[ux, uy, uz]`.
//! Each element carries only axial force (no bending, no shear, no torsion).
//!
//! # Relationship to [`crate::truss`]
//!
//! - [`truss`](crate::truss) — 2D truss (2 DOF/node, `ux`, `uy`).
//! - [`truss3d`] — 3D truss (3 DOF/node, `ux`, `uy`, `uz`).
//!
//! The two modules are independent: separate types, no generic dimension
//! abstraction, no shared element trait.  They reuse only the genuinely
//! dimension-independent infrastructure ([`SparseMatrix`],
//! [`LinearSolver`], [`diagnose_reduced`]).
//!
//! # Mathematical model
//!
//! For an element with direction cosines `lx = dx/L`, `ly = dy/L`,
//! `lz = dz/L` and axial stiffness `EA/L`, the 6×6 global stiffness is:
//!
//! ```text
//! EA/L * [
//!  lx²   lx·ly lx·lz -lx²   -lx·ly -lx·lz
//!  lx·ly ly²   ly·lz -lx·ly -ly²   -ly·lz
//!  lx·lz ly·lz lz²   -lx·lz -ly·lz -lz²
//!  -lx²  -lx·ly -lx·lz lx²   lx·ly  lx·lz
//!  -lx·ly -ly²  -ly·lz lx·ly ly²    ly·lz
//!  -lx·lz -ly·lz -lz²  lx·lz ly·lz  lz²
//! ]
//! ```
//!
//! # DOF ordering
//!
//! Each node has 3 global DOFs: `dof(node, d) = 3·node + d`.
//!
//! ```text
//! node 0: ux=0, uy=1, uz=2
//! node 1: ux=3, uy=4, uz=5
//! ...
//! ```
//!
//! # Sign convention
//!
//! - **Axial force**: tension positive (element pulled apart along
//!   `node_i → node_j`).
//! - **Nodal loads**: global coordinates (`+Fx → +X`, `+Fy → +Y`,
//!   `+Fz → +Z`).
//! - **Reactions**: `R = K·u − F` (global).
//!
//! # MVP scope
//!
//! - 3 DOF per node, axial-only formulation.
//! - No bending, no torsion, no shear.
//! - No member distributed loads.
//! - No self-weight.
//! - `Envelope` support via [`Truss3DEnvelope`](crate::postprocessing::Truss3DEnvelope).
//! - No 3D Frame.
//! - No generic dimension abstraction.

#![allow(non_snake_case)]

use section_properties::fea::{
    SparseMatrix,
    solver::{LinearSolver, SolverRegistry, SolverSelection},
};
use section_properties::material::Material;

use crate::beam_fem::FemError;
use crate::frame::{load_case_source, load_combination_source};
use crate::load::{LoadCase, LoadCombination, LoadSource};
use crate::mechanism::{StructuralDiagnostic, diagnose_reduced};

// ---------------------------------------------------------------------------
// DOF
// ---------------------------------------------------------------------------

/// A translational degree of freedom of a 3D truss node.
///
/// 3D truss nodes have 3 DOF: `Ux`, `Uy`, `Uz` (no rotation).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TrussDof3D {
    /// Horizontal displacement along X.
    Ux,
    /// Vertical displacement along Y.
    Uy,
    /// Displacement along Z.
    Uz,
}

impl TrussDof3D {
    /// All DOFs in canonical order `[Ux, Uy, Uz]`.
    pub const ALL: [TrussDof3D; 3] = [TrussDof3D::Ux, TrussDof3D::Uy, TrussDof3D::Uz];

    /// Raw index: `Ux → 0`, `Uy → 1`, `Uz → 2`.
    pub const fn index(self) -> usize {
        match self {
            TrussDof3D::Ux => 0,
            TrussDof3D::Uy => 1,
            TrussDof3D::Uz => 2,
        }
    }

    /// Human-readable name (`"ux"`, `"uy"`, `"uz"`).
    pub const fn name(self) -> &'static str {
        match self {
            TrussDof3D::Ux => "ux",
            TrussDof3D::Uy => "uy",
            TrussDof3D::Uz => "uz",
        }
    }
}

// ---------------------------------------------------------------------------
// Node
// ---------------------------------------------------------------------------

/// 3D truss node with 3 DOF: `[ux, uy, uz]`.
#[derive(Debug, Clone, Copy)]
pub struct TrussNode3D {
    /// Node index (position in the model's node list).
    pub id: usize,
    /// X coordinate `m`.
    pub x: f64,
    /// Y coordinate `m`.
    pub y: f64,
    /// Z coordinate `m`.
    pub z: f64,
}

impl TrussNode3D {
    /// Create a node at `(x, y, z)` with the given `id`.
    pub fn new(id: usize, x: f64, y: f64, z: f64) -> Self {
        Self { id, x, y, z }
    }

    /// Return coordinates as `(x, y, z)`.
    pub fn coords(&self) -> (f64, f64, f64) {
        (self.x, self.y, self.z)
    }
}

// ---------------------------------------------------------------------------
// Element
// ---------------------------------------------------------------------------

/// 3D pin-jointed truss element (axial-only, 6 DOF).
///
/// # Sign convention
///
/// Axial force is **tension positive**: a positive value means the element is
/// being pulled apart along the `node_i → node_j` direction.
#[derive(Debug, Clone)]
pub struct TrussElement3D {
    /// Index of the first node.
    pub node_i: usize,
    /// Index of the second node.
    pub node_j: usize,
    /// Young's modulus `Pa`.
    pub E: f64,
    /// Cross-sectional area `m²`.
    pub A: f64,
}

impl TrussElement3D {
    /// Create a 3D truss element connecting `node_i` and `node_j`.
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
    pub fn length(&self, pi: (f64, f64, f64), pj: (f64, f64, f64)) -> f64 {
        let dx = pj.0 - pi.0;
        let dy = pj.1 - pi.1;
        let dz = pj.2 - pi.2;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Direction cosines `(lx, ly, lz)` where `lx = dx/L`, etc.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if the element has zero length.
    pub fn direction(
        &self,
        pi: (f64, f64, f64),
        pj: (f64, f64, f64),
    ) -> Result<(f64, f64, f64), FemError> {
        let l = self.length(pi, pj);
        if l <= 0.0 {
            return Err(FemError::InvalidInput(
                "truss element has zero length".to_string(),
            ));
        }
        Ok(((pj.0 - pi.0) / l, (pj.1 - pi.1) / l, (pj.2 - pi.2) / l))
    }

    /// 6×6 global stiffness matrix.
    ///
    /// DOF ordering: `[ux_i, uy_i, uz_i, ux_j, uy_j, uz_j]` in **global**
    /// coordinates.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if the element has zero length.
    pub fn global_stiffness(
        &self,
        pi: (f64, f64, f64),
        pj: (f64, f64, f64),
    ) -> Result<[[f64; 6]; 6], FemError> {
        let l = self.length(pi, pj);
        if l <= 0.0 {
            return Err(FemError::InvalidInput(
                "truss element has zero length for stiffness".to_string(),
            ));
        }
        let (lx, ly, lz) = self.direction(pi, pj)?;
        let k = self.E * self.A / l;

        let lx2 = lx * lx;
        let ly2 = ly * ly;
        let lz2 = lz * lz;
        let lxly = lx * ly;
        let lxlz = lx * lz;
        let lylz = ly * lz;

        Ok([
            [k * lx2, k * lxly, k * lxlz, -k * lx2, -k * lxly, -k * lxlz],
            [k * lxly, k * ly2, k * lylz, -k * lxly, -k * ly2, -k * lylz],
            [k * lxlz, k * lylz, k * lz2, -k * lxlz, -k * lylz, -k * lz2],
            [-k * lx2, -k * lxly, -k * lxlz, k * lx2, k * lxly, k * lxlz],
            [-k * lxly, -k * ly2, -k * lylz, k * lxly, k * ly2, k * lylz],
            [-k * lxlz, -k * lylz, -k * lz2, k * lxlz, k * lylz, k * lz2],
        ])
    }

    /// Axial force from the 6 global nodal displacements
    /// `[ux_i, uy_i, uz_i, ux_j, uy_j, uz_j]`.
    ///
    /// **Tension positive**:
    /// `N = EA/L · (lx·Δux + ly·Δuy + lz·Δuz)`
    /// where `Δu = u_j − u_i`.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if the element has zero length.
    pub fn axial_force(
        &self,
        pi: (f64, f64, f64),
        pj: (f64, f64, f64),
        u: [f64; 6],
    ) -> Result<f64, FemError> {
        let l = self.length(pi, pj);
        if l <= 0.0 {
            return Err(FemError::InvalidInput(
                "truss element has zero length for axial force".to_string(),
            ));
        }
        let (lx, ly, lz) = self.direction(pi, pj)?;
        let delta = lx * (u[3] - u[0]) + ly * (u[4] - u[1]) + lz * (u[5] - u[2]);
        Ok(self.E * self.A / l * delta)
    }
}

// ---------------------------------------------------------------------------
// Model
// ---------------------------------------------------------------------------

/// 3D truss model: nodes, elements, loads, and boundary conditions.
///
/// # Conventions
///
/// - Each node has 3 **global** DOFs `[ux, uy, uz]`.
/// - DOF mapping: `dof(node, d) = 3·node + d`.
/// - Nodal loads are in **global** coordinates.
/// - Boundary conditions are enforced by static condensation (no penalties).
///
/// # Snapshot semantics
///
/// [`TrussSolver3D::from_model`] clones the model. Mutating a
/// `TrussModel3D` after building a solver does not affect that solver.
#[derive(Debug, Clone)]
pub struct TrussModel3D {
    /// Nodes.
    pub nodes: Vec<TrussNode3D>,
    /// Elements.
    pub elements: Vec<TrussElement3D>,
    /// Nodal forces: `(node_idx, dof, value)`.
    pub nodal_forces: Vec<(usize, usize, f64)>,
    /// Fixed DOFs: `(node_idx, dof, prescribed_value)`.
    pub fixed_dofs: Vec<(usize, usize, f64)>,
}

impl Default for TrussModel3D {
    fn default() -> Self {
        Self::new()
    }
}

impl TrussModel3D {
    /// Create an empty 3D truss model.
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            elements: Vec::new(),
            nodal_forces: Vec::new(),
            fixed_dofs: Vec::new(),
        }
    }

    /// Add a node to the model.
    pub fn add_node(&mut self, node: TrussNode3D) {
        self.nodes.push(node);
    }

    /// Add a truss element to the model.
    pub fn add_element(&mut self, element: TrussElement3D) {
        self.elements.push(element);
    }

    /// Apply a nodal force `(fx, fy, fz)` at `node_idx` in **global**
    /// coordinates.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if `node_idx` is out of bounds.
    /// [`FemError::InvalidInput`] if any component is non-finite.
    pub fn add_nodal_force(
        &mut self,
        node_idx: usize,
        fx: f64,
        fy: f64,
        fz: f64,
    ) -> Result<(), FemError> {
        if node_idx >= self.nodes.len() {
            return Err(FemError::InvalidNode(format!(
                "node index {node_idx} out of bounds (max {})",
                self.nodes.len().saturating_sub(1)
            )));
        }
        if !fx.is_finite() || !fy.is_finite() || !fz.is_finite() {
            return Err(FemError::InvalidInput(format!(
                "nodal force must be finite, got fx = {fx}, fy = {fy}, fz = {fz}"
            )));
        }
        self.nodal_forces.push((node_idx, 0, fx));
        self.nodal_forces.push((node_idx, 1, fy));
        self.nodal_forces.push((node_idx, 2, fz));
        Ok(())
    }

    /// Restrain a single DOF of a node to a prescribed `value` (usually 0.0).
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if `node_idx` is out of bounds.
    /// [`FemError::InvalidInput`] if `value` is non-finite.
    pub fn fix_dof(
        &mut self,
        node_idx: usize,
        dof: TrussDof3D,
        value: f64,
    ) -> Result<(), FemError> {
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

    /// Fix a node: restrain `ux`, `uy`, and `uz` to zero.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if `node_idx` is out of bounds.
    pub fn fix_node(&mut self, node_idx: usize) -> Result<(), FemError> {
        self.fix_dof(node_idx, TrussDof3D::Ux, 0.0)?;
        self.fix_dof(node_idx, TrussDof3D::Uy, 0.0)?;
        self.fix_dof(node_idx, TrussDof3D::Uz, 0.0)
    }

    /// Total number of DOFs.
    pub fn n_dof(&self) -> usize {
        self.nodes.len() * 3
    }

    /// Global DOF index: `3·node + dof`.
    pub(crate) fn dof_index(&self, node_idx: usize, dof: usize) -> usize {
        node_idx * 3 + dof
    }

    /// Solve the 3D truss under a single [`LoadCase`].
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
    /// The result's [`load_source`](TrussAnalysisResult3D::load_source) is
    /// `LoadSource::LoadCase`.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if a load or prescribed displacement in the
    /// case references a node that does not exist.
    /// [`FemError::InvalidModel`] if the model is invalid (see
    /// [`TrussSolver3D::from_model`]).
    /// [`FemError::SolverError`] if the linear solve fails.
    pub fn solve_case(&self, case: &LoadCase) -> Result<TrussAnalysisResult3D, FemError> {
        for &(node_idx, _, _) in case.nodal_forces() {
            if node_idx >= self.nodes.len() {
                return Err(FemError::InvalidNode(format!(
                    "load case '{}' references node {node_idx} out of bounds (max {})",
                    case.name(),
                    self.nodes.len().saturating_sub(1)
                )));
            }
        }
        for &(node_idx, _, _) in case.prescribed_displacements() {
            if node_idx >= self.nodes.len() {
                return Err(FemError::InvalidNode(format!(
                    "load case '{}' prescribes displacement at node {node_idx} out of bounds (max {})",
                    case.name(),
                    self.nodes.len().saturating_sub(1)
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

        let mut solver = TrussSolver3D::from_model(&model)?;
        solver.solve_configured()?;
        let mut result = solver.results()?;
        result.load_source = load_case_source(case);
        Ok(result)
    }

    /// Solve the 3D truss under a [`LoadCombination`].
    ///
    /// The right-hand side is merged: `f = Σ factor_i × f_case_i`, then
    /// `K u = f` is solved **once**. This is mathematically equivalent to
    /// solving each case separately and superposing the results (by linearity
    /// of the solve), but more efficient (a single factorisation).
    ///
    /// The result's [`load_source`](TrussAnalysisResult3D::load_source) is
    /// `LoadSource::LoadCombination` and records every scaled term.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if any case in the combination contains
    /// prescribed displacements (load combinations do not support prescribed
    /// displacement combination).
    /// [`FemError::InvalidNode`] if a load in any case references a node that
    /// does not exist.
    /// [`FemError::InvalidModel`] if the model is invalid.
    /// [`FemError::SolverError`] if the linear solve fails.
    pub fn solve_combination(
        &self,
        combo: &LoadCombination,
    ) -> Result<TrussAnalysisResult3D, FemError> {
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
                model.nodal_forces.push((node_idx, dof, factor * value));
            }
        }

        let mut solver = TrussSolver3D::from_model(&model)?;
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
struct ReducedSystem3D {
    k_ff: SparseMatrix,
    free_to_global: Vec<usize>,
}

/// 3D truss solver: assembles the global system, applies boundary conditions,
/// and delegates the linear solve to a `LinearSolver` backend.
///
/// # Workflow
///
/// ```rust
/// use structural_analysis::truss3d::{
///     TrussElement3D, TrussModel3D, TrussNode3D, TrussSolver3D, TrussDof3D,
/// };
/// use section_properties::Material;
///
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// let mut model = TrussModel3D::new();
/// model.add_node(TrussNode3D::new(0, 0.0, 0.0, 0.0));
/// model.add_node(TrussNode3D::new(1, 1.0, 0.0, 0.0));
/// model.add_element(TrussElement3D::new(0, 1, &Material::new(1.0, 0.3, 1.0, "unit"), 1.0)?);
/// model.fix_node(0)?;
/// model.fix_dof(1, TrussDof3D::Uy, 0.0)?;
/// model.fix_dof(1, TrussDof3D::Uz, 0.0)?;
/// model.add_nodal_force(1, 10.0, 0.0, 0.0)?;
///
/// let mut solver = TrussSolver3D::from_model(&model)?;
/// solver.solve_configured()?;
/// let ux = solver.displacement(1, TrussDof3D::Ux)?;
/// assert!((ux - 10.0).abs() < 1e-9); // delta = FL/EA = 10
/// # Ok(())
/// # }
/// ```
pub struct TrussSolver3D {
    k_global: SparseMatrix,
    f_global: Vec<f64>,
    u_global: Vec<f64>,
    fixed_dofs: Vec<bool>,
    prescribed_values: Vec<Option<f64>>,
    k_original: SparseMatrix,
    n_dof: usize,
    model: TrussModel3D,
    solver_selection: SolverSelection,
    solver_name: Option<String>,
}

impl TrussSolver3D {
    /// Create a solver from a `TrussModel3D`.
    ///
    /// Clones the model, assembles the global stiffness matrix, and applies
    /// boundary conditions (static condensation).
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidModel`] if the model has no nodes, or if any element
    /// references an out-of-bounds node or has zero length.
    pub fn from_model(model: &TrussModel3D) -> Result<Self, FemError> {
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
            let pi = model.nodes[el.node_i].coords();
            let pj = model.nodes[el.node_j].coords();
            if el.length(pi, pj) <= 0.0 {
                return Err(FemError::InvalidModel(format!(
                    "element {idx}: zero length (nodes {} and {} coincide)",
                    el.node_i, el.node_j
                )));
            }
        }

        let mut k_global = SparseMatrix::new(n_dof);
        for el in &model.elements {
            let pi = model.nodes[el.node_i].coords();
            let pj = model.nodes[el.node_j].coords();
            let k_e = el.global_stiffness(pi, pj)?;
            let dof_map = [
                model.dof_index(el.node_i, 0),
                model.dof_index(el.node_i, 1),
                model.dof_index(el.node_i, 2),
                model.dof_index(el.node_j, 0),
                model.dof_index(el.node_j, 1),
                model.dof_index(el.node_j, 2),
            ];
            for a in 0..6 {
                for b in 0..6 {
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

    /// Compute the reduced (free-free) system and reduced RHS via static
    /// condensation.
    ///
    /// For prescribed displacements `u_c ≠ 0` the RHS is corrected:
    /// `F_f' = F_f − K_fc · u_c`.
    fn compute_reduced(&self) -> Result<(ReducedSystem3D, Vec<f64>), FemError> {
        let n = self.n_dof;
        let mut free_to_global = Vec::new();
        let mut global_to_free = vec![None; n];
        for (i, gf) in global_to_free.iter_mut().enumerate() {
            if !self.fixed_dofs[i] {
                *gf = Some(free_to_global.len());
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
            ReducedSystem3D {
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
    /// [`FemError::SolverError`] if the linear solver fails (singular matrix,
    /// etc.).
    pub fn solve(&mut self, solver: &mut dyn LinearSolver) -> Result<(), FemError> {
        let (reduced, f_reduced) = self.compute_reduced()?;
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

        Ok(())
    }

    /// Solve using the configured (or auto-selected) backend.
    ///
    /// # Errors
    ///
    /// [`FemError::SolverError`] if the linear solver fails.
    pub fn solve_configured(&mut self) -> Result<(), FemError> {
        self.solver_name = None;
        let (reduced, f_reduced) = self.compute_reduced()?;
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

        self.solver_name = Some(name);
        Ok(())
    }

    /// Full displacement vector `[ux, uy, uz]` per node.
    pub fn displacements(&self) -> &[f64] {
        &self.u_global
    }

    /// Displacement of a single DOF.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if `node_idx` is out of bounds.
    pub fn displacement(&self, node_idx: usize, dof: TrussDof3D) -> Result<f64, FemError> {
        if node_idx >= self.model.nodes.len() {
            return Err(FemError::InvalidNode(format!(
                "node index {node_idx} out of bounds (max {})",
                self.model.nodes.len().saturating_sub(1)
            )));
        }
        Ok(self.u_global[self.model.dof_index(node_idx, dof.index())])
    }

    /// Global reactions `R = K·u − f`, laid out `[Rx, Ry, Rz]` per node.
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
    pub fn reaction(&self, node_idx: usize, dof: TrussDof3D) -> Result<f64, FemError> {
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
        let pi = self.model.nodes[el.node_i].coords();
        let pj = self.model.nodes[el.node_j].coords();
        let u = [
            self.u_global[self.model.dof_index(el.node_i, 0)],
            self.u_global[self.model.dof_index(el.node_i, 1)],
            self.u_global[self.model.dof_index(el.node_i, 2)],
            self.u_global[self.model.dof_index(el.node_j, 0)],
            self.u_global[self.model.dof_index(el.node_j, 1)],
            self.u_global[self.model.dof_index(el.node_j, 2)],
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
    /// This can be called **without** a prior [`solve_configured`] call: it
    /// assembles the reduced system internally and classifies it with the
    /// shared [`diagnose_reduced`] function.
    ///
    /// The 3D truss has 6 rigid-body modes (Tx, Ty, Tz, Rx, Ry, Rz). A
    /// structure whose null space is entirely rigid-body motions is
    /// under-restrained ([`StructuralDiagnostic::RigidBodyMode`]); a null
    /// direction that is *not* a rigid-body motion is an internal mechanism
    /// ([`StructuralDiagnostic::Mechanism`]).
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidModel`] if the reduced system cannot be formed.
    ///
    /// [`solve_configured`]: Self::solve_configured
    pub fn diagnostic(&self) -> Result<StructuralDiagnostic, FemError> {
        let (reduced, _) = self.compute_reduced()?;
        let candidates = self.rigid_candidates(&reduced.free_to_global);
        Ok(diagnose_reduced(&reduced.k_ff, &candidates))
    }

    /// Build 3D rigid-body mode candidates restricted to the free DOFs.
    ///
    /// For 3 DOF/node: `global / 3` gives the node index, `global % 3` gives
    /// `0 = ux`, `1 = uy`, `2 = uz`. Six modes: Tx, Ty, Tz, Rx, Ry, Rz
    /// (rotations about the global origin).
    fn rigid_candidates(&self, free_to_global: &[usize]) -> Vec<Vec<f64>> {
        let n_free = free_to_global.len();
        let mut tx = vec![0.0; n_free];
        let mut ty = vec![0.0; n_free];
        let mut tz = vec![0.0; n_free];
        let mut rx = vec![0.0; n_free];
        let mut ry = vec![0.0; n_free];
        let mut rz = vec![0.0; n_free];
        for (r, &global) in free_to_global.iter().enumerate() {
            let node = global / 3;
            let (x, y, z) = self.model.nodes[node].coords();
            match global % 3 {
                0 => {
                    // ux
                    tx[r] = 1.0;
                    ry[r] = z;
                    rz[r] = -y;
                }
                1 => {
                    // uy
                    ty[r] = 1.0;
                    rx[r] = -z;
                    rz[r] = x;
                }
                2 => {
                    // uz
                    tz[r] = 1.0;
                    rx[r] = y;
                    ry[r] = -x;
                }
                _ => {}
            }
        }
        vec![tx, ty, tz, rx, ry, rz]
    }

    /// Borrow the underlying model.
    pub fn model(&self) -> &TrussModel3D {
        &self.model
    }
}

// ---------------------------------------------------------------------------
// Analysis result
// ---------------------------------------------------------------------------

/// Result of a 3D truss analysis: displacements, reactions, axial forces, and
/// geometry snapshot for post-processing.
///
/// This is an **owned snapshot** — it does not borrow the solver or model.
/// Queries on the result are cheap (array reads), with no recomputation of
/// the analysis.
#[derive(Debug, Clone)]
pub struct TrussAnalysisResult3D {
    /// Full displacement vector `[ux, uy, uz]` per node.
    pub displacements: Vec<f64>,
    /// Global reactions `[Rx, Ry, Rz]` per node.
    pub reactions: Vec<f64>,
    /// Axial force per element (tension positive).
    pub axial_forces: Vec<f64>,
    /// Name of the solver backend used.
    pub solver_name: Option<String>,
    /// Node coordinates `(x, y, z)`, same order as the model.
    pub node_coords: Vec<(f64, f64, f64)>,
    /// Element connectivity `(node_i, node_j)`, same order as the model.
    pub element_nodes: Vec<(usize, usize)>,
    /// Nodal forces `(node_idx, dof, value)` that were applied.
    pub nodal_forces: Vec<(usize, usize, f64)>,
    /// Constrained DOFs `(node_idx, dof)` — DOFs with physical support reactions.
    ///
    /// Populated from the solver's boundary conditions at result construction
    /// time. Free DOFs are excluded; only DOFs that carry a physical reaction
    /// appear here.
    pub constrained_dofs: Vec<(usize, usize)>,
    /// Provenance of the load that produced this result.
    load_source: LoadSource,
    n_nodes: usize,
    n_elements: usize,
}

impl TrussAnalysisResult3D {
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

    /// Constrained DOFs `(node_idx, dof)` — DOFs with physical support reactions.
    ///
    /// Only DOFs that were constrained during the solve appear here. Free DOFs
    /// are excluded because their "reaction" is a round-off residual, not a
    /// physical support force.
    pub fn constrained_dofs(&self) -> &[(usize, usize)] {
        &self.constrained_dofs
    }

    /// Coordinates of a node `(x, y, z)`, or `None` if out of range.
    pub fn node_position(&self, node_index: usize) -> Option<(f64, f64, f64)> {
        self.node_coords.get(node_index).copied()
    }

    /// End-node indices `(node_i, node_j)` of an element, or `None` if out
    /// of range.
    pub fn element_endpoints(&self, element_index: usize) -> Option<(usize, usize)> {
        self.element_nodes.get(element_index).copied()
    }

    /// Displacement of a single DOF at a node.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if `node_index` is out of bounds.
    pub fn displacement(&self, node_index: usize, dof: TrussDof3D) -> Result<f64, FemError> {
        if node_index >= self.n_nodes {
            return Err(FemError::InvalidNode(format!(
                "node index {node_index} out of bounds (max {})",
                self.n_nodes.saturating_sub(1)
            )));
        }
        Ok(self.displacements[node_index * 3 + dof.index()])
    }

    /// Reaction at a single DOF at a node.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if `node_index` is out of bounds.
    pub fn reaction(&self, node_index: usize, dof: TrussDof3D) -> Result<f64, FemError> {
        if node_index >= self.n_nodes {
            return Err(FemError::InvalidNode(format!(
                "node index {node_index} out of bounds (max {})",
                self.n_nodes.saturating_sub(1)
            )));
        }
        Ok(self.reactions[node_index * 3 + dof.index()])
    }

    /// Axial force in a member (tension positive).
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidMember`] if `element_index` is out of bounds.
    pub fn member_axial_force(&self, element_index: usize) -> Result<f64, FemError> {
        if element_index >= self.n_elements {
            return Err(FemError::InvalidMember(format!(
                "element index {element_index} out of bounds (max {})",
                self.n_elements.saturating_sub(1)
            )));
        }
        Ok(self.axial_forces[element_index])
    }

    /// Axial forces for all members.
    pub fn member_axial_forces(&self) -> &[f64] {
        &self.axial_forces
    }

    /// Global equilibrium check: `ΣFx`, `ΣFy`, `ΣFz`, `ΣMx`, `ΣMy`, `ΣMz`
    /// about the origin.
    ///
    /// Combines applied nodal forces and support reactions. Truss elements
    /// carry only axial force, so there are no member-level distributed or
    /// point loads to include.
    ///
    /// Returns a [`TrussEquilibriumReport3D`] with force/moment residuals
    /// and a balanced verdict.
    pub fn equilibrium(&self) -> TrussEquilibriumReport3D {
        let mut applied_fx = 0.0;
        let mut applied_fy = 0.0;
        let mut applied_fz = 0.0;
        let mut applied_mx = 0.0;
        let mut applied_my = 0.0;
        let mut applied_mz = 0.0;
        let mut applied_f_mag = 0.0;

        for &(node, dof, v) in &self.nodal_forces {
            let (x, y, z) = self
                .node_coords
                .get(node)
                .copied()
                .unwrap_or((0.0, 0.0, 0.0));
            match dof {
                0 => {
                    applied_fx += v;
                    applied_my += z * v;
                    applied_mz += -y * v;
                    applied_f_mag += v.abs();
                }
                1 => {
                    applied_fy += v;
                    applied_mx += -z * v;
                    applied_mz += x * v;
                    applied_f_mag += v.abs();
                }
                2 => {
                    applied_fz += v;
                    applied_mx += y * v;
                    applied_my += -x * v;
                    applied_f_mag += v.abs();
                }
                _ => {}
            }
        }

        let mut reaction_fx = 0.0;
        let mut reaction_fy = 0.0;
        let mut reaction_fz = 0.0;
        let mut reaction_mx = 0.0;
        let mut reaction_my = 0.0;
        let mut reaction_mz = 0.0;
        let mut reaction_f_mag = 0.0;
        for (idx, &(x, y, z)) in self.node_coords.iter().enumerate() {
            let rx = self.reactions.get(3 * idx).copied().unwrap_or(0.0);
            let ry = self.reactions.get(3 * idx + 1).copied().unwrap_or(0.0);
            let rz = self.reactions.get(3 * idx + 2).copied().unwrap_or(0.0);
            reaction_fx += rx;
            reaction_fy += ry;
            reaction_fz += rz;
            reaction_mx += y * rz - z * ry;
            reaction_my += z * rx - x * rz;
            reaction_mz += x * ry - y * rx;
            reaction_f_mag += rx.abs() + ry.abs() + rz.abs();
        }

        let f_mag = applied_f_mag + reaction_f_mag;
        let l_char: f64 = self
            .node_coords
            .iter()
            .map(|&(x, y, z)| x.abs().max(y.abs()).max(z.abs()))
            .fold(0.0_f64, f64::max);
        let rel = 1e-6;
        let f_scale = f_mag + l_char * f_mag;

        TrussEquilibriumReport3D {
            fx_residual: applied_fx + reaction_fx,
            fy_residual: applied_fy + reaction_fy,
            fz_residual: applied_fz + reaction_fz,
            mx_residual: applied_mx + reaction_mx,
            my_residual: applied_my + reaction_my,
            mz_residual: applied_mz + reaction_mz,
            applied_fx,
            applied_fy,
            applied_fz,
            applied_mx,
            applied_my,
            applied_mz,
            reaction_fx,
            reaction_fy,
            reaction_fz,
            reaction_mx,
            reaction_my,
            reaction_mz,
            tolerance: rel * f_scale,
        }
    }
}

/// Equilibrium report for a 3D truss analysis.
///
/// Truss nodes have 3 DOF (`ux, uy, uz`), so there are no reaction moments
/// at nodes. Moment equilibrium is checked about the global origin using node
/// coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrussEquilibriumReport3D {
    /// Residual of `ΣFx` (applied + reaction).
    pub fx_residual: f64,
    /// Residual of `ΣFy` (applied + reaction).
    pub fy_residual: f64,
    /// Residual of `ΣFz` (applied + reaction).
    pub fz_residual: f64,
    /// Residual of `ΣMx` about the origin (applied + reaction).
    pub mx_residual: f64,
    /// Residual of `ΣMy` about the origin (applied + reaction).
    pub my_residual: f64,
    /// Residual of `ΣMz` about the origin (applied + reaction).
    pub mz_residual: f64,
    /// Total applied horizontal load (X).
    pub applied_fx: f64,
    /// Total applied vertical load (Y).
    pub applied_fy: f64,
    /// Total applied load (Z).
    pub applied_fz: f64,
    /// Total applied moment about X.
    pub applied_mx: f64,
    /// Total applied moment about Y.
    pub applied_my: f64,
    /// Total applied moment about Z.
    pub applied_mz: f64,
    /// Sum of support reaction forces in X.
    pub reaction_fx: f64,
    /// Sum of support reaction forces in Y.
    pub reaction_fy: f64,
    /// Sum of support reaction forces in Z.
    pub reaction_fz: f64,
    /// Sum of reaction moments about X.
    pub reaction_mx: f64,
    /// Sum of reaction moments about Y.
    pub reaction_my: f64,
    /// Sum of reaction moments about Z.
    pub reaction_mz: f64,
    /// Force tolerance: `1e-6 * (1 + L_char) * Σ|F|`.
    pub tolerance: f64,
}

impl TrussEquilibriumReport3D {
    /// Whether all six residuals are within relative tolerance.
    pub fn is_balanced(&self) -> bool {
        self.fx_residual.abs() <= self.tolerance
            && self.fy_residual.abs() <= self.tolerance
            && self.fz_residual.abs() <= self.tolerance
            && self.mx_residual.abs() <= self.tolerance
            && self.my_residual.abs() <= self.tolerance
            && self.mz_residual.abs() <= self.tolerance
    }

    /// Force tolerance used by [`Self::is_balanced`].
    pub fn force_tolerance(&self) -> f64 {
        self.tolerance
    }
}

impl TrussSolver3D {
    /// Collect all results into a [`TrussAnalysisResult3D`].
    ///
    /// # Errors
    ///
    /// Propagates errors from [`Self::axial_forces`].
    pub fn results(&self) -> Result<TrussAnalysisResult3D, FemError> {
        Ok(TrussAnalysisResult3D {
            displacements: self.u_global.clone(),
            reactions: self.reactions(),
            axial_forces: self.axial_forces()?,
            solver_name: self.solver_name.clone(),
            node_coords: self.model.nodes.iter().map(|n| (n.x, n.y, n.z)).collect(),
            element_nodes: self
                .model
                .elements
                .iter()
                .map(|e| (e.node_i, e.node_j))
                .collect(),
            nodal_forces: self.model.nodal_forces.clone(),
            constrained_dofs: self
                .fixed_dofs
                .iter()
                .enumerate()
                .filter(|&(_, &fixed)| fixed)
                .map(|(i, _)| (i / 3, i % 3))
                .collect(),
            load_source: LoadSource::ModelLoads,
            n_nodes: self.model.nodes.len(),
            n_elements: self.model.elements.len(),
        })
    }
}
