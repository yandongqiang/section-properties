//! 2D Euler-Bernoulli Beam FEM
//!
//! Implements a 2D frame/beam element with 3 DOF per node: [u, v, θ]
//! where u = axial displacement, v = transverse displacement, θ = rotation.
//!
//! The element has 6 DOF total: [u_i, v_i, θ_i, u_j, v_j, θ_j]

use crate::fea::{
    SparseMatrix,
    solver::{LinearSolver, SolverError},
};
use crate::geometry::Point;
use crate::material::Material;

/// Beam cross-section properties for 2D Euler-Bernoulli beam
#[derive(Debug, Clone, Copy)]
pub struct BeamSection {
    /// Cross-sectional area [m²]
    pub area: f64,
    /// Second moment of area about local z-axis (bending in local x-y plane) [m⁴]
    pub i: f64,
}

impl BeamSection {
    /// Create a new beam section
    pub fn new(area: f64, i: f64) -> Self {
        Self { area, i }
    }

    /// Create from a rectangle section
    pub fn rectangle(width: f64, height: f64) -> Self {
        let area = width * height;
        let i = width * height.powi(3) / 12.0;
        Self { area, i }
    }

    /// Create from a circular section
    pub fn circle(radius: f64) -> Self {
        let area = std::f64::consts::PI * radius * radius;
        let i = std::f64::consts::PI * radius.powi(4) / 4.0;
        Self { area, i }
    }

    /// Create from a circular hollow section
    pub fn circle_hollow(outer_radius: f64, inner_radius: f64) -> Self {
        let area =
            std::f64::consts::PI * (outer_radius * outer_radius - inner_radius * inner_radius);
        let i = std::f64::consts::PI * (outer_radius.powi(4) - inner_radius.powi(4)) / 4.0;
        Self { area, i }
    }
}

/// 2D Beam Element
#[derive(Debug, Clone)]
pub struct BeamElement {
    /// Node i index
    pub node_i: usize,
    /// Node j index
    pub node_j: usize,
    /// Material
    pub material: Material,
    /// Cross-section properties
    pub section: BeamSection,
}

impl BeamElement {
    /// Create a new beam element
    pub fn new(
        node_i: usize,
        node_j: usize,
        material: Material,
        section: BeamSection,
    ) -> Result<Self, FemError> {
        if node_i == node_j {
            return Err(FemError::InvalidModel(
                "Beam element cannot have same start and end node".to_string(),
            ));
        }
        Ok(Self {
            node_i,
            node_j,
            material,
            section,
        })
    }

    /// Get element length from node coordinates
    pub fn length(&self, node_i: Point, node_j: Point) -> f64 {
        let dx = node_j.x - node_i.x;
        let dy = node_j.y - node_i.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Compute local stiffness matrix (6x6) in local coordinates
    ///
    /// Local DOF ordering: [u_i, v_i, θ_i, u_j, v_j, θ_j]
    ///
    /// Returns a 6x6 matrix in row-major order (Vec<Vec<f64>>)
    pub fn local_stiffness(&self, node_i: Point, node_j: Point) -> [[f64; 6]; 6] {
        let L = self.length(node_i, node_j);
        if L <= 0.0 {
            return [[0.0; 6]; 6];
        }

        let E = self.material.youngs_modulus;
        let A = self.section.area;
        let I = self.section.i;

        let EA_L = E * A / L;
        let EI_L3 = E * I / L.powi(3);
        let EI_L2 = E * I / L.powi(2);
        let EI_L = E * I / L;

        // Standard 2D Euler-Bernoulli beam stiffness matrix
        // DOF order: [u_i, v_i, θ_i, u_j, v_j, θ_j]
        let mut k = [[0.0; 6]; 6];

        // Axial terms
        k[0][0] = EA_L;
        k[0][3] = -EA_L;
        k[3][0] = -EA_L;
        k[3][3] = EA_L;

        // Bending terms
        // v_i terms
        k[1][1] = 12.0 * EI_L3;
        k[1][2] = 6.0 * EI_L2;
        k[1][4] = -12.0 * EI_L3;
        k[1][5] = 6.0 * EI_L2;

        // θ_i terms
        k[2][1] = 6.0 * EI_L2;
        k[2][2] = 4.0 * EI_L;
        k[2][4] = -6.0 * EI_L2;
        k[2][5] = 2.0 * EI_L;

        // v_j terms
        k[4][1] = -12.0 * EI_L3;
        k[4][2] = -6.0 * EI_L2;
        k[4][4] = 12.0 * EI_L3;
        k[4][5] = -6.0 * EI_L2;

        // θ_j terms
        k[5][1] = 6.0 * EI_L2;
        k[5][2] = 2.0 * EI_L;
        k[5][4] = -6.0 * EI_L2;
        k[5][5] = 4.0 * EI_L;

        k
    }

    /// Compute coordinate transformation matrix (6x6)
    ///
    /// Transforms from local to global coordinates
    /// Local DOF: [u_i, v_i, θ_i, u_j, v_j, θ_j]
    /// Global DOF: [U_i, V_i, Θ_i, U_j, V_j, Θ_j]
    pub fn transformation_matrix(&self, node_i: Point, node_j: Point) -> [[f64; 6]; 6] {
        let dx = node_j.x - node_i.x;
        let dy = node_j.y - node_i.y;
        let L = (dx * dx + dy * dy).sqrt();

        if L <= 0.0 {
            return [[0.0; 6]; 6];
        }

        let c = dx / L;
        let s = dy / L;

        // Transformation matrix T (6x6)
        // [ c  s  0  0  0  0 ]
        // [-s  c  0  0  0  0 ]
        // [ 0  0  1  0  0  0 ]
        // [ 0  0  0  c  s  0 ]
        // [ 0  0  0 -s  c  0 ]
        // [ 0  0  0  0  0  1 ]
        let mut T = [[0.0; 6]; 6];

        // Node i block
        T[0][0] = c;
        T[0][1] = s;
        T[1][0] = -s;
        T[1][1] = c;
        T[2][2] = 1.0;

        // Node j block
        T[3][3] = c;
        T[3][4] = s;
        T[4][3] = -s;
        T[4][4] = c;
        T[5][5] = 1.0;

        T
    }

    /// Compute global stiffness matrix (6x6) by transforming local stiffness
    pub fn global_stiffness(&self, node_i: Point, node_j: Point) -> [[f64; 6]; 6] {
        let k_local = self.local_stiffness(node_i, node_j);
        let T = self.transformation_matrix(node_i, node_j);

        // k_global = T^T * k_local * T
        let mut k_global = [[0.0; 6]; 6];

        for i in 0..6 {
            for j in 0..6 {
                let mut sum = 0.0;
                for k in 0..6 {
                    for l in 0..6 {
                        sum += T[k][i] * k_local[k][l] * T[l][j];
                    }
                }
                k_global[i][j] = sum;
            }
        }

        k_global
    }
}

/// Beam node with 3 DOF: [u, v, θ]
#[derive(Debug, Clone, Copy)]
pub struct BeamNode {
    /// Node index
    pub id: usize,
    /// X coordinate
    pub x: f64,
    /// Y coordinate
    pub y: f64,
}

impl BeamNode {
    pub fn new(id: usize, x: f64, y: f64) -> Self {
        Self { id, x, y }
    }

    pub fn point(&self) -> Point {
        Point::new(self.x, self.y)
    }
}

/// Beam model with nodes, elements, loads, and boundary conditions
#[derive(Debug, Clone)]
pub struct BeamModel {
    /// Nodes
    pub nodes: Vec<BeamNode>,
    /// Elements
    pub elements: Vec<BeamElement>,
    /// Nodal forces: (node_idx, dof, value)
    /// dof: 0=u, 1=v, 2=θ
    /// node_idx is the index in the nodes Vec (0, 1, 2, ...)
    pub nodal_forces: Vec<(usize, usize, f64)>,
    /// Fixed DOFs: (node_idx, dof, value) - value is the prescribed displacement (usually 0.0)
    /// node_idx is the index in the nodes Vec (0, 1, 2, ...)
    pub fixed_dofs: Vec<(usize, usize, f64)>,
}

impl BeamModel {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            elements: Vec::new(),
            nodal_forces: Vec::new(),
            fixed_dofs: Vec::new(),
        }
    }

    pub fn add_node(&mut self, node: BeamNode) {
        self.nodes.push(node);
    }

    pub fn add_element(&mut self, element: BeamElement) {
        self.elements.push(element);
    }

    /// Add nodal force
    /// dof: 0=u, 1=v, 2=θ
    /// node_idx is the index in the nodes Vec (0, 1, 2, ...)
    pub fn add_nodal_force(&mut self, node_idx: usize, dof: usize, value: f64) {
        self.nodal_forces.push((node_idx, dof, value));
    }

    /// Fix a DOF to a prescribed value
    /// dof: 0=u, 1=v, 2=θ
    /// node_idx is the index in the nodes Vec (0, 1, 2, ...)
    pub fn fix_dof(&mut self, node_idx: usize, dof: usize, value: f64) {
        self.fixed_dofs.push((node_idx, dof, value));
    }

    /// Fix a node completely (all 3 DOFs to 0)
    /// node_idx is the index in the nodes Vec (0, 1, 2, ...)
    pub fn fix_node(&mut self, node_idx: usize) {
        self.fix_dof(node_idx, 0, 0.0);
        self.fix_dof(node_idx, 1, 0.0);
        self.fix_dof(node_idx, 2, 0.0);
    }

    /// Get total number of DOFs
    pub fn n_dof(&self) -> usize {
        self.nodes.len() * 3
    }

    /// Map (node_idx, dof) to global DOF index
    ///
    /// Note: node_idx must be the index in the nodes Vec (0, 1, 2, ...).
    /// This is NOT an arbitrary user-defined ID. Node IDs are implicitly their
    /// position in the nodes vector.
    pub fn dof_index(&self, node_idx: usize, dof: usize) -> usize {
        node_idx * 3 + dof
    }
}

/// Beam FEM solver using the unified LinearSolver abstraction
pub struct BeamSolver {
    /// Global stiffness matrix
    k_global: SparseMatrix,
    /// Global force vector
    f_global: Vec<f64>,
    /// Displacement solution
    u_global: Vec<f64>,
    /// Fixed DOFs (true if constrained)
    fixed_dofs: Vec<bool>,
    /// Prescribed values for constrained DOFs (None = 0.0, Some(value) = prescribed)
    prescribed_values: Vec<Option<f64>>,
    /// Original stiffness matrix (for reaction computation)
    k_original: SparseMatrix,
    /// Number of DOFs
    n_dof: usize,
    /// Model reference
    model: BeamModel,
}

impl BeamSolver {
    /// Create a new BeamSolver from a BeamModel
    pub fn from_model(model: &BeamModel) -> Result<Self, FemError> {
        let n_dof = model.n_dof();
        if n_dof == 0 {
            return Err(FemError::InvalidModel("Model has no nodes".to_string()));
        }

        // Build global stiffness matrix
        let mut k_global = SparseMatrix::new(n_dof);

        // Assemble element stiffness matrices
        for element in &model.elements {
            let node_i = model.nodes[element.node_i].point();
            let node_j = model.nodes[element.node_j].point();

            // Check for zero-length beam
            let dx = node_j.x - node_i.x;
            let dy = node_j.y - node_i.y;
            let L = (dx * dx + dy * dy).sqrt();
            if L <= 0.0 {
                return Err(FemError::InvalidModel(format!(
                    "Beam element {} has zero or negative length (nodes {} and {} at same position)",
                    model
                        .elements
                        .iter()
                        .position(|e| e.node_i == element.node_i && e.node_j == element.node_j)
                        .unwrap_or(0),
                    element.node_i,
                    element.node_j
                )));
            }

            let k_global_elem = element.global_stiffness(node_i, node_j);

            // Map element DOFs to global DOFs
            let dof_map = [
                model.dof_index(element.node_i, 0), // u_i
                model.dof_index(element.node_i, 1), // v_i
                model.dof_index(element.node_i, 2), // θ_i
                model.dof_index(element.node_j, 0), // u_j
                model.dof_index(element.node_j, 1), // v_j
                model.dof_index(element.node_j, 2), // θ_j
            ];

            // Assemble into global matrix
            for a in 0..6 {
                for b in 0..6 {
                    let val = k_global_elem[a][b];
                    k_global.add(dof_map[a], dof_map[b], val);
                }
            }
        }

        // Build force vector
        let mut f_global = vec![0.0; n_dof];
        for (node_id, dof, value) in &model.nodal_forces {
            if *node_id < model.nodes.len() && *dof < 3 {
                let idx = model.dof_index(*node_id, *dof);
                f_global[idx] += value;
            }
        }

        // Fixed DOFs and prescribed values
        let mut fixed_dofs = vec![false; n_dof];
        let mut prescribed_values = vec![None; n_dof];
        for (node_id, dof, value) in &model.fixed_dofs {
            if *node_id < model.nodes.len() && *dof < 3 {
                let idx = model.dof_index(*node_id, *dof);
                fixed_dofs[idx] = true;
                prescribed_values[idx] = Some(*value);
            }
        }

        // Store original matrix for reaction computation
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
        })
    }

    /// Apply boundary conditions using static condensation (exact enforcement)
    /// Returns (reduced stiffness matrix, reduced force vector, free_to_global mapping, constrained_indices, constrained_values)
    fn apply_boundary_conditions(
        &self,
    ) -> (SparseMatrix, Vec<f64>, Vec<usize>, Vec<usize>, Vec<f64>) {
        let n = self.n_dof;
        let fixed = &self.fixed_dofs;
        let prescribed = &self.prescribed_values;

        // Identify free and constrained DOFs
        let mut free_dofs = Vec::new();
        let mut constrained_dofs = Vec::new();
        let mut constrained_values = Vec::new();
        let mut free_to_global = Vec::new();
        let mut global_to_free = vec![None; n];

        for i in 0..n {
            if !fixed[i] {
                global_to_free[i] = Some(free_dofs.len());
                free_dofs.push(i);
                free_to_global.push(i);
            } else {
                let constrained_idx = constrained_dofs.len();
                constrained_dofs.push(i);
                constrained_values.push(prescribed[i].unwrap_or(0.0));
            }
        }

        let n_free = free_dofs.len();
        if n_free == 0 {
            // All DOFs fixed - return empty system
            return (
                SparseMatrix::new(0),
                Vec::new(),
                Vec::new(),
                constrained_dofs,
                constrained_values,
            );
        }

        // Extract free-free submatrix K_ff, free-constrained K_fc, and force vector f_f
        let mut k_ff = SparseMatrix::new(n_free);
        let mut k_fc = SparseMatrix::new(n_free); // Only n_free rows, n columns but we'll only use constrained cols
        let mut f_f = vec![0.0; n_free];

        // Compress original matrix to access CSR data
        let mut k_orig = self.k_global.clone();
        k_orig.compress();

        // Extract K_ff and K_fc
        for (free_idx, &global_i) in free_dofs.iter().enumerate() {
            // Force vector
            f_f[free_idx] = self.f_global[global_i];

            // Stiffness matrix row
            let row_ptr = k_orig.row_ptr();
            let csr_cols = k_orig.csr_cols();
            let csr_vals = k_orig.csr_vals();

            for idx in row_ptr[global_i]..row_ptr[global_i + 1] {
                let global_j = csr_cols[idx];
                let val = csr_vals[idx];

                if let Some(free_j) = global_to_free[global_j] {
                    // free-free
                    k_ff.add(free_idx, free_j, val);
                } else if fixed[global_j] {
                    // free-constrained
                    let constr_idx = constrained_dofs
                        .iter()
                        .position(|&x| x == global_j)
                        .unwrap();
                    k_fc.add(free_idx, constr_idx, val);
                }
            }
        }

        k_ff.compress();
        k_fc.compress();

        // Compute reduced RHS: F_reduced = F_f - K_fc * U_c
        let mut f_reduced = f_f;
        for (free_idx, _) in free_dofs.iter().enumerate() {
            let mut sum = 0.0;
            let row_ptr = k_fc.row_ptr();
            let csr_cols = k_fc.csr_cols();
            let csr_vals = k_fc.csr_vals();
            for idx in row_ptr[free_idx]..row_ptr[free_idx + 1] {
                let constr_j = csr_cols[idx];
                sum += csr_vals[idx] * constrained_values[constr_j];
            }
            f_reduced[free_idx] -= sum;
        }

        (
            k_ff,
            f_reduced,
            free_to_global,
            constrained_dofs,
            constrained_values,
        )
    }

    fn find_diagonal_index(&self, matrix: &SparseMatrix, row: usize) -> Option<usize> {
        let row_ptr = matrix.row_ptr();
        let csr_cols = matrix.csr_cols();
        for idx in row_ptr[row]..row_ptr[row + 1] {
            if csr_cols[idx] == row {
                return Some(idx);
            }
        }
        None
    }

    /// Solve the system using the provided LinearSolver
    pub fn solve(&mut self, solver: &mut dyn LinearSolver) -> Result<(), FemError> {
        // Apply boundary conditions using static condensation
        let (k_ff, f_reduced, free_to_global, constrained_dofs, constrained_values) =
            self.apply_boundary_conditions();

        if k_ff.n == 0 {
            // All DOFs fixed - just set prescribed values
            self.u_global = vec![0.0; self.n_dof];
            for (i, &global_idx) in constrained_dofs.iter().enumerate() {
                self.u_global[global_idx] = constrained_values[i];
            }
            return Ok(());
        }

        // Compress matrix if needed
        let mut k_compressed = k_ff.clone();
        k_compressed.compress();

        // Factorize
        solver
            .factor(&k_compressed)
            .map_err(|e| FemError::SolverError(e.to_string()))?;

        // Solve reduced system
        let u_free = solver
            .solve(&f_reduced)
            .map_err(|e| FemError::SolverError(e.to_string()))?;

        // Expand solution back to full DOF space
        self.u_global = vec![0.0; self.n_dof];
        for (free_idx, &global_idx) in free_to_global.iter().enumerate() {
            self.u_global[global_idx] = u_free[free_idx];
        }
        // Set prescribed values for constrained DOFs
        for (i, &global_idx) in constrained_dofs.iter().enumerate() {
            self.u_global[global_idx] = constrained_values[i];
        }

        Ok(())
    }

    /// Get displacement at a specific DOF
    pub fn displacement(&self, node_idx: usize, dof: usize) -> f64 {
        let idx = self.model.dof_index(node_idx, dof);
        if idx < self.u_global.len() {
            self.u_global[idx]
        } else {
            0.0
        }
    }

    /// Get all displacements
    pub fn displacements(&self) -> &[f64] {
        &self.u_global
    }

    /// Compute reaction forces: R = K_original * u - f_applied
    pub fn reactions(&self) -> Vec<f64> {
        // R = K_original * u - f_global
        let mut reactions = vec![0.0; self.n_dof];

        // Compute K_original * u using public API
        let mut k_orig = self.k_original.clone();
        k_orig.compress();
        for i in 0..self.n_dof {
            let mut sum = 0.0;
            let row_ptr = k_orig.row_ptr();
            let csr_cols = k_orig.csr_cols();
            let csr_vals = k_orig.csr_vals();
            for idx in row_ptr[i]..row_ptr[i + 1] {
                let j = csr_cols[idx];
                sum += csr_vals[idx] * self.u_global[j];
            }
            reactions[i] = sum - self.f_global[i];
        }

        reactions
    }

    /// Get reaction at a specific DOF
    pub fn reaction(&self, node_idx: usize, dof: usize) -> f64 {
        let idx = self.model.dof_index(node_idx, dof);
        if idx < self.reactions().len() {
            self.reactions()[idx]
        } else {
            0.0
        }
    }
}

/// FemError for beam FEM
#[derive(Debug, Clone, thiserror::Error)]
pub enum FemError {
    #[error("Invalid model: {0}")]
    InvalidModel(String),
    #[error("Solver error: {0}")]
    SolverError(String),
    #[error("Invalid input: {0}")]
    InvalidInput(String),
    #[error("Singular matrix: {0}")]
    SingularMatrix(String),
}

impl From<SolverError> for FemError {
    fn from(e: SolverError) -> Self {
        FemError::SolverError(e.to_string())
    }
}

/// High-level API for beam analysis
pub struct BeamAnalysis;

impl BeamAnalysis {
    /// Analyze a cantilever beam with tip load
    pub fn cantilever_tip_load(L: f64, E: f64, A: f64, I: f64, P: f64) -> (f64, f64, f64) {
        // Analytical solutions for cantilever with tip load P at free end
        // Fixed at x=0, load P downward (transverse) at x=L
        let u_tip = 0.0; // No axial displacement for transverse load
        let v_tip = P * L.powi(3) / (3.0 * E * I); // Transverse deflection
        let theta_tip = P * L.powi(2) / (2.0 * E * I); // Rotation

        (u_tip, v_tip, theta_tip)
    }

    /// Analyze a simply supported beam with central point load
    pub fn simply_supported_central_load(L: f64, E: f64, I: f64, P: f64) -> (f64, f64) {
        // Central deflection and rotation at support
        let v_mid = P * L.powi(3) / (48.0 * E * I);
        let theta_support = P * L.powi(2) / (16.0 * E * I);
        (v_mid, theta_support)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::Material;

    #[test]
    fn test_beam_section_creation() {
        let sec = BeamSection::new(1.0, 1.0);
        assert_eq!(sec.area, 1.0);
        assert_eq!(sec.i, 1.0);

        let rect = BeamSection::rectangle(0.1, 0.2);
        assert!((rect.area - 0.02).abs() < 1e-10);
        assert!((rect.i - 0.1 * 0.2_f64.powi(3) / 12.0).abs() < 1e-10);

        let circ = BeamSection::circle(0.05);
        assert!((circ.area - std::f64::consts::PI * 0.0025).abs() < 1e-10);
    }

    #[test]
    fn test_local_stiffness() {
        let material = Material::new(200e9, 0.3, 7850.0, "Steel");
        let section = BeamSection::new(0.01, 8.333e-6);
        let element = BeamElement::new(0, 1, material, section);

        let node_i = Point::new(0.0, 0.0);
        let node_j = Point::new(1.0, 0.0);

        let k = element.local_stiffness(node_i, node_j);

        // Check symmetry
        for i in 0..6 {
            for j in 0..6 {
                assert!(
                    (k[i][j] - k[j][i]).abs() < 1e-10,
                    "Matrix not symmetric at ({},{})",
                    i,
                    j
                );
            }
        }

        // Check axial terms
        let E = 200e9;
        let A = 0.01;
        let L = 1.0;
        let EA_L = E * A / L;
        assert!((k[0][0] - EA_L).abs() < 1e-6);
        assert!((k[0][3] + EA_L).abs() < 1e-6);
        assert!((k[3][3] - EA_L).abs() < 1e-6);

        // Check bending terms
        let I = 8.333e-6;
        let EI_L3 = E * I / L.powi(3);
        let EI_L2 = E * I / L.powi(2);
        let EI_L = E * I / L;

        assert!((k[1][1] - 12.0 * EI_L3).abs() < 1e-6);
        assert!((k[1][2] - 6.0 * EI_L2).abs() < 1e-6);
        assert!((k[2][2] - 4.0 * EI_L).abs() < 1e-6);
        assert!((k[4][4] - 12.0 * EI_L3).abs() < 1e-6);
        assert!((k[5][5] - 4.0 * EI_L).abs() < 1e-6);
    }

    #[test]
    fn test_transformation_matrix() {
        let material = Material::new(200e9, 0.3, 7850.0, "Steel");
        let section = BeamSection::new(0.01, 8.333e-6);
        let element = BeamElement::new(0, 1, material, section);

        // Horizontal beam
        let node_i = Point::new(0.0, 0.0);
        let node_j = Point::new(1.0, 0.0);
        let T = element.transformation_matrix(node_i, node_j);

        // For horizontal beam: c=1, s=0
        assert!((T[0][0] - 1.0).abs() < 1e-10);
        assert!((T[1][1] - 1.0).abs() < 1e-10);
        assert!((T[2][2] - 1.0).abs() < 1e-10);
        assert!((T[3][3] - 1.0).abs() < 1e-10);
        assert!((T[4][4] - 1.0).abs() < 1e-10);
        assert!((T[5][5] - 1.0).abs() < 1e-10);

        // Vertical beam
        let node_i = Point::new(0.0, 0.0);
        let node_j = Point::new(0.0, 1.0);
        let T = element.transformation_matrix(node_i, node_j);

        // For vertical beam: c=0, s=1
        assert!((T[0][0] - 0.0).abs() < 1e-10); // c
        assert!((T[0][1] - 1.0).abs() < 1e-10); // s
        assert!((T[1][0] - -1.0).abs() < 1e-10); // -s
        assert!((T[1][1] - 0.0).abs() < 1e-10); // c
    }

    #[test]
    fn test_global_stiffness_horizontal() {
        let material = Material::new(200e9, 0.3, 7850.0, "Steel");
        let section = BeamSection::new(0.01, 8.333e-6);
        let element = BeamElement::new(0, 1, material, section);

        let node_i = Point::new(0.0, 0.0);
        let node_j = Point::new(1.0, 0.0);

        let k_global = element.global_stiffness(node_i, node_j);
        let k_local = element.local_stiffness(node_i, node_j);

        // For horizontal beam, global should equal local
        for i in 0..6 {
            for j in 0..6 {
                assert!((k_global[i][j] - k_local[i][j]).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_global_stiffness_vertical() {
        let material = Material::new(200e9, 0.3, 7850.0, "Steel");
        let section = BeamSection::new(0.01, 8.333e-6);
        let element = BeamElement::new(0, 1, material, section);

        let node_i = Point::new(0.0, 0.0);
        let node_j = Point::new(0.0, 1.0);

        let k_global = element.global_stiffness(node_i, node_j);

        // Check symmetry
        for i in 0..6 {
            for j in 0..6 {
                assert!((k_global[i][j] - k_global[j][i]).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_cantilever_analytical() {
        let L = 1.0;
        let E = 200e9;
        let A = 0.01;
        let I = 8.333e-6;
        let P = 1000.0;

        let (u_tip, v_tip, theta_tip) = BeamAnalysis::cantilever_tip_load(L, E, A, I, P);

        // For tip load P downward: axial = 0, v = PL³/3EI, θ = PL²/2EI
        let expected_v = P * L.powi(3) / (3.0 * E * I);
        let expected_theta = P * L.powi(2) / (2.0 * E * I);

        assert!((v_tip - expected_v).abs() < 1e-10);
        assert!((theta_tip - expected_theta).abs() < 1e-10);
        assert!(u_tip.abs() < 1e-10); // No axial load in this case
    }

    #[test]
    fn test_beam_model() {
        let mut model = BeamModel::new();

        model.add_node(BeamNode::new(0, 0.0, 0.0));
        model.add_node(BeamNode::new(1, 1.0, 0.0));

        let material = Material::new(200e9, 0.3, 7850.0, "Steel");
        let section = BeamSection::new(0.01, 8.333e-6);
        model.add_element(BeamElement::new(0, 1, material, section));

        model.fix_node(0);
        model.add_nodal_force(1, 1, -1000.0); // Downward force at tip

        assert_eq!(model.n_dof(), 6);
        assert_eq!(model.dof_index(0, 0), 0);
        assert_eq!(model.dof_index(0, 1), 1);
        assert_eq!(model.dof_index(0, 2), 2);
        assert_eq!(model.dof_index(1, 0), 3);
        assert_eq!(model.dof_index(1, 1), 4);
        assert_eq!(model.dof_index(1, 2), 5);
    }
}
