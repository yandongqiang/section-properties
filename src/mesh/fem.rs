//! Finite Element Method (FEM) solver for linear elastic analysis.
//!
//! Provides 2D plane stress/strain and plate bending elements,
//! stiffness matrix assembly, displacement solution, and stress recovery.

use crate::geometry::Point;
use crate::material::Material;
use crate::mesh::{Mesh, MeshParams};
use crate::section::Section;
use std::collections::HashMap;

/// 2D element types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ElementType {
    /// 3-node triangle (CST - Constant Strain Triangle)
    Tri3,
    /// 6-node triangle (quadratic)
    Tri6,
    /// 4-node quadrilateral (bilinear)
    Quad4,
    /// 8-node quadrilateral (quadratic)
    Quad8,
}

/// Material properties for FEM (plane stress/strain).
#[derive(Debug, Clone, Copy)]
pub struct MaterialProps {
    /// Young's modulus
    pub youngs_modulus: f64,
    /// Poisson's ratio
    pub poissons_ratio: f64,
    /// Shear modulus (G = E / (2(1+nu)))
    pub shear_modulus: f64,
    /// Density (for mass matrix)
    pub density: f64,
    /// Plane stress (true) or plane strain (false)
    pub plane_stress: bool,
    /// Shear correction factor (for plates)
    pub shear_correction: f64,
}

impl MaterialProps {
    /// Create from Material with plane stress assumption.
    pub fn from_material(material: &Material, plane_stress: bool) -> Self {
        Self {
            youngs_modulus: material.youngs_modulus,
            poissons_ratio: material.poissons_ratio,
            shear_modulus: material.shear_modulus,
            density: material.density,
            plane_stress,
            shear_correction: 5.0 / 6.0,
        }
    }

    /// Create for plane strain.
    pub fn plane_strain(material: &Material) -> Self {
        Self::from_material(material, false)
    }

    /// Create for plane stress.
    pub fn plane_stress(material: &Material) -> Self {
        Self::from_material(material, true)
    }

    /// Constitutive matrix D for plane stress.
    pub fn d_matrix_plane_stress(&self) -> [[f64; 3]; 3] {
        let e = self.youngs_modulus;
        let nu = self.poissons_ratio;
        let factor = e / (1.0 - nu * nu);
        [
            [factor, factor * nu, 0.0],
            [factor * nu, factor, 0.0],
            [0.0, 0.0, factor * (1.0 - nu) / 2.0],
        ]
    }

    /// Constitutive matrix D for plane strain.
    pub fn d_matrix_plane_strain(&self) -> [[f64; 3]; 3] {
        let e = self.youngs_modulus;
        let nu = self.poissons_ratio;
        let factor = e / ((1.0 + nu) * (1.0 - 2.0 * nu));
        [
            [factor * (1.0 - nu), factor * nu, 0.0],
            [factor * nu, factor * (1.0 - nu), 0.0],
            [0.0, 0.0, factor * (1.0 - 2.0 * nu) / 2.0],
        ]
    }

    /// Get constitutive matrix D (3x3 for 2D).
    pub fn d_matrix(&self) -> [[f64; 3]; 3] {
        if self.plane_stress {
            self.d_matrix_plane_stress()
        } else {
            self.d_matrix_plane_strain()
        }
    }
}

/// Stress/strain result at a point.
#[derive(Debug, Clone, Copy)]
pub struct StressResult {
    /// Normal stress in x direction
    pub sigma_x: f64,
    /// Normal stress in y direction
    pub sigma_y: f64,
    /// Shear stress xy
    pub tau_xy: f64,
    /// Strain in x direction
    pub epsilon_x: f64,
    /// Strain in y direction
    pub epsilon_y: f64,
    /// Shear strain xy
    pub gamma_xy: f64,
    /// Von Mises stress
    pub von_mises: f64,
    /// Principal stress 1 (major)
    pub sigma_1: f64,
    /// Principal stress 2 (minor)
    pub sigma_2: f64,
    /// Principal angle (radians)
    pub principal_angle: f64,
}

impl StressResult {
    /// Create from stress components.
    pub fn from_stress(sigma_x: f64, sigma_y: f64, tau_xy: f64) -> Self {
        let sigma_avg = (sigma_x + sigma_y) * 0.5;
        let r = ((sigma_x - sigma_y) * 0.5).powi(2) + tau_xy * tau_xy;
        let r = r.sqrt();
        let sigma_1 = sigma_avg + r;
        let sigma_2 = sigma_avg - r;
        let principal_angle = 0.5 * (2.0 * tau_xy).atan2(sigma_x - sigma_y);
        let von_mises =
            (sigma_x * sigma_x - sigma_x * sigma_y + sigma_y * sigma_y + 3.0 * tau_xy * tau_xy)
                .sqrt();

        Self {
            sigma_x,
            sigma_y,
            tau_xy,
            epsilon_x: 0.0,
            epsilon_y: 0.0,
            gamma_xy: 0.0,
            von_mises,
            sigma_1,
            sigma_2,
            principal_angle,
        }
    }

    /// Create from strain components using material.
    pub fn from_strain(
        epsilon_x: f64,
        epsilon_y: f64,
        gamma_xy: f64,
        props: &MaterialProps,
    ) -> Self {
        let d = props.d_matrix();
        let sigma_x = d[0][0] * epsilon_x + d[0][1] * epsilon_y + d[0][2] * gamma_xy;
        let sigma_y = d[1][0] * epsilon_x + d[1][1] * epsilon_y + d[1][2] * gamma_xy;
        let tau_xy = d[2][0] * epsilon_x + d[2][1] * epsilon_y + d[2][2] * gamma_xy;

        let mut result = Self::from_stress(sigma_x, sigma_y, tau_xy);
        result.epsilon_x = epsilon_x;
        result.epsilon_y = epsilon_y;
        result.gamma_xy = gamma_xy;
        result
    }
}

/// FEM model containing mesh, materials, boundary conditions, and loads.
#[derive(Debug, Clone)]
pub struct FemModel {
    pub mesh: Mesh,
    pub materials: Vec<MaterialProps>,
    /// Boundary conditions: (node_index, dof, value) where dof: 0=ux, 1=uy
    pub dirichlet_bcs: Vec<(usize, usize, f64)>,
    /// Nodal forces: (node_index, dof, value)
    pub nodal_forces: Vec<(usize, usize, f64)>,
    /// Distributed loads on elements: (element_index, load_vector)
    pub element_loads: Vec<(usize, [f64; 3])>, // body force per unit area
    /// Problem type
    pub problem_type: ProblemType,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProblemType {
    PlaneStress,
    PlaneStrain,
    Axisymmetric,
}

impl FemModel {
    /// Create a new FEM model from a mesh.
    pub fn new(mesh: Mesh) -> Self {
        Self {
            mesh,
            materials: Vec::new(),
            dirichlet_bcs: Vec::new(),
            nodal_forces: Vec::new(),
            element_loads: Vec::new(),
            problem_type: ProblemType::PlaneStress,
        }
    }

    /// Add a material.
    pub fn add_material(&mut self, props: MaterialProps) -> usize {
        self.materials.push(props);
        self.materials.len() - 1
    }

    /// Add Dirichlet boundary condition.
    pub fn add_dirichlet_bc(&mut self, node: usize, dof: usize, value: f64) {
        self.dirichlet_bcs.push((node, dof, value));
    }

    /// Fix a node completely (both DOFs).
    pub fn fix_node(&mut self, node: usize) {
        self.add_dirichlet_bc(node, 0, 0.0);
        self.add_dirichlet_bc(node, 1, 0.0);
    }

    /// Add nodal force.
    pub fn add_nodal_force(&mut self, node: usize, dof: usize, value: f64) {
        self.nodal_forces.push((node, dof, value));
    }

    /// Add body force (gravity, etc.) to all elements.
    pub fn add_body_force(&mut self, fx: f64, fy: f64) {
        for i in 0..self.mesh.n_elements() {
            self.element_loads.push((i, [fx, fy, 0.0]));
        }
    }

    /// Set problem type.
    pub fn set_problem_type(&mut self, ptype: ProblemType) {
        self.problem_type = ptype;
    }
}

/// FEM solver for linear static analysis.
#[derive(Debug)]
pub struct FemSolver {
    /// Global stiffness matrix (sparse CSR format)
    pub k_global: SprsMatrix,
    /// Global force vector
    pub f_global: Vec<f64>,
    /// Displacement solution
    pub u_global: Vec<f64>,
    /// Number of DOFs
    pub n_dof: usize,
    /// DOF mapping: (node, dof) -> global_dof_index
    pub dof_map: HashMap<(usize, usize), usize>,
    /// Fixed DOFs
    pub fixed_dofs: Vec<bool>,
    /// Original (unmodified) global stiffness matrix, before any Dirichlet
    /// boundary conditions are applied. Used to compute reaction forces as
    /// `K_original * u`; `k_global` is mutated in place during assembly.
    k_original: SprsMatrix,
}

impl FemSolver {
    /// Create solver from FEM model.
    pub fn from_model(model: &FemModel) -> Result<Self, FemError> {
        let n_nodes = model.mesh.n_nodes();
        let n_dof = n_nodes * 2; // 2 DOFs per node (ux, uy)

        // Build DOF map
        let mut dof_map = HashMap::new();
        let mut dof_index = 0;
        for node in 0..n_nodes {
            for dof in 0..2 {
                dof_map.insert((node, dof), dof_index);
                dof_index += 1;
            }
        }

        // Fixed DOFs
        let mut fixed_dofs = vec![false; n_dof];
        for (node, dof, _) in &model.dirichlet_bcs {
            if let Some(&idx) = dof_map.get(&(*node, *dof)) {
                fixed_dofs[idx] = true;
            }
        }

        // Build sparse stiffness matrix
        let mut k_global = SprsMatrix::new(n_dof);

        // Assemble element stiffness matrices
        for (elem_idx, element) in model.mesh.elements.iter().enumerate() {
            let mat_idx = if elem_idx < model.mesh.element_materials.len() {
                model.mesh.element_materials[elem_idx]
            } else {
                0
            };
            let props = &model.materials[mat_idx];

            let ke = element_stiffness_tri3(&model.mesh, element, props);

            // Assemble into global matrix
            let mut dof_indices = [0; 6];
            for (i, &node) in element.iter().enumerate() {
                dof_indices[2 * i] = dof_map[&(node, 0)];
                dof_indices[2 * i + 1] = dof_map[&(node, 1)];
            }

            for i in 0..6 {
                for j in 0..6 {
                    k_global.add(dof_indices[i], dof_indices[j], ke[i][j]);
                }
            }
        }

        // Build global force vector
        let mut f_global = vec![0.0; n_dof];

        // Nodal forces
        for (node, dof, value) in &model.nodal_forces {
            if let Some(&idx) = dof_map.get(&(*node, *dof)) {
                f_global[idx] += value;
            }
        }

        // Element body forces (consistent load vector)
        for (elem_idx, load) in &model.element_loads {
            let element = model.mesh.elements[*elem_idx];
            let fe = element_body_force_tri3(&model.mesh, &element, load);

            let mut dof_indices = [0; 6];
            for (i, &node) in element.iter().enumerate() {
                dof_indices[2 * i] = dof_map[&(node, 0)];
                dof_indices[2 * i + 1] = dof_map[&(node, 1)];
            }

            for i in 0..6 {
                f_global[dof_indices[i]] += fe[i];
            }
        }

        // Apply Dirichlet BCs to force vector
        for (node, dof, value) in &model.dirichlet_bcs {
            if let Some(&idx) = dof_map.get(&(*node, *dof)) {
                f_global[idx] = *value;
            }
        }

        let mut k_original = k_global.clone();
        k_original.finalize();

        Ok(Self {
            k_global,
            f_global,
            u_global: vec![0.0; n_dof],
            n_dof,
            dof_map,
            fixed_dofs,
            k_original,
        })
    }

    /// Solve the linear system K*u = F.
    pub fn solve(&mut self) -> Result<&[f64], FemError> {
        // Apply boundary conditions to stiffness matrix
        self.apply_boundary_conditions();

        // Solve using Cholesky decomposition (for symmetric positive definite)
        self.u_global = self
            .k_global
            .solve_cholesky(&self.f_global)
            .ok_or(FemError::SingularMatrix)?;

        Ok(&self.u_global)
    }

    /// Apply Dirichlet boundary conditions to stiffness matrix.
    fn apply_boundary_conditions(&mut self) {
        for i in 0..self.n_dof {
            if self.fixed_dofs[i] {
                // Zero out row and column, set diagonal to 1
                self.k_global.zero_row_col(i);
                self.k_global.set(i, i, 1.0);
            }
        }
    }

    /// Get displacement at a node.
    pub fn displacement(&self, node: usize) -> Point {
        let ux = self.u_global[self.dof_map[&(node, 0)]];
        let uy = self.u_global[self.dof_map[&(node, 1)]];
        Point::new(ux, uy)
    }

    /// Get stress at element centroid.
    pub fn element_stress(&self, model: &FemModel, elem_idx: usize) -> StressResult {
        let element = model.mesh.elements[elem_idx];
        let props = &model.materials[model.mesh.element_materials[elem_idx]];

        // Get element displacements
        let mut u_elem = [0.0; 6];
        for (i, &node) in element.iter().enumerate() {
            u_elem[2 * i] = self.u_global[self.dof_map[&(node, 0)]];
            u_elem[2 * i + 1] = self.u_global[self.dof_map[&(node, 1)]];
        }

        // Compute B matrix (strain-displacement)
        let b = strain_displacement_matrix_tri3(&model.mesh, &element);

        // Strain = B * u
        let mut strain = [0.0; 3];
        for i in 0..3 {
            for j in 0..6 {
                strain[i] += b[i][j] * u_elem[j];
            }
        }

        // Stress = D * strain
        let d = props.d_matrix();
        let mut stress = [0.0; 3];
        for i in 0..3 {
            for j in 0..3 {
                stress[i] += d[i][j] * strain[j];
            }
        }

        StressResult::from_strain(strain[0], strain[1], strain[2], props)
    }

    /// Get stress at all element centroids.
    pub fn all_element_stresses(&self, model: &FemModel) -> Vec<StressResult> {
        (0..model.mesh.n_elements())
            .map(|i| self.element_stress(model, i))
            .collect()
    }

    /// Get nodal stresses by averaging from elements.
    pub fn nodal_stresses(&self, model: &FemModel) -> Vec<StressResult> {
        let n_nodes = model.mesh.n_nodes();
        let mut stress_sum = vec![StressResult::from_stress(0.0, 0.0, 0.0); n_nodes];
        let mut stress_count = vec![0; n_nodes];

        for elem_idx in 0..model.mesh.n_elements() {
            let stress = self.element_stress(model, elem_idx);
            let element = model.mesh.elements[elem_idx];
            for &node in &element {
                stress_sum[node].sigma_x += stress.sigma_x;
                stress_sum[node].sigma_y += stress.sigma_y;
                stress_sum[node].tau_xy += stress.tau_xy;
                stress_sum[node].von_mises += stress.von_mises;
                stress_sum[node].sigma_1 += stress.sigma_1;
                stress_sum[node].sigma_2 += stress.sigma_2;
                stress_count[node] += 1;
            }
        }

        let mut nodal_stresses = Vec::with_capacity(n_nodes);
        for i in 0..n_nodes {
            if stress_count[i] > 0 {
                let c = stress_count[i] as f64;
                let mut s = stress_sum[i];
                s.sigma_x /= c;
                s.sigma_y /= c;
                s.tau_xy /= c;
                s.von_mises /= c;
                s.sigma_1 /= c;
                s.sigma_2 /= c;
                nodal_stresses.push(s);
            } else {
                nodal_stresses.push(StressResult::from_stress(0.0, 0.0, 0.0));
            }
        }
        nodal_stresses
    }

    /// Compute reaction forces at fixed DOFs.
    ///
    /// Reactions are `K_original * u` evaluated at the fixed DOFs, using the
    /// unmodified stiffness matrix (before Dirichlet conditions were applied)
    /// so that the true support forces are recovered rather than the prescribed
    /// displacement values.
    pub fn reactions(&self, model: &FemModel) -> Vec<(usize, usize, f64)> {
        let mut reactions = Vec::new();
        for (node, dof, _) in &model.dirichlet_bcs {
            if let Some(&idx) = self.dof_map.get(&(*node, *dof)) {
                // Reaction = K_original * u
                let mut reaction = 0.0;
                for j in 0..self.n_dof {
                    reaction += self.k_original.get(idx, j) * self.u_global[j];
                }
                reactions.push((*node, *dof, reaction));
            }
        }
        reactions
    }
}

/// Sparse matrix in CSR format (simplified for small problems).
#[derive(Debug, Clone)]
pub struct SprsMatrix {
    n: usize,
    /// Row pointers
    row_ptr: Vec<usize>,
    /// Column indices
    col_idx: Vec<usize>,
    /// Values
    values: Vec<f64>,
    /// Temporary storage for assembly (triplet format)
    triplets: Vec<(usize, usize, f64)>,
    /// Whether matrix is finalized
    finalized: bool,
}

impl SprsMatrix {
    pub fn new(n: usize) -> Self {
        Self {
            n,
            row_ptr: vec![0; n + 1],
            col_idx: Vec::new(),
            values: Vec::new(),
            triplets: Vec::new(),
            finalized: false,
        }
    }

    /// Add value to matrix (accumulates).
    pub fn add(&mut self, row: usize, col: usize, value: f64) {
        if value.abs() < 1e-15 {
            return;
        }
        self.triplets.push((row, col, value));
        self.finalized = false;
    }

    /// Set value (replaces existing).
    pub fn set(&mut self, row: usize, col: usize, value: f64) {
        self.finalize();
        for i in self.row_ptr[row]..self.row_ptr[row + 1] {
            if self.col_idx[i] == col {
                self.values[i] = value;
                return;
            }
        }
        // Entry doesn't exist — need to insert into CSR
        let pos = self.row_ptr[row + 1];
        self.col_idx.insert(pos, col);
        self.values.insert(pos, value);
        for r in (row + 1)..=self.n {
            self.row_ptr[r] += 1;
        }
    }

    /// Get value.
    pub fn get(&self, row: usize, col: usize) -> f64 {
        if !self.finalized {
            return 0.0; // Not valid until finalized
        }
        for i in self.row_ptr[row]..self.row_ptr[row + 1] {
            if self.col_idx[i] == col {
                return self.values[i];
            }
        }
        0.0
    }

    /// Finalize matrix (convert to CSR).
    fn finalize(&mut self) {
        if self.finalized {
            return;
        }
        // Sort triplets
        self.triplets.sort_by_key(|&(r, c, _)| (r, c));

        // Combine duplicates
        let mut combined: Vec<(usize, usize, f64)> = Vec::new();
        for (r, c, v) in self.triplets.drain(..) {
            if let Some(last) = combined.last_mut() {
                if last.0 == r && last.1 == c {
                    last.2 += v;
                } else {
                    combined.push((r, c, v));
                }
            } else {
                combined.push((r, c, v));
            }
        }

        // Build CSR
        self.row_ptr.fill(0);
        for (r, _, _) in &combined {
            self.row_ptr[*r + 1] += 1;
        }
        for i in 1..=self.n {
            self.row_ptr[i] += self.row_ptr[i - 1];
        }

        self.col_idx = Vec::with_capacity(combined.len());
        self.values = Vec::with_capacity(combined.len());
        for (_, c, v) in combined {
            self.col_idx.push(c);
            self.values.push(v);
        }

        self.finalized = true;
    }

    /// Zero out row and column.
    pub fn zero_row_col(&mut self, idx: usize) {
        self.finalize();
        let row_start = self.row_ptr[idx];
        let row_end = self.row_ptr[idx + 1];
        for i in row_start..row_end {
            self.values[i] = 0.0;
        }
        // Also zero column entries
        for r in 0..self.n {
            let rs = self.row_ptr[r];
            let re = self.row_ptr[r + 1];
            for i in rs..re {
                if self.col_idx[i] == idx {
                    self.values[i] = 0.0;
                }
            }
        }
    }

    /// Solve using Cholesky decomposition (simplified - uses dense for small systems).
    pub fn solve_cholesky(&mut self, rhs: &[f64]) -> Option<Vec<f64>> {
        self.finalize();

        // For small systems, convert to dense and use Cholesky
        if self.n <= 500 {
            let mut a = vec![vec![0.0; self.n]; self.n];
            for i in 0..self.n {
                for j_idx in self.row_ptr[i]..self.row_ptr[i + 1] {
                    let j = self.col_idx[j_idx];
                    a[i][j] = self.values[j_idx];
                }
            }

            // Cholesky decomposition
            let mut l = vec![vec![0.0; self.n]; self.n];
            for i in 0..self.n {
                for j in 0..=i {
                    let mut sum = a[i][j];
                    for k in 0..j {
                        sum -= l[i][k] * l[j][k];
                    }
                    if i == j {
                        if sum <= 1e-12 {
                            return None; // Not positive definite
                        }
                        l[i][j] = sum.sqrt();
                    } else {
                        l[i][j] = sum / l[j][j];
                    }
                }
            }

            // Forward substitution L*y = b
            let mut y = vec![0.0; self.n];
            for i in 0..self.n {
                let mut sum = rhs[i];
                for k in 0..i {
                    sum -= l[i][k] * y[k];
                }
                y[i] = sum / l[i][i];
            }

            // Back substitution L^T*x = y
            let mut x = vec![0.0; self.n];
            for i in (0..self.n).rev() {
                let mut sum = y[i];
                for k in i + 1..self.n {
                    sum -= l[k][i] * x[k];
                }
                x[i] = sum / l[i][i];
            }

            Some(x)
        } else {
            // For larger systems, would use iterative solver
            None
        }
    }
}

/// Element stiffness matrix for 3-node triangle (CST).
pub fn element_stiffness_tri3(
    mesh: &Mesh,
    element: &[usize; 3],
    props: &MaterialProps,
) -> [[f64; 6]; 6] {
    let p1 = mesh.nodes[element[0]];
    let p2 = mesh.nodes[element[1]];
    let p3 = mesh.nodes[element[2]];

    // Area
    let area = 0.5 * ((p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y)).abs();

    // B matrix (3x6)
    let b = strain_displacement_matrix_tri3(mesh, element);

    // D matrix
    let d = props.d_matrix();

    // ke = B^T * D * B * area * thickness (thickness = 1 for plane stress/strain)
    let thickness = 1.0;
    let mut ke = [[0.0; 6]; 6];

    for i in 0..6 {
        for j in 0..6 {
            let mut sum = 0.0;
            for k in 0..3 {
                for l in 0..3 {
                    sum += b[k][i] * d[k][l] * b[l][j];
                }
            }
            ke[i][j] = sum * area * thickness;
        }
    }

    ke
}

/// Strain-displacement matrix B for 3-node triangle (3x6).
pub fn strain_displacement_matrix_tri3(mesh: &Mesh, element: &[usize; 3]) -> [[f64; 6]; 3] {
    let p1 = mesh.nodes[element[0]];
    let p2 = mesh.nodes[element[1]];
    let p3 = mesh.nodes[element[2]];

    let area2 = (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y);
    let _area = 0.5 * area2.abs();

    // Shape function derivatives (constant for linear triangle)
    let b1 = (p2.y - p3.y) / area2;
    let c1 = (p3.x - p2.x) / area2;
    let b2 = (p3.y - p1.y) / area2;
    let c2 = (p1.x - p3.x) / area2;
    let b3 = (p1.y - p2.y) / area2;
    let c3 = (p2.x - p1.x) / area2;

    [
        [b1, 0.0, b2, 0.0, b3, 0.0],
        [0.0, c1, 0.0, c2, 0.0, c3],
        [c1, b1, c2, b2, c3, b3],
    ]
}

/// Consistent body force vector for 3-node triangle.
pub fn element_body_force_tri3(
    mesh: &Mesh,
    element: &[usize; 3],
    body_force: &[f64; 3], // [fx, fy, fz] per unit volume
) -> [f64; 6] {
    let p1 = mesh.nodes[element[0]];
    let p2 = mesh.nodes[element[1]];
    let p3 = mesh.nodes[element[2]];

    let area = 0.5 * ((p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y)).abs();
    let thickness = 1.0;
    let volume = area * thickness;

    // Consistent load: N^T * b * volume, N = [1/3, 1/3, 1/3] for centroid
    let factor = volume / 3.0;
    [
        body_force[0] * factor,
        body_force[1] * factor,
        body_force[0] * factor,
        body_force[1] * factor,
        body_force[0] * factor,
        body_force[1] * factor,
    ]
}

/// High-level function to solve a section under load.
pub fn analyze_section(
    section: &Section,
    material: &Material,
    params: MeshParams,
    loads: &[LoadCase],
) -> Result<AnalysisResults, FemError> {
    let mesh = crate::mesh::mesh_section(section, params);

    let mut model = FemModel::new(mesh);
    let mat_props = MaterialProps::from_material(material, true); // plane stress
    model.add_material(mat_props);
    model.set_problem_type(ProblemType::PlaneStress);

    // Apply loads
    for load in loads {
        match load {
            LoadCase::NodalForce { node, fx, fy } => {
                model.add_nodal_force(*node, 0, *fx);
                model.add_nodal_force(*node, 1, *fy);
            }
            LoadCase::BodyForce { fx, fy } => {
                model.add_body_force(*fx, *fy);
            }
            LoadCase::FixedNode { node } => {
                model.fix_node(*node);
            }
            LoadCase::RollerX { node } => {
                model.add_dirichlet_bc(*node, 0, 0.0);
            }
            LoadCase::RollerY { node } => {
                model.add_dirichlet_bc(*node, 1, 0.0);
            }
        }
    }

    let mut solver = FemSolver::from_model(&model)?;
    solver.solve()?;

    Ok(AnalysisResults {
        displacements: (0..model.mesh.n_nodes())
            .map(|i| solver.displacement(i))
            .collect(),
        element_stresses: solver.all_element_stresses(&model),
        nodal_stresses: solver.nodal_stresses(&model),
        reactions: solver.reactions(&model),
    })
}

/// Load cases for section analysis.
#[derive(Debug, Clone)]
pub enum LoadCase {
    NodalForce { node: usize, fx: f64, fy: f64 },
    BodyForce { fx: f64, fy: f64 },
    FixedNode { node: usize },
    RollerX { node: usize },
    RollerY { node: usize },
}

/// Analysis results.
#[derive(Debug, Clone)]
pub struct AnalysisResults {
    pub displacements: Vec<Point>,
    pub element_stresses: Vec<StressResult>,
    pub nodal_stresses: Vec<StressResult>,
    pub reactions: Vec<(usize, usize, f64)>,
}

/// FEM errors.
#[derive(Debug, Clone, PartialEq)]
pub enum FemError {
    SingularMatrix,
    InvalidMesh,
    MaterialNotFound,
    ConvergenceFailed,
    DegenerateElement,
    InvalidElementOrientation,
}

impl std::fmt::Display for FemError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FemError::SingularMatrix => write!(f, "Singular stiffness matrix"),
            FemError::InvalidMesh => write!(f, "Invalid mesh"),
            FemError::MaterialNotFound => write!(f, "Material not found"),
            FemError::ConvergenceFailed => write!(f, "Solver did not converge"),
            FemError::DegenerateElement => write!(f, "Degenerate element (zero or negative Jacobian)"),
            FemError::InvalidElementOrientation => write!(f, "Invalid element orientation (negative Jacobian, CW winding)"),
        }
    }
}

impl std::error::Error for FemError {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;
    use crate::material::presets::STEEL_S355;
    

    #[test]
    fn material_props_d_matrix() {
        let props = MaterialProps::plane_stress(&STEEL_S355);
        let d = props.d_matrix();
        assert!(d[0][0] > 0.0);
        assert!(d[1][1] > 0.0);
        assert!(d[2][2] > 0.0);
    }

    #[test]
    fn stress_result_von_mises() {
        let stress = StressResult::from_stress(100.0, 50.0, 30.0);
        let expected =
            (100.0_f64.powi(2) - 100.0 * 50.0 + 50.0_f64.powi(2) + 3.0 * 30.0_f64.powi(2)).sqrt();
        assert!((stress.von_mises - expected).abs() < 1e-6);
    }

    #[test]
    fn tri3_stiffness() {
        let mut mesh = Mesh::new();
        mesh.nodes = vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(0.0, 1.0),
        ];
        mesh.elements = vec![[0, 1, 2]];

        let props = MaterialProps::plane_stress(&STEEL_S355);
        let ke = element_stiffness_tri3(&mesh, &[0, 1, 2], &props);

        // Stiffness matrix should be symmetric
        for i in 0..6 {
            for j in 0..6 {
                assert!((ke[i][j] - ke[j][i]).abs() < 1e-10);
            }
        }
        // Diagonal should be positive
        for i in 0..6 {
            assert!(ke[i][i] > 0.0);
        }
    }

    #[test]
    fn simple_cantilever() {
        // Simple cantilever beam: fixed at x=0, force at tip
        let mut mesh = Mesh::new();
        mesh.nodes = vec![
            Point::new(0.0, 0.0), // 0 - fixed
            Point::new(1.0, 0.0), // 1
            Point::new(0.0, 0.1), // 2 - fixed
            Point::new(1.0, 0.1), // 3 - tip load
        ];
        mesh.elements = vec![[0, 1, 2], [1, 3, 2]];

        let mut model = FemModel::new(mesh);
        let props = MaterialProps::plane_stress(&STEEL_S355);
        model.add_material(props);

        // Fix left end
        model.fix_node(0);
        model.fix_node(2);

        // Apply vertical force at tip
        model.add_nodal_force(3, 1, -1000.0);

        let mut solver = FemSolver::from_model(&model).unwrap();
        solver.solve().unwrap();

        // Tip should deflect downward
        let disp = solver.displacement(3);
        assert!(disp.y < 0.0);
    }

    #[test]
    fn reactions_balance_applied_load() {
        // A single downward tip load on a cantilever must be balanced by
        // upward reactions at the fixed support. This guards against the
        // regression where reactions were computed from the already
        // boundary-condition-modified stiffness matrix (returning ~0).
        let mut mesh = Mesh::new();
        mesh.nodes = vec![
            Point::new(0.0, 0.0), // 0 - fixed
            Point::new(1.0, 0.0), // 1
            Point::new(0.0, 0.1), // 2 - fixed
            Point::new(1.0, 0.1), // 3 - tip load
        ];
        mesh.elements = vec![[0, 1, 2], [1, 3, 2]];

        let mut model = FemModel::new(mesh);
        let props = MaterialProps::plane_stress(&STEEL_S355);
        model.add_material(props);

        model.fix_node(0);
        model.fix_node(2);
        model.add_nodal_force(3, 1, -1000.0);

        let mut solver = FemSolver::from_model(&model).unwrap();
        solver.solve().unwrap();

        let reactions = solver.reactions(&model);

        // Sum the vertical (dof 1) reactions at the two fixed nodes.
        let mut vertical_reaction = 0.0;
        for (node, dof, value) in &reactions {
            assert!(node == &0 || node == &2, "reactions only at fixed nodes");
            if *dof == 1 {
                vertical_reaction += value;
            }
        }

        // The fixed support must push up to balance the -1000 downward load.
        assert!(
            (vertical_reaction - 1000.0).abs() < 1e-6,
            "vertical reactions should balance the applied load, got {vertical_reaction}"
        );
    }
}
