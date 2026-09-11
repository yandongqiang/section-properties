//! 2D Euler-Bernoulli Beam FEM
//!
//! Implements a 2D frame/beam element with 3 DOF per node: [u, v, θ]
//! where u = axial displacement, v = transverse displacement, θ = rotation.
//!
//! The element has 6 DOF total: [u_i, v_j, θ_i, u_j, v_j, θ_j]
//!
//! Uses engineering-mnemonic local variable names (`L`, `E`, `A`, `I`, `P`,
//! `T`, `N1`..`N4`, etc.) throughout, so `non_snake_case` is allowed here.
#![allow(non_snake_case)]

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
    pub second_moment: f64,
}

impl BeamSection {
    /// Create a new beam section
    pub fn new(area: f64, second_moment: f64) -> Self {
        Self {
            area,
            second_moment,
        }
    }

    /// Create from a rectangle section
    pub fn rectangle(width: f64, height: f64) -> Self {
        let area = width * height;
        let second_moment = width * height.powi(3) / 12.0;
        Self {
            area,
            second_moment,
        }
    }

    /// Create from a circular section
    pub fn circle(radius: f64) -> Self {
        let area = std::f64::consts::PI * radius * radius;
        let second_moment = std::f64::consts::PI * radius.powi(4) / 4.0;
        Self {
            area,
            second_moment,
        }
    }

    /// Create from a circular hollow section
    pub fn circle_hollow(outer_radius: f64, inner_radius: f64) -> Self {
        let area =
            std::f64::consts::PI * (outer_radius * outer_radius - inner_radius * inner_radius);
        let second_moment =
            std::f64::consts::PI * (outer_radius.powi(4) - inner_radius.powi(4)) / 4.0;
        Self {
            area,
            second_moment,
        }
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
        let I = self.section.second_moment;

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

    /// Compute consistent nodal load vector for uniform distributed load (6 DOF) in LOCAL coordinates
    ///
    /// Local DOF ordering: [u_i, v_i, θ_i, u_j, v_j, θ_j]
    /// Sign convention:
    /// - qx > 0: tensile axial load (pulling beam in +x direction)
    /// - qy > 0: upward transverse load (in +y direction)
    ///
    /// Returns a 6-element vector in local coordinates
    pub fn consistent_nodal_load(
        &self,
        node_i: Point,
        node_j: Point,
        qx: f64,
        qy: f64,
    ) -> Result<[f64; 6], FemError> {
        let L = self.length(node_i, node_j);
        if L <= 0.0 {
            return Err(FemError::InvalidInput(
                "Beam element has zero or negative length for distributed load".to_string(),
            ));
        }

        // Consistent nodal load vector for uniform distributed load
        // Axial (qx): linear shape functions
        // f_u_i = qx * L / 2, f_u_j = qx * L / 2
        //
        // Transverse (qy): Hermite cubic shape functions
        // f_v_i = qy * L / 2
        // f_θ_i = qy * L^2 / 12  (positive = counterclockwise in local coords)
        // f_v_j = qy * L / 2
        // f_θ_j = -qy * L^2 / 12 (negative = clockwise in local coords)
        //
        // Note: Local v is positive upward, local θ is positive counterclockwise
        let qx_L2 = qx * L / 2.0;
        let qy_L2 = qy * L / 2.0;
        let qy_L2_12 = qy * L * L / 12.0;

        Ok([
            qx_L2,     // u_i: axial
            qy_L2,     // v_i: transverse
            qy_L2_12,  // θ_i: moment (CCW positive)
            qx_L2,     // u_j: axial
            qy_L2,     // v_j: transverse
            -qy_L2_12, // θ_j: moment (CW negative)
        ])
    }

    /// Compute consistent nodal load vector for a point load at given position (6 DOF) in LOCAL coordinates
    ///
    /// Local DOF ordering: [u_i, v_i, θ_i, u_j, v_j, θ_j]
    /// Position is measured from node_i (0.0) to node_j (1.0)
    /// Sign convention:
    /// - fx > 0: tensile axial force (pulling in +x direction)
    /// - fy > 0: upward transverse force (in +y direction)
    /// - mz > 0: counterclockwise moment about z-axis (positive θ direction)
    ///
    /// Uses shape functions:
    /// - Axial: linear shape functions N1 = 1-ξ, N2 = ξ
    /// - Transverse: Hermite cubic shape functions
    /// - Moment: shape functions for moment
    pub fn consistent_nodal_load_point(
        &self,
        node_i: Point,
        node_j: Point,
        position: f64,
        fx: f64,
        fy: f64,
        mz: f64,
    ) -> Result<[f64; 6], FemError> {
        let L = self.length(node_i, node_j);
        if L <= 0.0 {
            return Err(FemError::InvalidInput(
                "Beam element has zero or negative length for point load".to_string(),
            ));
        }
        if !(0.0..=1.0).contains(&position) {
            return Err(FemError::InvalidInput(format!(
                "Point load position must be in [0, 1], got {}",
                position
            )));
        }

        let xi = position; // normalized position
        let one_minus_xi = 1.0 - xi;

        // Axial: linear shape functions
        // N1 = 1-ξ, N2 = ξ
        // f_u_i = fx * (1-ξ), f_u_j = fx * ξ
        let f_u_i = fx * one_minus_xi;
        let f_u_j = fx * xi;

        // Transverse: Hermite cubic shape functions
        // N1 = 1 - 3ξ² + 2ξ³
        // N2 = L(ξ - 2ξ² + ξ³)
        // N3 = 3ξ² - 2ξ³
        // N4 = L(-ξ² + ξ³)
        let xi2 = xi * xi;
        let xi3 = xi2 * xi;

        let N1 = 1.0 - 3.0 * xi2 + 2.0 * xi3;
        let N2 = L * (xi - 2.0 * xi2 + xi3);
        let N3 = 3.0 * xi2 - 2.0 * xi3;
        let N4 = L * (-xi2 + xi3);

        // f_v_i = fy * N1, f_θ_i = fy * N2
        // f_v_j = fy * N3, f_θ_j = fy * N4
        let f_v_i = fy * N1;
        let f_theta_i = fy * N2;
        let f_v_j = fy * N3;
        let f_theta_j = fy * N4;

        // Moment: shape functions for applied moment (from virtual work: δW = M * δθ(x_p))
        // Rotation shape functions: θ(x) = dN1/dx * v_i + dN2/dx * θ_i + dN3/dx * v_j + dN4/dx * θ_j
        // M1 = dN1/dx = (6/L)(ξ² - ξ) = -(6/L)ξ(1-ξ)
        // M2 = dN2/dx = 1 - 4ξ + 3ξ²
        // M3 = dN3/dx = (6/L)(ξ - ξ²) = (6/L)ξ(1-ξ)
        // M4 = dN4/dx = -2ξ + 3ξ²
        // For a point moment mz at ξ, the equivalent nodal loads are:
        // f_v_i = mz * M1 = -6mz/L * ξ(1-ξ)
        // f_θ_i = mz * M2 = mz * (1 - 4ξ + 3ξ²)
        // f_v_j = mz * M3 = 6mz/L * ξ(1-ξ)
        // f_θ_j = mz * M4 = mz * (-2ξ + 3ξ²)
        let one_minus_xi = 1.0 - xi;
        let f_v_i_moment = mz * (-6.0 * xi * one_minus_xi / L); // M1 = -(6/L)ξ(1-ξ)
        let f_theta_i_moment = mz * (1.0 - 4.0 * xi + 3.0 * xi2); // M2 = 1 - 4ξ + 3ξ²
        let f_v_j_moment = mz * (6.0 * xi * one_minus_xi / L); // M3 = (6/L)ξ(1-ξ)
        let f_theta_j_moment = mz * (-2.0 * xi + 3.0 * xi2); // M4 = -2ξ + 3ξ²

        Ok([
            f_u_i,                        // u_i: axial
            f_v_i + f_v_i_moment,         // v_i: transverse + moment
            f_theta_i + f_theta_i_moment, // θ_i: rotation + moment
            f_u_j,                        // u_j: axial
            f_v_j + f_v_j_moment,         // v_j: transverse + moment
            f_theta_j + f_theta_j_moment, // θ_j: rotation + moment
        ])
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

/// Distributed load on a beam element in LOCAL coordinates
///
/// Local coordinate system:
/// - x axis: along beam from node_i to node_j (axial direction)
/// - y axis: transverse direction (positive v)
/// - z axis: out of plane (rotation θ about z)
///
/// Sign convention:
/// - qx > 0: tensile axial load (pulling beam in +x direction)
/// - qy > 0: upward transverse load (in +y direction)
#[derive(Debug, Clone, Copy)]
pub struct DistributedLoad {
    /// Element index (index in elements Vec)
    pub element_idx: usize,
    /// Uniform axial load per unit length [N/m] in local x direction
    pub qx: f64,
    /// Uniform transverse load per unit length [N/m] in local y direction
    pub qy: f64,
}

impl DistributedLoad {
    /// Create a new distributed load on an element
    pub fn new(element_idx: usize, qx: f64, qy: f64) -> Self {
        Self {
            element_idx,
            qx,
            qy,
        }
    }

    /// Uniform transverse load only (qy)
    pub fn transverse(element_idx: usize, qy: f64) -> Self {
        Self {
            element_idx,
            qx: 0.0,
            qy,
        }
    }

    /// Uniform axial load only (qx)
    pub fn axial(element_idx: usize, qx: f64) -> Self {
        Self {
            element_idx,
            qx,
            qy: 0.0,
        }
    }
}

/// Point load applied at a specific position along an element in LOCAL coordinates
///
/// Local coordinate system:
/// - x axis: along beam from node_i to node_j (axial direction)
/// - y axis: transverse direction (positive v)
/// - z axis: out of plane (rotation θ about z)
///
/// Position is measured from node_i (x=0) to node_j (x=L)
///
/// Sign convention:
/// - fx > 0: tensile axial force (pulling in +x direction)
/// - fy > 0: upward transverse force (in +y direction)
/// - mz > 0: counterclockwise moment about z-axis (positive θ direction)
#[derive(Debug, Clone, Copy)]
pub struct PointLoad {
    /// Element index (index in elements Vec)
    pub element_idx: usize,
    /// Position along element from node_i (0.0 = node_i, 1.0 = node_j)
    pub position: f64,
    /// Axial force [N] in local x direction
    pub fx: f64,
    /// Transverse force [N] in local y direction
    pub fy: f64,
    /// Moment [Nm] about local z axis (positive = CCW)
    pub mz: f64,
}

impl PointLoad {
    /// Create a new point load on an element
    pub fn new(element_idx: usize, position: f64, fx: f64, fy: f64, mz: f64) -> Self {
        Self {
            element_idx,
            position,
            fx,
            fy,
            mz,
        }
    }

    /// Create a transverse point force only
    pub fn transverse_force(element_idx: usize, position: f64, fy: f64) -> Self {
        Self {
            element_idx,
            position,
            fx: 0.0,
            fy,
            mz: 0.0,
        }
    }

    /// Create an axial point force only
    pub fn axial_force(element_idx: usize, position: f64, fx: f64) -> Self {
        Self {
            element_idx,
            position,
            fx,
            fy: 0.0,
            mz: 0.0,
        }
    }

    /// Create a moment only
    pub fn moment(element_idx: usize, position: f64, mz: f64) -> Self {
        Self {
            element_idx,
            position,
            fx: 0.0,
            fy: 0.0,
            mz,
        }
    }
}

/// Applied moment at a specific node (in GLOBAL coordinates)
///
/// Sign convention:
/// - value > 0: counterclockwise moment (positive θ direction)
#[derive(Debug, Clone, Copy)]
pub struct AppliedMoment {
    /// Node index (index in nodes Vec)
    pub node_idx: usize,
    /// Moment value [Nm] in global coordinates (positive = CCW)
    pub value: f64,
}

impl AppliedMoment {
    pub fn new(node_idx: usize, value: f64) -> Self {
        Self { node_idx, value }
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
    /// Distributed loads on elements (in LOCAL coordinates)
    pub distributed_loads: Vec<DistributedLoad>,
    /// Point loads on elements (in LOCAL coordinates)
    pub point_loads: Vec<PointLoad>,
    /// Applied moments at nodes (in GLOBAL coordinates)
    pub applied_moments: Vec<AppliedMoment>,
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
            distributed_loads: Vec::new(),
            point_loads: Vec::new(),
            applied_moments: Vec::new(),
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
        if node_idx >= self.nodes.len() {
            panic!(
                "Invalid node index: {} (max: {})",
                node_idx,
                self.nodes.len().saturating_sub(1)
            );
        }
        if dof >= 3 {
            panic!("Invalid DOF: {} (must be 0, 1, or 2)", dof);
        }
        self.nodal_forces.push((node_idx, dof, value));
    }

    /// Fix a DOF to a prescribed value
    /// dof: 0=u, 1=v, 2=θ
    /// node_idx is the index in the nodes Vec (0, 1, 2, ...)
    pub fn fix_dof(&mut self, node_idx: usize, dof: usize, value: f64) {
        if node_idx >= self.nodes.len() {
            panic!(
                "Invalid node index: {} (max: {})",
                node_idx,
                self.nodes.len().saturating_sub(1)
            );
        }
        if dof >= 3 {
            panic!("Invalid DOF: {} (must be 0, 1, or 2)", dof);
        }
        self.fixed_dofs.push((node_idx, dof, value));
    }

    /// Fix a node completely (all 3 DOFs to 0)
    /// node_idx is the index in the nodes Vec (0, 1, 2, ...)
    pub fn fix_node(&mut self, node_idx: usize) {
        if node_idx >= self.nodes.len() {
            panic!(
                "Invalid node index: {} (max: {})",
                node_idx,
                self.nodes.len().saturating_sub(1)
            );
        }
        self.fix_dof(node_idx, 0, 0.0);
        self.fix_dof(node_idx, 1, 0.0);
        self.fix_dof(node_idx, 2, 0.0);
    }

    /// Add a distributed load on an element (in LOCAL coordinates)
    /// element_idx is the index in the elements Vec (0, 1, 2, ...)
    pub fn add_distributed_load(
        &mut self,
        element_idx: usize,
        qx: f64,
        qy: f64,
    ) -> Result<(), FemError> {
        if element_idx >= self.elements.len() {
            return Err(FemError::InvalidInput(format!(
                "Invalid element index: {} (max: {})",
                element_idx,
                self.elements.len().saturating_sub(1)
            )));
        }
        self.distributed_loads
            .push(DistributedLoad::new(element_idx, qx, qy));
        Ok(())
    }

    /// Add a point load on an element (in LOCAL coordinates)
    /// element_idx is the index in the elements Vec (0, 1, 2, ...)
    /// position is measured from node_i (0.0) to node_j (1.0)
    pub fn add_point_load(
        &mut self,
        element_idx: usize,
        position: f64,
        fx: f64,
        fy: f64,
        mz: f64,
    ) -> Result<(), FemError> {
        if element_idx >= self.elements.len() {
            return Err(FemError::InvalidInput(format!(
                "Invalid element index: {} (max: {})",
                element_idx,
                self.elements.len().saturating_sub(1)
            )));
        }
        if !(0.0..=1.0).contains(&position) {
            return Err(FemError::InvalidInput(format!(
                "Point load position must be in [0, 1], got {}",
                position
            )));
        }
        self.point_loads
            .push(PointLoad::new(element_idx, position, fx, fy, mz));
        Ok(())
    }

    /// Add a point force only (transverse or axial) on an element
    pub fn add_point_force(
        &mut self,
        element_idx: usize,
        position: f64,
        fx: f64,
        fy: f64,
    ) -> Result<(), FemError> {
        self.add_point_load(element_idx, position, fx, fy, 0.0)
    }

    /// Add a point moment only on an element
    pub fn add_point_moment(
        &mut self,
        element_idx: usize,
        position: f64,
        mz: f64,
    ) -> Result<(), FemError> {
        self.add_point_load(element_idx, position, 0.0, 0.0, mz)
    }

    /// Add an applied moment at a node (in GLOBAL coordinates)
    pub fn add_applied_moment(&mut self, node_idx: usize, value: f64) -> Result<(), FemError> {
        if node_idx >= self.nodes.len() {
            return Err(FemError::InvalidInput(format!(
                "Invalid node index: {} (max: {})",
                node_idx,
                self.nodes.len().saturating_sub(1)
            )));
        }
        self.applied_moments
            .push(AppliedMoment::new(node_idx, value));
        Ok(())
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

        // Validate element node indices
        for (elem_idx, element) in model.elements.iter().enumerate() {
            if element.node_i >= model.nodes.len() {
                return Err(FemError::InvalidModel(format!(
                    "Element {}: node_i index {} out of bounds (max: {})",
                    elem_idx,
                    element.node_i,
                    model.nodes.len().saturating_sub(1)
                )));
            }
            if element.node_j >= model.nodes.len() {
                return Err(FemError::InvalidModel(format!(
                    "Element {}: node_j index {} out of bounds (max: {})",
                    elem_idx,
                    element.node_j,
                    model.nodes.len().saturating_sub(1)
                )));
            }
            if element.node_i == element.node_j {
                return Err(FemError::InvalidModel(format!(
                    "Element {}: node_i and node_j cannot be the same",
                    elem_idx
                )));
            }
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
                    "Beam element has zero or negative length (nodes {} and {} at same position)",
                    element.node_i, element.node_j
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
            if *node_id >= model.nodes.len() {
                return Err(FemError::InvalidModel(format!(
                    "Nodal force: node index {} out of bounds (max: {})",
                    node_id,
                    model.nodes.len().saturating_sub(1)
                )));
            }
            if *dof >= 3 {
                return Err(FemError::InvalidModel(format!(
                    "Nodal force: DOF {} invalid (must be 0, 1, or 2)",
                    dof
                )));
            }
            let idx = model.dof_index(*node_id, *dof);
            f_global[idx] += value;
        }

        // Assemble distributed loads
        for dl in &model.distributed_loads {
            if dl.element_idx >= model.elements.len() {
                return Err(FemError::InvalidModel(format!(
                    "Distributed load: element index {} out of bounds (max: {})",
                    dl.element_idx,
                    model.elements.len().saturating_sub(1)
                )));
            }
            let element = &model.elements[dl.element_idx];
            let node_i = model.nodes[element.node_i].point();
            let node_j = model.nodes[element.node_j].point();

            // Check for zero-length beam
            let dx = node_j.x - node_i.x;
            let dy = node_j.y - node_i.y;
            let L = (dx * dx + dy * dy).sqrt();
            if L <= 0.0 {
                return Err(FemError::InvalidModel(format!(
                    "Distributed load: beam element has zero or negative length (nodes {} and {} at same position)",
                    element.node_i, element.node_j
                )));
            }

            // Compute consistent nodal load in local coordinates
            let f_local = element.consistent_nodal_load(node_i, node_j, dl.qx, dl.qy)?;

            // Get transformation matrix
            let T = element.transformation_matrix(node_i, node_j);

            // Transform to global coordinates: f_global_elem = T^T * f_local
            let mut f_global_elem = [0.0; 6];
            for i in 0..6 {
                for j in 0..6 {
                    f_global_elem[i] += T[j][i] * f_local[j];
                }
            }

            // Map element DOFs to global DOFs
            let dof_map = [
                model.dof_index(element.node_i, 0), // u_i
                model.dof_index(element.node_i, 1), // v_i
                model.dof_index(element.node_i, 2), // θ_i
                model.dof_index(element.node_j, 0), // u_j
                model.dof_index(element.node_j, 1), // v_j
                model.dof_index(element.node_j, 2), // θ_j
            ];

            // Assemble into global force vector
            for a in 0..6 {
                f_global[dof_map[a]] += f_global_elem[a];
            }
        }

        // Assemble point loads
        for pl in &model.point_loads {
            if pl.element_idx >= model.elements.len() {
                return Err(FemError::InvalidModel(format!(
                    "Point load: element index {} out of bounds (max: {})",
                    pl.element_idx,
                    model.elements.len().saturating_sub(1)
                )));
            }
            let element = &model.elements[pl.element_idx];
            let node_i = model.nodes[element.node_i].point();
            let node_j = model.nodes[element.node_j].point();

            // Check for zero-length beam
            let dx = node_j.x - node_i.x;
            let dy = node_j.y - node_i.y;
            let L = (dx * dx + dy * dy).sqrt();
            if L <= 0.0 {
                return Err(FemError::InvalidModel(format!(
                    "Point load: beam element has zero or negative length (nodes {} and {} at same position)",
                    element.node_i, element.node_j
                )));
            }

            // Compute consistent nodal load in local coordinates
            let f_local = element.consistent_nodal_load_point(
                node_i,
                node_j,
                pl.position,
                pl.fx,
                pl.fy,
                pl.mz,
            )?;

            // Get transformation matrix
            let T = element.transformation_matrix(node_i, node_j);

            // Transform to global coordinates: f_global_elem = T^T * f_local
            let mut f_global_elem = [0.0; 6];
            for i in 0..6 {
                for j in 0..6 {
                    f_global_elem[i] += T[j][i] * f_local[j];
                }
            }

            // Map element DOFs to global DOFs
            let dof_map = [
                model.dof_index(element.node_i, 0), // u_i
                model.dof_index(element.node_i, 1), // v_i
                model.dof_index(element.node_i, 2), // θ_i
                model.dof_index(element.node_j, 0), // u_j
                model.dof_index(element.node_j, 1), // v_j
                model.dof_index(element.node_j, 2), // θ_j
            ];

            // Assemble into global force vector
            for a in 0..6 {
                f_global[dof_map[a]] += f_global_elem[a];
            }
        }

        // Assemble applied moments (already in global coordinates)
        for am in &model.applied_moments {
            if am.node_idx >= model.nodes.len() {
                return Err(FemError::InvalidModel(format!(
                    "Applied moment: node index {} out of bounds (max: {})",
                    am.node_idx,
                    model.nodes.len().saturating_sub(1)
                )));
            }
            // Applied moment is in global coordinates, DOF 2 = θ
            let idx = model.dof_index(am.node_idx, 2);
            f_global[idx] += am.value;
        }

        // Fixed DOFs and prescribed values
        let mut fixed_dofs = vec![false; n_dof];
        let mut prescribed_values = vec![None; n_dof];
        for (node_id, dof, value) in &model.fixed_dofs {
            if *node_id >= model.nodes.len() {
                return Err(FemError::InvalidModel(format!(
                    "Fixed DOF: node index {} out of bounds (max: {})",
                    node_id,
                    model.nodes.len().saturating_sub(1)
                )));
            }
            if *dof >= 3 {
                return Err(FemError::InvalidModel(format!(
                    "Fixed DOF: DOF {} invalid (must be 0, 1, or 2)",
                    dof
                )));
            }
            let idx = model.dof_index(*node_id, *dof);
            fixed_dofs[idx] = true;
            prescribed_values[idx] = Some(*value);
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
                let _constrained_idx = constrained_dofs.len();
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

        // Extract free-free submatrix K_ff and force vector f_f
        let mut k_ff = SparseMatrix::new(n_free);
        let mut f_f = vec![0.0; n_free];

        // Compress original matrix to access CSR data
        let mut k_orig = self.k_global.clone();
        k_orig.compress();

        // Extract K_ff and compute K_fc * U_c directly
        let mut kfc_uc = vec![0.0; n_free];

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
                    // free-constrained: accumulate K_fc * U_c directly
                    let constr_idx = constrained_dofs
                        .iter()
                        .position(|&x| x == global_j)
                        .unwrap();
                    kfc_uc[free_idx] += val * constrained_values[constr_idx];
                }
            }
        }

        k_ff.compress();

        // Compute reduced RHS: F_reduced = F_f - K_fc * U_c
        let mut f_reduced = f_f;
        for free_idx in 0..n_free {
            f_reduced[free_idx] -= kfc_uc[free_idx];
        }

        (
            k_ff,
            f_reduced,
            free_to_global,
            constrained_dofs,
            constrained_values,
        )
    }

    #[allow(dead_code)]
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

    /// Access the original (unconstrained) global stiffness matrix.
    ///
    /// This is exposed crate-internally for debugging and diagnostics; it is
    /// not part of the public API surface.
    #[allow(dead_code)]
    pub(crate) fn stiffness_matrix(&self) -> &SparseMatrix {
        &self.k_original
    }

    /// Compute element end forces in LOCAL coordinates.
    ///
    /// Returns a vector of 6 forces per element in local coordinates:
    /// `[N_i, V_i, M_i, N_j, V_j, M_j]`, where `N` is axial force, `V` is
    /// transverse shear force, and `M` is bending moment.
    ///
    /// # Sign convention
    ///
    /// These are the forces the ELEMENT applies to its end NODES (reaction
    /// sense), i.e. `f_end = f_equiv - K_local·u_local`. Positive `N` is
    /// tension on the node, positive `V` is upward on the node, and positive
    /// `M` is counter-clockwise on the node.
    ///
    /// Applied moments at nodes are external concentrated loads whose effect is
    /// carried by the displacement field `u` (through the global force vector),
    /// so no separate correction is needed here. Recovered end moments therefore
    /// automatically satisfy nodal moment equilibrium:
    ///
    /// - free-end node with applied moment `M`: `M_element_j + M ≈ 0`;
    /// - internal node with applied moment `M` shared by two elements:
    ///   `M_left_j + M_right_i + M ≈ 0`.
    pub fn element_end_forces(&self) -> Vec<[f64; 6]> {
        let mut results = Vec::with_capacity(self.model.elements.len());

        for (elem_idx, element) in self.model.elements.iter().enumerate() {
            let node_i = self.model.nodes[element.node_i].point();
            let node_j = self.model.nodes[element.node_j].point();

            // Element displacement in global coordinates.
            let dof_map = [
                self.model.dof_index(element.node_i, 0), // u_i
                self.model.dof_index(element.node_i, 1), // v_i
                self.model.dof_index(element.node_i, 2), // θ_i
                self.model.dof_index(element.node_j, 0), // u_j
                self.model.dof_index(element.node_j, 1), // v_j
                self.model.dof_index(element.node_j, 2), // θ_j
            ];
            let u_global_elem = [
                self.u_global[dof_map[0]],
                self.u_global[dof_map[1]],
                self.u_global[dof_map[2]],
                self.u_global[dof_map[3]],
                self.u_global[dof_map[4]],
                self.u_global[dof_map[5]],
            ];

            // Transform to local coordinates: u_local = T * u_global.
            let t = element.transformation_matrix(node_i, node_j);
            let mut u_local = [0.0; 6];
            for i in 0..6 {
                for j in 0..6 {
                    u_local[i] += t[i][j] * u_global_elem[j];
                }
            }

            // Internal nodal force: f_stiffness = K_local * u_local.
            let k_local = element.local_stiffness(node_i, node_j);
            let mut f_stiffness = [0.0; 6];
            for i in 0..6 {
                for j in 0..6 {
                    f_stiffness[i] += k_local[i][j] * u_local[j];
                }
            }

            // Equivalent nodal forces from distributed and point loads on this
            // element. Applied moments at nodes are external concentrated loads,
            // excluded from the equivalent element loads.
            let f_equiv = Self::element_equivalent_nodal_forces(
                &self.model,
                elem_idx,
                element,
                node_i,
                node_j,
            );

            // Element-on-node end forces: f_end = f_equiv - f_stiffness.
            let end_forces = [
                f_equiv[0] - f_stiffness[0], // N_i: axial at node i
                f_equiv[1] - f_stiffness[1], // V_i: shear at node i
                f_equiv[2] - f_stiffness[2], // M_i: moment at node i
                f_equiv[3] - f_stiffness[3], // N_j: axial at node j
                f_equiv[4] - f_stiffness[4], // V_j: shear at node j
                f_equiv[5] - f_stiffness[5], // M_j: moment at node j
            ];

            results.push(end_forces);
        }

        results
    }

    /// Compute equivalent nodal forces (in LOCAL coordinates) for an element
    /// from DISTRIBUTED and POINT loads (NOT including applied moments at
    /// nodes, which are external concentrated loads handled in the global
    /// force vector).
    ///
    /// Returns `[N_i, V_i, M_i, N_j, V_j, M_j]` in local coordinates.
    fn element_equivalent_nodal_forces(
        model: &BeamModel,
        elem_idx: usize,
        element: &BeamElement,
        node_i: Point,
        node_j: Point,
    ) -> [f64; 6] {
        let mut f_equiv = [0.0; 6];

        // Distributed loads on this element.
        for dl in &model.distributed_loads {
            if dl.element_idx == elem_idx {
                let L = element.length(node_i, node_j);
                // Consistent nodal load for uniform distributed load:
                // f_u_i = qx*L/2, f_u_j = qx*L/2
                // f_v_i = qy*L/2, f_theta_i = qy*L^2/12
                // f_v_j = qy*L/2, f_theta_j = -qy*L^2/12
                let qx = dl.qx;
                let qy = dl.qy;
                let qx_L2 = qx * L / 2.0;
                let qy_L2 = qy * L / 2.0;
                let qy_L2_12 = qy * L * L / 12.0;

                f_equiv[0] += qx_L2; // N_i
                f_equiv[1] += qy_L2; // V_i
                f_equiv[2] += qy_L2_12; // M_i (CCW positive)
                f_equiv[3] += qx_L2; // N_j
                f_equiv[4] += qy_L2; // V_j
                f_equiv[5] -= qy_L2_12; // M_j (CW negative for qy > 0 upward)
            }
        }

        // Point loads on this element.
        for pl in &model.point_loads {
            if pl.element_idx == elem_idx {
                let f_local = element
                    .consistent_nodal_load_point(node_i, node_j, pl.position, pl.fx, pl.fy, pl.mz)
                    .unwrap_or([0.0; 6]);
                for i in 0..6 {
                    f_equiv[i] += f_local[i];
                }
            }
        }

        f_equiv
    }

    /// Compute element end forces in GLOBAL coordinates
    ///
    /// Returns a vector of 6 forces per element in global coordinates:
    /// [Fx_i, Fy_i, Mz_i, Fx_j, Fy_j, Mz_j]
    pub fn element_end_forces_global(&self) -> Vec<[f64; 6]> {
        let local_forces = self.element_end_forces();
        let mut results = Vec::new();

        for (idx, forces) in local_forces.iter().enumerate() {
            let element = &self.model.elements[idx];
            let node_i = self.model.nodes[element.node_i].point();
            let node_j = self.model.nodes[element.node_j].point();

            let T = element.transformation_matrix(node_i, node_j);

            // Transform local forces to global: f_global = T^T * f_local
            let mut f_global_elem = [0.0; 6];
            for i in 0..6 {
                for j in 0..6 {
                    f_global_elem[i] += T[j][i] * forces[j];
                }
            }

            results.push(f_global_elem);
        }

        results
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
    pub fn cantilever_tip_load(L: f64, E: f64, _A: f64, I: f64, P: f64) -> (f64, f64, f64) {
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
        assert_eq!(sec.second_moment, 1.0);

        let rect = BeamSection::rectangle(0.1, 0.2);
        assert!((rect.area - 0.02).abs() < 1e-10);
        assert!((rect.second_moment - 0.1 * 0.2_f64.powi(3) / 12.0).abs() < 1e-10);

        let circ = BeamSection::circle(0.05);
        assert!((circ.area - std::f64::consts::PI * 0.0025).abs() < 1e-10);
    }

    #[test]
    fn test_local_stiffness() {
        let material = Material::new(200e9, 0.3, 7850.0, "Steel");
        let section = BeamSection::new(0.01, 8.333e-6);
        let element = BeamElement::new(0, 1, material, section).unwrap();

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
        let E: f64 = 200e9;
        let A: f64 = 0.01;
        let L: f64 = 1.0;
        let EA_L = E * A / L;
        assert!((k[0][0] - EA_L).abs() < 1e-6);
        assert!((k[0][3] + EA_L).abs() < 1e-6);
        assert!((k[3][3] - EA_L).abs() < 1e-6);

        // Check bending terms
        let I: f64 = 8.333e-6;
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
        let element = BeamElement::new(0, 1, material, section).unwrap();

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
        let element = BeamElement::new(0, 1, material, section).unwrap();

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
        let element = BeamElement::new(0, 1, material, section).unwrap();

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
        model.add_element(BeamElement::new(0, 1, material, section).unwrap());

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
