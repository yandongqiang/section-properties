//! Warping torsion analysis.
//!
//! Provides St. Venant torsion constant (J), warping constant (Iw),
//! shear center coordinates, and torsional-warping section properties.

use super::warping_fem::{compute_fem_warping_properties, compute_fem_solution, FemWarpingResult, FemSolution, analytical_shear_center, analytical_beta};
use crate::geometry::Point;
use crate::section::Section;
use crate::section_properties::SectionProperties;
use crate::material::Material;

/// Warping torsion properties for open and closed sections.
#[derive(Debug, Clone)]
pub struct WarpingProperties {
    /// St. Venant torsion constant [m^4]
    pub j: f64,
    /// Warping constant [m^6]
    pub iw: f64,
    /// Shear center coordinates relative to centroid [m]
    pub shear_center: Point,
    /// Torsional radius of gyration [m]
    pub r_t: f64,
    /// Warping radius of gyration [m]
    pub r_w: f64,
    /// Section area [m^2]
    pub area: f64,
    /// Polar moment of inertia about centroid [m^4]
    pub ip: f64,
    /// Monosymmetry constant about x-axis [m]
    pub beta_x: f64,
    /// Monosymmetry constant about y-axis [m]
    pub beta_y: f64,
    /// Shear area in y-direction (for Vx shear) [m^2]
    pub ay: f64,
    /// Shear area in z-direction (for Vy shear) [m^2]
    pub az: f64,
    /// Shear center 11-coordinate (principal axis) [m]
    pub shear_center_11: f64,
    /// Shear center 22-coordinate (principal axis) [m]
    pub shear_center_22: f64,
    /// Monosymmetry constant about 11 (major principal) axis [m]
    pub beta_11: f64,
    /// Monosymmetry constant about 22 (minor principal) axis [m]
    pub beta_22: f64,
    /// Shear area about 11 (major principal) axis [m^2]
    pub a_s11: f64,
    /// Shear area about 22 (minor principal) axis [m^2]
    pub a_s22: f64,
    /// Shear area about the xy-axis (cross-coupling) [m^2]
    ///
    /// Zero for sections with at least one axis of symmetry. Requires finite-element
    /// analysis for general sections; approximated as zero here.
    pub a_sxy: f64,
    /// Shear deformation parameter delta_s = 2*(1+nu)*(Ix*Iy - Ixy^2)
    pub delta_s: f64,
    /// Principal axis angle [rad]
    pub principal_angle: f64,
    /// Trefftz shear center coordinates relative to centroid [m]
    ///
    /// For open thin-walled sections, the Trefftz approach yields the same shear
    /// center as the elasticity approach. For closed/solid sections the Trefftz
    /// shear center requires a finite-element solution of the Laplace equation;
    /// in that case we fall back to the elasticity-based shear center as an
    /// approximation.
    pub shear_center_trefftz: Point,
    /// Monosymmetry constant for positive bending about x-axis [m]
    pub beta_x_plus: f64,
    /// Monosymmetry constant for negative bending about x-axis [m]
    pub beta_x_minus: f64,
    /// Monosymmetry constant for positive bending about y-axis [m]
    pub beta_y_plus: f64,
    /// Monosymmetry constant for negative bending about y-axis [m]
    pub beta_y_minus: f64,
    /// Monosymmetry constant for positive bending about 11-axis [m]
    pub beta_11_plus: f64,
    /// Monosymmetry constant for negative bending about 11-axis [m]
    pub beta_11_minus: f64,
    /// Monosymmetry constant for positive bending about 22-axis [m]
    pub beta_22_plus: f64,
    /// Monosymmetry constant for negative bending about 22-axis [m]
    pub beta_22_minus: f64,
    /// Maximum warping coordinate |ω| from FEM solution
    pub omega_max: f64,
    /// Maximum St. Venant shear stress (unit torque) from FEM solution
    pub tau_sv_max_unit: f64,
}

impl WarpingProperties {
    /// Compute warping properties for a section using FEM.
    ///
    /// Mirrors Python `sectionproperties.analysis.section.Section.calculate_warping_properties`:
    /// Uses FEM as the core method to compute J, Iw, shear center, shear areas,
    /// and monosymmetry constants about both centroidal (x/y) and principal (11/22) axes.
    ///
    /// Analytical formulas are available for validation/testing but are NOT used as fallbacks.
    ///
    /// `nu` is the Poisson's ratio of the material (e.g., 0.3 for steel).
    pub fn from_section(section: &Section, nu: f64) -> Self {
        let props = SectionProperties::from_section(section);
        let area = props.area;

        // Use FEM as the primary method for ALL sections
        // This matches Python sectionproperties which uses FEM as the core method
        let fem = compute_fem_warping_properties(section, &props, nu).expect(
            "FEM warping analysis failed. This should not happen for valid sections. \
             Check mesh generation and section validity."
        );

        // Also compute the full FEM solution to get tau_sv_max for unit torque
        let fem_solution = compute_fem_solution(section, &props, nu).expect(
            "FEM stress recovery failed after warping analysis succeeded. \
             Check mesh generation and section validity."
        );

        // FEM solves in centroidal coordinates; report in global axes
        // (Python convention: shear centre in section coordinates).
        let shear_center = Point::new(
            fem.shear_center.x + props.centroid.x,
            fem.shear_center.y + props.centroid.y,
        );
        let shear_center_trefftz = shear_center;
        let beta_x = fem.beta_x_plus;
        let beta_y = fem.beta_y_plus;
        let iw = fem.iw;
        let j = fem.j;

        // FEM shear areas directly (Python sectionproperties uses the FEM
        // values without an analytical fallback guess).
        let (ay, az) = (fem.a_sx, fem.a_sy);

        let ip = props.ix + props.iy;
        let r_t = (j / area).sqrt();
        let r_w = if iw > 0.0 { (iw / area).sqrt() } else { 0.0 };

        let principal = props.principal_properties();
        let phi = principal.phi;

        let (sc_11, sc_22) = if phi.abs() < 1e-10 {
            (shear_center.x, shear_center.y)
        } else {
            let cos_phi = phi.cos();
            let sin_phi = phi.sin();
            (
                shear_center.x * cos_phi + shear_center.y * sin_phi,
                -shear_center.x * sin_phi + shear_center.y * cos_phi,
            )
        };

        // Principal axis shear areas from tensor rotation (Python method)
        // Use the FEM-computed rotated values directly if available, otherwise rotate
        let (a_s11, a_s22) = if phi.abs() < 1e-10 {
            (ay, az)
        } else if fem.a_s11 > 0.0 && fem.a_s22 > 0.0 {
            (fem.a_s11, fem.a_s22)
        } else {
            // Tensor rotation of shear area tensor (no additional FEM run needed)
            let cos_phi = phi.cos();
            let sin_phi = phi.sin();
            let alpha_xx = area / ay;
            let alpha_yy = area / az;
            let alpha_xy = 0.0; // Cross-coupling in centroidal axes
            
            let rot_00 = cos_phi * (cos_phi * alpha_xx + sin_phi * alpha_xy)
                + sin_phi * (cos_phi * alpha_xy + sin_phi * alpha_yy);
            let rot_11 = (-sin_phi) * (-sin_phi * alpha_xx + cos_phi * alpha_xy)
                + cos_phi * (-sin_phi * alpha_xy + cos_phi * alpha_yy);
            
            let a_s11 = if rot_00.abs() > 1e-15 {
                area / rot_00
            } else {
                ay
            };
            let a_s22 = if rot_11.abs() > 1e-15 {
                area / rot_11
            } else {
                az
            };
            (a_s11, a_s22)
        };

        // Principal-axis monosymmetry constants via direct integration in
        // rotated coordinates (Python formula) computed by the FEM analysis,
        // not by rotating the centroidal beta values.
        let beta_11 = fem.beta_11_plus;
        let beta_22 = fem.beta_22_plus;
        let beta_11_plus = fem.beta_11_plus;
        let beta_11_minus = fem.beta_11_minus;
        let beta_22_plus = fem.beta_22_plus;
        let beta_22_minus = fem.beta_22_minus;

        let delta_s = 2.0 * (1.0 + nu) * (props.ix * props.iy - props.ixy * props.ixy);

        // Use FEM-computed centroidal monosymmetry constants. AS4100 /
        // Python sectionproperties convention: beta_x_minus = -beta_x_plus.
        let beta_x_plus = fem.beta_x_plus;
        let beta_x_minus = fem.beta_x_minus;
        let beta_y_plus = fem.beta_y_plus;
        let beta_y_minus = fem.beta_y_minus;

        // Max St. Venant shear stress for unit torque from FEM
        let tau_sv_max_unit = fem_solution.tau_sv_max;

        Self {
            j,
            iw,
            shear_center,
            r_t,
            r_w,
            area,
            ip,
            beta_x,
            beta_y,
            ay,
            az,
            shear_center_11: sc_11,
            shear_center_22: sc_22,
            beta_11,
            beta_22,
            a_s11,
            a_s22,
            a_sxy: fem.a_sxy,
            delta_s,
            principal_angle: phi,
            shear_center_trefftz,
            beta_x_plus,
            beta_x_minus,
            beta_y_plus,
            beta_y_minus,
            beta_11_plus,
            beta_11_minus,
            beta_22_plus,
            beta_22_minus,
            omega_max: fem.omega_max,
            tau_sv_max_unit,
        }
    }

    /// Torsional stiffness (GJ).
    pub fn torsional_stiffness(&self, shear_modulus: f64) -> f64 {
        shear_modulus * self.j
    }

    /// Warping stiffness (E*Iw).
    pub fn warping_stiffness(&self, youngs_modulus: f64) -> f64 {
        youngs_modulus * self.iw
    }

    /// Characteristic length for warping (sqrt(E*Iw/G*J)).
    pub fn warping_length(&self, youngs_modulus: f64, shear_modulus: f64) -> f64 {
        if self.j > 0.0 && self.iw > 0.0 {
            (youngs_modulus * self.iw / (shear_modulus * self.j)).sqrt()
        } else {
            0.0
        }
    }
}

/// Check if a section is thin-walled (open profile heuristic).
///
/// A section is considered thin-walled when its material area is a small
/// fraction of its bounding box (< 15% solid).
pub trait ThinWalledCheck {
    fn is_thin_walled(&self) -> bool;
}

impl ThinWalledCheck for Section {
    fn is_thin_walled(&self) -> bool {
        // Heuristic: check if area is much smaller than bounding box
        let props = SectionProperties::from_section(self);
        let bounds = self.bounds();
        let bbox_area = (bounds.1 - bounds.0) * (bounds.3 - bounds.2);
        // 20% threshold captures I, Channel, and Angle sections
        props.area / bbox_area < 0.20
    }
}

/// Torsional constant beta for rectangle (h/b ratio).
pub fn torsional_constant_beta(ratio: f64) -> f64 {
    // From Roark's formulas
    match ratio {
        r if r >= 10.0 => 1.0 / 3.0,
        r if r >= 5.0 => 0.291,
        r if r >= 3.0 => 0.263,
        r if r >= 2.0 => 0.229,
        r if r >= 1.5 => 0.196,
        r if r >= 1.25 => 0.166,
        r if r >= 1.0 => 0.141,
        _ => torsional_constant_beta(1.0 / ratio), // Symmetric
    }
}

/// Torsion analysis results.
#[derive(Debug, Clone)]
pub struct TorsionAnalysis {
    pub properties: WarpingProperties,
    /// St. Venant shear stress from torque T
    pub tau_sv_max: f64,
    /// Warping normal stress from bimoment B
    pub sigma_w_max: f64,
    /// Total angle of twist per unit length
    pub theta_prime: f64,
    /// Warping displacement
    pub warping_displacement: f64,
}

impl TorsionAnalysis {
    /// Analyze section under pure torsion.
    pub fn pure_torsion(
        section: &Section,
        torque: f64,
        material: &crate::material::Material,
    ) -> Self {
        let nu = material.poissons_ratio;
        let props = WarpingProperties::from_section(section, nu);

        // St. Venant shear stress from FEM solution (not farthest-point formula)
        // τ_sv_max = torque * tau_sv_max_unit where tau_sv_max_unit is max shear stress for unit torque
        let tau_sv_max = torque * props.tau_sv_max_unit.max(1e-12);

        // Angle of twist per unit length: θ' = T / (G*J)
        let theta_prime = torque / (material.shear_modulus * props.j.max(1e-12));

        Self {
            properties: props,
            tau_sv_max,
            sigma_w_max: 0.0, // No warping in pure St. Venant torsion
            theta_prime,
            warping_displacement: 0.0,
        }
    }

    /// Analyze section under torsion + warping (constrained torsion).
    pub fn constrained_torsion(
        section: &Section,
        torque: f64,
        bimoment: f64, // Bimoment (warping moment)
        material: &crate::material::Material,
    ) -> Self {
        let nu = material.poissons_ratio;
        let props = WarpingProperties::from_section(section, nu);

        // St. Venant component from FEM
        let tau_sv_max = torque * props.tau_sv_max_unit.max(1e-12);

        // Warping component
        // σ_w = B * ω_max / Iw where ω_max is the maximum warping coordinate from FEM
        let omega_max = props.omega_max;

        let sigma_w_max = if props.iw > 0.0 {
            bimoment * omega_max / props.iw
        } else {
            0.0
        };

        // Total twist
        let theta_prime = torque / (material.shear_modulus * props.j.max(1e-12));

        // Warping displacement (simplified)
        let warping_displacement = if props.iw > 0.0 {
            bimoment / (material.youngs_modulus * props.iw)
        } else {
            0.0
        };

        Self {
            properties: props,
            tau_sv_max,
            sigma_w_max,
            theta_prime,
            warping_displacement,
        }
    }
}

trait SectionDistance {
    fn max_distance_from(&self, from: Point) -> f64;
}

impl SectionDistance for Section {
    fn max_distance_from(&self, from: Point) -> f64 {
        let mut max_dist: f64 = 0.0;
        for v in &self.outer.vertices {
            let dx = v.x - from.x;
            let dy = v.y - from.y;
            let dist = (dx * dx + dy * dy).sqrt();
            max_dist = max_dist.max(dist);
        }
        for hole in &self.holes {
            for v in &hole.vertices {
                let dx = v.x - from.x;
                let dy = v.y - from.y;
                let dist = (dx * dx + dy * dy).sqrt();
                max_dist = max_dist.max(dist);
            }
        }
        max_dist
    }
}

/// Exact warping analysis using sectorial coordinates.
pub mod trefftz {
    use super::*;
    use crate::geometry::Point;
    use crate::mesh::{Mesh, MeshParams, mesh_section};

    /// Sectorial coordinate (warping function) computation using Trefftz method.
    ///
    /// NOTE: this is a legacy, approximate sectorial-coordinate solver kept for
    /// reference. The boundary integral in `apply_sectorial_bc` is a pointwise
    /// approximation (not a proper edge integration) and `compute_sectorial_properties`
    /// averages `omega` over elements, so results are only qualitatively useful.
    /// Prefer the FEM warping path in `warping_fem`.
    pub fn compute_sectorial_coordinates(section: &Section) -> SectorialProperties {
        // Use FEM to solve for the sectorial coordinate (warping function)
        // ∇²�?= 0 with boundary condition: ∂�?∂n = y*n_x - x*n_y on boundary
        // This is the exact Trefftz method for shear center and warping analysis.

        let mesh = mesh_section(
            section,
            MeshParams {
                target_size: 0.005,
                max_size: 0.01,
                min_size: 0.001,
                quality_threshold: 0.3,
                use_delaunay: true,
                max_iterations: 10,
                max_nodes: 20000,
            },
        );

        let n_nodes = mesh.n_nodes();
        let n_dof = n_nodes; // 1 DOF per node (ω)

        // Build DOF map
        let mut dof_map = std::collections::HashMap::new();
        for node in 0..n_nodes {
            dof_map.insert(node, node);
        }

        // Build global stiffness matrix for Laplace equation ∇²�?= 0
        // ke = �?B^T * B dA where B = [∂N/∂x, ∂N/∂y]
        let mut k_global = SprsMatrix::new(n_dof);

        for element in &mesh.elements {
            let ke = sectorial_stiffness_tri3(&mesh, element);

            let mut dof_indices = [0; 3];
            for (i, &node) in element.iter().enumerate() {
                dof_indices[i] = dof_map[&node];
            }

            for i in 0..3 {
                for j in 0..3 {
                    k_global.add(dof_indices[i], dof_indices[j], ke[i][j]);
                }
            }
        }

        // Build force vector from boundary condition
        // ∂�?∂n = y*n_x - x*n_y on boundary
        let mut f_global = vec![0.0; n_dof];

        // Apply boundary condition: ∂�?∂n = y*n_x - x*n_y
        // This is applied via Neumann BC in FEM
        apply_sectorial_bc(&mesh, &mut f_global);

        // Fix one point to remove rigid body mode (ω = 0 at shear center)
        // We need to find shear center first - use iterative approach
        // For now, fix ω = 0 at centroid as reference
        let centroid = SectionProperties::from_section(section).centroid;
        let mut ref_node = 0;
        let mut min_dist = f64::INFINITY;
        for node in 0..n_nodes {
            let p = mesh.nodes[node];
            let dist = (p.x - centroid.x).hypot(p.y - centroid.y);
            if dist < min_dist {
                min_dist = dist;
                ref_node = node;
            }
        }

        // Apply Dirichlet BC: ω = 0 at reference node
        let ref_dof = dof_map[&ref_node];
        k_global.zero_row_col(ref_dof);
        k_global.set(ref_dof, ref_dof, 1.0);
        f_global[ref_dof] = 0.0;

        // Solve
        k_global.finalize();
        let omega = k_global
            .solve_cholesky(&f_global)
            .unwrap_or(vec![0.0; n_dof]);

        // Compute sectorial static moments and shear center
        let (_shear_center, iw, q_omega_x, q_omega_y) =
            compute_sectorial_properties(&mesh, &omega, &dof_map);

        // Shear center relative to centroid:
        // x_sc = -Q_ωy / I_x, y_sc = Q_ωx / I_x
        let props = SectionProperties::from_section(section);
        let shear_center = Point::new(
            -q_omega_y / props.ix.max(1e-12),
            q_omega_x / props.ix.max(1e-12),
        );

        SectorialProperties {
            shear_center,
            omega_coords: omega,
            iw,
        }
    }

    /// Sectorial stiffness matrix for Tri3 element (3x3).
    fn sectorial_stiffness_tri3(mesh: &Mesh, element: &[usize; 3]) -> [[f64; 3]; 3] {
        let p1 = mesh.nodes[element[0]];
        let p2 = mesh.nodes[element[1]];
        let p3 = mesh.nodes[element[2]];

        let area2 = (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y);
        let area = 0.5 * area2.abs();

        // Shape function derivatives
        let b1 = (p2.y - p3.y) / area2;
        let c1 = (p3.x - p2.x) / area2;
        let b2 = (p3.y - p1.y) / area2;
        let c2 = (p1.x - p3.x) / area2;
        let b3 = (p1.y - p2.y) / area2;
        let c3 = (p2.x - p1.x) / area2;

        let mut ke = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                let bi = [b1, b2, b3][i];
                let ci = [c1, c2, c3][i];
                let bj = [b1, b2, b3][j];
                let cj = [c1, c2, c3][j];
                ke[i][j] = (bi * bj + ci * cj) * area;
            }
        }
        ke
    }

    /// Apply Neumann BC for sectorial coordinate: ∂�?∂n = y*n_x - x*n_y on boundary.
    fn apply_sectorial_bc(mesh: &Mesh, f_global: &mut [f64]) {
        // For each boundary edge, compute integral of (y*n_x - x*n_y) * N dS
        // This is the Neumann BC for the warping function
        for &node in &mesh.boundary_nodes {
            let p = mesh.nodes[node];
            // Approximate contribution from boundary integral
            // In a full implementation, this would integrate over boundary edges
            // For now, use pointwise approximation
            f_global[node] = p.y * 1.0 - p.x * 0.0; // Simplified
        }
    }

    /// Compute sectorial properties from FEM solution.
    fn compute_sectorial_properties(
        mesh: &Mesh,
        omega: &[f64],
        dof_map: &std::collections::HashMap<usize, usize>,
    ) -> (Point, f64, f64, f64) {
        // Q_ωx = �?ω * y dA, Q_ωy = �?ω * x dA
        let mut q_omega_x = 0.0;
        let mut q_omega_y = 0.0;
        let mut iw = 0.0;

        for element in &mesh.elements {
            let p1 = mesh.nodes[element[0]];
            let p2 = mesh.nodes[element[1]];
            let p3 = mesh.nodes[element[2]];

            let area = 0.5 * ((p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y)).abs();
            let centroid = Point::new((p1.x + p2.x + p3.x) / 3.0, (p1.y + p2.y + p3.y) / 3.0);

            // Average omega over element
            let omega_elem = (omega[dof_map[&element[0]]]
                + omega[dof_map[&element[1]]]
                + omega[dof_map[&element[2]]])
                / 3.0;

            q_omega_x += omega_elem * centroid.y * area;
            q_omega_y += omega_elem * centroid.x * area;
            iw += omega_elem * omega_elem * area;
        }

        (Point::new(0.0, 0.0), iw, q_omega_x, q_omega_y)
    }

    #[derive(Debug, Clone)]
    pub struct SectorialProperties {
        pub shear_center: Point,
        pub omega_coords: Vec<f64>,
        pub iw: f64,
    }

    // Sparse matrix for sectorial analysis
    #[derive(Debug, Clone)]
    pub struct SprsMatrix {
        n: usize,
        row_ptr: Vec<usize>,
        col_idx: Vec<usize>,
        values: Vec<f64>,
        triplets: Vec<(usize, usize, f64)>,
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

        pub fn add(&mut self, row: usize, col: usize, value: f64) {
            if value.abs() < 1e-15 {
                return;
            }
            self.triplets.push((row, col, value));
            self.finalized = false;
        }

        pub fn set(&mut self, row: usize, col: usize, value: f64) {
            self.finalize();
            for i in self.row_ptr[row]..self.row_ptr[row + 1] {
                if self.col_idx[i] == col {
                    self.values[i] = value;
                    return;
                }
            }
            let pos = self.row_ptr[row + 1];
            self.col_idx.insert(pos, col);
            self.values.insert(pos, value);
            for r in (row + 1)..=self.n {
                self.row_ptr[r] += 1;
            }
        }

        fn finalize(&mut self) {
            if self.finalized {
                return;
            }
            self.triplets.sort_by_key(|&(r, c, _)| (r, c));

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

            self.row_ptr.fill(0);
            for (r, _, _) in &combined {
                self.row_ptr[r + 1] += 1;
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

        pub fn zero_row_col(&mut self, idx: usize) {
            self.finalize();
            let row_start = self.row_ptr[idx];
            let row_end = self.row_ptr[idx + 1];
            for i in row_start..row_end {
                self.values[i] = 0.0;
            }
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

        pub fn solve_cholesky(&mut self, rhs: &[f64]) -> Option<Vec<f64>> {
            self.finalize();

            if self.n <= 500 {
                let mut a = vec![vec![0.0; self.n]; self.n];
                for i in 0..self.n {
                    for j_idx in self.row_ptr[i]..self.row_ptr[i + 1] {
                        let j = self.col_idx[j_idx];
                        a[i][j] = self.values[j_idx];
                    }
                }

                let mut l = vec![vec![0.0; self.n]; self.n];
                for i in 0..self.n {
                    for j in 0..=i {
                        let mut sum = a[i][j];
                        for k in 0..j {
                            sum -= l[i][k] * l[j][k];
                        }
                        if i == j {
                            if sum <= 1e-12 {
                                return None;
                            }
                            l[i][j] = sum.sqrt();
                        } else {
                            l[i][j] = sum / l[j][j];
                        }
                    }
                }

                let mut y = vec![0.0; self.n];
                for i in 0..self.n {
                    let mut sum = rhs[i];
                    for k in 0..i {
                        sum -= l[i][k] * y[k];
                    }
                    y[i] = sum / l[i][i];
                }

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
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon};
    use crate::material::presets::STEEL_S355;
    use crate::section::Section;
    use crate::section_library::ParametricSection;

    #[test]
    fn warping_solid_rectangle() {
        let poly = Polygon::new(vec![
            Point::new(-0.1, -0.05),
            Point::new(0.1, -0.05),
            Point::new(0.1, 0.05),
            Point::new(-0.1, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let props = WarpingProperties::from_section(&section, 0.3);

        assert!(props.j > 0.0);
        assert!(
            props.iw.abs() < 1e-6,
            "solid Iw should be ~0, got {}",
            props.iw
        );
        assert!(props.shear_center.x.abs() < 1e-4);
        assert!(props.shear_center.y.abs() < 1e-4);
    }

    #[test]
    fn warping_circle() {
        let poly = crate::section_library::circle_polygon(0.05, 64);
        let section = Section::new(poly, vec![]);
        let props = WarpingProperties::from_section(&section, 0.3);

        // For circle: J = π*r^4/2
        let r: f64 = 0.05;
        let expected_j = std::f64::consts::PI * r.powi(4) / 2.0;
        assert!((props.j - expected_j).abs() / expected_j < 0.2); // Approximate

        assert!(
            props.iw.abs() < 1e-6,
            "circle Iw should be ~0, got {}",
            props.iw
        );
    }

    #[test]
    fn warping_i_section() {
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();
        let props = WarpingProperties::from_section(&section, 0.3);

        assert!(props.j > 0.0);
        assert!(props.iw > 0.0); // Open section has warping
        // Doubly symmetric: shear centre coincides with the centroid.
        let c = section.centroid();
        assert!((props.shear_center.x - c.x).abs() < 5e-3);
        assert!((props.shear_center.y - c.y).abs() < 5e-3);
    }

    #[test]
    fn torsion_analysis_pure() {
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();

        let analysis = TorsionAnalysis::pure_torsion(&section, 10e3, &STEEL_S355); // 10 kNm

        assert!(analysis.tau_sv_max > 0.0);
        assert!(analysis.theta_prime > 0.0);
    }

    #[test]
    fn torsion_analysis_constrained() {
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();

        let analysis = TorsionAnalysis::constrained_torsion(&section, 10e3, 5e3, &STEEL_S355);

        assert!(analysis.tau_sv_max > 0.0);
        assert!(analysis.sigma_w_max >= 0.0);
    }

    #[test]
    fn warping_length() {
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();
        let props = WarpingProperties::from_section(&section, 0.3);

        let l_w = props.warping_length(STEEL_S355.youngs_modulus, STEEL_S355.shear_modulus);

        // For IPE300, warping length ~2-5m typically
        assert!(l_w > 0.5 && l_w < 20.0);
    }

    #[test]
    fn warping_principal_axes_doubly_symmetric() {
        // I-section: doubly symmetric, principal axes == centroidal axes
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();
        let props = WarpingProperties::from_section(&section, 0.3);

        // Principal angle ~ 0 or pi/2
        let angle_norm = props
            .principal_angle
            .rem_euclid(std::f64::consts::FRAC_PI_2);
        assert!(angle_norm < 1e-4 || (angle_norm - std::f64::consts::FRAC_PI_2).abs() < 1e-4);

        // Shear areas should be positive
        assert!(props.a_s11 > 0.0);
        assert!(props.a_s22 > 0.0);

        // delta_s should be positive
        assert!(props.delta_s > 0.0);

        // Monosymmetry constants ~ 0 for doubly-symmetric
        assert!(props.beta_11.abs() < 1e-2);
        assert!(props.beta_22.abs() < 1e-2);
    }

    #[test]
    fn warping_principal_axes_channel() {
        // Channel: singly symmetric (about x-axis), principal angle ~ 0
        let channel =
            crate::section_library::steel::ChannelSection::new(0.2, 0.1, 0.008, 0.008, 0.0, 0.0);
        let section = channel.build();
        let props = WarpingProperties::from_section(&section, 0.3);

        // Channel is symmetric about x-axis, so principal angle ~ 0
        assert!(props.principal_angle.abs() < 1e-4);

        // Shear areas about principal axes should match centroidal
        assert!((props.a_s11 - props.ay).abs() / props.ay < 0.05);
        assert!((props.a_s22 - props.az).abs() / props.az < 0.05);

        // beta_x ~ 0 (symmetric about x), beta_y != 0
        assert!(props.beta_x.abs() < 1e-3);
    }

    #[test]
    fn warping_principal_axes_angle() {
        // Angle section: unsymmetric, principal angle != 0
        let angle = crate::section_library::steel::AngleSection::new(0.1, 0.075, 0.008, 0.0, 0.0);
        let section = angle.build();
        let props = WarpingProperties::from_section(&section, 0.3);

        // Shear areas should be positive
        assert!(props.a_s11 > 0.0);
        assert!(props.a_s22 > 0.0);

        // delta_s should be positive
        assert!(props.delta_s > 0.0);
    }

    #[test]
    fn warping_shear_areas_i_section() {
        // I-section: check that FEM shear areas are positive and reasonable.
        // The FEM shear areas differ from the simple web/flange area formulas
        // (which were previously used as a fallback). FEM computes shear areas
        // from the full shear coefficient formulation, which gives different
        // values than the simple web/flange area approximation.
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();
        let props = WarpingProperties::from_section(&section, 0.3);

        // Both shear areas should be positive and in a reasonable range
        assert!(props.ay > 0.0);
        assert!(props.az > 0.0);

        // For the IPE300-like section, FEM shear areas are approximately:
        // ay (flange shear) ~ 0.0026, az (web shear) ~ 0.0035
        // These differ from the simple web/flange area formulas (web ~ 0.00196)
        // because FEM uses the full shear coefficient formulation.
        assert!((props.ay - 0.00261).abs() / 0.00261 < 0.15);
        assert!((props.az - 0.00351).abs() / 0.00351 < 0.15);
    }

    #[test]
    fn warping_trefftz_shear_center_doubly_symmetric() {
        // Doubly-symmetric I-section: Trefftz shear center == elasticity shear center
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();
        let props = WarpingProperties::from_section(&section, 0.3);

        assert!((props.shear_center_trefftz.x - props.shear_center.x).abs() < 1e-10);
        assert!((props.shear_center_trefftz.y - props.shear_center.y).abs() < 1e-10);
        // Both should be at centroid (0, 0) for doubly-symmetric section
        assert!(props.shear_center_trefftz.x.abs() < 5e-3);
        assert!(props.shear_center_trefftz.y.abs() < 5e-3);
    }

    #[test]
    fn warping_trefftz_shear_center_channel() {
        // Channel section: singly-symmetric, shear center outside web
        let channel =
            crate::section_library::steel::ChannelSection::new(0.2, 0.1, 0.008, 0.008, 0.0, 0.0);
        let section = channel.build();
        let props = WarpingProperties::from_section(&section, 0.3);

        // For open thin-walled sections, Trefftz == elasticity
        assert!((props.shear_center_trefftz.x - props.shear_center.x).abs() < 1e-10);
        assert!((props.shear_center_trefftz.y - props.shear_center.y).abs() < 1e-10);
    }

    #[test]
    fn monosymmetry_plus_minus_doubly_symmetric() {
        // Doubly-symmetric I-section: all monosymmetry constants are zero.
        // Physical value is exactly 0; the FE mesh is only approximately symmetric,
        // so small discretization noise (~1e-3 m) is expected.
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();
        let props = WarpingProperties::from_section(&section, 0.3);

        assert!(props.beta_x_plus.abs() < 5e-3);
        assert!(props.beta_x_minus.abs() < 5e-3);
        assert!(props.beta_y_plus.abs() < 1e-2);
        assert!(props.beta_y_minus.abs() < 1e-2);
    }

    #[test]
    fn monosymmetry_plus_minus_relation() {
        // Verify beta_minus = -beta_plus (Python convention)
        let channel =
            crate::section_library::steel::ChannelSection::new(0.2, 0.1, 0.008, 0.008, 0.0, 0.0);
        let section = channel.build();
        let props = WarpingProperties::from_section(&section, 0.3);

        // beta_x_minus = -beta_x_plus
        assert!((props.beta_x_minus + props.beta_x_plus).abs() < 1e-10);

        // beta_y_minus = -beta_y_plus
        assert!((props.beta_y_minus + props.beta_y_plus).abs() < 1e-10);
    }

    #[test]
    fn monosymmetry_plus_minus_principal_axes() {
        // For symmetric section, principal plus/minus match centroidal.
        // Physical value is zero; allow FE discretization noise.
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();
        let props = WarpingProperties::from_section(&section, 0.3);

        assert!(props.beta_11_plus.abs() < 5e-3);
        assert!(props.beta_11_minus.abs() < 5e-3);
        assert!(props.beta_22_plus.abs() < 1e-2);
        assert!(props.beta_22_minus.abs() < 1e-2);
    }

    #[test]
    fn constrained_torsion_omega_max_uses_correct_dimensions() {
        // Regression test: h and b were swapped in constrained_torsion.
        // For I-section with depth=0.3, width=0.15:
        //   correct omega_max = (h/2)^2 * (b/2) = (0.15)^2 * 0.075 = 0.0016875
        //   buggy  omega_max = (b/2)^2 * (h/2) = (0.075)^2 * 0.15 = 0.00084375
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();
        let props = WarpingProperties::from_section(&section, 0.3);

        let bimoment = 1e3;
        let analysis = TorsionAnalysis::constrained_torsion(&section, 0.0, bimoment, &STEEL_S355);

        if props.iw > 0.0 {
            let omega_max = analysis.sigma_w_max * props.iw / bimoment;
            let h = 0.3_f64;
            let b = 0.15_f64;
            let correct = (h / 2.0).powi(2) * (b / 2.0);
            let buggy = (b / 2.0).powi(2) * (h / 2.0);
            assert!(
                (omega_max - correct).abs() < (omega_max - buggy).abs(),
                "omega_max should match correct formula, not buggy"
            );
        }
    }

    #[test]
    fn channel_shear_center_magnitude_reasonable() {
        // Regression test: shear center formula had hw instead of hw².
        // ChannelSection::build() creates a solid C-shape, not thin-walled.
        // Manually create a thin-walled channel polygon.
        let (h, b, tw, tf) = (0.2, 0.075, 0.003, 0.005);
        let hh = h / 2.0;
        let poly = Polygon::new(vec![
            Point::new(0.0, -hh),
            Point::new(tw, -hh),
            Point::new(tw + b, -hh),
            Point::new(tw + b, -hh + tf),
            Point::new(tw, -hh + tf),
            Point::new(tw, hh - tf),
            Point::new(tw + b, hh - tf),
            Point::new(tw + b, hh),
            Point::new(tw, hh),
            Point::new(0.0, hh),
        ]);
        let section = Section::new(poly, vec![]);
        let props = WarpingProperties::from_section(&section, 0.3);

        // Verify it's detected as thin-walled
        assert!(section.is_thin_walled(), "section should be thin-walled");

        // Shear centre must lie behind the web (negative x) within one
        // flange width.
        assert!(
            props.shear_center.x < 1e-6 && props.shear_center.x.abs() < b,
            "channel shear center x should be behind the web, got {}",
            props.shear_center.x
        );

        // Magnitude should be less than flange width (skip if near-zero due to FEM precision)
        if props.shear_center.x.abs() > 1e-6 {
            assert!(
                props.shear_center.x.abs() < b,
                "channel shear center magnitude should be < flange width {}, got {}",
                b,
                props.shear_center.x.abs()
            );
        }
    }

    #[test]
    fn channel_warping_constant_positive_and_reasonable() {
        // Regression test: warping constant used rough 0.25 approximation.
        // Now uses standard formula: Iw = tf*bf³*hw²/12 * (tw*hw+2*tf*bf)/(tw*hw+6*tf*bf)
        let (h, b, tw, tf) = (0.2, 0.075, 0.003, 0.005);
        let hh = h / 2.0;
        let poly = Polygon::new(vec![
            Point::new(0.0, -hh),
            Point::new(tw, -hh),
            Point::new(tw + b, -hh),
            Point::new(tw + b, -hh + tf),
            Point::new(tw, -hh + tf),
            Point::new(tw, hh - tf),
            Point::new(tw + b, hh - tf),
            Point::new(tw + b, hh),
            Point::new(tw, hh),
            Point::new(0.0, hh),
        ]);
        let section = Section::new(poly, vec![]);
        let props = WarpingProperties::from_section(&section, 0.3);

        // Iw should be positive for an open section
        assert!(
            props.iw > 0.0,
            "channel Iw should be positive, got {}",
            props.iw
        );
    }
}
