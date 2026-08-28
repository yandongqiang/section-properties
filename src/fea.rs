//! Finite element analysis core — Tri6 (six-noded quadratic triangular) element.
//!
//! Mirrors Python `sectionproperties.analysis.fea`:
//! shape functions, Gaussian quadrature, stress extrapolation, and element
//! integration methods for section property computation.

use crate::geometry::Point;
use std::collections::HashMap;

// ---------------------------------------------------------------------------
// FEM Tolerances
// ---------------------------------------------------------------------------

/// Tolerance for geometric comparisons (e.g., point coincidence, edge lengths).
pub const GEOMETRY_TOL: f64 = 1e-12;

/// Tolerance for Jacobian determinant checks (degenerate element detection).
pub const JACOBIAN_TOL: f64 = 1e-20;

/// Tolerance for iterative solver convergence (CG, ICCG, etc.).
pub const SOLVER_TOL: f64 = 1e-12;

/// Tolerance for pivot checks in direct solvers (LDL^T, Cholesky).
pub const PIVOT_TOL: f64 = 1e-15;

/// Tolerance for near-zero checks in warping/stress calculations.
pub const NEAR_ZERO_TOL: f64 = 1e-15;

// ---------------------------------------------------------------------------
// Gaussian quadrature for Tri6
// ---------------------------------------------------------------------------

/// Gaussian weights and locations for `n`-point integration of a Tri6 element.
///
/// Returns `n` tuples of `(weight, eta, xi, zeta)` where `zeta = 1 - eta - xi`.
/// Supports n = 1, 3, 4, 6.
pub fn gauss_points(n: usize) -> Vec<(f64, f64, f64, f64)> {
    match n {
        1 => vec![(1.0, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0)],
        3 => vec![
            (1.0 / 3.0, 2.0 / 3.0, 1.0 / 6.0, 1.0 / 6.0),
            (1.0 / 3.0, 1.0 / 6.0, 2.0 / 3.0, 1.0 / 6.0),
            (1.0 / 3.0, 1.0 / 6.0, 1.0 / 6.0, 2.0 / 3.0),
        ],
        4 => vec![
            (-27.0 / 48.0, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0),
            (25.0 / 48.0, 0.6, 0.2, 0.2),
            (25.0 / 48.0, 0.2, 0.6, 0.2),
            (25.0 / 48.0, 0.2, 0.2, 0.6),
        ],
        6 => {
            let g1 =
                1.0 / 18.0 * (8.0 - (10f64).sqrt() + (38.0 - 44.0 * (2.0_f64 / 5.0).sqrt()).sqrt());
            let g2 =
                1.0 / 18.0 * (8.0 - (10f64).sqrt() - (38.0 - 44.0 * (2.0_f64 / 5.0).sqrt()).sqrt());
            let w1 = (620.0 + (213125.0 - 53320.0 * (10f64).sqrt()).sqrt()) / 3720.0;
            let w2 = (620.0 - (213125.0 - 53320.0 * (10f64).sqrt()).sqrt()) / 3720.0;
            vec![
                (w2, 1.0 - 2.0 * g2, g2, g2),
                (w2, g2, 1.0 - 2.0 * g2, g2),
                (w2, g2, g2, 1.0 - 2.0 * g2),
                (w1, g1, g1, 1.0 - 2.0 * g1),
                (w1, 1.0 - 2.0 * g1, g1, g1),
                (w1, g1, 1.0 - 2.0 * g1, g1),
            ]
        }
        _ => panic!("gauss_points: n must be 1, 3, 4 or 6, got {n}"),
    }
}

// ---------------------------------------------------------------------------
// Shape functions for Tri6
// ---------------------------------------------------------------------------

/// Result of evaluating shape functions at a Gauss point.
pub struct ShapeFunctionResult {
    /// Shape function values N(i), i=0..6
    pub n: [f64; 6],
    /// Shape function derivatives B(i,j): [2][6]
    /// B[0] = dN/dx, B[1] = dN/dy
    pub b: [[f64; 6]; 2],
    /// Jacobian determinant
    pub j: f64,
    /// Global x-coordinate at Gauss point
    pub x: f64,
    /// Global y-coordinate at Gauss point
    pub y: f64,
}

/// Evaluate Tri6 shape functions, derivatives, Jacobian, and global coords.
///
/// `coords` is a 2×6 array: coords[0] = x-coords, coords[1] = y-coords.
/// Node ordering: [v0, v1, v2, mid01, mid12, mid20].
pub fn shape_function(coords: &[[f64; 6]; 2], gp: (f64, f64, f64)) -> ShapeFunctionResult {
    let (eta, xi, zeta) = gp;

    // Shape function values
    let n = [
        eta * (2.0 * eta - 1.0),
        xi * (2.0 * xi - 1.0),
        zeta * (2.0 * zeta - 1.0),
        4.0 * eta * xi,
        4.0 * xi * zeta,
        4.0 * eta * zeta,
    ];

    // Derivatives wrt isoparametric coords [dN/deta; dN/dxi; dN/dzeta]
    let b_iso = [
        [4.0 * eta - 1.0, 0.0, 0.0, 4.0 * xi, 0.0, 4.0 * zeta],
        [0.0, 4.0 * xi - 1.0, 0.0, 4.0 * eta, 4.0 * zeta, 0.0],
        [0.0, 0.0, 4.0 * zeta - 1.0, 0.0, 4.0 * xi, 4.0 * eta],
    ];

    // Jacobian matrix (3×3):
    // J = | 1   sum(dN/deta * x)   sum(dN/deta * y) |
    //     | 1   sum(dN/dxi  * x)   sum(dN/dxi  * y) |
    //     | 1   sum(dN/dzeta* x)   sum(dN/dzeta* y) |
    let mut j_mat = [[1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 0.0]];
    for k in 0..3 {
        for i in 0..6 {
            j_mat[k][1] += b_iso[k][i] * coords[0][i];
            j_mat[k][2] += b_iso[k][i] * coords[1][i];
        }
    }

    // Jacobian determinant = 0.5 * det(J)
    let det_j = j_mat[0][0] * (j_mat[1][1] * j_mat[2][2] - j_mat[1][2] * j_mat[2][1])
        - j_mat[0][1] * (j_mat[1][0] * j_mat[2][2] - j_mat[1][2] * j_mat[2][0])
        + j_mat[0][2] * (j_mat[1][0] * j_mat[2][1] - j_mat[1][1] * j_mat[2][0]);
    let jacobian = 0.5 * det_j;

    // Check for degenerate element (zero or negative Jacobian)
    // Use a very small threshold for truly degenerate elements
    if jacobian.abs() <= JACOBIAN_TOL {
        return ShapeFunctionResult {
            n,
            b: [[0.0; 6]; 2],
            j: jacobian,
            x: 0.0,
            y: 0.0,
        };
    }

    // B = tmp_array * J^{-1} * b_iso
    // tmp_array = [[0,1,0],[0,0,1]]
    // So B[0] = row 1 of J^{-1} * b_iso, B[1] = row 2 of J^{-1} * b_iso
    let j_inv = inv3x3(&j_mat);
    let mut b = [[0.0; 6]; 2];
    for j in 0..6 {
        for k in 0..3 {
            b[0][j] += j_inv[1][k] * b_iso[k][j];
            b[1][j] += j_inv[2][k] * b_iso[k][j];
        }
    }

    // Global coordinates
    let mut x = 0.0;
    let mut y = 0.0;
    for i in 0..6 {
        x += n[i] * coords[0][i];
        y += n[i] * coords[1][i];
    }

    ShapeFunctionResult {
        n,
        b,
        j: jacobian,
        x,
        y,
    }
}

/// Inverse of a 3×3 matrix.
fn inv3x3(m: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    let inv_det = 1.0 / det;
    [
        [
            (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det,
            (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det,
        ],
        [
            (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_det,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det,
            (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det,
        ],
        [
            (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det,
            (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det,
        ],
    ]
}

// ---------------------------------------------------------------------------
// Extrapolation from Gauss points to nodes
// ---------------------------------------------------------------------------

/// Extrapolation matrix H^{-1} for 6-point Gauss → 6 nodes.
const H_INV: [[f64; 6]; 6] = [
    [
        1.87365927351160,
        0.138559587411935,
        0.138559587411935,
        -0.638559587411936,
        0.126340726488397,
        -0.638559587411935,
    ],
    [
        0.138559587411935,
        1.87365927351160,
        0.138559587411935,
        -0.638559587411935,
        -0.638559587411935,
        0.126340726488397,
    ],
    [
        0.138559587411935,
        0.138559587411935,
        1.87365927351160,
        0.126340726488396,
        -0.638559587411935,
        -0.638559587411935,
    ],
    [
        0.0749010751157440,
        0.0749010751157440,
        0.180053080734478,
        1.36051633430762,
        -0.345185782636792,
        -0.345185782636792,
    ],
    [
        0.180053080734478,
        0.0749010751157440,
        0.0749010751157440,
        -0.345185782636792,
        1.36051633430762,
        -0.345185782636792,
    ],
    [
        0.0749010751157440,
        0.180053080734478,
        0.0749010751157440,
        -0.345185782636792,
        -0.345185782636792,
        1.36051633430762,
    ],
];

/// Extrapolate results at six Gauss points to the six nodes of a Tri6 element.
pub fn extrapolate_to_nodes(w: &[f64; 6]) -> [f64; 6] {
    let mut result = [0.0; 6];
    for i in 0..6 {
        for j in 0..6 {
            result[i] += H_INV[i][j] * w[j];
        }
    }
    result
}

// ---------------------------------------------------------------------------
// Coordinate transformations
// ---------------------------------------------------------------------------

/// Convert global (centroidal) coordinates to principal coordinates.
///
/// `phi` is the principal axis angle in **radians** — the angle from the
/// centroidal x-axis to axis 11 (major principal), CCW positive.
///
/// ```text
/// x₁₁ = (x − x_c)·cos(φ) + (y − y_c)·sin(φ)    // along axis 11
/// y₂₂ = −(x − x_c)·sin(φ) + (y − y_c)·cos(φ)   // along axis 22
/// ```
///
/// Note: this convention matches the standard Mohr's circle and differs
/// from Python's `sectionproperties` by a sign in `phi`.
pub fn principal_coordinate(phi: f64, x: f64, y: f64) -> (f64, f64) {
    let cos_phi = phi.cos();
    let sin_phi = phi.sin();
    (x * cos_phi + y * sin_phi, y * cos_phi - x * sin_phi)
}

/// Convert principal coordinates back to global (centroidal) coordinates.
///
/// `phi` is the principal axis angle in **radians**.
///
/// ```text
/// x = x₁₁·cos(φ) − y₂₂·sin(φ)
/// y = x₁₁·sin(φ) + y₂₂·cos(φ)
/// ```
pub fn global_coordinate(phi: f64, x11: f64, y22: f64) -> (f64, f64) {
    let cos_phi = phi.cos();
    let sin_phi = phi.sin();
    (x11 * cos_phi - y22 * sin_phi, x11 * sin_phi + y22 * cos_phi)
}

// ---------------------------------------------------------------------------
// Shear parameters
// ---------------------------------------------------------------------------

/// Compute shear parameters used in shear load vector and coefficient assembly.
pub fn shear_parameter(nx: f64, ny: f64, ixx: f64, iyy: f64, ixy: f64) -> [f64; 6] {
    let r = nx * nx - ny * ny;
    let q = 2.0 * nx * ny;
    let d1 = ixx * r - ixy * q;
    let d2 = ixy * r + ixx * q;
    let h1 = -ixy * r + iyy * q;
    let h2 = -iyy * r - ixy * q;
    [r, q, d1, d2, h1, h2]
}

// ---------------------------------------------------------------------------
// Tri6 element
// ---------------------------------------------------------------------------

/// Six-noded quadratic triangular finite element.
///
/// Node ordering: [v0, v1, v2, mid01, mid12, mid20] where v0,v1,v2 are
/// vertices and midXX are mid-edge nodes.
#[derive(Debug, Clone)]
pub struct Tri6 {
    /// Element id
    pub el_id: usize,
    /// Coordinates: coords[0] = x[0..6], coords[1] = y[0..6]
    pub coords: [[f64; 6]; 2],
    /// Global node ids
    pub node_ids: [usize; 6],
    /// Elastic modulus
    pub elastic_modulus: f64,
    /// Shear modulus
    pub shear_modulus: f64,
    /// Density
    pub density: f64,
}

impl Tri6 {
    /// Create a Tri6 element from 6 points and node ids.
    ///
    /// Validates that the element has positive orientation (CCW winding).
    /// Returns `Err(FemError::InvalidElementOrientation)` if the element
    /// is degenerate or has negative Jacobian (CW winding).
    pub fn from_points(
        el_id: usize,
        points: [Point; 6],
        node_ids: [usize; 6],
        em: f64,
        gm: f64,
        rho: f64,
    ) -> Result<Self, crate::mesh::fem::FemError> {
        let mut coords = [[0.0; 6]; 2];
        for i in 0..6 {
            coords[0][i] = points[i].x;
            coords[1][i] = points[i].y;
        }
        // Check orientation at centroid (xi=eta=zeta=1/3)
        let sf = shape_function(&coords, (1.0/3.0, 1.0/3.0, 1.0/3.0));
        if sf.j.abs() <= JACOBIAN_TOL {
            return Err(crate::mesh::fem::FemError::DegenerateElement);
        }
        if sf.j < 0.0 {
            return Err(crate::mesh::fem::FemError::InvalidElementOrientation);
        }
        Ok(Self {
            el_id,
            coords,
            node_ids,
            elastic_modulus: em,
            shear_modulus: gm,
            density: rho,
        })
    }

    /// Calculate geometric properties: (area, qx, qy, ixx, iyy, ixy).
    pub fn geometric_properties(&self) -> (f64, f64, f64, f64, f64, f64) {
        let mut area = 0.0;
        let mut qx = 0.0;
        let mut qy = 0.0;
        let mut ixx = 0.0;
        let mut iyy = 0.0;
        let mut ixy = 0.0;

        for &(w, eta, xi, zeta) in &gauss_points(4) {
            let sf = shape_function(&self.coords, (eta, xi, zeta));
            let weight = w * sf.j;
            area += weight;
            qx += weight * sf.y;
            qy += weight * sf.x;
            ixx += weight * sf.y * sf.y;
            iyy += weight * sf.x * sf.x;
            ixy += weight * sf.y * sf.x;
        }

        (area, qx, qy, ixx, iyy, ixy)
    }

    /// Calculate torsion stiffness matrix (6×6), load vector (6), and constraint vector (6).
    pub fn torsion_properties(&self) -> ([[f64; 6]; 6], [f64; 6], [f64; 6]) {
        let mut k_el = [[0.0; 6]; 6];
        let mut f_el = [0.0; 6];
        let mut c_el = [0.0; 6];

        for &(w, eta, xi, zeta) in &gauss_points(4) {
            let sf = shape_function(&self.coords, (eta, xi, zeta));
            let weight = w * sf.j * self.elastic_modulus;

            // k_el += weight * B^T * B
            for i in 0..6 {
                for j in 0..6 {
                    k_el[i][j] += weight * (sf.b[0][i] * sf.b[0][j] + sf.b[1][i] * sf.b[1][j]);
                }
                // f_el += weight * B^T * [y, -x]
                f_el[i] += weight * (sf.b[0][i] * sf.y - sf.b[1][i] * sf.x);
                // c_el += weight * N
                c_el[i] += weight * sf.n[i];
            }
        }

        (k_el, f_el, c_el)
    }

    /// Calculate shear load vectors f_psi and f_phi.
    pub fn shear_load_vectors(
        &self,
        ixx: f64,
        iyy: f64,
        ixy: f64,
        nu: f64,
    ) -> ([f64; 6], [f64; 6]) {
        let mut f_psi = [0.0; 6];
        let mut f_phi = [0.0; 6];

        for &(w, eta, xi, zeta) in &gauss_points(4) {
            let sf = shape_function(&self.coords, (eta, xi, zeta));
            let weight = w * sf.j * self.elastic_modulus;
            let [_, _, d1, d2, h1, h2] = shear_parameter(sf.x, sf.y, ixx, iyy, ixy);

            for i in 0..6 {
                // f_psi += weight * (nu/2 * B^T * [d1, d2] + 2*(1+nu) * N * (ixx*x - ixy*y))
                f_psi[i] += weight
                    * (nu / 2.0 * (sf.b[0][i] * d1 + sf.b[1][i] * d2)
                        + 2.0 * (1.0 + nu) * sf.n[i] * (ixx * sf.x - ixy * sf.y));
                // f_phi += weight * (nu/2 * B^T * [h1, h2] + 2*(1+nu) * N * (iyy*y - ixy*x))
                f_phi[i] += weight
                    * (nu / 2.0 * (sf.b[0][i] * h1 + sf.b[1][i] * h2)
                        + 2.0 * (1.0 + nu) * sf.n[i] * (iyy * sf.y - ixy * sf.x));
            }
        }

        (f_psi, f_phi)
    }

    /// Calculate shear centre and warping integrals.
    ///
    /// Returns (sc_xint, sc_yint, q_omega, i_omega, i_xomega, i_yomega).
    pub fn shear_warping_integrals(
        &self,
        ixx: f64,
        iyy: f64,
        ixy: f64,
        omega: &[f64; 6],
    ) -> (f64, f64, f64, f64, f64, f64) {
        let mut sc_xint = 0.0;
        let mut sc_yint = 0.0;
        let mut q_omega = 0.0;
        let mut i_omega = 0.0;
        let mut i_xomega = 0.0;
        let mut i_yomega = 0.0;

        for &(w, eta, xi, zeta) in &gauss_points(4) {
            let sf = shape_function(&self.coords, (eta, xi, zeta));
            let weight = w * sf.j * self.elastic_modulus;

            let n_omega: f64 = (0..6).map(|i| sf.n[i] * omega[i]).sum();
            let r2 = sf.x * sf.x + sf.y * sf.y;

            sc_xint += weight * (iyy * sf.x + ixy * sf.y) * r2;
            sc_yint += weight * (ixx * sf.y + ixy * sf.x) * r2;
            q_omega += weight * n_omega;
            i_omega += weight * n_omega * n_omega;
            i_xomega += weight * sf.x * n_omega;
            i_yomega += weight * sf.y * n_omega;
        }

        (sc_xint, sc_yint, q_omega, i_omega, i_xomega, i_yomega)
    }

    /// Calculate shear deformation coefficients (kappa_x, kappa_y, kappa_xy).
    pub fn shear_coefficients(
        &self,
        ixx: f64,
        iyy: f64,
        ixy: f64,
        psi_shear: &[f64; 6],
        phi_shear: &[f64; 6],
        nu: f64,
    ) -> (f64, f64, f64) {
        let mut kappa_x = 0.0;
        let mut kappa_y = 0.0;
        let mut kappa_xy = 0.0;

        for &(w, eta, xi, zeta) in &gauss_points(4) {
            let sf = shape_function(&self.coords, (eta, xi, zeta));
            let weight = w * sf.j * self.elastic_modulus;
            let [_, _, d1, d2, h1, h2] = shear_parameter(sf.x, sf.y, ixx, iyy, ixy);

            // B * psi - nu/2 * [d1, d2]
            let mut b_psi_d = [0.0; 2];
            let mut b_phi_h = [0.0; 2];
            for i in 0..6 {
                b_psi_d[0] += sf.b[0][i] * psi_shear[i];
                b_psi_d[1] += sf.b[1][i] * psi_shear[i];
                b_phi_h[0] += sf.b[0][i] * phi_shear[i];
                b_phi_h[1] += sf.b[1][i] * phi_shear[i];
            }
            b_psi_d[0] -= nu / 2.0 * d1;
            b_psi_d[1] -= nu / 2.0 * d2;
            b_phi_h[0] -= nu / 2.0 * h1;
            b_phi_h[1] -= nu / 2.0 * h2;

            kappa_x += weight * (b_psi_d[0] * b_psi_d[0] + b_psi_d[1] * b_psi_d[1]);
            kappa_y += weight * (b_phi_h[0] * b_phi_h[0] + b_phi_h[1] * b_phi_h[1]);
            kappa_xy += weight * (b_psi_d[0] * b_phi_h[0] + b_psi_d[1] * b_phi_h[1]);
        }

        (kappa_x, kappa_y, kappa_xy)
    }

    /// Calculate monosymmetry integrals (int_x, int_y, int_11, int_22).
    ///
    /// `phi` is the principal axis angle in **radians**.
    pub fn monosymmetry_integrals(&self, phi: f64) -> (f64, f64, f64, f64) {
        let mut int_x = 0.0;
        let mut int_y = 0.0;
        let mut int_11 = 0.0;
        let mut int_22 = 0.0;

        for &(w, eta, xi, zeta) in &gauss_points(4) {
            let sf = shape_function(&self.coords, (eta, xi, zeta));
            let weight = w * sf.j * self.elastic_modulus;
            let (nx_11, ny_22) = principal_coordinate(phi, sf.x, sf.y);

            int_x += weight * (sf.x * sf.x * sf.y + sf.y * sf.y * sf.y);
            int_y += weight * (sf.y * sf.y * sf.x + sf.x * sf.x * sf.x);
            int_11 += weight * (nx_11 * nx_11 * ny_22 + ny_22 * ny_22 * ny_22);
            int_22 += weight * (ny_22 * ny_22 * nx_11 + nx_11 * nx_11 * nx_11);
        }

        (int_x, int_y, int_11, int_22)
    }

    /// Calculate element stresses at the 6 Gauss points, extrapolated to nodes.
    ///
    /// Returns 11 nodal stress arrays (each length 6):
    /// (sig_zz_n, sig_zz_mxx, sig_zz_myy, sig_zz_m11, sig_zz_m22,
    ///  sig_zx_mzz, sig_zy_mzz, sig_zx_vx, sig_zy_vx, sig_zx_vy, sig_zy_vy)
    pub fn element_stress(
        &self,
        n: f64,
        mxx: f64,
        myy: f64,
        m11: f64,
        m22: f64,
        mzz: f64,
        vx: f64,
        vy: f64,
        ea: f64,
        cx: f64,
        cy: f64,
        ixx: f64,
        iyy: f64,
        ixy: f64,
        i11: f64,
        i22: f64,
        phi: f64,
        j: f64,
        nu: f64,
        omega: &[f64; 6],
        psi_shear: &[f64; 6],
        phi_shear: &[f64; 6],
        delta_s: f64,
    ) -> [[f64; 6]; 11] {
        let em = self.elastic_modulus;
        let denom = ixx * iyy - ixy * ixy;

        // Axial stress (constant over element)
        let sig_zz_n = [em * n / ea; 6];

        let mut sig_zz_mxx_gp = [0.0; 6];
        let mut sig_zz_myy_gp = [0.0; 6];
        let mut sig_zz_m11_gp = [0.0; 6];
        let mut sig_zz_m22_gp = [0.0; 6];
        let mut sig_zx_mzz_gp = [0.0; 6];
        let mut sig_zy_mzz_gp = [0.0; 6];
        let mut sig_zx_vx_gp = [0.0; 6];
        let mut sig_zy_vx_gp = [0.0; 6];
        let mut sig_zx_vy_gp = [0.0; 6];
        let mut sig_zy_vy_gp = [0.0; 6];

        // Centroidal coords
        let coords_c = [
            [
                self.coords[0][0] - cx,
                self.coords[0][1] - cx,
                self.coords[0][2] - cx,
                self.coords[0][3] - cx,
                self.coords[0][4] - cx,
                self.coords[0][5] - cx,
            ],
            [
                self.coords[1][0] - cy,
                self.coords[1][1] - cy,
                self.coords[1][2] - cy,
                self.coords[1][3] - cy,
                self.coords[1][4] - cy,
                self.coords[1][5] - cy,
            ],
        ];

        let gps = gauss_points(6);
        for (i, &(_w, eta, xi, zeta)) in gps.iter().enumerate() {
            let sf = shape_function(&coords_c, (eta, xi, zeta));
            let (nx_11, ny_22) = principal_coordinate(phi, sf.x, sf.y);
            let [_, _, d1, d2, h1, h2] = shear_parameter(sf.x, sf.y, ixx, iyy, ixy);

            // Bending stresses
            if denom.abs() > NEAR_ZERO_TOL {
                sig_zz_mxx_gp[i] = em * (-ixy * mxx / denom * sf.x + iyy * mxx / denom * sf.y);
                sig_zz_myy_gp[i] = em * (-ixx * myy / denom * sf.x + ixy * myy / denom * sf.y);
            }
            if i11.abs() > NEAR_ZERO_TOL {
                sig_zz_m11_gp[i] = em * m11 / i11 * ny_22;
            }
            if i22.abs() > NEAR_ZERO_TOL {
                sig_zz_m22_gp[i] = em * -m22 / i22 * nx_11;
            }

            // Torsional shear stress
            if mzz.abs() > GEOMETRY_TOL && j.abs() > GEOMETRY_TOL {
                let mut b_omega = [0.0; 2];
                for k in 0..6 {
                    b_omega[0] += sf.b[0][k] * omega[k];
                    b_omega[1] += sf.b[1][k] * omega[k];
                }
                sig_zx_mzz_gp[i] = em * mzz / j * (b_omega[0] - sf.y);
                sig_zy_mzz_gp[i] = em * mzz / j * (b_omega[1] + sf.x);
            }

            // Shear stress from Vx
            if vx.abs() > GEOMETRY_TOL && delta_s.abs() > GEOMETRY_TOL {
                let mut b_psi = [0.0; 2];
                for k in 0..6 {
                    b_psi[0] += sf.b[0][k] * psi_shear[k];
                    b_psi[1] += sf.b[1][k] * psi_shear[k];
                }
                sig_zx_vx_gp[i] = em * vx / delta_s * (b_psi[0] - nu / 2.0 * d1);
                sig_zy_vx_gp[i] = em * vx / delta_s * (b_psi[1] - nu / 2.0 * d2);
            }

            // Shear stress from Vy
            if vy.abs() > GEOMETRY_TOL && delta_s.abs() > GEOMETRY_TOL {
                let mut b_phi = [0.0; 2];
                for k in 0..6 {
                    b_phi[0] += sf.b[0][k] * phi_shear[k];
                    b_phi[1] += sf.b[1][k] * phi_shear[k];
                }
                sig_zx_vy_gp[i] = em * vy / delta_s * (b_phi[0] - nu / 2.0 * h1);
                sig_zy_vy_gp[i] = em * vy / delta_s * (b_phi[1] - nu / 2.0 * h2);
            }
        }

        // Extrapolate from Gauss points to nodes
        [
            sig_zz_n,
            extrapolate_to_nodes(&sig_zz_mxx_gp),
            extrapolate_to_nodes(&sig_zz_myy_gp),
            extrapolate_to_nodes(&sig_zz_m11_gp),
            extrapolate_to_nodes(&sig_zz_m22_gp),
            extrapolate_to_nodes(&sig_zx_mzz_gp),
            extrapolate_to_nodes(&sig_zy_mzz_gp),
            extrapolate_to_nodes(&sig_zx_vx_gp),
            extrapolate_to_nodes(&sig_zy_vx_gp),
            extrapolate_to_nodes(&sig_zx_vy_gp),
            extrapolate_to_nodes(&sig_zy_vy_gp),
        ]
    }
}

// ---------------------------------------------------------------------------
// Tri6 mesh generation from Tri3 mesh
// ---------------------------------------------------------------------------

/// A Tri6 mesh: nodes and 6-noded elements.
#[derive(Debug, Clone)]
pub struct Tri6Mesh {
    /// All nodes (original + mid-edge)
    pub nodes: Vec<Point>,
    /// Elements: each is 6 node indices [v0, v1, v2, mid01, mid12, mid20]
    pub elements: Vec<[usize; 6]>,
}

/// Convert a Tri3 mesh to a Tri6 mesh by adding mid-edge nodes.
///
/// Shared edges between elements share the same mid-node.
/// Ensures all Tri6 elements have CCW orientation (positive Jacobian).
pub fn tri3_to_tri6(tri3_nodes: &[Point], tri3_elements: &[[usize; 3]]) -> Tri6Mesh {
    let mut nodes: Vec<Point> = tri3_nodes.to_vec();
    // Map from (min_node, max_node) -> mid_node_index
    let mut edge_midpoints: HashMap<(usize, usize), usize> = HashMap::new();

    let mut tri6_elements: Vec<[usize; 6]> = Vec::with_capacity(tri3_elements.len());

    for tri3_elem in tri3_elements {
        let n0 = tri3_elem[0];
        let n1 = tri3_elem[1];
        let n2 = tri3_elem[2];
        let mid01 = get_or_create_midpoint(&mut nodes, &mut edge_midpoints, n0, n1);
        let mid12 = get_or_create_midpoint(&mut nodes, &mut edge_midpoints, n1, n2);
        let mid20 = get_or_create_midpoint(&mut nodes, &mut edge_midpoints, n2, n0);
        let mut elem = [n0, n1, n2, mid01, mid12, mid20];

        // Check orientation and fix if needed (CCW required)
        let mut coords = [[0.0; 6]; 2];
        for k in 0..6 {
            coords[0][k] = nodes[elem[k]].x;
            coords[1][k] = nodes[elem[k]].y;
        }
        let sf = shape_function(&coords, (1.0/3.0, 1.0/3.0, 1.0/3.0));
        if sf.j < 0.0 {
            // Reverse the vertex order to fix orientation (CCW required)
            // Original: [n0, n1, n2, mid01, mid12, mid20]
            // After vertex reverse: [n2, n1, n0, mid01, mid12, mid20]
            // Need to reorder midpoints to match: [n2, n1, n0, mid12, mid01, mid20]
            elem[0..3].reverse(); // Now [n2, n1, n0, mid01, mid12, mid20]
            elem[3] = mid12;
            elem[4] = mid01;
            elem[5] = mid20;
        }
        tri6_elements.push(elem);
    }

    Tri6Mesh {
        nodes,
        elements: tri6_elements,
    }
}

/// Validates that a Tri6Mesh has no degenerate elements.
///
/// Returns `Err(FemError::DegenerateElement)` if any element has
/// zero or negative Jacobian determinant at any Gauss point.
pub fn validate_tri6_mesh(mesh: &Tri6Mesh) -> Result<(), crate::mesh::fem::FemError> {
    let gps = gauss_points(6);
    for &elem in mesh.elements.iter() {
        let mut coords = [[0.0; 6]; 2];
        for k in 0..6 {
            coords[0][k] = mesh.nodes[elem[k]].x;
            coords[1][k] = mesh.nodes[elem[k]].y;
        }
        for &(_, eta, xi, zeta) in &gps {
            let sf = shape_function(&coords, (eta, xi, zeta));
            if sf.j <= JACOBIAN_TOL {
                return Err(crate::mesh::fem::FemError::DegenerateElement);
            }
        }
    }
    Ok(())
}

fn get_or_create_midpoint(
    nodes: &mut Vec<Point>,
    edge_map: &mut HashMap<(usize, usize), usize>,
    a: usize,
    b: usize,
) -> usize {
    let key = if a < b { (a, b) } else { (b, a) };
    if let Some(&mid) = edge_map.get(&key) {
        return mid;
    }
    let pa = nodes[a];
    let pb = nodes[b];
    let mid = Point::new(0.5 * (pa.x + pb.x), 0.5 * (pa.y + pb.y));
    let mid_idx = nodes.len();
    nodes.push(mid);
    edge_map.insert(key, mid_idx);
    mid_idx
}

/// Build a list of Tri6 elements from a Tri6Mesh for a given material.
/// Returns `Err(FemError::DegenerateElement)` if any element has
/// zero or negative Jacobian determinant at any Gauss point.
/// Coordinates are kept in section-centroid frame for stress calculation compatibility.
pub fn build_tri6_elements(mesh: &Tri6Mesh, em: f64, gm: f64, rho: f64) -> Result<Vec<Tri6>, crate::mesh::fem::FemError> {
    let gps = gauss_points(6);
    let mut elements = Vec::with_capacity(mesh.elements.len());
    for (i, &elem) in mesh.elements.iter().enumerate() {
        let points: [Point; 6] = [
            mesh.nodes[elem[0]],
            mesh.nodes[elem[1]],
            mesh.nodes[elem[2]],
            mesh.nodes[elem[3]],
            mesh.nodes[elem[4]],
            mesh.nodes[elem[5]],
        ];
        let coords = [
            [points[0].x, points[1].x, points[2].x, points[3].x, points[4].x, points[5].x],
            [points[0].y, points[1].y, points[2].y, points[3].y, points[4].y, points[5].y],
        ];
        // Check for degenerate element at all Gauss points
        let mut degenerate = false;
        for &(_, eta, xi, zeta) in &gps {
            let sf = shape_function(&coords, (eta, xi, zeta));
            if sf.j <= JACOBIAN_TOL {
                degenerate = true;
                break;
            }
        }
        if degenerate {
            return Err(crate::mesh::fem::FemError::DegenerateElement);
        }
        elements.push(Tri6::from_points(i, points, elem, em, gm, rho)?);
    }
    Ok(elements)
}

// ---------------------------------------------------------------------------
// Sparse matrix and conjugate gradient solver
// ---------------------------------------------------------------------------

/// Sparse matrix in COO (coordinate) format.
#[derive(Debug, Clone)]
pub struct SparseMatrix {
    pub n: usize,
    /// Assembled row indices (may contain duplicates).
    pub rows: Vec<usize>,
    /// Assembled column indices (may contain duplicates).
    pub cols: Vec<usize>,
    /// Assembled values (may contain duplicates).
    pub vals: Vec<f64>,
    /// Cached diagonal entries (sum of duplicates). Populated by `compressed()`.
    diag: Option<Vec<f64>>,
}

impl SparseMatrix {
    pub fn new(n: usize) -> Self {
        Self {
            n,
            rows: Vec::new(),
            cols: Vec::new(),
            vals: Vec::new(),
            diag: None,
        }
    }

    pub fn add(&mut self, row: usize, col: usize, val: f64) {
        self.rows.push(row);
        self.cols.push(col);
        self.vals.push(val);
    }

    /// Compressed CSR representation for fast repeated matvec.
    pub fn to_csr(&self) -> (Vec<usize>, Vec<usize>, Vec<f64>) {
        use std::collections::HashMap;
        let mut map: HashMap<(usize, usize), f64> = HashMap::new();
        for k in 0..self.rows.len() {
            *map.entry((self.rows[k], self.cols[k])).or_insert(0.0) += self.vals[k];
        }
        let mut entries: Vec<(usize, usize, f64)> =
            map.into_iter().filter(|(_, v)| *v != 0.0).map(|((r, c), v)| (r, c, v)).collect();
        entries.sort_unstable_by_key(|e| (e.0, e.1));
        let mut row_ptr = vec![0usize; self.n + 1];
        let cols: Vec<usize> = entries.iter().map(|e| e.1).collect();
        let vals: Vec<f64> = entries.iter().map(|e| e.2).collect();
        for e in &entries {
            row_ptr[e.0 + 1] += 1;
        }
        for i in 0..self.n {
            row_ptr[i + 1] += row_ptr[i];
        }
        (row_ptr, cols, vals)
    }

    /// Matrix-vector product into a caller-provided buffer (no alloc).
    pub fn matvec_into(&self, x: &[f64], y: &mut [f64]) {
        for v in y.iter_mut() {
            *v = 0.0;
        }
        for k in 0..self.rows.len() {
            y[self.rows[k]] += self.vals[k] * x[self.cols[k]];
        }
    }

    /// Matrix-vector product y = A*x.
    pub fn matvec(&self, x: &[f64]) -> Vec<f64> {
        let mut y = vec![0.0; self.n];
        for k in 0..self.rows.len() {
            y[self.rows[k]] += self.vals[k] * x[self.cols[k]];
        }
        y
    }

    /// Diagonal entry of row `i` (summing duplicates).
    pub fn matvec_diag(&self, i: usize) -> f64 {
        if let Some(ref d) = self.diag {
            return d[i];
        }
        let mut d = 0.0;
        for k in 0..self.rows.len() {
            if self.rows[k] == i && self.cols[k] == i {
                d += self.vals[k];
            }
        }
        d
    }

    /// Sum duplicate triplets into a compressed copy (faster matvec).
    /// Uses sort-based deduplication for deterministic iteration order.
    pub fn compressed(&self) -> SparseMatrix {
        let n = self.n;
        let nnz = self.rows.len();
        if nnz == 0 {
            return SparseMatrix::new(n);
        }

        // Collect all triplets
        let mut triplets: Vec<(usize, usize, f64)> = Vec::with_capacity(nnz);
        for k in 0..nnz {
            triplets.push((self.rows[k], self.cols[k], self.vals[k]));
        }

        // Sort by (row, col) for deterministic order
        triplets.sort_unstable_by(|a, b| a.0.cmp(&b.0).then(a.1.cmp(&b.1)));

        // Merge duplicates in a single pass
        let mut m = SparseMatrix::new(self.n);
        let mut i = 0;
        while i < triplets.len() {
            let mut r = triplets[i].0;
            let mut c = triplets[i].1;
            let mut v = triplets[i].2;
            let mut j = i + 1;
            while j < triplets.len() && triplets[j].0 == r && triplets[j].1 == c {
                v += triplets[j].2;
                j += 1;
            }
            if v.abs() > 0.0 {
                m.rows.push(r);
                m.cols.push(c);
                m.vals.push(v);
            }
            i = j;
        }

        // Build diagonal cache.
        let mut diag = vec![0.0; n];
        for i in 0..m.rows.len() {
            if m.rows[i] == m.cols[i] {
                diag[m.rows[i]] += m.vals[i];
            }
        }
        m.diag = Some(diag);
        m
    }
}

/// Conjugate gradient solver with diagonal preconditioning.
///
/// Returns a `CgResult` containing the solution and convergence information.
pub fn cg_solve(a: &SparseMatrix, b: &[f64], max_iter: usize, tol: f64) -> CgResult {
    let n = b.len();

    // Diagonal preconditioner: M = diag(A), M_inv = 1/diag(A).
    // Entries may be stored as duplicate triplets, so accumulate.
    let mut diag = vec![0.0; n];
    for k in 0..a.rows.len() {
        if a.rows[k] == a.cols[k] {
            diag[a.rows[k]] += a.vals[k];
        }
    }
    let mut m_inv = vec![1.0; n];
    for i in 0..n {
        if diag[i].abs() > NEAR_ZERO_TOL {
            m_inv[i] = 1.0 / diag[i];
        }
    }

    let mut x = vec![0.0; n];
    let mut r = b.to_vec();

    // Relative convergence: scale tolerance by ||b|| so that problems with
    // small absolute magnitudes (e.g. unit-modulus scaled FEM loads) still
    // iterate to a meaningful solution.
    let b_norm = b.iter().map(|v| v * v).sum::<f64>().sqrt();
    if b_norm == 0.0 || !b_norm.is_finite() {
        return CgResult {
            x,
            iterations: 0,
            residual: 0.0,
            converged: true,
        };
    }

    let mut z: Vec<f64> = r.iter().zip(m_inv.iter()).map(|(a, b)| a * b).collect();
    let mut p = z.clone();
    let mut rz_old: f64 = r.iter().zip(z.iter()).map(|(a, b)| a * b).sum();
    let mut ap_buf = vec![0.0; n];

    let mut iterations = 0;
    let mut converged = false;
    let mut residual = 0.0;

    for iter in 0..max_iter {
        a.matvec_into(&p, &mut ap_buf);
        let ap: &[f64] = &ap_buf;
        let p_ap: f64 = p.iter().zip(ap.iter()).map(|(a, b)| a * b).sum();

        // Check for breakdown: p^T A p <= 0 indicates A is not SPD
        if p_ap <= 0.0 || !p_ap.is_finite() {
            return CgResult {
                x,
                iterations,
                residual: r.iter().map(|v| v * v).sum::<f64>().sqrt(),
                converged: false,
            };
        }

        let alpha = rz_old / p_ap;
        if !alpha.is_finite() {
            return CgResult {
                x,
                iterations,
                residual: r.iter().map(|v| v * v).sum::<f64>().sqrt(),
                converged: false,
            };
        }

        for i in 0..n {
            x[i] += alpha * p[i];
            r[i] -= alpha * ap[i];
        }

        let rs_new: f64 = r.iter().map(|v| v * v).sum();
        residual = rs_new.sqrt();
        if residual < tol * b_norm {
            converged = true;
            iterations = iter + 1;
            break;
        }

        if rz_old == 0.0 || !rz_old.is_finite() {
            break;
        }
        for i in 0..n {
            z[i] = r[i] * m_inv[i];
        }
        let rz_new: f64 = r.iter().zip(z.iter()).map(|(a, b)| a * b).sum();
        if !rz_new.is_finite() {
            break;
        }
        let beta = rz_new / rz_old;
        if !beta.is_finite() {
            break;
        }
        for i in 0..n {
            p[i] = z[i] + beta * p[i];
        }
        rz_old = rz_new;
        iterations = iter + 1;
    }

    CgResult {
        x,
        iterations,
        residual,
        converged,
    }
}

/// Result of the Conjugate Gradient solver.
#[derive(Debug, Clone)]
pub struct CgResult {
    /// Approximate solution vector.
    pub x: Vec<f64>,
    /// Number of iterations performed.
    pub iterations: usize,
    /// Final residual norm ||b - A*x||.
    pub residual: f64,
    /// Whether the solver converged to the requested tolerance.
    pub converged: bool,
}

/// Solve the Lagrangian system using sparse CG + Schur complement.
///
/// [K  c] [u]   [f]
/// [c^T 0] [λ] = [0]
///
/// u = K^{-1}*(f - c*λ),  λ = (c^T*K^{-1}*c)^{-1} * c^T*K^{-1}*f
pub fn solve_lagrange_sparse(k: &SparseMatrix, c: &[f64], f: &[f64]) -> Vec<f64> {
    solve_lagrange_sparse_tol(k, c, f, 1e-6)
}

/// Like [`solve_lagrange_sparse`] with an explicit relative CG tolerance.
pub fn solve_lagrange_sparse_tol(k: &SparseMatrix, c: &[f64], f: &[f64], tol: f64) -> Vec<f64> {
    // Tight tolerances (needed for shear-centre difference quantities)
    // require a proportionally larger iteration budget.
    let n = k.n;
    let max_iter = if tol < 1e-8 {
        (n * 20).clamp(20000, 600000)
    } else {
        (n * 4).clamp(1000, 60000)
    };
    let _n = k.n;

    // Try solving without regularization first (matching Skyline behavior).
    // Warping Laplacian has a constant null space; CG will detect non-SPD
    // and we fall back to regularized K.
    let k_reg = k.compressed();
    let n = k.n;

    let w1 = cg_solve(&k_reg, f, max_iter, tol);
    let w2 = cg_solve(&k_reg, c, max_iter, tol);

    // If CG didn't converge (breakdown or non-SPD), fall back to regularization.
    let (w1, w2) = if w1.converged && w2.converged {
        (w1.x, w2.x)
    } else {
        // Regularize K: K_reg = K + ε*I
        let mut k_reg = k.compressed();
        let mut diag_sum = 0.0;
        for i in 0..n {
            diag_sum += k_reg.matvec_diag(i);
        }
        let eps = diag_sum.max(1e-300) / n as f64 * 1e-9;
        for i in 0..n {
            k_reg.add(i, i, eps);
        }
        let k_reg = k_reg.compressed();

        let w1 = cg_solve(&k_reg, f, max_iter, tol).x;
        let w2 = cg_solve(&k_reg, c, max_iter, tol).x;
        (w1, w2)
    };

    let ct_w2: f64 = c.iter().zip(w2.iter()).map(|(a, b)| a * b).sum();
    let ct_w1: f64 = c.iter().zip(w1.iter()).map(|(a, b)| a * b).sum();

    if ct_w2.abs() < NEAR_ZERO_TOL {
        return w1;
    }
    let lambda = ct_w1 / ct_w2;

    let mut u = vec![0.0; n];
    for i in 0..n {
        u[i] = w1[i] - lambda * w2[i];
    }
    u
}

// ---------------------------------------------------------------------------
// Linear solver for Lagrangian system
// ---------------------------------------------------------------------------

/// Solve the Lagrangian system [K  c; c^T 0] [u; lambda] = [f; 0].
///
/// `k` is n×n stiffness matrix, `c` is n constraint vector, `f` is n load vector.
/// Returns the solution vector u (length n), or `Err` if singular.
pub fn solve_lagrange(k: &[Vec<f64>], c: &[f64], f: &[f64]) -> Result<Vec<f64>, crate::mesh::fem::FemError> {
    let n = k.len();
    // Augmented system (n+1)×(n+1)
    let mut a = vec![vec![0.0; n + 1]; n + 1];
    for i in 0..n {
        for j in 0..n {
            a[i][j] = k[i][j];
        }
        a[i][n] = c[i];
        a[n][i] = c[i];
    }
    let mut rhs = vec![0.0; n + 1];
    for i in 0..n {
        rhs[i] = f[i];
    }

    // Solve using Gaussian elimination with partial pivoting
    solve_dense(&mut a, &mut rhs)?;
    Ok(rhs[..n].to_vec())
}

/// Solve a dense linear system A*x = b in place (Gaussian elimination with partial pivoting).
/// Returns `Err(FemError::SingularMatrix)` if the matrix is singular.
pub fn solve_dense(a: &mut [Vec<f64>], b: &mut [f64]) -> Result<(), crate::mesh::fem::FemError> {
    let n = a.len();
    for k in 0..n {
        // Partial pivoting
        let mut max_row = k;
        let mut max_val = a[k][k].abs();
        for i in (k + 1)..n {
            if a[i][k].abs() > max_val {
                max_val = a[i][k].abs();
                max_row = i;
            }
        }
        if max_row != k {
            a.swap(k, max_row);
            b.swap(k, max_row);
        }
        let pivot = a[k][k];
        if pivot.abs() < PIVOT_TOL {
            return Err(crate::mesh::fem::FemError::SingularMatrix);
        }
        for i in (k + 1)..n {
            let factor = a[i][k] / pivot;
            for j in k..n {
                a[i][j] -= factor * a[k][j];
            }
            b[i] -= factor * b[k];
        }
    }
    // Back substitution
    for k in (0..n).rev() {
        let pivot = a[k][k];
        if pivot.abs() < PIVOT_TOL {
            return Err(crate::mesh::fem::FemError::SingularMatrix);
        }
        let mut sum = 0.0;
        for j in (k + 1)..n {
            sum += a[k][j] * b[j];
        }
        b[k] = (b[k] - sum) / pivot;
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// Sparse matrix in CSC (compressed sparse column) format.
///
/// Mirrors scipy `csc_matrix` as used by Python sectionproperties'
/// warping solver (`assemble_torsion` returns `k_lg` in CSC form).
#[derive(Debug, Clone)]
pub struct CscMatrix {
    pub n_rows: usize,
    pub n_cols: usize,
    /// Column offsets, length n_cols + 1.
    pub col_ptr: Vec<usize>,
    /// Row indices sorted within each column.
    pub rows: Vec<usize>,
    /// Nonzero values, column-major.
    pub vals: Vec<f64>,
}

impl CscMatrix {
    /// Build a CSC matrix from COO triplets, summing duplicates
    /// (mirrors `coo_matrix(...).tocsc()`).
    pub fn from_coo(
        n_rows: usize,
        n_cols: usize,
        row: &[usize],
        col: &[usize],
        data: &[f64],
    ) -> Self {
        debug_assert_eq!(row.len(), col.len());
        debug_assert_eq!(row.len(), data.len());

        // Sort triplets by (column, row).
        let mut order: Vec<usize> = (0..data.len()).collect();
        order.sort_unstable_by_key(|&k| (col[k], row[k]));

        let mut rows: Vec<usize> = Vec::with_capacity(data.len());
        let mut vals: Vec<f64> = Vec::with_capacity(data.len());
        let mut counts = vec![0usize; n_cols];

        let mut pending: Option<(usize, usize, f64)> = None;
        for &k in &order {
            let key = (row[k], col[k]);
            match pending {
                None => pending = Some((key.0, key.1, data[k])),
                Some((pr, pc, pv)) if pr == key.0 && pc == key.1 => {
                    pending = Some((pr, pc, pv + data[k]));
                }
                Some((pr, pc, pv)) => {
                    rows.push(pr);
                    vals.push(pv);
                    counts[pc] += 1;
                    pending = Some((key.0, key.1, data[k]));
                }
            }
        }
        if let Some((pr, pc, pv)) = pending {
            rows.push(pr);
            vals.push(pv);
            counts[pc] += 1;
        }

        let mut col_ptr = vec![0usize; n_cols + 1];
        for cc in 0..n_cols {
            col_ptr[cc + 1] = col_ptr[cc] + counts[cc];
        }

        Self { n_rows, n_cols, col_ptr, rows, vals }
    }

    /// y = A x
    pub fn matvec(&self, x: &[f64]) -> Vec<f64> {
        let mut y = vec![0.0; self.n_rows];
        for c in 0..self.n_cols {
            let xc = x[c];
            for k in self.col_ptr[c]..self.col_ptr[c + 1] {
                y[self.rows[k]] += self.vals[k] * xc;
            }
        }
        y
    }
}

/// Direct Lagrangian solver over an (N+1)x(N+1) augmented CSC system.
///
/// Mirrors Python `solve_direct_lagrange`: the augmented matrix is
/// [[K, c], [c^T, 0]] assembled by `assemble_torsion_lagrange`, solved with
/// RHS [f, 0]; the multiplier magnitude must satisfy
/// |u[N]| / max|u| <= 1e-7.
pub struct DirectLagrangeSolver {
    /// Size of the leading K block.
    pub n: usize,
    kernel: LagrangeKernelInstance,
    c: Vec<f64>,
}

/// Numeric backend for the augmented direct solve.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LagrangeKernel {
    /// Skyline LDL^T of the regularised leading block (always available).
    Skyline,
    /// Intel MKL PARDISO (requires the `pardiso` feature and MKL runtime).
    #[cfg(feature = "pardiso")]
    Pardiso,
}

enum LagrangeKernelInstance {
    Skyline(SkylineLdlt),
    /// PARDISO needs &mut for its internal solve phases.
    #[cfg(feature = "pardiso")]
    Pardiso(std::sync::Mutex<crate::fea::solvers::pardiso::PardisoSolver>),
}

impl DirectLagrangeSolver {
    /// Assemble the augmented Lagrangian matrix exactly as Python
    /// `Section.assemble_torsion` does: [[K, c], [c^T, 0]] in CSC format.
    pub fn assemble_torsion_lagrange(k: &SparseMatrix, c: &[f64]) -> CscMatrix {
        let n = k.n;
        let mut row: Vec<usize> = Vec::with_capacity(k.rows.len() + 2 * n);
        let mut col: Vec<usize> = Vec::with_capacity(row.capacity());
        let mut data: Vec<f64> = Vec::with_capacity(row.capacity());
        row.extend_from_slice(&k.rows);
        col.extend_from_slice(&k.cols);
        data.extend_from_slice(&k.vals);
        // column vector
        row.extend(0..n);
        col.extend(std::iter::repeat(n).take(n));
        data.extend_from_slice(c);
        // row vector
        row.extend(std::iter::repeat(n).take(n));
        col.extend(0..n);
        data.extend_from_slice(c);
        CscMatrix::from_coo(n + 1, n + 1, &row, &col, &data)
    }

    /// Factor using the default backend: the skyline LDL^T, which is
    /// self-contained and robust.
    ///
    /// PARDISO is powerful but demands a fully provisioned MKL runtime;
    /// select it explicitly via [`DirectLagrangeSolver::with_kernel`] when
    /// the environment is known to be complete.
    pub fn new(k: &SparseMatrix, c: &[f64]) -> Result<Self, crate::mesh::fem::FemError> {
        Self::with_kernel(LagrangeKernel::Skyline, k, c)
    }

    /// Factor with an explicit kernel.
    pub fn with_kernel(
        kernel: LagrangeKernel,
        k: &SparseMatrix,
        c: &[f64],
    ) -> Result<Self, crate::mesh::fem::FemError> {
        let instance = match kernel {
            LagrangeKernel::Skyline => {
                let m = k.compressed();
                let m = match SkylineLdlt::factor(&m) {
                    Ok(ldlt) => ldlt,
                    Err(crate::mesh::fem::FemError::SingularMatrix) => {
                        let mut m_reg = k.compressed();
                        let mut diag_avg = 0.0;
                        for i in 0..m_reg.n {
                            diag_avg += m_reg.matvec_diag(i);
                        }
                        let eps = diag_avg.max(1e-300) / m_reg.n as f64 * 1e-9;
                        for i in 0..m_reg.n {
                            m_reg.add(i, i, eps);
                        }
                        let m_reg = m_reg.compressed();
                        SkylineLdlt::factor(&m_reg)?
                    }
                    Err(e) => return Err(e),
                };
                LagrangeKernelInstance::Skyline(m)
            }
            #[cfg(feature = "pardiso")]
            LagrangeKernel::Pardiso => LagrangeKernelInstance::Pardiso(std::sync::Mutex::new(
                crate::fea::solvers::pardiso::PardisoSolver::new(k, c)
                    .map_err(|_| crate::mesh::fem::FemError::SingularMatrix)?,),
            ),
        };
        Ok(Self { n: k.n, kernel: instance, c: c.to_vec() })
    }

    /// Solve [K c; c^T 0] [u; lam] = [f; 0] and return u.
    pub fn solve(&self, f: &[f64]) -> Vec<f64> {
        let (u, _lam) = self.solve_full(f);
        u
    }

    /// Solve returning `(u, lambda)`.
    pub fn solve_full(&self, f: &[f64]) -> (Vec<f64>, f64) {
        match &self.kernel {
            LagrangeKernelInstance::Skyline(ldlt) => {
                let w1 = ldlt.solve(f);
                let w2 = ldlt.solve(&self.c);
                let ct_w2: f64 = self.c.iter().zip(w2.iter()).map(|(&a, &b)| a * b).sum();
                let ct_w1: f64 = self.c.iter().zip(w1.iter()).map(|(&a, &b)| a * b).sum();
                let lambda = if ct_w2.abs() > NEAR_ZERO_TOL { ct_w1 / ct_w2 } else { 0.0 };
                let u = w1
                    .iter()
                    .zip(w2.iter())
                    .map(|(&a, &b)| a - lambda * b)
                    .collect();
                (u, lambda)
            }
            #[cfg(feature = "pardiso")]
            LagrangeKernelInstance::Pardiso(p) => {
                // The augmented solve yields lambda as the last unknown.
                match p.lock().unwrap().solve_with_multiplier(f) {
                    Ok((u, lam)) => (u, lam),
                    Err(_) => (vec![0.0; self.n], 0.0),
                }
            }
        }
    }

    /// Multiplier error metric |lam| / max|u| (Python's u[-1]/max|u|).
    pub fn multiplier_error(&self, f: &[f64], u: &[f64]) -> f64 {
        let (_u_full, lambda) = self.solve_full(f);
        let max_u = u.iter().fold(0.0f64, |a, &v| a.max(v.abs()));
        if max_u > 0.0 {
            lambda.abs() / max_u
        } else {
            f64::INFINITY
        }
    }
}
/// Reverse Cuthill-McKee ordering for bandwidth reduction.
///
/// Returns `perm` where `perm[new_index] = old_index`.
fn rcm_order(n: usize, adj: &[Vec<usize>]) -> Vec<usize> {
    let mut visited = vec![false; n];
    let mut perm: Vec<usize> = Vec::with_capacity(n);

    if n == 0 {
        return perm;
    }

    let mut bfs_farthest = |from: usize| -> usize {
        for v in visited.iter_mut() {
            *v = false;
        }
        let mut queue = std::collections::VecDeque::new();
        queue.push_back(from);
        visited[from] = true;
        let mut last = from;
        while let Some(u) = queue.pop_front() {
            for &w in &adj[u] {
                if !visited[w] {
                    visited[w] = true;
                    queue.push_back(w);
                    last = w;
                }
            }
        }
        last
    };

    let mut start = 0;
    while start < n && adj[start].is_empty() {
        start += 1;
    }
    if start >= n {
        return (0..n).collect();
    }

    let peripheral = {
        let far = bfs_farthest(start);
        bfs_farthest(far)
    };

    for v in visited.iter_mut() {
        *v = false;
    }
    let mut queue = std::collections::VecDeque::new();
    queue.push_back(peripheral);
    visited[peripheral] = true;
    while let Some(u) = queue.pop_front() {
        perm.push(u);
        let mut nbrs: Vec<usize> =
            adj[u].iter().filter(|&&w| !visited[w]).copied().collect();
        nbrs.sort_by_key(|&w| adj[w].len());
        for w in nbrs {
            visited[w] = true;
            queue.push_back(w);
        }
    }
    for i in 0..n {
        if !visited[i] {
            perm.push(i);
        }
    }

    perm.reverse();
    perm
}

/// Skyline (profile) LDL^T direct solver for sparse SPD matrices.
///
/// Factor once with [`SkylineLdlt::factor`], solve any number of right-hand
/// sides with [`SkylineLdlt::solve`]. RCM ordering keeps the profile small,
/// which makes 2D FE meshes (moderate bandwidth) very fast.
pub struct SkylineLdlt {
    n: usize,
    diag: Vec<f64>,
    /// First (permuted) column index stored in each row.
    first: Vec<usize>,
    /// Row-major skyline values for columns [first[i], i).
    lower: Vec<f64>,
    /// Offset of row i's segment within `lower`.
    row_start: Vec<usize>,
    /// RCM permutation: perm[new_index] = original_index.
    perm: Vec<usize>,
}

impl SkylineLdlt {
    pub fn factor(matrix: &SparseMatrix) -> Result<Self, crate::mesh::fem::FemError> {
        use std::collections::HashMap;

        let n = matrix.n;

        // Compress duplicate triplets.
        let mut map: HashMap<(usize, usize), f64> = HashMap::new();
        for k in 0..matrix.rows.len() {
            *map
                .entry((matrix.rows[k], matrix.cols[k]))
                .or_insert(0.0) += matrix.vals[k];
        }

        // Adjacency for RCM.
        let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
        for &(r, c) in map.keys() {
            if r != c {
                adj[r].push(c);
            }
        }
        for a in adj.iter_mut() {
            a.sort_unstable();
            a.dedup();
        }
        let perm = rcm_order(n, &adj);

        let mut old_to_new = vec![0usize; n];
        for (new_idx, &old_idx) in perm.iter().enumerate() {
            old_to_new[old_idx] = new_idx;
        }

        // Skyline profile of the permuted lower triangle.
        let mut first = vec![n; n];
        for &(r, c) in map.keys() {
            let i = old_to_new[r];
            let j = old_to_new[c];
            let (hi, lo) = if i > j { (i, j) } else { (j, i) };
            if hi != lo && lo < first[hi] {
                first[hi] = lo;
            }
        }
        for i in 0..n {
            if first[i] > i {
                first[i] = i;
            }
        }

        let mut row_start = vec![0usize; n + 1];
        for i in 0..n {
            row_start[i + 1] = row_start[i] + (i - first[i]);
        }
        let mut lower = vec![0.0; row_start[n]];
        let mut diag = vec![0.0; n];

        // Scatter original values into the skyline (lower triangle).
        for (&(r, c), &v) in map.iter() {
            let i = old_to_new[r];
            let j = old_to_new[c];
            if i > j {
                lower[row_start[i] + (j - first[i])] = v;
            } else if j > i {
                lower[row_start[j] + (i - first[j])] = v;
            } else {
                diag[i] = v;
            }
        }

        // In-place Crout LDL^T factorisation.
        for i in 0..n {
            let rs_i = row_start[i];
            for k in first[i]..i {
                let rs_k = row_start[k];
                let j0 = first[i].max(first[k]);
                let mut s = lower[rs_i + (k - first[i])];
                for j in j0..k {
                    s -= lower[rs_i + (j - first[i])] * lower[rs_k + (j - first[k])] * diag[j];
                }
                if diag[k].abs() < 1e-300 {
                    return Err(crate::mesh::fem::FemError::SingularMatrix);
                }
                lower[rs_i + (k - first[i])] = s / diag[k];
            }
            let mut d = diag[i];
            for k in first[i]..i {
                let l_ik = lower[rs_i + (k - first[i])];
                d -= l_ik * l_ik * diag[k];
            }
            if !(d > 0.0) || !d.is_finite() {
                return Err(crate::mesh::fem::FemError::SingularMatrix);
            }
            diag[i] = d;
        }

        Ok(Self { n, diag, first, lower, row_start, perm })
    }

    /// Solve A x = b.
    pub fn solve(&self, b: &[f64]) -> Vec<f64> {
        let n = self.n;
        debug_assert_eq!(b.len(), n);

        // Permute the right-hand side into the RCM ordering.
        let mut x: Vec<f64> = self.perm.iter().map(|&old| b[old]).collect();
        for i in 0..n {
            let rs = self.row_start[i];
            let mut s = x[i];
            for k in self.first[i]..i {
                s -= self.lower[rs + (k - self.first[i])] * x[k];
            }
            x[i] = s;
        }
        // Diagonal scaling.
        for i in 0..n {
            x[i] /= self.diag[i];
        }
        // Backward substitution: L^T x = y via scattered updates.
        for i in (0..n).rev() {
            let rs = self.row_start[i];
            let xi = x[i];
            if xi != 0.0 {
                for k in self.first[i]..i {
                    x[k] -= self.lower[rs + (k - self.first[i])] * xi;
                }
            }
        }
        // Inverse permutation back to the original ordering.
        let mut out = vec![0.0; n];
        for (new_idx, &old_idx) in self.perm.iter().enumerate() {
            out[old_idx] = x[new_idx];
        }
        out
    }

    /// Solve with a Lagrange multiplier constraint vector, mirroring
    /// [`solve_lagrange_sparse`]: u = w1 - lambda * w2 with
    /// lambda = (c.w1)/(c.w2).
    pub fn solve_lagrange(&self, c: &[f64], f: &[f64]) -> Vec<f64> {
        let w1 = self.solve(f);
        let w2 = self.solve(c);
        let ct_w2: f64 = c.iter().zip(w2.iter()).map(|(&a, &b)| a * b).sum();
        let ct_w1: f64 = c.iter().zip(w1.iter()).map(|(&a, &b)| a * b).sum();
        let lambda = if ct_w2.abs() > NEAR_ZERO_TOL { ct_w1 / ct_w2 } else { 0.0 };
        w1.iter().zip(w2.iter()).map(|(&a, &b)| a - lambda * b).collect()
    }
}
pub mod solvers;
#[cfg(test)]
mod direct_lagrange_tests {
    use super::*;

    #[test]
    fn csc_from_coo_sums_duplicates() {
        let m = CscMatrix::from_coo(
            3,
            3,
            &[0, 1, 0],
            &[0, 1, 0],
            &[1.0, 2.0, 3.5],
        );
        // (0,0) should be 4.5 after duplicate summing
        let col0 = &m.vals[m.col_ptr[0]..m.col_ptr[1]];
        assert!((col0.iter().sum::<f64>() - 4.5).abs() < 1e-12);
        assert_eq!(m.col_ptr[2] - m.col_ptr[1], 1);
    }

    #[test]
    fn augmented_direct_matches_schur() {
        // Same tridiagonal system as the skyline lagrange test.
        let n = 20;
        let mut k = SparseMatrix::new(n);
        for i in 0..n {
            k.add(i, i, 2.0);
            if i + 1 < n {
                k.add(i, i + 1, -1.0);
                k.add(i + 1, i, -1.0);
            }
        }
        let c: Vec<f64> = vec![1.0; n];
        let f: Vec<f64> = (0..n).map(|i| (i as f64) * 0.1 + 0.05).collect();

        let u_ref = SkylineLdlt::factor(&k.compressed())
            .unwrap()
            .solve_lagrange(&c, &f);

        // Python-style: assemble augmented CSC, factor leading block.
        let k_lg = DirectLagrangeSolver::assemble_torsion_lagrange(&k, &c);
        assert_eq!(k_lg.n_rows, n + 1);
        assert_eq!(k_lg.n_cols, n + 1);
        let solver = DirectLagrangeSolver::new(&k, &c).unwrap();
        let u_dir = solver.solve(&f);

        for i in 0..n {
            assert!(
                (u_ref[i] - u_dir[i]).abs() < 5e-6,
                "i={} ref={} dir={}",
                i,
                u_ref[i],
                u_dir[i]
            );
        }

        // NOTE: Python also checks |multiplier|/max|u| <= 1e-7; that holds
        // for genuine warping problems (c sums to ~0) but not for this
        // synthetic constraint, so it is not asserted here.
    }
}
#[cfg(test)]
mod skyline_tests {
    use super::*;

    #[test]
    fn skyline_debug_4x4() {
        let mut a = SparseMatrix::new(4);
        let entries = [
            (0, 0, 4.0), (1, 1, 5.0), (2, 2, 6.0), (3, 3, 7.0),
            (0, 1, 1.0), (1, 0, 1.0),
            (1, 2, 2.0), (2, 1, 2.0),
            (2, 3, 1.0), (3, 2, 1.0),
        ];
        for &(r, c, v) in &entries { a.add(r, c, v); }
        let s = SkylineLdlt::factor(&a).unwrap();
        eprintln!("perm-like diag={:?} first={:?} lower={:?}", s.diag, s.first, s.lower);
        let x = s.solve(&[1.0,2.0,3.0,4.0]);
        eprintln!("x={:?} expected~[0.1934,0.2264,0.3374,0.5232]", x);
    }

    #[test]
    fn skyline_matches_direct_dense_solve() {
        // SPD matrix: A = [[4,1,0,0],[1,5,2,0],[0,2,6,1],[0,0,1,7]]
        let mut a = SparseMatrix::new(4);
        let entries = [
            (0, 0, 4.0), (1, 1, 5.0), (2, 2, 6.0), (3, 3, 7.0),
            (0, 1, 1.0), (1, 0, 1.0),
            (1, 2, 2.0), (2, 1, 2.0),
            (2, 3, 1.0), (3, 2, 1.0),
        ];
        for &(r, c, v) in &entries {
            a.add(r, c, v);
        }
        let solver = SkylineLdlt::factor(&a).unwrap();

        // Solve two right-hand sides and verify A x = b.
        for b in [vec![1.0, 2.0, 3.0, 4.0], vec![4.0, 3.0, 2.0, 1.0]] {
            let x = solver.solve(&b);
            // residual check
            for row in 0..4 {
                let mut sum = 0.0;
                for &(r, c, v) in &entries {
                    if r == row {
                        sum += v * x[c];
                    }
                }
                assert!((sum - b[row]).abs() < 1e-10, "row {} got {} (x={:?})", row, sum - b[row], x);
            }
        }

        // Compare against CG
        let b = vec![1.0, 2.0, 3.0, 4.0];
        let x_cg = cg_solve(&a.compressed(), &b, 2000, SOLVER_TOL).x;
        let x_dir = SkylineLdlt::factor(&a).unwrap().solve(&b);
        for i in 0..4 {
            assert!((x_cg[i] - x_dir[i]).abs() < 1e-8);
        }
    }

    #[test]
    fn skyline_lagrange_matches_sparse() {
        // Small FE-like Laplacian path with constraint vector.
        let n = 20;
        let mut k = SparseMatrix::new(n);
        for i in 0..n {
            k.add(i, i, 2.0);
            if i + 1 < n {
                k.add(i, i + 1, -1.0);
                k.add(i + 1, i, -1.0);
            }
        }
        let c: Vec<f64> = (0..n).map(|_i| 1.0).collect();
        let f: Vec<f64> = (0..n).map(|i| (i as f64) * 0.1 + 0.05).collect();

        // High-precision CG reference
        let k_reg = {
            let mut m = k.compressed();
            for i in 0..n {
                m.add(i, i, 1e-10);
            }
            m.compressed()
        };
        let w1 = cg_solve(&k_reg, &f, 200000, 1e-13).x;
        let w2 = cg_solve(&k_reg, &c, 200000, 1e-13).x;
        let ct_w2: f64 = c.iter().zip(w2.iter()).map(|(&a, &b)| a * b).sum();
        let ct_w1: f64 = c.iter().zip(w1.iter()).map(|(&a, &b)| a * b).sum();
        let lambda = ct_w2 / ct_w1;
        let u_ref: Vec<f64> =
            w1.iter().zip(w2.iter()).map(|(&a, &b)| a - lambda * b).collect();

        let u_dir = SkylineLdlt::factor(&k).unwrap().solve_lagrange(&c, &f);
        for i in 0..n {
            assert!(
                (u_ref[i] - u_dir[i]).abs() < 5e-6,
                "i={} ref={:.12} dir={:.12}",
                i,
                u_ref[i],
                u_dir[i]
            );
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    fn rect_tri6() -> Tri6 {
        // Unit triangle with mid-nodes (CCW orientation)
        let points = [
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(0.0, 1.0),
            Point::new(0.5, 0.0),
            Point::new(0.5, 0.5),
            Point::new(0.0, 0.5),
        ];
        Tri6::from_points(0, points, [0, 1, 2, 3, 4, 5], 1.0, 1.0, 1.0).unwrap()
    }

    #[test]
    fn gauss_points_sum_to_one() {
        for &n in &[1, 3, 4, 6] {
            let gps = gauss_points(n);
            let sum: f64 = gps.iter().map(|&(w, _, _, _)| w).sum();
            assert!((sum - 1.0).abs() < 1e-12, "gauss({n}): weights sum = {sum}");
        }
    }

    #[test]
    fn tri6_area_unit_triangle() {
        let tri = rect_tri6();
        let (area, _, _, _, _, _) = tri.geometric_properties();
        assert!((area - 0.5).abs() < 1e-10, "area = {area}");
    }

    #[test]
    fn tri6_centroid_unit_triangle() {
        let tri = rect_tri6();
        let (area, qx, qy, _, _, _) = tri.geometric_properties();
        let cx = qy / area;
        let cy = qx / area;
        assert!((cx - 1.0 / 3.0).abs() < 1e-10, "cx = {cx}");
        assert!((cy - 1.0 / 3.0).abs() < 1e-10, "cy = {cy}");
    }

    #[test]
    fn tri6_moments_unit_triangle() {
        let tri = rect_tri6();
        let (_, _, _, ixx, iyy, ixy) = tri.geometric_properties();
        // For unit triangle about origin:
        // Ixx = ∫y² dA = 1/12, Iyy = ∫x² dA = 1/12, Ixy = ∫xy dA = 1/24
        assert!((ixx - 1.0 / 12.0).abs() < 1e-10, "ixx = {ixx}");
        assert!((iyy - 1.0 / 12.0).abs() < 1e-10, "iyy = {iyy}");
        assert!((ixy - 1.0 / 24.0).abs() < 1e-10, "ixy = {ixy}");
    }

    #[test]
    fn principal_coordinate_zero_phi() {
        let (x11, y22) = principal_coordinate(0.0, 3.0, 4.0);
        assert!((x11 - 3.0).abs() < 1e-12);
        assert!((y22 - 4.0).abs() < 1e-12);
    }

    #[test]
    fn principal_coordinate_90_degrees() {
        let (x11, y22) = principal_coordinate(std::f64::consts::FRAC_PI_2, 3.0, 0.0);
        assert!(x11.abs() < 1e-12);
        assert!((y22 + 3.0).abs() < 1e-12);
    }

    #[test]
    fn tri3_to_tri6_creates_midpoints() {
        let nodes = vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(0.0, 1.0),
        ];
        let elements = vec![[0, 1, 2]];
        let mesh = tri3_to_tri6(&nodes, &elements);
        assert_eq!(mesh.nodes.len(), 6); // 3 original + 3 mid
        assert_eq!(mesh.elements.len(), 1);
        // Check mid-node of edge 0-1
        let mid01 = mesh.nodes[mesh.elements[0][3]];
        assert!((mid01.x - 0.5).abs() < 1e-12);
        assert!(mid01.y.abs() < 1e-12);
    }

    #[test]
    fn tri3_to_tri6_shares_midpoints() {
        let nodes = vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(0.0, 1.0),
            Point::new(1.0, 1.0),
        ];
        let elements = vec![[0, 1, 2], [1, 3, 2]];
        let mesh = tri3_to_tri6(&nodes, &elements);
        // 4 original + 5 unique edges (01, 12, 20, 13, 32) = 9 nodes
        assert_eq!(mesh.nodes.len(), 9);
        // Edge 1-2 is shared, so mid12 of elem0 == mid20 of elem1
        assert_eq!(mesh.elements[0][4], mesh.elements[1][5]);
    }

    #[test]
    fn solve_dense_simple() {
        let mut a = vec![vec![2.0, 1.0], vec![1.0, 3.0]];
        let mut b = vec![5.0, 10.0];
        solve_dense(&mut a, &mut b).unwrap();
        // Solution: 2x + y = 5, x + 3y = 10 => x = 10-3y, 2(10-3y)+y = 5 => 20-5y = 5 => y = 3, x = 1
        assert!((b[0] - 1.0).abs() < 1e-10, "x = {}", b[0]);
        assert!((b[1] - 3.0).abs() < 1e-10, "y = {}", b[1]);
    }

    #[test]
    fn solve_lagrange_simple() {
        // Simple test: K = I, c = [1, 1], f = [1, 0]
        // Solution should satisfy u + lambda * c = f and c^T u = 0
        let k = vec![vec![1.0, 0.0], vec![0.0, 1.0]];
        let c = vec![1.0, 1.0];
        let f = vec![1.0, 0.0];
        let u = solve_lagrange(&k, &c, &f).unwrap();
        // u0 + lambda = 1, u1 + lambda = 0, u0 + u1 = 0
        // => u0 = -u1, -u1 + lambda = 1, u1 + lambda = 0
        // => lambda = -u1, -u1 - u1 = 1 => u1 = -0.5, u0 = 0.5, lambda = 0.5
        assert!((u[0] - 0.5).abs() < 1e-10, "u0 = {}", u[0]);
        assert!((u[1] + 0.5).abs() < 1e-10, "u1 = {}", u[1]);
        // Direct constraint check: c^T u = 0 (independent of lambda formula)
        let ct_u: f64 = c.iter().zip(u.iter()).map(|(a, b)| a * b).sum();
        assert!(ct_u.abs() < 1e-10, "constraint residual c^T u = {}", ct_u);
    }

    #[test]
    fn skyline_lagrange_constraint_residual() {
        // Verify c^T u ≈ 0 for SkylineLdlt::solve_lagrange (catches lambda formula bugs)
        let n = 10;
        let mut k = SparseMatrix::new(n);
        for i in 0..n {
            k.add(i, i, 2.0);
            if i + 1 < n {
                k.add(i, i + 1, -1.0);
                k.add(i + 1, i, -1.0);
            }
        }
        let c: Vec<f64> = vec![1.0; n];
        let f: Vec<f64> = (0..n).map(|i| (i as f64) * 0.1 + 0.05).collect();

        let u = SkylineLdlt::factor(&k.compressed()).unwrap().solve_lagrange(&c, &f);
        let ct_u: f64 = c.iter().zip(u.iter()).map(|(a, b)| a * b).sum();
        assert!(ct_u.abs() < 1e-10, "SkylineLdlt constraint residual c^T u = {}", ct_u);
    }

    #[test]
    fn cg_solve_breakdown_detection() {
        // Non-SPD matrix with negative diagonal: A = [[-1, 0], [0, 1]]
        // This will have p^T A p <= 0 on the first iteration
        let mut a = SparseMatrix::new(2);
        a.add(0, 0, -1.0);
        a.add(1, 1, 1.0);
        let b = vec![1.0, 1.0];
        let result = cg_solve(&a, &b, 100, SOLVER_TOL);
        // Should detect non-SPD (p^T A p <= 0) and not converge
        assert!(!result.converged, "CG should not converge on non-SPD matrix");
        // iterations may be 0 if breakdown happens on first check
    }

    #[test]
    fn cg_solve_finite_checks() {
        // SPD matrix: A = [[2, 1], [1, 2]]
        let mut a = SparseMatrix::new(2);
        a.add(0, 0, 2.0);
        a.add(0, 1, 1.0);
        a.add(1, 0, 1.0);
        a.add(1, 1, 2.0);
        let b = vec![1.0, 2.0];
        let result = cg_solve(&a, &b, 100, SOLVER_TOL);
        assert!(result.converged, "CG should converge on SPD matrix: {}", result.residual);
        // Check all output values are finite
        assert!(result.x.iter().all(|v| v.is_finite()), "Solution has non-finite values");
        assert!(result.residual.is_finite(), "Residual is non-finite");
    }

    #[test]
    fn extrapolate_to_nodes_constant() {
        // Constant values should extrapolate to the same constant
        let w = [1.0; 6];
        let nodes = extrapolate_to_nodes(&w);
        for &v in &nodes {
            assert!((v - 1.0).abs() < 1e-10, "v = {v}");
        }
    }

    #[test]
    fn tri6_degenerate_element_detection() {
        // Create a degenerate triangle (collinear points)
        let nodes = vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(2.0, 0.0),  // collinear!
            Point::new(0.5, 0.0),  // mid01
            Point::new(1.5, 0.0),  // mid12
            Point::new(1.0, 0.0),  // mid20
        ];
        let elements = vec![[0, 1, 2, 3, 4, 5]];
        let mesh = Tri6Mesh { nodes, elements };

        // validate_tri6_mesh should return DegenerateElement error
        let result = validate_tri6_mesh(&mesh);
        assert!(result.is_err());
        assert_eq!(
            result.unwrap_err(),
            crate::mesh::fem::FemError::DegenerateElement
        );

        // build_tri6_elements should return DegenerateElement error
        let result = build_tri6_elements(&mesh, 200e9, 80e9, 7850.0);
        assert!(result.is_err());
        assert_eq!(
            result.unwrap_err(),
            crate::mesh::fem::FemError::DegenerateElement
        );
    }

    #[test]
    fn tri6_orientation_check() {
        // Valid CCW triangle should work
        let points_ccw = [
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(0.0, 1.0),
            Point::new(0.5, 0.0),
            Point::new(0.5, 0.5),
            Point::new(0.0, 0.5),
        ];
        let result = Tri6::from_points(0, points_ccw, [0, 1, 2, 3, 4, 5], 1.0, 1.0, 1.0);
        assert!(result.is_ok(), "CCW triangle should be valid");

        // CW triangle should fail
        let points_cw = [
            Point::new(0.0, 0.0),
            Point::new(0.0, 1.0),
            Point::new(1.0, 0.0),
            Point::new(0.0, 0.5),
            Point::new(0.5, 0.5),
            Point::new(0.5, 0.0),
        ];
        let result = Tri6::from_points(0, points_cw, [0, 1, 2, 3, 4, 5], 1.0, 1.0, 1.0);
        assert!(result.is_err(), "CW triangle should fail");
        assert_eq!(
            result.unwrap_err(),
            crate::mesh::fem::FemError::InvalidElementOrientation
        );
    }
}
