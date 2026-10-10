//! 3D frame element foundation — Euler–Bernoulli formulation.
//!
//! This module provides the element-level foundation for 3D frame analysis:
//! section/material parameters, element geometry with orientation, the 12×12
//! local elastic stiffness matrix, and the local-to-global coordinate
//! transformation.
//!
//! **No global assembly, solver, boundary conditions, load cases, or result
//! recovery is implemented here.** Those belong to later phases (134+).
//!
//! # Mathematical model
//!
//! A conventional linear-elastic 3D Euler–Bernoulli frame element with 6 DOF
//! per node: `[u, v, w, rx, ry, rz]` (3 translations + 3 rotations). The
//! element has 12 DOF total.
//!
//! The local stiffness combines four independent actions:
//!
//! - **Axial** (`EA/L`): 2×2 block coupling `u_i`, `u_j`.
//! - **Torsion** (`GJ/L`): 2×2 block coupling `rx_i`, `rx_j`.
//! - **Bending about local z** (`EIz`): couples `v` and `rz` (bending in the
//!   local x-y plane).
//! - **Bending about local y** (`EIy`): couples `w` and `ry` (bending in the
//!   local x-z plane).
//!
//! Shear deformation (Timoshenko) is **not** included. The formulation is
//! pure Euler–Bernoulli (thin beam, plane sections remain plane and
//! perpendicular to the axis).
//!
//! # Local DOF ordering
//!
//! ```text
//! [u_i, v_i, w_i, rx_i, ry_i, rz_i, u_j, v_j, w_j, rx_j, ry_j, rz_j]
//!    0    1    2     3     4     5    6    7    8     9    10    11
//! ```
//!
//! - `u` — axial displacement (along local x).
//! - `v` — transverse displacement along local y.
//! - `w` — transverse displacement along local z.
//! - `rx` — rotation about local x (torsion).
//! - `ry` — rotation about local y (bending in x-z plane).
//! - `rz` — rotation about local z (bending in x-y plane).
//!
//! # Local-axis convention
//!
//! 1. **Local x** runs from `node_i` to `node_j`: `x̂ = (p_j − p_i) / L`.
//! 2. A caller-provided **reference vector** `ref` defines the roll
//!    orientation. It must not be parallel (or nearly parallel) to the
//!    member axis.
//! 3. **Local z** = `(x̂ × ref) / |x̂ × ref|`.
//! 4. **Local y** = `ẑ × x̂`.
//!
//! The resulting `(x̂, ŷ, ẑ)` triple is right-handed and orthonormal.
//! The reference vector is typically global Z (or global Y for vertical
//! members), but any non-parallel vector is accepted.
//!
//! # Transformation convention
//!
//! The 12×12 transformation matrix `T` maps **global** element DOFs to
//! **local** element DOFs:
//!
//! ```text
//! u_local = T · u_global
//! ```
//!
//! `T` is block-diagonal with four copies of the 3×3 direction cosine matrix
//! `R` (rows = local axes in global coordinates). The global stiffness is
//! obtained by congruence:
//!
//! ```text
//! K_global = Tᵀ · K_local · T
//! ```
//!
//! # Stiffness matrix reference
//!
//! The local stiffness matrix follows the standard 3D Euler–Bernoulli frame
//! element formulation as given in:
//!
//! - Bathe, K.-J. *Finite Element Procedures*, 2nd ed. (2014), §5.6.
//! - Cook, R.D., Malkus, D.S., Plesha, M.E. *Concepts and Applications of
//!   Finite Element Analysis*, 4th ed. (2002), §4.3.
//! - Logan, D.L. *A First Course in the Finite Element Method*, 4th ed.
//!   (2007), §5.6.
//!
//! The sign convention for bending about y (x-z plane) has **opposite**
//! coupling signs compared to bending about z (x-y plane), because the
//! right-hand rule gives `θy = −dw/dx` whereas `θz = +dv/dx`.
//!
//! # Torsion constant semantics
//!
//! `J` is the **Saint-Venant torsional constant**, *not* the polar second
//! moment of area (`I_p = I_y + I_z`). For circular cross-sections they
//! coincide; for non-circular sections (I-sections, rectangles, etc.) `J`
//! differs significantly and must be obtained from a torsion analysis (e.g.
//! `WarpingProperties.j` in `section-properties`). Do not silently substitute
//! `I_p` for `J`.
//!
//! # Units
//!
//! All quantities use a consistent unit system supplied by the caller. No
//! implicit unit conversion is performed. Typical SI: `E`, `G` in Pa;
//! `A` in m²; `Iy`, `Iz`, `J` in m⁴; coordinates in m.

#![allow(non_snake_case)]

use crate::beam_fem::FemError;

// ---------------------------------------------------------------------------
// Section / material parameters
// ---------------------------------------------------------------------------

/// Section and material parameters for a 3D Euler–Bernoulli frame element.
///
/// Combines Young's modulus `E`, shear modulus `G`, and the four geometric
/// section properties needed for the 12×12 local stiffness: area `A`,
/// second moments `Iy` and `Iz`, and the Saint-Venant torsional constant `J`.
///
/// # Validation
///
/// All six scalars must be finite and strictly positive. The constructor
/// rejects zero, negative, NaN, or infinite values.
///
/// # Euler–Bernoulli assumptions
///
/// This type does **not** include shear areas (`Ay`, `Az`); transverse shear
/// deformation is excluded. The formulation is pure Euler–Bernoulli (thin
/// beam theory). Timoshenko shear deformation is a future enhancement.
///
/// # Torsion constant
///
/// `j` is the Saint-Venant torsional constant, *not* the polar second moment
/// of area. For non-circular sections, use the value from a torsion analysis
/// (e.g. `WarpingProperties.j`).
#[derive(Debug, Clone, Copy)]
pub struct FrameSection3D {
    /// Young's modulus `E` (Pa in SI).
    pub E: f64,
    /// Shear modulus `G` (Pa in SI).
    pub G: f64,
    /// Cross-sectional area `A` (m² in SI).
    pub area: f64,
    /// Second moment of area about the local y-axis `Iy` (m⁴ in SI).
    pub iy: f64,
    /// Second moment of area about the local z-axis `Iz` (m⁴ in SI).
    pub iz: f64,
    /// Saint-Venant torsional constant `J` (m⁴ in SI).
    pub j: f64,
}

impl FrameSection3D {
    /// Create a validated 3D frame section.
    ///
    /// # Errors
    ///
    /// Returns [`FemError::InvalidInput`] if any parameter is non-finite or
    /// non-positive.
    pub fn new(E: f64, G: f64, area: f64, iy: f64, iz: f64, j: f64) -> Result<Self, FemError> {
        for (label, val) in [
            ("Young's modulus E", E),
            ("shear modulus G", G),
            ("cross-sectional area A", area),
            ("second moment Iy", iy),
            ("second moment Iz", iz),
            ("torsional constant J", j),
        ] {
            if !val.is_finite() {
                return Err(FemError::InvalidInput(format!(
                    "{label} must be finite, got {val}"
                )));
            }
            if val <= 0.0 {
                return Err(FemError::InvalidInput(format!(
                    "{label} must be positive, got {val}"
                )));
            }
        }
        Ok(Self {
            E,
            G,
            area,
            iy,
            iz,
            j,
        })
    }
}

// ---------------------------------------------------------------------------
// Element
// ---------------------------------------------------------------------------

/// Tolerance for detecting near-parallel orientation vectors.
///
/// The check is on `sin(angle)` between the member axis and the reference
/// vector, which is dimensionless and therefore scale-aware. A value of
/// `1e-8` corresponds to an angle of approximately `5.7 × 10⁻⁷` radians
/// (~0.00003°), well below any practically meaningful orientation.
const PARALLEL_TOL: f64 = 1e-8;

/// A 3D Euler–Bernoulli frame element (12 DOF).
///
/// Stores node indices, section/material parameters, and a reference vector
/// for local-axis orientation. Coordinates are supplied to methods (not
/// stored), following the same pattern as [`crate::beam_fem::BeamElement`]
/// and [`crate::truss3d::TrussElement3D`].
///
/// # Local axes
///
/// See the [module-level documentation](crate::frame3d) for the full
/// convention. In brief:
///
/// 1. Local x = `(p_j − p_i) / L`.
/// 2. Local z = `(x̂ × ref) / |x̂ × ref|`.
/// 3. Local y = `ẑ × x̂`.
///
/// The reference vector must not be parallel (or nearly parallel) to the
/// member axis.
#[derive(Debug, Clone)]
pub struct FrameElement3D {
    /// Index of the first (start) node.
    pub node_i: usize,
    /// Index of the second (end) node.
    pub node_j: usize,
    /// Section and material parameters.
    pub section: FrameSection3D,
    /// Reference vector for local-axis orientation. Need not be unit length;
    /// will be normalized internally. Must not be parallel to the member axis.
    pub ref_vec: [f64; 3],
}

impl FrameElement3D {
    /// Create a 3D frame element with rigid end connections.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidModel`] if `node_i == node_j`.
    /// [`FemError::InvalidInput`] if `ref_vec` contains non-finite values or
    /// is a zero vector.
    pub fn new(
        node_i: usize,
        node_j: usize,
        section: FrameSection3D,
        ref_vec: [f64; 3],
    ) -> Result<Self, FemError> {
        if node_i == node_j {
            return Err(FemError::InvalidModel(
                "frame element cannot have same start and end node".to_string(),
            ));
        }
        if !ref_vec[0].is_finite() || !ref_vec[1].is_finite() || !ref_vec[2].is_finite() {
            return Err(FemError::InvalidInput(format!(
                "ref_vec must be finite, got [{}, {}, {}]",
                ref_vec[0], ref_vec[1], ref_vec[2]
            )));
        }
        let ref_norm =
            (ref_vec[0] * ref_vec[0] + ref_vec[1] * ref_vec[1] + ref_vec[2] * ref_vec[2]).sqrt();
        if ref_norm == 0.0 {
            return Err(FemError::InvalidInput(
                "ref_vec must be nonzero".to_string(),
            ));
        }
        Ok(Self {
            node_i,
            node_j,
            section,
            ref_vec,
        })
    }

    /// Element length from node coordinates.
    pub fn length(&self, pi: (f64, f64, f64), pj: (f64, f64, f64)) -> f64 {
        let dx = pj.0 - pi.0;
        let dy = pj.1 - pi.1;
        let dz = pj.2 - pi.2;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Compute the orthonormal local-axis triple `(x_hat, y_hat, z_hat)`.
    ///
    /// Each axis is a unit vector in global coordinates. The triple is
    /// right-handed: `x̂ × ŷ = ẑ`.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if any coordinate is non-finite.
    /// [`FemError::ZeroLengthMember`] if the element has zero length.
    /// [`FemError::InvalidInput`] if the reference vector is parallel (or
    /// nearly parallel) to the member axis.
    #[allow(clippy::type_complexity)]
    pub fn local_axes(
        &self,
        pi: (f64, f64, f64),
        pj: (f64, f64, f64),
    ) -> Result<([f64; 3], [f64; 3], [f64; 3]), FemError> {
        if !pi.0.is_finite() || !pi.1.is_finite() || !pi.2.is_finite() {
            return Err(FemError::InvalidInput(format!(
                "node i coordinates must be finite, got ({}, {}, {})",
                pi.0, pi.1, pi.2
            )));
        }
        if !pj.0.is_finite() || !pj.1.is_finite() || !pj.2.is_finite() {
            return Err(FemError::InvalidInput(format!(
                "node j coordinates must be finite, got ({}, {}, {})",
                pj.0, pj.1, pj.2
            )));
        }

        let L = self.length(pi, pj);
        if L <= 0.0 {
            return Err(FemError::ZeroLengthMember(format!(
                "element from ({}, {}, {}) to ({}, {}, {}) has zero length",
                pi.0, pi.1, pi.2, pj.0, pj.1, pj.2
            )));
        }

        // Local x-axis
        let x_hat = [(pj.0 - pi.0) / L, (pj.1 - pi.1) / L, (pj.2 - pi.2) / L];

        // Cross product x_hat × ref
        let cross = [
            x_hat[1] * self.ref_vec[2] - x_hat[2] * self.ref_vec[1],
            x_hat[2] * self.ref_vec[0] - x_hat[0] * self.ref_vec[2],
            x_hat[0] * self.ref_vec[1] - x_hat[1] * self.ref_vec[0],
        ];
        let cross_norm = (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt();

        let ref_norm = (self.ref_vec[0] * self.ref_vec[0]
            + self.ref_vec[1] * self.ref_vec[1]
            + self.ref_vec[2] * self.ref_vec[2])
            .sqrt();

        // sin(angle) between x_hat and ref
        let sin_angle = cross_norm / ref_norm;
        if sin_angle < PARALLEL_TOL {
            return Err(FemError::InvalidInput(format!(
                "ref_vec is parallel (or nearly parallel) to member axis; \
                 sin(angle) = {sin_angle:.3e} < tolerance {PARALLEL_TOL:.3e}"
            )));
        }

        // Local z-axis
        let z_hat = [
            cross[0] / cross_norm,
            cross[1] / cross_norm,
            cross[2] / cross_norm,
        ];

        // Local y-axis = z_hat × x_hat
        let y_hat = [
            z_hat[1] * x_hat[2] - z_hat[2] * x_hat[1],
            z_hat[2] * x_hat[0] - z_hat[0] * x_hat[2],
            z_hat[0] * x_hat[1] - z_hat[1] * x_hat[0],
        ];

        Ok((x_hat, y_hat, z_hat))
    }

    /// Compute the 12×12 local stiffness matrix in local coordinates.
    ///
    /// Local DOF ordering: `[u_i, v_i, w_i, rx_i, ry_i, rz_i,
    /// u_j, v_j, w_j, rx_j, ry_j, rz_j]`.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if any coordinate is non-finite.
    /// [`FemError::ZeroLengthMember`] if the element has zero length.
    /// [`FemError::InvalidInput`] if the reference vector is parallel to the
    /// member axis.
    pub fn local_stiffness(
        &self,
        pi: (f64, f64, f64),
        pj: (f64, f64, f64),
    ) -> Result<[[f64; 12]; 12], FemError> {
        let L = self.length(pi, pj);
        if L <= 0.0 {
            if !pi.0.is_finite() || !pi.1.is_finite() || !pi.2.is_finite() {
                return Err(FemError::InvalidInput(format!(
                    "node i coordinates must be finite, got ({}, {}, {})",
                    pi.0, pi.1, pi.2
                )));
            }
            if !pj.0.is_finite() || !pj.1.is_finite() || !pj.2.is_finite() {
                return Err(FemError::InvalidInput(format!(
                    "node j coordinates must be finite, got ({}, {}, {})",
                    pj.0, pj.1, pj.2
                )));
            }
            return Err(FemError::ZeroLengthMember(
                "element has zero length".to_string(),
            ));
        }

        let E = self.section.E;
        let G = self.section.G;
        let A = self.section.area;
        let Iy = self.section.iy;
        let Iz = self.section.iz;
        let J = self.section.j;

        let EA_L = E * A / L;
        let GJ_L = G * J / L;

        let EIy_L3 = E * Iy / L.powi(3);
        let EIy_L2 = E * Iy / L.powi(2);
        let EIy_L = E * Iy / L;

        let EIz_L3 = E * Iz / L.powi(3);
        let EIz_L2 = E * Iz / L.powi(2);
        let EIz_L = E * Iz / L;

        let mut k = [[0.0f64; 12]; 12];

        // --- Axial (EA/L): DOFs 0, 6 ---
        k[0][0] = EA_L;
        k[0][6] = -EA_L;
        k[6][0] = -EA_L;
        k[6][6] = EA_L;

        // --- Torsion (GJ/L): DOFs 3, 9 ---
        k[3][3] = GJ_L;
        k[3][9] = -GJ_L;
        k[9][3] = -GJ_L;
        k[9][9] = GJ_L;

        // --- Bending about local z (Iz, x-y plane): DOFs 1, 5, 7, 11 ---
        // Same signs as 2D beam (theta_z = +dv/dx)
        k[1][1] = 12.0 * EIz_L3;
        k[1][5] = 6.0 * EIz_L2;
        k[1][7] = -12.0 * EIz_L3;
        k[1][11] = 6.0 * EIz_L2;

        k[5][1] = 6.0 * EIz_L2;
        k[5][5] = 4.0 * EIz_L;
        k[5][7] = -6.0 * EIz_L2;
        k[5][11] = 2.0 * EIz_L;

        k[7][1] = -12.0 * EIz_L3;
        k[7][5] = -6.0 * EIz_L2;
        k[7][7] = 12.0 * EIz_L3;
        k[7][11] = -6.0 * EIz_L2;

        k[11][1] = 6.0 * EIz_L2;
        k[11][5] = 2.0 * EIz_L;
        k[11][7] = -6.0 * EIz_L2;
        k[11][11] = 4.0 * EIz_L;

        // --- Bending about local y (Iy, x-z plane): DOFs 2, 4, 8, 10 ---
        // Opposite coupling signs (theta_y = -dw/dx by right-hand rule)
        k[2][2] = 12.0 * EIy_L3;
        k[2][4] = -6.0 * EIy_L2;
        k[2][8] = -12.0 * EIy_L3;
        k[2][10] = -6.0 * EIy_L2;

        k[4][2] = -6.0 * EIy_L2;
        k[4][4] = 4.0 * EIy_L;
        k[4][8] = 6.0 * EIy_L2;
        k[4][10] = 2.0 * EIy_L;

        k[8][2] = -12.0 * EIy_L3;
        k[8][4] = 6.0 * EIy_L2;
        k[8][8] = 12.0 * EIy_L3;
        k[8][10] = 6.0 * EIy_L2;

        k[10][2] = -6.0 * EIy_L2;
        k[10][4] = 2.0 * EIy_L;
        k[10][8] = 6.0 * EIy_L2;
        k[10][10] = 4.0 * EIy_L;

        Ok(k)
    }

    /// Compute the 12×12 coordinate transformation matrix `T`.
    ///
    /// Maps **global** element DOFs to **local** element DOFs:
    /// `u_local = T · u_global`.
    ///
    /// `T` is block-diagonal with four copies of the 3×3 direction cosine
    /// matrix `R` (rows = local axes in global coordinates). Translations
    /// and rotations transform identically (both are 3D vectors).
    ///
    /// # Errors
    ///
    /// Same as [`local_axes`](Self::local_axes).
    pub fn transformation_matrix(
        &self,
        pi: (f64, f64, f64),
        pj: (f64, f64, f64),
    ) -> Result<[[f64; 12]; 12], FemError> {
        let (x_hat, y_hat, z_hat) = self.local_axes(pi, pj)?;

        // Direction cosine matrix R: rows = local axes in global coordinates
        // R = [ x_hat ]
        //     [ y_hat ]
        //     [ z_hat ]
        let R = [
            [x_hat[0], x_hat[1], x_hat[2]],
            [y_hat[0], y_hat[1], y_hat[2]],
            [z_hat[0], z_hat[1], z_hat[2]],
        ];

        let mut T = [[0.0f64; 12]; 12];

        // Block-diagonal: four copies of R for (trans_i, rot_i, trans_j, rot_j)
        for block in 0..4 {
            let off = block * 3;
            for i in 0..3 {
                for j in 0..3 {
                    T[off + i][off + j] = R[i][j];
                }
            }
        }

        Ok(T)
    }

    /// Compute the 12×12 element stiffness matrix in **global** coordinates.
    ///
    /// `K_global = Tᵀ · K_local · T`, with the DOF order
    /// `[ux_i, uy_i, uz_i, rx_i, ry_i, rz_i, ux_j, uy_j, uz_j, rx_j, ry_j, rz_j]`
    /// (global axes).
    ///
    /// # Errors
    ///
    /// Same as [`local_stiffness`](Self::local_stiffness).
    pub fn global_stiffness(
        &self,
        pi: (f64, f64, f64),
        pj: (f64, f64, f64),
    ) -> Result<[[f64; 12]; 12], FemError> {
        let k_local = self.local_stiffness(pi, pj)?;
        let T = self.transformation_matrix(pi, pj)?;

        // K_global = T^T * K_local * T
        let mut k_global = [[0.0f64; 12]; 12];
        for i in 0..12 {
            for j in 0..12 {
                let mut sum = 0.0;
                for k in 0..12 {
                    for l in 0..12 {
                        sum += T[k][i] * k_local[k][l] * T[l][j];
                    }
                }
                k_global[i][j] = sum;
            }
        }

        Ok(k_global)
    }
}
