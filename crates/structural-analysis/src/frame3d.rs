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
//!
//! # API invariants and validation (Phase 136)
//!
//! All structural types (`FrameSection3D`, `FrameElement3D`, `FrameNode3D`,
//! `FrameModel3D`) encapsulate their fields as **private**. External code
//! reads values through getter methods (`E()`, `G()`, `area()`, …) and
//! modifies them through validated setters (`set_E()`, `set_G()`, …) that
//! enforce the same constraints as the constructor. This prevents
//! post-construction mutation to NaN, infinity, or non-positive values.
//!
//! ## Numerical robustness
//!
//! - `local_stiffness` validates coordinate finiteness **before** computing
//!   the element length, so NaN coordinates cannot bypass the `L > 0` check
//!   (because `NaN <= 0.0` is `false` in IEEE 754).
//! - Element length and axis norms use `f64::hypot` for improved numerical
//!   stability against intermediate overflow/underflow.
//! - Every entry of the local stiffness matrix is checked for finiteness
//!   before returning.
//!
//! ## Constraint semantics
//!
//! `fix_dof` detects duplicate constraints on the same DOF:
//! - Re-fixing the same DOF to the **same** value is a no-op (idempotent).
//! - Re-fixing to a **different** value returns [`FemError::InvalidInput`].
//!
//! ## Solver-boundary re-validation
//!
//! `FrameAnalysisResult3D::from_model` re-validates all model data at the
//! solver boundary: node coordinates, nodal-load indices and values, fixed-DOF
//! indices and values, section parameters, and member-load equivalent forces.
//! This provides defense-in-depth against any future code path that might
//! bypass the encapsulated validation.

#![allow(non_snake_case)]

use crate::beam_fem::FemError;
use section_properties::fea::{
    SparseMatrix,
    solver::{SolverRegistry, SolverSelection},
};

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
/// rejects zero, negative, NaN, or infinite values. Fields are private;
/// use the getter methods (`E()`, `G()`, `area()`, `iy()`, `iz()`, `j()`)
/// to read and the validated setters (`set_E()`, …) to modify.
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
    E: f64,
    G: f64,
    area: f64,
    iy: f64,
    iz: f64,
    j: f64,
}

impl FrameSection3D {
    /// Create a validated 3D frame section.
    ///
    /// # Errors
    ///
    /// Returns [`FemError::InvalidInput`] if any parameter is non-finite or
    /// non-positive.
    pub fn new(E: f64, G: f64, area: f64, iy: f64, iz: f64, j: f64) -> Result<Self, FemError> {
        Self::validate_params(E, G, area, iy, iz, j)?;
        Ok(Self {
            E,
            G,
            area,
            iy,
            iz,
            j,
        })
    }

    fn validate_params(
        E: f64,
        G: f64,
        area: f64,
        iy: f64,
        iz: f64,
        j: f64,
    ) -> Result<(), FemError> {
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
        Ok(())
    }

    /// Re-validate all parameters (solver-boundary defense).
    pub(crate) fn validate(&self) -> Result<(), FemError> {
        Self::validate_params(self.E, self.G, self.area, self.iy, self.iz, self.j)
    }

    /// Young's modulus `E`.
    pub fn E(&self) -> f64 {
        self.E
    }
    /// Shear modulus `G`.
    pub fn G(&self) -> f64 {
        self.G
    }
    /// Cross-sectional area `A`.
    pub fn area(&self) -> f64 {
        self.area
    }
    /// Second moment of area about local y `Iy`.
    pub fn iy(&self) -> f64 {
        self.iy
    }
    /// Second moment of area about local z `Iz`.
    pub fn iz(&self) -> f64 {
        self.iz
    }
    /// Saint-Venant torsional constant `J`.
    pub fn j(&self) -> f64 {
        self.j
    }

    /// Set Young's modulus with validation.
    pub fn set_E(&mut self, E: f64) -> Result<(), FemError> {
        Self::validate_params(E, self.G, self.area, self.iy, self.iz, self.j)?;
        self.E = E;
        Ok(())
    }
    /// Set shear modulus with validation.
    pub fn set_G(&mut self, G: f64) -> Result<(), FemError> {
        Self::validate_params(self.E, G, self.area, self.iy, self.iz, self.j)?;
        self.G = G;
        Ok(())
    }
    /// Set cross-sectional area with validation.
    pub fn set_area(&mut self, area: f64) -> Result<(), FemError> {
        Self::validate_params(self.E, self.G, area, self.iy, self.iz, self.j)?;
        self.area = area;
        Ok(())
    }
    /// Set `Iy` with validation.
    pub fn set_iy(&mut self, iy: f64) -> Result<(), FemError> {
        Self::validate_params(self.E, self.G, self.area, iy, self.iz, self.j)?;
        self.iy = iy;
        Ok(())
    }
    /// Set `Iz` with validation.
    pub fn set_iz(&mut self, iz: f64) -> Result<(), FemError> {
        Self::validate_params(self.E, self.G, self.area, self.iy, iz, self.j)?;
        self.iz = iz;
        Ok(())
    }
    /// Set `J` with validation.
    pub fn set_j(&mut self, j: f64) -> Result<(), FemError> {
        Self::validate_params(self.E, self.G, self.area, self.iy, self.iz, j)?;
        self.j = j;
        Ok(())
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
///
/// Fields are private; use getter methods (`node_i()`, `node_j()`,
/// `section()`, `ref_vec()`) to read.
#[derive(Debug, Clone)]
pub struct FrameElement3D {
    node_i: usize,
    node_j: usize,
    section: FrameSection3D,
    ref_vec: [f64; 3],
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
        let ref_norm = ref_vec[0].hypot(ref_vec[1]).hypot(ref_vec[2]);
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

    /// Start node index.
    pub fn node_i(&self) -> usize {
        self.node_i
    }
    /// End node index.
    pub fn node_j(&self) -> usize {
        self.node_j
    }
    /// Section parameters.
    pub fn section(&self) -> &FrameSection3D {
        &self.section
    }
    /// Reference vector (orientation).
    pub fn ref_vec(&self) -> [f64; 3] {
        self.ref_vec
    }

    /// Element length from node coordinates.
    pub fn length(&self, pi: (f64, f64, f64), pj: (f64, f64, f64)) -> f64 {
        let dx = pj.0 - pi.0;
        let dy = pj.1 - pi.1;
        let dz = pj.2 - pi.2;
        dx.hypot(dy).hypot(dz)
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
        let cross_norm = cross[0].hypot(cross[1]).hypot(cross[2]);

        let ref_norm = self.ref_vec[0]
            .hypot(self.ref_vec[1])
            .hypot(self.ref_vec[2]);

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

        for i in 0..12 {
            for j in 0..12 {
                if !k[i][j].is_finite() {
                    return Err(FemError::InvalidInput(format!(
                        "stiffness matrix entry [{i}][{j}] is non-finite: {} \
                         (E={E}, G={G}, A={A}, Iy={Iy}, Iz={Iz}, J={J}, L={L})",
                        k[i][j]
                    )));
                }
            }
        }

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

// ---------------------------------------------------------------------------
// DOF enum
// ---------------------------------------------------------------------------

/// A degree of freedom of a 3D frame node.
///
/// 3D frame nodes have 6 DOF: `Ux`, `Uy`, `Uz` (translations) and `Rx`,
/// `Ry`, `Rz` (rotations).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Dof3D {
    /// Translation along global X.
    Ux,
    /// Translation along global Y.
    Uy,
    /// Translation along global Z.
    Uz,
    /// Rotation about global X.
    Rx,
    /// Rotation about global Y.
    Ry,
    /// Rotation about global Z.
    Rz,
}

impl Dof3D {
    /// All DOFs in canonical order `[Ux, Uy, Uz, Rx, Ry, Rz]`.
    pub const ALL: [Dof3D; 6] = [
        Dof3D::Ux,
        Dof3D::Uy,
        Dof3D::Uz,
        Dof3D::Rx,
        Dof3D::Ry,
        Dof3D::Rz,
    ];

    /// Raw index: `Ux → 0`, `Uy → 1`, …, `Rz → 5`.
    pub const fn index(self) -> usize {
        match self {
            Dof3D::Ux => 0,
            Dof3D::Uy => 1,
            Dof3D::Uz => 2,
            Dof3D::Rx => 3,
            Dof3D::Ry => 4,
            Dof3D::Rz => 5,
        }
    }

    /// Human-readable name (`"ux"`, …, `"rz"`).
    pub const fn name(self) -> &'static str {
        match self {
            Dof3D::Ux => "ux",
            Dof3D::Uy => "uy",
            Dof3D::Uz => "uz",
            Dof3D::Rx => "rx",
            Dof3D::Ry => "ry",
            Dof3D::Rz => "rz",
        }
    }
}

// ---------------------------------------------------------------------------
// Member loads
// ---------------------------------------------------------------------------

/// A distributed member load for a 3D frame element.
///
/// All intensities are in **local** member coordinates. Positive intensity
/// follows the positive local axis direction.
///
/// # Sign conventions
///
/// - **Axial** (`q_x`): positive in local +x.
/// - **Transverse y** (`q_y`): positive in local +y. Bending about local z
///   with `θ_z = +dv/dx`. Consistent moments: `+qL²/12` at i, `−qL²/12` at j.
/// - **Transverse z** (`q_z`): positive in local +z. Bending about local y
///   with `θ_y = −dw/dx`. Consistent moments: `−qL²/12` at i, `+qL²/12` at j
///   (signs flipped relative to `q_y` due to the right-hand rule).
///
/// For linearly varying loads, `q_i` is the intensity at node i (x=0) and
/// `q_j` at node j (x=L).
#[derive(Debug, Clone, Copy)]
pub enum FrameMemberLoad {
    /// Uniform distributed force along local x (axial). `q_x` in N/m.
    UniformAxial {
        /// Member index.
        member_idx: usize,
        /// Force per unit length (N/m), positive in local +x.
        q_x: f64,
    },
    /// Uniform distributed force along local y. `q_y` in N/m.
    UniformY {
        /// Member index.
        member_idx: usize,
        /// Force per unit length (N/m), positive in local +y.
        q_y: f64,
    },
    /// Uniform distributed force along local z. `q_z` in N/m.
    UniformZ {
        /// Member index.
        member_idx: usize,
        /// Force per unit length (N/m), positive in local +z.
        q_z: f64,
    },
    /// Linearly varying distributed force along local y.
    /// `q_i` at node i (x=0), `q_j` at node j (x=L). N/m.
    LinearY {
        /// Member index.
        member_idx: usize,
        /// Intensity at node i (N/m).
        q_i: f64,
        /// Intensity at node j (N/m).
        q_j: f64,
    },
    /// Linearly varying distributed force along local z.
    /// `q_i` at node i (x=0), `q_j` at node j (x=L). N/m.
    LinearZ {
        /// Member index.
        member_idx: usize,
        /// Intensity at node i (N/m).
        q_i: f64,
        /// Intensity at node j (N/m).
        q_j: f64,
    },
}

impl FrameMemberLoad {
    /// Return the member index referenced by this load.
    pub fn member_idx(&self) -> usize {
        match *self {
            FrameMemberLoad::UniformAxial { member_idx, .. }
            | FrameMemberLoad::UniformY { member_idx, .. }
            | FrameMemberLoad::UniformZ { member_idx, .. }
            | FrameMemberLoad::LinearY { member_idx, .. }
            | FrameMemberLoad::LinearZ { member_idx, .. } => member_idx,
        }
    }

    /// Compute the 12-component equivalent nodal load vector in **local**
    /// coordinates.
    ///
    /// Derived from the Euler–Bernoulli consistent load formulation
    /// (shape-function integration). For uniform loads the familiar
    /// `qL/2` force and `qL²/12` moment coefficients appear. For linearly
    /// varying loads the exact consistent vector is used (no averaging).
    ///
    /// # Bending sign conventions
    ///
    /// - **Bending about z** (local y load): `θ_z = +dv/dx`, moments
    ///   `+qL²/12` at i, `−qL²/12` at j.
    /// - **Bending about y** (local z load): `θ_y = −dw/dx`, moments
    ///   `−qL²/12` at i, `+qL²/12` at j.
    pub fn equivalent_nodal_loads_local(&self, L: f64) -> [f64; 12] {
        let mut f = [0.0f64; 12];
        match *self {
            FrameMemberLoad::UniformAxial { q_x, .. } => {
                f[0] = q_x * L / 2.0;
                f[6] = q_x * L / 2.0;
            }
            FrameMemberLoad::UniformY { q_y, .. } => {
                f[1] = q_y * L / 2.0;
                f[5] = q_y * L * L / 12.0;
                f[7] = q_y * L / 2.0;
                f[11] = -q_y * L * L / 12.0;
            }
            FrameMemberLoad::UniformZ { q_z, .. } => {
                f[2] = q_z * L / 2.0;
                f[4] = -q_z * L * L / 12.0;
                f[8] = q_z * L / 2.0;
                f[10] = q_z * L * L / 12.0;
            }
            FrameMemberLoad::LinearY { q_i, q_j, .. } => {
                f[1] = L * (7.0 * q_i + 3.0 * q_j) / 20.0;
                f[5] = L * L * (3.0 * q_i + 2.0 * q_j) / 60.0;
                f[7] = L * (3.0 * q_i + 7.0 * q_j) / 20.0;
                f[11] = -L * L * (2.0 * q_i + 3.0 * q_j) / 60.0;
            }
            FrameMemberLoad::LinearZ { q_i, q_j, .. } => {
                f[2] = L * (7.0 * q_i + 3.0 * q_j) / 20.0;
                f[4] = -L * L * (3.0 * q_i + 2.0 * q_j) / 60.0;
                f[8] = L * (3.0 * q_i + 7.0 * q_j) / 20.0;
                f[10] = L * L * (2.0 * q_i + 3.0 * q_j) / 60.0;
            }
        }
        f
    }
}

// ---------------------------------------------------------------------------
// Node
// ---------------------------------------------------------------------------

/// 3D frame node with 6 DOF: `[ux, uy, uz, rx, ry, rz]`.
///
/// Fields are private; use getter methods (`id()`, `x()`, `y()`, `z()`)
/// to read.
#[derive(Debug, Clone, Copy)]
pub struct FrameNode3D {
    id: usize,
    x: f64,
    y: f64,
    z: f64,
}

impl FrameNode3D {
    /// Create a node at `(x, y, z)` with the given `id`.
    pub fn new(id: usize, x: f64, y: f64, z: f64) -> Self {
        Self { id, x, y, z }
    }

    /// Node index.
    pub fn id(&self) -> usize {
        self.id
    }
    /// X coordinate.
    pub fn x(&self) -> f64 {
        self.x
    }
    /// Y coordinate.
    pub fn y(&self) -> f64 {
        self.y
    }
    /// Z coordinate.
    pub fn z(&self) -> f64 {
        self.z
    }
    /// Return coordinates as `(x, y, z)`.
    pub fn coords(&self) -> (f64, f64, f64) {
        (self.x, self.y, self.z)
    }
}

// ---------------------------------------------------------------------------
// Model
// ---------------------------------------------------------------------------

/// 3D frame model: nodes, members, nodal loads, and support conditions.
///
/// # Global DOF mapping
///
/// Each node has 6 **global** DOFs `[ux, uy, uz, rx, ry, rz]`.
/// DOF index: `dof(node, d) = 6·node + d`.
///
/// # Conventions
///
/// - Nodal loads are in **global** coordinates.
/// - Prescribed DOFs are enforced by static condensation (no penalty).
/// - Multiple loads at the same node/DOF **accumulate**.
/// - `fix_dof` is **idempotent** for the same value; re-fixing the same DOF
///   to a different value returns [`FemError::InvalidInput`].
///
/// Fields are private; use accessor methods (`n_nodes()`, `n_members()`,
/// `node()`, `member()`, …) to read.
///
/// # Snapshot semantics
///
/// [`FrameSolver3D::from_model`] clones the model. Mutating a `FrameModel3D`
/// after building a solver does not affect that solver.
#[derive(Debug, Clone)]
pub struct FrameModel3D {
    nodes: Vec<FrameNode3D>,
    members: Vec<FrameElement3D>,
    nodal_forces: Vec<(usize, usize, f64)>,
    fixed_dofs: Vec<(usize, usize, f64)>,
    member_loads: Vec<FrameMemberLoad>,
}

impl Default for FrameModel3D {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameModel3D {
    /// Create an empty 3D frame model.
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            members: Vec::new(),
            nodal_forces: Vec::new(),
            fixed_dofs: Vec::new(),
            member_loads: Vec::new(),
        }
    }

    /// Number of nodes.
    pub fn n_nodes(&self) -> usize {
        self.nodes.len()
    }
    /// Number of members.
    pub fn n_members(&self) -> usize {
        self.members.len()
    }
    /// Borrow node at `idx`, or `None` if out of bounds.
    pub fn node(&self, idx: usize) -> Option<&FrameNode3D> {
        self.nodes.get(idx)
    }
    /// Borrow member at `idx`, or `None` if out of bounds.
    pub fn member(&self, idx: usize) -> Option<&FrameElement3D> {
        self.members.get(idx)
    }

    /// Add a node at `(x, y, z)` and return its index.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if any coordinate is non-finite.
    pub fn add_node(&mut self, x: f64, y: f64, z: f64) -> Result<usize, FemError> {
        if !x.is_finite() || !y.is_finite() || !z.is_finite() {
            return Err(FemError::InvalidInput(format!(
                "node coordinates must be finite, got ({x}, {y}, {z})"
            )));
        }
        let id = self.nodes.len();
        self.nodes.push(FrameNode3D::new(id, x, y, z));
        Ok(id)
    }

    /// Add a member between `node_i` and `node_j` with the given section and
    /// orientation reference vector. Returns the member index.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if `node_i` or `node_j` is out of bounds.
    /// [`FemError::InvalidModel`] if `node_i == node_j`.
    /// [`FemError::InvalidInput`] if `ref_vec` is non-finite or zero.
    pub fn add_member(
        &mut self,
        node_i: usize,
        node_j: usize,
        section: FrameSection3D,
        ref_vec: [f64; 3],
    ) -> Result<usize, FemError> {
        let n = self.nodes.len();
        if node_i >= n {
            return Err(FemError::InvalidNode(format!(
                "node_i {node_i} out of bounds (max {})",
                n.saturating_sub(1)
            )));
        }
        if node_j >= n {
            return Err(FemError::InvalidNode(format!(
                "node_j {node_j} out of bounds (max {})",
                n.saturating_sub(1)
            )));
        }
        let element = FrameElement3D::new(node_i, node_j, section, ref_vec)?;
        let idx = self.members.len();
        self.members.push(element);
        Ok(idx)
    }

    /// Apply a nodal force `(fx, fy, fz)` and moments `(mx, my, mz)` at
    /// `node_idx` in **global** coordinates.
    ///
    /// Multiple calls accumulate.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if `node_idx` is out of bounds.
    /// [`FemError::InvalidInput`] if any component is non-finite.
    #[allow(clippy::too_many_arguments)]
    pub fn add_nodal_load(
        &mut self,
        node_idx: usize,
        fx: f64,
        fy: f64,
        fz: f64,
        mx: f64,
        my: f64,
        mz: f64,
    ) -> Result<(), FemError> {
        if node_idx >= self.nodes.len() {
            return Err(FemError::InvalidNode(format!(
                "node index {node_idx} out of bounds (max {})",
                self.nodes.len().saturating_sub(1)
            )));
        }
        for (label, val) in [
            ("fx", fx),
            ("fy", fy),
            ("fz", fz),
            ("mx", mx),
            ("my", my),
            ("mz", mz),
        ] {
            if !val.is_finite() {
                return Err(FemError::InvalidInput(format!(
                    "nodal load {label} must be finite, got {val}"
                )));
            }
        }
        self.nodal_forces.push((node_idx, 0, fx));
        self.nodal_forces.push((node_idx, 1, fy));
        self.nodal_forces.push((node_idx, 2, fz));
        self.nodal_forces.push((node_idx, 3, mx));
        self.nodal_forces.push((node_idx, 4, my));
        self.nodal_forces.push((node_idx, 5, mz));
        Ok(())
    }

    /// Restrain a single DOF of a node to a prescribed `value` (usually 0.0).
    ///
    /// # Duplicate constraints
    ///
    /// If the same `(node_idx, dof)` is constrained again with the **same**
    /// value, the call is a no-op (idempotent). If the value **differs**, an
    /// [`FemError::InvalidInput`] is returned to prevent contradictory
    /// constraints.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if `node_idx` is out of bounds.
    /// [`FemError::InvalidInput`] if `value` is non-finite or conflicts with
    /// an existing constraint on the same DOF.
    pub fn fix_dof(&mut self, node_idx: usize, dof: Dof3D, value: f64) -> Result<(), FemError> {
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
        let dof_idx = dof.index();
        for &(n, d, v) in &self.fixed_dofs {
            if n == node_idx && d == dof_idx {
                if v == value {
                    return Ok(());
                }
                return Err(FemError::InvalidInput(format!(
                    "conflicting constraint at node {node_idx}, dof {}: \
                     existing value {v}, new value {value}",
                    dof.name()
                )));
            }
        }
        self.fixed_dofs.push((node_idx, dof_idx, value));
        Ok(())
    }

    /// Fix all 6 DOFs of a node to zero (fully built-in support).
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if `node_idx` is out of bounds.
    pub fn fix_node(&mut self, node_idx: usize) -> Result<(), FemError> {
        for dof in Dof3D::ALL {
            self.fix_dof(node_idx, dof, 0.0)?;
        }
        Ok(())
    }

    /// Pin a node: restrain all 3 translations to zero, rotations free.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if `node_idx` is out of bounds.
    pub fn pin_node(&mut self, node_idx: usize) -> Result<(), FemError> {
        self.fix_dof(node_idx, Dof3D::Ux, 0.0)?;
        self.fix_dof(node_idx, Dof3D::Uy, 0.0)?;
        self.fix_dof(node_idx, Dof3D::Uz, 0.0)
    }

    // --- Member loads -------------------------------------------------------

    /// Add a uniform distributed axial force (local x) to a member.
    ///
    /// `q_x` is in N/m, positive in local +x. Multiple calls accumulate.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidMember`] if `member_idx` is out of bounds.
    /// [`FemError::InvalidInput`] if `q_x` is non-finite.
    pub fn add_uniform_axial(&mut self, member_idx: usize, q_x: f64) -> Result<(), FemError> {
        self.validate_member_load(member_idx, &[q_x])?;
        self.member_loads
            .push(FrameMemberLoad::UniformAxial { member_idx, q_x });
        Ok(())
    }

    /// Add a uniform distributed transverse force (local y) to a member.
    ///
    /// `q_y` is in N/m, positive in local +y. Multiple calls accumulate.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidMember`] if `member_idx` is out of bounds.
    /// [`FemError::InvalidInput`] if `q_y` is non-finite.
    pub fn add_uniform_y(&mut self, member_idx: usize, q_y: f64) -> Result<(), FemError> {
        self.validate_member_load(member_idx, &[q_y])?;
        self.member_loads
            .push(FrameMemberLoad::UniformY { member_idx, q_y });
        Ok(())
    }

    /// Add a uniform distributed transverse force (local z) to a member.
    ///
    /// `q_z` is in N/m, positive in local +z. Multiple calls accumulate.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidMember`] if `member_idx` is out of bounds.
    /// [`FemError::InvalidInput`] if `q_z` is non-finite.
    pub fn add_uniform_z(&mut self, member_idx: usize, q_z: f64) -> Result<(), FemError> {
        self.validate_member_load(member_idx, &[q_z])?;
        self.member_loads
            .push(FrameMemberLoad::UniformZ { member_idx, q_z });
        Ok(())
    }

    /// Add a linearly varying distributed transverse force (local y).
    ///
    /// `q_i` at node i (x=0), `q_j` at node j (x=L), in N/m. Multiple calls
    /// accumulate.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidMember`] if `member_idx` is out of bounds.
    /// [`FemError::InvalidInput`] if any intensity is non-finite.
    pub fn add_linear_y(&mut self, member_idx: usize, q_i: f64, q_j: f64) -> Result<(), FemError> {
        self.validate_member_load(member_idx, &[q_i, q_j])?;
        self.member_loads.push(FrameMemberLoad::LinearY {
            member_idx,
            q_i,
            q_j,
        });
        Ok(())
    }

    /// Add a linearly varying distributed transverse force (local z).
    ///
    /// `q_i` at node i (x=0), `q_j` at node j (x=L), in N/m. Multiple calls
    /// accumulate.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidMember`] if `member_idx` is out of bounds.
    /// [`FemError::InvalidInput`] if any intensity is non-finite.
    pub fn add_linear_z(&mut self, member_idx: usize, q_i: f64, q_j: f64) -> Result<(), FemError> {
        self.validate_member_load(member_idx, &[q_i, q_j])?;
        self.member_loads.push(FrameMemberLoad::LinearZ {
            member_idx,
            q_i,
            q_j,
        });
        Ok(())
    }

    fn validate_member_load(&self, member_idx: usize, intensities: &[f64]) -> Result<(), FemError> {
        if member_idx >= self.members.len() {
            return Err(FemError::InvalidMember(format!(
                "member index {member_idx} out of bounds (max {})",
                self.members.len().saturating_sub(1)
            )));
        }
        for &q in intensities {
            if !q.is_finite() {
                return Err(FemError::InvalidInput(format!(
                    "load intensity must be finite, got {q}"
                )));
            }
        }
        Ok(())
    }

    /// Total number of DOFs.
    pub fn n_dof(&self) -> usize {
        self.nodes.len() * 6
    }

    /// Global DOF index: `6·node + dof`.
    pub(crate) fn dof_index(&self, node_idx: usize, dof: usize) -> usize {
        node_idx * 6 + dof
    }

    /// Solve the 3D frame model and return an owned analysis result.
    ///
    /// This is a convenience method equivalent to:
    /// ```text
    /// FrameSolver3D::from_model(&model)?.solve()
    /// ```
    ///
    /// # Errors
    ///
    /// See [`FrameSolver3D::from_model`] and [`FrameSolver3D::solve`].
    pub fn solve(&self) -> Result<FrameAnalysisResult3D, FemError> {
        FrameSolver3D::from_model(self)?.solve()
    }
}

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

/// 3D frame solver: assembles the global system, applies boundary conditions,
/// and delegates the linear solve to a `LinearSolver` backend.
///
/// The solver is immutable after construction. [`solve`](Self::solve) returns
/// an owned [`FrameAnalysisResult3D`] snapshot.
pub struct FrameSolver3D {
    k_global: SparseMatrix,
    f_global: Vec<f64>,
    fixed_dofs: Vec<bool>,
    prescribed_values: Vec<Option<f64>>,
    k_original: SparseMatrix,
    n_dof: usize,
    model: FrameModel3D,
    solver_selection: SolverSelection,
    member_equiv_loads: Vec<[f64; 12]>,
}

impl FrameSolver3D {
    /// Create a solver from a [`FrameModel3D`].
    ///
    /// Clones the model, assembles the global stiffness matrix, and applies
    /// boundary conditions.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidModel`] if the model has no nodes, or if any member
    /// references an out-of-bounds node or has zero length / invalid
    /// orientation.
    pub fn from_model(model: &FrameModel3D) -> Result<Self, FemError> {
        let n_dof = model.n_dof();
        if n_dof == 0 {
            return Err(FemError::InvalidModel("model has no nodes".to_string()));
        }

        // Solver-boundary re-validation: node coordinates
        for (idx, n) in model.nodes.iter().enumerate() {
            if !n.x.is_finite() || !n.y.is_finite() || !n.z.is_finite() {
                return Err(FemError::InvalidInput(format!(
                    "node {idx} has non-finite coordinates ({}, {}, {})",
                    n.x, n.y, n.z
                )));
            }
        }

        // Solver-boundary re-validation: nodal loads
        for &(node_idx, dof, value) in &model.nodal_forces {
            if node_idx >= model.nodes.len() {
                return Err(FemError::InvalidNode(format!(
                    "nodal load references node {node_idx} out of bounds (max {})",
                    model.nodes.len().saturating_sub(1)
                )));
            }
            if !value.is_finite() {
                return Err(FemError::InvalidInput(format!(
                    "nodal load at node {node_idx}, dof {dof} is non-finite: {value}"
                )));
            }
        }

        // Solver-boundary re-validation: fixed DOFs
        for &(node_idx, dof, value) in &model.fixed_dofs {
            if node_idx >= model.nodes.len() {
                return Err(FemError::InvalidNode(format!(
                    "fixed dof references node {node_idx} out of bounds (max {})",
                    model.nodes.len().saturating_sub(1)
                )));
            }
            if !value.is_finite() {
                return Err(FemError::InvalidInput(format!(
                    "fixed dof at node {node_idx}, dof {dof} is non-finite: {value}"
                )));
            }
        }

        // Validate members (including solver-boundary section re-validation)
        for (idx, m) in model.members.iter().enumerate() {
            if m.node_i >= model.nodes.len() {
                return Err(FemError::InvalidModel(format!(
                    "member {idx}: node_i {} out of bounds (max {})",
                    m.node_i,
                    model.nodes.len().saturating_sub(1)
                )));
            }
            if m.node_j >= model.nodes.len() {
                return Err(FemError::InvalidModel(format!(
                    "member {idx}: node_j {} out of bounds (max {})",
                    m.node_j,
                    model.nodes.len().saturating_sub(1)
                )));
            }
            m.section.validate()?;
            let pi = model.nodes[m.node_i].coords();
            let pj = model.nodes[m.node_j].coords();
            m.local_axes(pi, pj)?;
        }

        // Assemble global stiffness
        let mut k_global = SparseMatrix::new(n_dof);
        for m in &model.members {
            let pi = model.nodes[m.node_i].coords();
            let pj = model.nodes[m.node_j].coords();
            let k_e = m.global_stiffness(pi, pj)?;
            let dof_map = [
                model.dof_index(m.node_i, 0),
                model.dof_index(m.node_i, 1),
                model.dof_index(m.node_i, 2),
                model.dof_index(m.node_i, 3),
                model.dof_index(m.node_i, 4),
                model.dof_index(m.node_i, 5),
                model.dof_index(m.node_j, 0),
                model.dof_index(m.node_j, 1),
                model.dof_index(m.node_j, 2),
                model.dof_index(m.node_j, 3),
                model.dof_index(m.node_j, 4),
                model.dof_index(m.node_j, 5),
            ];
            for a in 0..12 {
                for b in 0..12 {
                    if k_e[a][b] != 0.0 {
                        k_global.add(dof_map[a], dof_map[b], k_e[a][b]);
                    }
                }
            }
        }
        k_global.compress();

        // Assemble global load vector
        let mut f_global = vec![0.0; n_dof];
        for &(node_idx, dof, value) in &model.nodal_forces {
            let idx = model.dof_index(node_idx, dof);
            f_global[idx] += value;
        }

        // Compute equivalent nodal loads for member loads and add to global RHS.
        // Also store per-member local equivalent loads for end-force recovery.
        let mut member_equiv_loads = vec![[0.0f64; 12]; model.members.len()];
        for load in &model.member_loads {
            let mid = load.member_idx();
            if mid >= model.members.len() {
                return Err(FemError::InvalidMember(format!(
                    "member load references member {mid} out of bounds (max {})",
                    model.members.len().saturating_sub(1)
                )));
            }
            let m = &model.members[mid];
            let pi = model.nodes[m.node_i].coords();
            let pj = model.nodes[m.node_j].coords();
            let L = m.length(pi, pj);
            let f_eq_local = load.equivalent_nodal_loads_local(L);
            for i in 0..12 {
                if !f_eq_local[i].is_finite() {
                    return Err(FemError::InvalidInput(format!(
                        "member load on member {mid} produced non-finite \
                         equivalent load at DOF {i}: {}",
                        f_eq_local[i]
                    )));
                }
            }

            // Accumulate local equivalent loads for end-force recovery
            for i in 0..12 {
                member_equiv_loads[mid][i] += f_eq_local[i];
            }

            // Transform to global: f_eq_global = Tᵀ · f_eq_local, then scatter
            let T = m.transformation_matrix(pi, pj)?;
            let dof_map = [
                model.dof_index(m.node_i, 0),
                model.dof_index(m.node_i, 1),
                model.dof_index(m.node_i, 2),
                model.dof_index(m.node_i, 3),
                model.dof_index(m.node_i, 4),
                model.dof_index(m.node_i, 5),
                model.dof_index(m.node_j, 0),
                model.dof_index(m.node_j, 1),
                model.dof_index(m.node_j, 2),
                model.dof_index(m.node_j, 3),
                model.dof_index(m.node_j, 4),
                model.dof_index(m.node_j, 5),
            ];
            for a in 0..12 {
                let mut f_global_a = 0.0;
                for b in 0..12 {
                    f_global_a += T[b][a] * f_eq_local[b];
                }
                f_global[dof_map[a]] += f_global_a;
            }
        }

        // Apply boundary conditions
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
            fixed_dofs,
            prescribed_values,
            k_original,
            n_dof,
            model: model.clone(),
            solver_selection: SolverSelection::Auto,
            member_equiv_loads,
        })
    }

    /// Configure the solver backend.
    pub fn set_solver(&mut self, selection: SolverSelection) {
        self.solver_selection = selection;
    }

    /// Solve the 3D frame and return an owned analysis result.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidModel`] if all DOFs are constrained.
    /// [`FemError::SolverError`] if the reduced stiffness matrix is singular
    /// or the linear solve fails.
    pub fn solve(&self) -> Result<FrameAnalysisResult3D, FemError> {
        let n_dof = self.n_dof;

        // Partition free / constrained DOFs
        let mut free_to_global = Vec::new();
        let mut global_to_free = vec![None; n_dof];
        for (i, &fixed) in self.fixed_dofs.iter().enumerate().take(n_dof) {
            if !fixed {
                global_to_free[i] = Some(free_to_global.len());
                free_to_global.push(i);
            }
        }
        let n_free = free_to_global.len();

        // Assemble global load vector (already stored)
        let f_global = self.f_global.clone();

        // Build full displacement vector
        let mut u_global = vec![0.0; n_dof];

        if n_free == 0 {
            // All constrained — just use prescribed values
            for (i, u_slot) in u_global.iter_mut().enumerate().take(n_dof) {
                if self.fixed_dofs[i] {
                    *u_slot = self.prescribed_values[i].unwrap_or(0.0);
                }
            }
        } else {
            // Build reduced system: K_ff * u_f = F_f - K_fc * u_c
            let mut k_orig_csr = self.k_global.clone();
            k_orig_csr.compress();
            let row_ptr = k_orig_csr.row_ptr();
            let csr_cols = k_orig_csr.csr_cols();
            let csr_vals = k_orig_csr.csr_vals();

            let mut k_ff = SparseMatrix::new(n_free);
            let mut f_reduced = vec![0.0; n_free];

            for (free_idx, &global_i) in free_to_global.iter().enumerate() {
                f_reduced[free_idx] = f_global[global_i];
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

            // Solve
            let registry = SolverRegistry::default();
            let mut solver = registry
                .create_selected(&k_ff, &self.solver_selection)
                .map_err(FemError::from)?;
            let solver_name = solver.name().to_string();

            solver.factor(&k_ff)?;
            let u_free = solver.solve(&f_reduced)?;

            // Check for non-finite solution
            for &v in &u_free {
                if !v.is_finite() {
                    return Err(FemError::SolverError {
                        source: section_properties::fea::solver::SolverError::SingularMatrix(
                            "solution contains non-finite values".to_string(),
                        ),
                        message: "linear solve produced NaN or infinity".to_string(),
                    });
                }
            }

            // Reconstruct full displacement vector
            for (free_idx, &global_idx) in free_to_global.iter().enumerate() {
                u_global[global_idx] = u_free[free_idx];
            }
            for (i, u_slot) in u_global.iter_mut().enumerate().take(n_dof) {
                if self.fixed_dofs[i] {
                    *u_slot = self.prescribed_values[i].unwrap_or(0.0);
                }
            }

            let _ = solver_name; // used in result below
        }

        // Recover reactions: R = K·u - f
        let ku = self.k_original.matvec(&u_global);
        let reactions: Vec<f64> = (0..n_dof).map(|i| ku[i] - f_global[i]).collect();

        // Recover member end forces (local coordinates)
        let mut member_end_forces = Vec::with_capacity(self.model.members.len());
        for (member_idx, m) in self.model.members.iter().enumerate() {
            let pi = self.model.nodes[m.node_i].coords();
            let pj = self.model.nodes[m.node_j].coords();
            let k_local = m.local_stiffness(pi, pj)?;
            let T = m.transformation_matrix(pi, pj)?;

            // Extract 12 global DOFs for this member
            let u_global_e: [f64; 12] = [
                u_global[self.model.dof_index(m.node_i, 0)],
                u_global[self.model.dof_index(m.node_i, 1)],
                u_global[self.model.dof_index(m.node_i, 2)],
                u_global[self.model.dof_index(m.node_i, 3)],
                u_global[self.model.dof_index(m.node_i, 4)],
                u_global[self.model.dof_index(m.node_i, 5)],
                u_global[self.model.dof_index(m.node_j, 0)],
                u_global[self.model.dof_index(m.node_j, 1)],
                u_global[self.model.dof_index(m.node_j, 2)],
                u_global[self.model.dof_index(m.node_j, 3)],
                u_global[self.model.dof_index(m.node_j, 4)],
                u_global[self.model.dof_index(m.node_j, 5)],
            ];

            // Transform to local: u_local = T · u_global
            let mut u_local = [0.0f64; 12];
            for i in 0..12 {
                for j in 0..12 {
                    u_local[i] += T[i][j] * u_global_e[j];
                }
            }

            // Local end forces: f_local = K_local · u_local − f_eq_local
            // The equivalent load correction accounts for distributed member
            // loads. Without member loads f_eq_local = 0 and this reduces to
            // the Phase 134 formula.
            let mut f_local = [0.0f64; 12];
            for i in 0..12 {
                for j in 0..12 {
                    f_local[i] += k_local[i][j] * u_local[j];
                }
                f_local[i] -= self.member_equiv_loads[member_idx][i];
            }

            member_end_forces.push(f_local);
        }

        // Constrained DOFs as (node_idx, dof)
        let constrained_dofs: Vec<(usize, usize)> = self
            .fixed_dofs
            .iter()
            .enumerate()
            .filter(|&(_, &fixed)| fixed)
            .map(|(i, _)| (i / 6, i % 6))
            .collect();

        let node_coords: Vec<(f64, f64, f64)> =
            self.model.nodes.iter().map(|n| n.coords()).collect();
        let member_nodes: Vec<(usize, usize)> = self
            .model
            .members
            .iter()
            .map(|m| (m.node_i, m.node_j))
            .collect();

        Ok(FrameAnalysisResult3D {
            displacements: u_global,
            reactions,
            member_end_forces,
            node_coords,
            member_nodes,
            constrained_dofs,
            n_nodes: self.model.nodes.len(),
            n_members: self.model.members.len(),
        })
    }

    /// Borrow the underlying model.
    pub fn model(&self) -> &FrameModel3D {
        &self.model
    }
}

// ---------------------------------------------------------------------------
// Analysis result
// ---------------------------------------------------------------------------

/// Result of a 3D frame analysis: displacements, reactions, and member end
/// forces.
///
/// This is an **owned snapshot** — it does not borrow the solver or model.
///
/// # Displacements
///
/// Full vector `[ux, uy, uz, rx, ry, rz]` per node, in global coordinates.
/// Access via [`displacement`](Self::displacement) with a [`Dof3D`].
///
/// # Reactions
///
/// `R = K·u − F` (global), laid out `[Rx, Ry, Rz, Mx, My, Mz]` per node.
/// Reactions at **free** DOFs are zero by construction (equilibrium).
///
/// # Member end forces
///
/// 12 values per member in **local** coordinates:
/// `[fx_i, fy_i, fz_i, mx_i, my_i, mz_i, fx_j, fy_j, fz_j, mx_j, my_j, mz_j]`.
///
/// These are the forces that the nodes exert **on the member** (element end
/// actions). At a constrained node, they match the support reactions
/// (transformed to local axes). At a!loaded node, they match the applied
/// loads (transformed to local axes).
///
/// Computed as `f_local = K_local · T · u_global − f_eq_local`, where
/// `f_eq_local` is the local equivalent nodal load vector from distributed
/// member loads (zero if no member loads are applied).
#[derive(Debug, Clone)]
pub struct FrameAnalysisResult3D {
    /// Full displacement vector `[ux, uy, uz, rx, ry, rz]` per node.
    pub displacements: Vec<f64>,
    /// Global reactions `[Rx, Ry, Rz, Mx, My, Mz]` per node.
    pub reactions: Vec<f64>,
    /// Member end forces (12 per member, local coordinates).
    pub member_end_forces: Vec<[f64; 12]>,
    /// Node coordinates snapshot `(x, y, z)`.
    pub node_coords: Vec<(f64, f64, f64)>,
    /// Member node connectivity `(node_i, node_j)`.
    pub member_nodes: Vec<(usize, usize)>,
    /// Constrained DOFs as `(node_idx, dof)`.
    pub constrained_dofs: Vec<(usize, usize)>,
    /// Number of nodes.
    pub n_nodes: usize,
    /// Number of members.
    pub n_members: usize,
}

impl FrameAnalysisResult3D {
    /// Displacement of a single DOF at `node_idx`.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if `node_idx` is out of bounds.
    pub fn displacement(&self, node_idx: usize, dof: Dof3D) -> Result<f64, FemError> {
        if node_idx >= self.n_nodes {
            return Err(FemError::InvalidNode(format!(
                "node index {node_idx} out of bounds (max {})",
                self.n_nodes.saturating_sub(1)
            )));
        }
        Ok(self.displacements[node_idx * 6 + dof.index()])
    }

    /// Reaction at a single DOF at `node_idx`.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if `node_idx` is out of bounds.
    pub fn reaction(&self, node_idx: usize, dof: Dof3D) -> Result<f64, FemError> {
        if node_idx >= self.n_nodes {
            return Err(FemError::InvalidNode(format!(
                "node index {node_idx} out of bounds (max {})",
                self.n_nodes.saturating_sub(1)
            )));
        }
        Ok(self.reactions[node_idx * 6 + dof.index()])
    }

    /// Member end forces (12 local values) for `member_idx`.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidMember`] if `member_idx` is out of bounds.
    pub fn member_end_forces(&self, member_idx: usize) -> Result<&[f64; 12], FemError> {
        if member_idx >= self.n_members {
            return Err(FemError::InvalidMember(format!(
                "member index {member_idx} out of bounds (max {})",
                self.n_members.saturating_sub(1)
            )));
        }
        Ok(&self.member_end_forces[member_idx])
    }
}
