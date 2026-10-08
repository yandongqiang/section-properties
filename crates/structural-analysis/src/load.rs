//! Load cases and load combinations for structural analysis.
//!
//! A [`LoadCase`] is a named collection of loads (nodal forces, distributed
//! loads, point loads, applied moments) that can be solved independently. A
//! [`LoadCombination`] is a linear combination of load cases,
//! `C = Σ factor_i × LoadCase_i`, solved by merging the right-hand side and
//! solving `K u = f` once.
//!
//! Both types reuse the existing load types ([`DistributedLoad`],
//! [`PointLoad`], [`AppliedMoment`]) — no new load types are introduced.

use crate::beam_fem::{AppliedMoment, DistributedLoad, Dof, FemError, PointLoad};
use crate::frame::{FrameModel, MemberHandle, NodeHandle};

// ---------------------------------------------------------------------------
// LoadCase
// ---------------------------------------------------------------------------

/// A named collection of loads representing one analysis load case.
///
/// Reuses the existing load types ([`DistributedLoad`], [`PointLoad`],
/// [`AppliedMoment`]); no new load types are introduced. Loads are stored by
/// raw node/member index (extracted from [`NodeHandle`] / [`MemberHandle`]);
/// index validity is checked at solve time against the model geometry.
///
/// # Example
///
/// ```
/// # use structural_analysis::{FrameModel, LoadCase, BeamSection};
/// # use section_properties::Material;
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// let mut frame = FrameModel::new();
/// let a = frame.add_node(0.0, 0.0)?;
/// let b = frame.add_node(4.0, 0.0)?;
/// let m = frame.add_member(a, b, Material::new(200e9, 0.3, 7850.0, "Steel"),
///                         BeamSection::new(5e-3, 2e-5))?;
/// frame.fix(a)?;
/// frame.fix(b)?;
///
/// let mut dead = LoadCase::new("dead");
/// dead.nodal_load(b, 0.0, -1000.0)?;
/// dead.member_udl(m, 0.0, -500.0)?;
///
/// let result = frame.solve_case(&dead)?;
/// assert!(result.equilibrium().is_balanced());
/// # Ok(())
/// # }
/// ```
#[derive(Debug, Clone)]
pub struct LoadCase {
    name: String,
    nodal_forces: Vec<(usize, usize, f64)>,
    distributed_loads: Vec<DistributedLoad>,
    point_loads: Vec<PointLoad>,
    applied_moments: Vec<AppliedMoment>,
    prescribed_displacements: Vec<(usize, usize, f64)>,
}

impl LoadCase {
    /// Create an empty load case with the given name.
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            nodal_forces: Vec::new(),
            distributed_loads: Vec::new(),
            point_loads: Vec::new(),
            applied_moments: Vec::new(),
            prescribed_displacements: Vec::new(),
        }
    }

    /// Name of this load case.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `true` if the case contains no loads and no prescribed displacements.
    pub fn is_empty(&self) -> bool {
        self.nodal_forces.is_empty()
            && self.distributed_loads.is_empty()
            && self.point_loads.is_empty()
            && self.applied_moments.is_empty()
            && self.prescribed_displacements.is_empty()
    }

    /// Apply a **global** nodal force `(fx, fy)` at `node`.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if either component is non-finite.
    pub fn nodal_load(&mut self, node: NodeHandle, fx: f64, fy: f64) -> Result<(), FemError> {
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
        let i = node.index();
        self.nodal_forces.push((i, 0, fx));
        self.nodal_forces.push((i, 1, fy));
        Ok(())
    }

    /// Apply a **global** nodal moment `mz` (counter-clockwise positive) at `node`.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if `mz` is non-finite.
    pub fn nodal_moment(&mut self, node: NodeHandle, mz: f64) -> Result<(), FemError> {
        if !mz.is_finite() {
            return Err(FemError::InvalidInput(format!(
                "nodal moment must be finite, got {mz}"
            )));
        }
        self.applied_moments
            .push(AppliedMoment::new(node.index(), mz));
        Ok(())
    }

    /// Apply a uniform distributed load in the member's **local** axes.
    ///
    /// `qy > 0` is upward in local +y; `qx > 0` is tensile (towards `node_j`).
    /// Repeated calls accumulate on the same member.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if `qx` or `qy` is non-finite.
    pub fn member_udl(&mut self, member: MemberHandle, qx: f64, qy: f64) -> Result<(), FemError> {
        if !qx.is_finite() || !qy.is_finite() {
            return Err(FemError::InvalidInput(format!(
                "distributed load must be finite, got qx = {qx}, qy = {qy}"
            )));
        }
        self.distributed_loads
            .push(DistributedLoad::new(member.index(), qx, qy));
        Ok(())
    }

    /// Apply a trapezoidal distributed load in the member's **local** axes.
    ///
    /// `qx` / `qy` are the intensities at `node_i`; `qx_end` / `qy_end` at
    /// `node_j`. The load varies linearly along the member.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if any component is non-finite.
    pub fn member_trapezoidal(
        &mut self,
        member: MemberHandle,
        qx: f64,
        qy: f64,
        qx_end: f64,
        qy_end: f64,
    ) -> Result<(), FemError> {
        if !qx.is_finite() || !qy.is_finite() || !qx_end.is_finite() || !qy_end.is_finite() {
            return Err(FemError::InvalidInput(format!(
                "trapezoidal load must be finite, got qx={qx}, qy={qy}, qx_end={qx_end}, qy_end={qy_end}"
            )));
        }
        self.distributed_loads.push(DistributedLoad::trapezoidal(
            member.index(),
            qx,
            qy,
            qx_end,
            qy_end,
        ));
        Ok(())
    }

    /// Apply a point load in the member's **local** axes at normalised position
    /// `xi in [0, 1]`.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if any component is non-finite.
    pub fn member_point_load(
        &mut self,
        member: MemberHandle,
        xi: f64,
        fx: f64,
        fy: f64,
        mz: f64,
    ) -> Result<(), FemError> {
        if !xi.is_finite() || !fx.is_finite() || !fy.is_finite() || !mz.is_finite() {
            return Err(FemError::InvalidInput(format!(
                "point load must be finite, got xi = {xi}, fx = {fx}, fy = {fy}, mz = {mz}"
            )));
        }
        self.point_loads
            .push(PointLoad::new(member.index(), xi, fx, fy, mz));
        Ok(())
    }

    /// Add self-weight (body-force) loads for all members of a [`FrameModel`].
    ///
    /// For each member the equivalent line load `w = ρ · A · g` is computed in
    /// global coordinates, transformed to the member's local axes, and added
    /// as a [`DistributedLoad`].  This is a **load source** — it modifies only
    /// this `LoadCase`'s RHS, not the stiffness matrix.
    ///
    /// Repeated calls accumulate.  Self-weight added to one `LoadCase` does
    /// not appear in other load cases — there is no cross-case contamination.
    ///
    /// # Units
    ///
    /// The library does not perform unit conversion; the user must ensure
    /// consistent units.  With SI inputs (`density` in kg/m³, `area` in m²,
    /// `gx`/`gy` in m/s²) the resulting line load is in N/m.
    ///
    /// # Gravity convention
    ///
    /// `gx` and `gy` are the components of the gravitational acceleration
    /// vector in **global** coordinates.  For standard downward gravity on
    /// Earth use `gx = 0.0, gy = -9.81`.
    ///
    /// # Example
    ///
    /// ```
    /// # use structural_analysis::{FrameModel, LoadCase, BeamSection, Dof};
    /// # use section_properties::Material;
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut frame = FrameModel::new();
    /// let a = frame.add_node(0.0, 0.0)?;
    /// let b = frame.add_node(6.0, 0.0)?;
    /// frame.add_member(a, b, Material::new(200e9, 0.3, 7850.0, "Steel"),
    ///                   BeamSection::new(5e-3, 2e-5))?;
    /// frame.fix(a)?;
    /// frame.fix(b)?;
    ///
    /// let mut dead = LoadCase::new("dead");
    /// dead.add_self_weight(&frame, 0.0, -9.81)?;
    ///
    /// let result = frame.solve_case(&dead)?;
    /// assert!(result.equilibrium().is_balanced());
    /// # Ok(())
    /// # }
    /// ```
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if `gx` or `gy` is non-finite, or if any
    /// member has non-finite or negative density, or non-finite or
    /// non-positive area.
    pub fn add_self_weight(
        &mut self,
        model: &FrameModel,
        gx: f64,
        gy: f64,
    ) -> Result<(), FemError> {
        let loads = model.self_weight_distributed_loads(gx, gy)?;
        for dl in loads {
            self.distributed_loads.push(dl);
        }
        Ok(())
    }

    /// Prescribe a non-zero displacement `value` at `(node, dof)`.
    ///
    /// This is a **support settlement** or **prescribed displacement** boundary
    /// condition, not an external force.  The constrained DOF is removed from
    /// the free system via static condensation; the reduced RHS is corrected
    /// by `f_f -= K_fc · u_c` (see [`FrameModel::solve_case`]).
    ///
    /// # Units
    ///
    /// `value` is in the same units as the DOF: metres for `Ux`/`Uy`,
    /// radians for `Rz`.
    ///
    /// # Sign convention
    ///
    /// Positive `value` is in the positive global direction of the DOF
    /// (rightward for `Ux`, upward for `Uy`, counter-clockwise for `Rz`).
    ///
    /// # Interaction with model supports
    ///
    /// At solve time the prescribed displacement **overrides** any existing
    /// constraint on the same DOF (e.g. from [`FrameModel::fix`] or
    /// [`FrameModel::pin`]).  If the DOF is not already constrained, a new
    /// constraint is added — this changes the constrained DOF set and
    /// therefore `K_ff`; the factorisation in
    /// [`PreparedFrameAnalysis`](crate::frame::PreparedFrameAnalysis) is not
    /// reused in that case (an error is returned). A prescription that
    /// targets a spring or inclined roller is rejected because those supports
    /// have their own stiffness/rotated constraint basis; prescribe the
    /// inclined displacement through the roller API instead.
    ///
    /// # Interaction with [`LoadCombination`]
    ///
    /// Load combinations do **not** support prescribed displacements.  If any
    /// case in a combination has prescribed displacements, [`solve_combination`]
    /// returns [`FemError::InvalidInput`].
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if `value` is non-finite.
    ///
    /// [`solve_combination`]: FrameModel::solve_combination
    pub fn prescribed_displacement(
        &mut self,
        node: NodeHandle,
        dof: Dof,
        value: f64,
    ) -> Result<(), FemError> {
        if !value.is_finite() {
            return Err(FemError::InvalidInput(format!(
                "prescribed displacement must be finite, got {value}"
            )));
        }
        self.prescribed_displacements
            .push((node.index(), dof.index(), value));
        Ok(())
    }

    /// Apply a **global** 3D nodal force `(fx, fy, fz)` at `node_idx`.
    ///
    /// This is the 3D-truss analogue of [`nodal_load`](Self::nodal_load); it
    /// stores the three components as raw `(node_idx, dof, value)` triples
    /// with `dof = 0, 1, 2` for `ux, uy, uz`.  No [`NodeHandle`] is required
    /// — the raw node index is used directly, matching
    /// [`TrussModel3D`](crate::truss3d::TrussModel3D) conventions.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if any component is non-finite.
    pub fn nodal_load_3d(
        &mut self,
        node_idx: usize,
        fx: f64,
        fy: f64,
        fz: f64,
    ) -> Result<(), FemError> {
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

    /// Prescribe a non-zero displacement `value` at `(node_idx, dof)` for a
    /// 3D truss model.
    ///
    /// `dof` is the raw DOF index: `0 = ux`, `1 = uy`, `2 = uz`.  This is the
    /// 3D-truss analogue of
    /// [`prescribed_displacement`](Self::prescribed_displacement).
    ///
    /// # Interaction with [`LoadCombination`]
    ///
    /// Load combinations do **not** support prescribed displacements.  If any
    /// case in a combination has prescribed displacements,
    /// [`TrussModel3D::solve_combination`](crate::truss3d::TrussModel3D::solve_combination)
    /// returns [`FemError::InvalidInput`].
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if `value` is non-finite.
    pub fn prescribed_displacement_3d(
        &mut self,
        node_idx: usize,
        dof: usize,
        value: f64,
    ) -> Result<(), FemError> {
        if !value.is_finite() {
            return Err(FemError::InvalidInput(format!(
                "prescribed displacement must be finite, got {value}"
            )));
        }
        self.prescribed_displacements.push((node_idx, dof, value));
        Ok(())
    }

    /// `true` if this case contains any prescribed displacements.
    pub fn has_prescribed_displacements(&self) -> bool {
        !self.prescribed_displacements.is_empty()
    }

    /// Read access to the raw nodal forces (for assembly by the solver path).
    pub(crate) fn nodal_forces(&self) -> &[(usize, usize, f64)] {
        &self.nodal_forces
    }

    /// Read access to the raw distributed loads.
    pub(crate) fn distributed_loads(&self) -> &[DistributedLoad] {
        &self.distributed_loads
    }

    /// Read access to the raw point loads.
    pub(crate) fn point_loads(&self) -> &[PointLoad] {
        &self.point_loads
    }

    /// Read access to the raw applied moments.
    pub(crate) fn applied_moments(&self) -> &[AppliedMoment] {
        &self.applied_moments
    }

    /// Read access to the raw prescribed displacements `(node_idx, dof_idx, value)`.
    pub(crate) fn prescribed_displacements(&self) -> &[(usize, usize, f64)] {
        &self.prescribed_displacements
    }
}

// ---------------------------------------------------------------------------
// LoadSource
// ---------------------------------------------------------------------------

/// One term of a solved [`LoadCombination`].
#[derive(Debug, Clone, PartialEq)]
pub struct LoadCombinationTerm {
    /// Name of the contributing load case.
    pub case_name: String,
    /// Factor applied to that case.
    pub factor: f64,
}

/// Typed provenance of the load that produced an analysis result.
///
/// This replaces the previous `Option<String>` label so that model-resident
/// loads, a single load case, and a load combination are unambiguous. A
/// combination also carries its terms, so a result remains self-describing
/// without an external name lookup.
#[derive(Debug, Clone, PartialEq)]
pub enum LoadSource {
    /// Loads stored directly on the model (`FrameModel::solve`).
    ModelLoads,
    /// A single named [`LoadCase`].
    LoadCase {
        /// Case name supplied by the caller.
        name: String,
        /// Whether the case prescribes any displacement.
        has_prescribed_displacements: bool,
    },
    /// A named [`LoadCombination`] and its scaled terms.
    LoadCombination {
        /// Combination name supplied by the caller.
        name: String,
        /// Terms actually combined, in insertion order.
        terms: Vec<LoadCombinationTerm>,
    },
}

impl LoadSource {
    /// Case or combination name, or `None` for model-resident loads.
    pub fn name(&self) -> Option<&str> {
        match self {
            Self::ModelLoads => None,
            Self::LoadCase { name, .. } | Self::LoadCombination { name, .. } => Some(name),
        }
    }

    /// Whether this source prescribes any displacement.
    pub fn has_prescribed_displacements(&self) -> bool {
        matches!(
            self,
            Self::LoadCase {
                has_prescribed_displacements: true,
                ..
            }
        )
    }
}

impl std::fmt::Display for LoadSource {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ModelLoads => f.write_str("model loads"),
            Self::LoadCase { name, .. } => write!(f, "case:{name}"),
            Self::LoadCombination { name, .. } => write!(f, "combination:{name}"),
        }
    }
}

// ---------------------------------------------------------------------------
// LoadCombination
// ---------------------------------------------------------------------------

/// A linear combination of load cases: `C = Σ factor_i × LoadCase_i`.
///
/// Factors are validated to be finite real numbers (NaN and ±∞ are rejected;
/// negative factors are allowed). The combination owns clones of the added
/// load cases.
///
/// # Example
///
/// ```
/// # use structural_analysis::{FrameModel, LoadCase, LoadCombination, BeamSection};
/// # use section_properties::Material;
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// let mut frame = FrameModel::new();
/// let a = frame.add_node(0.0, 0.0)?;
/// let b = frame.add_node(4.0, 0.0)?;
/// let m = frame.add_member(a, b, Material::new(200e9, 0.3, 7850.0, "Steel"),
///                         BeamSection::new(5e-3, 2e-5))?;
/// frame.fix(a)?;
/// frame.fix(b)?;
///
/// let mut dead = LoadCase::new("dead");
/// dead.nodal_load(b, 0.0, -1000.0)?;
///
/// let mut live = LoadCase::new("live");
/// live.nodal_load(b, 0.0, -800.0)?;
///
/// let mut combo = LoadCombination::new("1.4D + 1.6L");
/// combo.add_case(&dead, 1.4)?;
/// combo.add_case(&live, 1.6)?;
///
/// let result = frame.solve_combination(&combo)?;
/// assert!(result.equilibrium().is_balanced());
/// # Ok(())
/// # }
/// ```
#[derive(Debug, Clone)]
pub struct LoadCombination {
    name: String,
    terms: Vec<(LoadCase, f64)>,
}

impl LoadCombination {
    /// Create an empty load combination with the given name.
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            terms: Vec::new(),
        }
    }

    /// Name of this load combination.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Number of load-case terms in the combination.
    pub fn n_terms(&self) -> usize {
        self.terms.len()
    }

    /// `true` if the combination has no terms.
    pub fn is_empty(&self) -> bool {
        self.terms.is_empty()
    }

    /// Add a load case with a scaling factor.
    ///
    /// The load case is **cloned** into the combination. Negative factors are
    /// allowed; NaN and ±∞ are rejected.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if `factor` is not finite.
    pub fn add_case(&mut self, case: &LoadCase, factor: f64) -> Result<(), FemError> {
        if !factor.is_finite() {
            return Err(FemError::InvalidInput(format!(
                "load factor must be finite, got {factor}"
            )));
        }
        self.terms.push((case.clone(), factor));
        Ok(())
    }

    /// Read access to the terms (for assembly by the solver path).
    pub(crate) fn terms(&self) -> &[(LoadCase, f64)] {
        &self.terms
    }
}
