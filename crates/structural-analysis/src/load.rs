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

use crate::beam_fem::{AppliedMoment, DistributedLoad, FemError, PointLoad};
use crate::frame::{MemberHandle, NodeHandle};

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
        }
    }

    /// Name of this load case.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `true` if the case contains no loads.
    pub fn is_empty(&self) -> bool {
        self.nodal_forces.is_empty()
            && self.distributed_loads.is_empty()
            && self.point_loads.is_empty()
            && self.applied_moments.is_empty()
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
