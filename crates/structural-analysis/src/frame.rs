//! 2D frame façade over the verified Beam FEM core (Phase 13).
//!
//! This module adds **no mechanics**. Every element stiffness, coordinate
//! transformation, load vector, static-condensation step, end-force recovery
//! and solver call is delegated to [`BeamModel`] / [`BeamSolver`]. What it adds
//! is a frame-shaped surface:
//!
//! * typed [`NodeHandle`] / [`MemberHandle`] addressing instead of positional
//!   indices;
//! * connectivity validation (orphan nodes, disconnected components, duplicate
//!   and zero-length members) with *targeted* diagnostics that replace the
//!   opaque "singular matrix" error;
//! * a support vocabulary ([`FrameModel::fix`], [`FrameModel::pin`],
//!   [`FrameModel::roller_y`], [`FrameModel::roller_x`],
//!   [`FrameModel::restrain`]);
//! * frame-level result access and a global equilibrium report.
//!
//! # Conventions (frozen - identical to `docs/beam_fem.md`)
//!
//! ```text
//! DOFs              [ux, uy, rz] per node (global), rz counter-clockwise positive
//! nodal loads       GLOBAL   (add via FrameModel::nodal_load / nodal_moment)
//! member loads      LOCAL    (add via FrameModel::member_udl / member_point_load)
//! member end forces f_end = f_equiv - K_e u_e   (element-on-node, LOCAL axes)
//! reactions         R = K_original u - f_global (global)
//! constraints       static condensation K_ff u_f = f_f - K_fc u_c (no penalties)
//! ```
//!
//! A member point load at `xi = 0` or `xi = 1` acts at a node and is **not**
//! double counted.
//!
//! # Current limitations (documented, not defects)
//!
//! * Parallel members between the same node pair are **not** supported: the
//!   second one is rejected as a duplicate. Member end releases (hinges) are
//!   supported via [`FrameModel::add_member_with_release`].
//! * A single connected structural system is required; multiple independent
//!   substructures are rejected as disconnected.
//! * Member loads are uniform (single `(qx, qy)` per member per call; repeated
//!   calls accumulate, matching the core behaviour).
//!
//! # Example
//!
//! ```rust
//! use structural_analysis::frame::FrameModel;
//! use structural_analysis::{BeamSection, Dof};
//! use section_properties::Material;
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! let mut frame = FrameModel::new();
//! let base = frame.add_node(0.0, 0.0)?;
//! let tip = frame.add_node(2.0, 0.0)?;
//! frame.add_member(base, tip, Material::new(200e9, 0.3, 7850.0, "Steel"),
//!                   BeamSection::new(5e-3, 2e-5))?;
//! frame.fix(base)?;
//! frame.nodal_load(tip, 0.0, -1.0e4)?;
//!
//! let result = frame.solve()?;
//! // v = -P L^3 / 3EI
//! let uy = result.displacement(tip, Dof::Uy)?;
//! assert!((uy + 1.0e4 * 2.0f64.powi(3) / (3.0 * 200e9 * 2e-5)).abs() < 1e-12);
//! assert!(result.equilibrium().is_balanced());
//! # Ok(())
//! # }
//! ```

use crate::beam_fem::{
    AppliedMoment, BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver, DistributedLoad, Dof,
    EndRelease, FemError, PointLoad, ReducedSystem,
};
use crate::load::{LoadCase, LoadCombination};
use crate::mechanism::diagnose_reduced;
use section_properties::SolverSelection;
use section_properties::material::Material;

use std::collections::VecDeque;

pub use crate::mechanism::StructuralDiagnostic;

// ---------------------------------------------------------------------------
// Handles
// ---------------------------------------------------------------------------

/// Handle to a node of a [`FrameModel`].
///
/// Handles are produced by [`FrameModel::add_node`] and are only valid for the
/// model that created them. They are deliberately distinct from raw DOF
/// indices so a node reference cannot be confused with a degree of freedom.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct NodeHandle(usize);

impl NodeHandle {
    /// The zero-based index of the node inside the model.
    pub fn index(self) -> usize {
        self.0
    }

    /// Build a handle from a raw index.
    ///
    /// The handle is **not** validated here; validity is checked when it is
    /// used against a model (returning [`FemError::InvalidNode`]). Prefer
    /// [`FrameModel::add_node`] in normal use.
    pub fn from_index(index: usize) -> Self {
        Self(index)
    }
}

/// Handle to a member (two-node frame element) of a [`FrameModel`].
///
/// Produced by [`FrameModel::add_member`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct MemberHandle(usize);

impl MemberHandle {
    /// The zero-based index of the member inside the model.
    pub fn index(self) -> usize {
        self.0
    }

    /// Build a handle from a raw index.
    ///
    /// The handle is **not** validated here; validity is checked when it is
    /// used against a model (returning [`FemError::InvalidMember`]). Prefer
    /// [`FrameModel::add_member`] in normal use.
    pub fn from_index(index: usize) -> Self {
        Self(index)
    }
}

// ---------------------------------------------------------------------------
// Equilibrium report
// ---------------------------------------------------------------------------

/// Relative equilibrium tolerance: residuals are compared against the analysed
/// system's **own** force and moment magnitudes, never against an absolute
/// floor, so the verdict is invariant under a change of units (N/m ↔ kN/mm).
const EQUILIBRIUM_REL_TOL: f64 = 1e-6;

/// Conditioning-relief coefficient for [`EquilibriumReport::is_balanced`].
///
/// A Euler–Bernoulli frame's stiffness matrix is ill-conditioned when its
/// transverse/axial stiffness ratio grows: for an element the dominant
/// conditioning driver is `A·L²/I = (L/r)²`, the square of the slenderness
/// ratio `L/r` (with `r² = I/A` the radius of gyration). The forward error of a
/// direct solve is `O((L/r)² · ε)`, so equilibrium residuals — which inherit
/// that error through the recovered reactions — can legitimately reach the same
/// relative magnitude at large slenderness. A fixed `EQUILIBRIUM_REL_TOL` would
/// then flag a physically balanced frame as unbalanced (catastrophic
/// cancellation of large opposing reactions).
///
/// `effective_rel_tol = max(EQUILIBRIUM_REL_TOL, C · λ²_max · ε)` recovers the
/// correct verdict: `λ²_max = max_elements(L²·A/I)` is a **dimensionless** and
/// **unit-invariant** element-level slenderness/conditioning proxy (a consistent
/// change of the length/area/inertia units multiplies numerator and denominator
/// by the same power, leaving the ratio unchanged), so the tolerance itself is
/// dimensionless and unit-invariant. It is not a strict bound on the full
/// assembled stiffness matrix condition number — the global conditioning may be
/// worse — but it captures the dominant per-element driver and biases the
/// tolerance toward being tight rather than loose.
///
/// `C = 1e-2` is an **empirically calibrated** safety factor, not a rigorous
/// error upper bound. It was calibrated against the measured residual of a
/// large-scale symmetric hyperstatic portal regression case (`λ² ≈ 2.5e14`,
/// measured residual `~1.7e-3 · λ² · ε`); the coefficient is ≪ 1 because
/// reaction cancellation reduces the residual far below the pessimistic
/// forward-error bound. The calibration is based on this specific topology and
/// load pattern; other frame configurations may exhibit different cancellation
/// behaviour. Below `λ² ≈ 4.5e9` (span ≈ 42 km for the reference section) the
/// term stays below `EQUILIBRIUM_REL_TOL` and the tolerance is exactly `1e-6`,
/// unchanged.
const EQUILIBRIUM_COND_FACTOR: f64 = 1e-2;

/// Result of the global equilibrium check, summed **about the global origin
/// `(0, 0)`**.
///
/// Residuals are `applied + reaction` and should be zero for a solved,
/// properly restrained frame. Both the residuals and the contributing totals
/// are exposed so a caller can diagnose which component is out of balance
/// instead of receiving a single boolean.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EquilibriumReport {
    /// Residual of `ΣFx` (applied + reaction).
    pub fx_residual: f64,
    /// Residual of `ΣFy` (applied + reaction).
    pub fy_residual: f64,
    /// Residual of `ΣMz` about the global origin (applied + reaction).
    pub mz_residual: f64,
    /// Total applied horizontal load (global nodal, plus member load resultants).
    pub applied_fx: f64,
    /// Total applied vertical load.
    pub applied_fy: f64,
    /// Total applied moment about the origin, including force moments `x·Fy − y·Fx`.
    pub applied_mz: f64,
    /// Sum of support reaction forces in X.
    pub reaction_fx: f64,
    /// Sum of support reaction forces in Y.
    pub reaction_fy: f64,
    /// Sum of reaction moments about the origin, including `x·Ry − y·Rx`.
    pub reaction_mz: f64,
    /// Absolute **force** tolerance applied by [`Self::is_balanced`], equal to
    /// `effective_rel_tol · force_scale` (see [`Self::force_tolerance`]). The
    /// moment tolerance is **not** stored in this field; use
    /// [`Self::moment_tolerance`] to obtain it. Using `tolerance` to
    /// threshold `mz_residual` gives a wrong verdict when `l_char != 1`.
    /// Private: use [`Self::force_tolerance`] or [`Self::moment_tolerance`].
    tolerance: f64,
    /// Σ of the absolute applied and reaction **force** magnitudes actually
    /// present (per load term, not per resultant — a self-cancelling load pair
    /// must not collapse the scale to zero). Private: it only feeds
    /// [`Self::is_balanced`].
    f_mag: f64,
    /// Σ of the absolute applied and reaction **moment** magnitudes actually
    /// present, again per term. Private, as above.
    m_mag: f64,
    /// Characteristic length of the structure: the largest absolute nodal
    /// coordinate (see [`characteristic_length`]). Private, as above.
    l_char: f64,
    /// Conditioning-relief relative floor `C · λ²_max · ε`, where `λ²_max` is
    /// the maximum element slenderness squared `L²·A/I` (dimensionless,
    /// unit-invariant). `0.0` when the frame has no element. Private: it only
    /// feeds [`Self::is_balanced`]; see [`EQUILIBRIUM_COND_FACTOR`].
    cond_rel_floor: f64,
}

/// The two scale factors behind [`EquilibriumReport::is_balanced`]:
/// `(force_scale, moment_scale)`.
///
/// Both are built **only** from magnitudes the analysed system actually carries,
/// so a pure change of units (N/m ↔ kN/mm) leaves the verdict unchanged:
///
/// * `force_scale = Σ|F| + Σ|M| / l_char` — the force magnitudes present
///   (applied plus recovered reactions) plus the equivalent force of the moments
///   present, so a pure-moment system still has a positive force scale.
/// * `moment_scale = Σ|M| + Σ|F| · l_char` — the moment magnitudes present plus
///   the moment the forces produce through their largest lever arm `l_char`.
///   The `Σ|F| · l_char` term is what keeps the moment check sensitive for a load
///   system that is symmetric about the origin (`applied_mz == 0`): the scale
///   then reflects the real force × length magnitudes rather than collapsing to
///   a constant.
///
/// `Σ|F|` and `Σ|M|` are sums of absolute **per-term** magnitudes (`f_mag`,
/// `m_mag`), so equal-and-opposite terms that cancel in the resultant still keep
/// the scale positive; `l_char` is the largest absolute nodal coordinate. All
/// three are zero only for a model with no load and no reactions, in which case
/// both scales are zero and the check degenerates to "the residual must be
/// exactly zero".
fn equilibrium_scales(f_mag: f64, m_mag: f64, l_char: f64) -> (f64, f64) {
    if l_char > 0.0 {
        (f_mag + m_mag / l_char, m_mag + f_mag * l_char)
    } else {
        (f_mag, m_mag)
    }
}

impl EquilibriumReport {
    /// Whether all three residuals are within a tolerance **relative** to the
    /// analysed system's own magnitudes (see `equilibrium_scales`).
    ///
    /// The force residuals (`fx`, `fy`) are compared against `force_scale` and
    /// the moment residual (`mz`) against `moment_scale`, both scaled by the
    /// conditioning-aware effective relative tolerance
    /// `Self::effective_rel_tol`. At normal slenderness this is exactly
    /// `EQUILIBRIUM_REL_TOL` (`1e-6`); for highly slender frames it is raised
    /// by a dimensionless element-level conditioning proxy (see
    /// `EQUILIBRIUM_COND_FACTOR`) so that pure solver round-off is not
    /// mistaken for a physical imbalance. The verdict is invariant under a
    /// change of units and the moment check stays sensitive even when a load
    /// system is symmetric about the origin (`applied_mz == 0`). There is no
    /// absolute floor: if every relevant magnitude is zero the scale is zero
    /// and the residual must then be **exactly** zero for the frame to be
    /// balanced.
    pub fn is_balanced(&self) -> bool {
        let (f_scale, m_scale) = equilibrium_scales(self.f_mag, self.m_mag, self.l_char);
        let rel = self.effective_rel_tol();
        self.fx_residual.abs() <= rel * f_scale
            && self.fy_residual.abs() <= rel * f_scale
            && self.mz_residual.abs() <= rel * m_scale
    }

    /// Effective relative tolerance: at least [`EQUILIBRIUM_REL_TOL`], relaxed
    /// by the conditioning floor for a slender frame so pure solver round-off
    /// is not mistaken for a physical imbalance.
    fn effective_rel_tol(&self) -> f64 {
        EQUILIBRIUM_REL_TOL.max(self.cond_rel_floor)
    }

    /// Force tolerance used by [`Self::is_balanced`]: `effective_rel_tol *
    /// force_scale`, where `force_scale = Σ|F| + Σ|M| / l_char` (see
    /// `equilibrium_scales`). The force residuals `fx_residual` and
    /// `fy_residual` are compared against this value.
    ///
    /// This is identical to the `tolerance` field, exposed as a
    /// method for clarity and symmetry with [`Self::moment_tolerance`].
    pub fn force_tolerance(&self) -> f64 {
        let (f_scale, _) = equilibrium_scales(self.f_mag, self.m_mag, self.l_char);
        self.effective_rel_tol() * f_scale
    }

    /// Moment tolerance used by [`Self::is_balanced`]: `effective_rel_tol *
    /// moment_scale`, where `moment_scale = Σ|M| + Σ|F| * l_char` (see
    /// `equilibrium_scales`). The moment residual `mz_residual` is
    /// compared against this value.
    ///
    /// This is generally **not** equal to `tolerance` (which is the
    /// force tolerance); using `tolerance` to threshold `mz_residual` would
    /// give a wrong verdict for structures with `l_char != 1`.
    pub fn moment_tolerance(&self) -> f64 {
        let (_, m_scale) = equilibrium_scales(self.f_mag, self.m_mag, self.l_char);
        self.effective_rel_tol() * m_scale
    }
}

// ---------------------------------------------------------------------------
// FrameModel
// ---------------------------------------------------------------------------

/// A 2D frame: nodes, members, supports and loads.
///
/// Wraps the existing [`BeamModel`] and delegates all mechanics to it; supports
/// arbitrary planar connectivity (not just a straight beam), per-DOF
/// constraints and prescribed displacements.
#[derive(Debug, Clone)]
pub struct FrameModel {
    inner: BeamModel,
}

impl Default for FrameModel {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameModel {
    /// Create an empty frame.
    pub fn new() -> Self {
        Self {
            inner: BeamModel::new(),
        }
    }

    /// Number of nodes.
    pub fn n_nodes(&self) -> usize {
        self.inner.nodes.len()
    }

    /// Number of members.
    pub fn n_members(&self) -> usize {
        self.inner.elements.len()
    }

    /// Add a node at global `(x, y)` and return its handle.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidInput`] if either coordinate is not finite.
    pub fn add_node(&mut self, x: f64, y: f64) -> Result<NodeHandle, FemError> {
        if !x.is_finite() || !y.is_finite() {
            return Err(FemError::InvalidInput(format!(
                "node coordinates must be finite, got ({x}, {y})"
            )));
        }
        let idx = self.inner.nodes.len();
        self.inner.add_node(BeamNode::new(idx, x, y));
        Ok(NodeHandle(idx))
    }

    /// Add a member between two existing nodes.
    ///
    /// The member stiffness uses `EA` (axial) and `EI` (bending) from the given
    /// material and section, exactly as the Beam FEM core does.
    ///
    /// # Errors
    ///
    /// * [`FemError::InvalidNode`] if either handle is not valid for this model;
    /// * [`FemError::ZeroLengthMember`] if the two nodes coincide;
    /// * [`FemError::DuplicateMember`] if a member already connects the same
    ///   node pair (in either order) - parallel members are not supported;
    /// * [`FemError::InvalidInput`] if `E`, `A` or `I` is non-finite or
    ///   non-positive (the same predicate the solver applies at assembly time).
    pub fn add_member(
        &mut self,
        a: NodeHandle,
        b: NodeHandle,
        material: Material,
        section: BeamSection,
    ) -> Result<MemberHandle, FemError> {
        let ia = self.check_node(a)?;
        let ib = self.check_node(b)?;
        if ia == ib {
            return Err(FemError::ZeroLengthMember(format!(
                "member connects node {ia} to itself"
            )));
        }
        let (pa, pb) = (self.inner.nodes[ia].point(), self.inner.nodes[ib].point());
        let (dx, dy) = (pb.x - pa.x, pb.y - pa.y);
        if !dx.is_finite() || !dy.is_finite() || dx * dx + dy * dy <= 0.0 {
            return Err(FemError::ZeroLengthMember(format!(
                "nodes {ia} and {ib} coincide at ({}, {})",
                pa.x, pa.y
            )));
        }

        let (e, area, inertia) = (material.youngs_modulus, section.area, section.second_moment);
        if !e.is_finite() || e <= 0.0 {
            return Err(FemError::InvalidInput(format!(
                "Young's modulus must be finite and positive, got {e}"
            )));
        }
        if !area.is_finite() || area <= 0.0 {
            return Err(FemError::InvalidInput(format!(
                "cross-section area must be finite and positive, got {area}"
            )));
        }
        if !inertia.is_finite() || inertia <= 0.0 {
            return Err(FemError::InvalidInput(format!(
                "second moment of area must be finite and positive, got {inertia}"
            )));
        }

        let (lo, hi) = if ia < ib { (ia, ib) } else { (ib, ia) };
        for existing in &self.inner.elements {
            let (elo, ehi) = if existing.node_i < existing.node_j {
                (existing.node_i, existing.node_j)
            } else {
                (existing.node_j, existing.node_i)
            };
            if elo == lo && ehi == hi {
                return Err(FemError::DuplicateMember(format!(
                    "a member already connects nodes {lo} and {hi}; parallel members are not supported"
                )));
            }
        }

        let element = BeamElement::new(ia, ib, material, section)?;
        self.inner.add_element(element);
        Ok(MemberHandle(self.inner.elements.len() - 1))
    }

    /// Add a member between two existing nodes with end releases.
    ///
    /// This is like [`add_member`](Self::add_member) but applies
    /// [`EndRelease`] to the member. The release is enforced via static
    /// condensation of the element stiffness matrix — the node DOFs are
    /// **not** removed or constrained.
    ///
    /// # Errors
    ///
    /// Same as [`add_member`](Self::add_member).
    ///
    /// # Example — fixed-fixed beam with one end released
    ///
    /// ```rust
    /// use structural_analysis::{FrameModel, BeamSection, EndRelease, MemberHandle};
    /// use section_properties::Material;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut frame = FrameModel::new();
    /// let a = frame.add_node(0.0, 0.0)?;
    /// let b = frame.add_node(4.0, 0.0)?;
    /// let m: MemberHandle = frame.add_member_with_release(a, b,
    ///     Material::new(200e9, 0.3, 7850.0, "Steel"),
    ///     BeamSection::new(5e-3, 2e-5),
    ///     EndRelease::end_pin())?; // hinge at b
    /// frame.fix(a)?;
    /// frame.fix(b)?;
    /// frame.member_udl(m, 0.0, -1000.0)?; // 1 kN/m downward
    /// let result = frame.solve()?;
    /// // Released end moment at b is zero
    /// let forces = result.member_end_forces(m)?;
    /// assert!(forces[5].abs() < 1.0); // M_j ≈ 0
    /// # Ok(())
    /// # }
    /// ```
    pub fn add_member_with_release(
        &mut self,
        a: NodeHandle,
        b: NodeHandle,
        material: Material,
        section: BeamSection,
        end_release: EndRelease,
    ) -> Result<MemberHandle, FemError> {
        let ia = self.check_node(a)?;
        let ib = self.check_node(b)?;
        if ia == ib {
            return Err(FemError::ZeroLengthMember(format!(
                "member connects node {ia} to itself"
            )));
        }
        let (pa, pb) = (self.inner.nodes[ia].point(), self.inner.nodes[ib].point());
        let (dx, dy) = (pb.x - pa.x, pb.y - pa.y);
        if !dx.is_finite() || !dy.is_finite() || dx * dx + dy * dy <= 0.0 {
            return Err(FemError::ZeroLengthMember(format!(
                "nodes {ia} and {ib} coincide at ({}, {})",
                pa.x, pa.y
            )));
        }

        let (e, area, inertia) = (material.youngs_modulus, section.area, section.second_moment);
        if !e.is_finite() || e <= 0.0 {
            return Err(FemError::InvalidInput(format!(
                "Young's modulus must be finite and positive, got {e}"
            )));
        }
        if !area.is_finite() || area <= 0.0 {
            return Err(FemError::InvalidInput(format!(
                "cross-section area must be finite and positive, got {area}"
            )));
        }
        if !inertia.is_finite() || inertia <= 0.0 {
            return Err(FemError::InvalidInput(format!(
                "second moment of area must be finite and positive, got {inertia}"
            )));
        }

        let (lo, hi) = if ia < ib { (ia, ib) } else { (ib, ia) };
        for existing in &self.inner.elements {
            let (elo, ehi) = if existing.node_i < existing.node_j {
                (existing.node_i, existing.node_j)
            } else {
                (existing.node_j, existing.node_i)
            };
            if elo == lo && ehi == hi {
                return Err(FemError::DuplicateMember(format!(
                    "a member already connects nodes {lo} and {hi}; parallel members are not supported"
                )));
            }
        }

        let element = BeamElement::with_end_release(ia, ib, material, section, end_release)?;
        self.inner.add_element(element);
        Ok(MemberHandle(self.inner.elements.len() - 1))
    }
    ///
    /// [`FemError::InvalidNode`] if the handle is not valid.
    /// [`FemError::ConflictingPrescribedDisplacement`] if any of the node's
    /// DOFs already has a conflicting prescribed value.
    pub fn fix(&mut self, node: NodeHandle) -> Result<(), FemError> {
        let i = self.check_node(node)?;
        self.inner.try_fix_node(i)
    }

    /// Pin a node: restrain `ux` and `uy`, leave the rotation free.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if the handle is not valid.
    /// [`FemError::ConflictingPrescribedDisplacement`] if either `ux` or `uy`
    /// already has a conflicting prescribed value.
    pub fn pin(&mut self, node: NodeHandle) -> Result<(), FemError> {
        let i = self.check_node(node)?;
        self.inner.try_fix_dof(i, Dof::Ux.index(), 0.0)?;
        self.inner.try_fix_dof(i, Dof::Uy.index(), 0.0)
    }

    /// Roller restraining the vertical translation (`uy = 0`), horizontal free.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if the handle is not valid.
    /// [`FemError::ConflictingPrescribedDisplacement`] if `uy` already has a
    /// conflicting prescribed value.
    pub fn roller_y(&mut self, node: NodeHandle) -> Result<(), FemError> {
        let i = self.check_node(node)?;
        self.inner.try_fix_dof(i, Dof::Uy.index(), 0.0)
    }

    /// Roller restraining the horizontal translation (`ux = 0`), vertical free.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if the handle is not valid.
    /// [`FemError::ConflictingPrescribedDisplacement`] if `ux` already has a
    /// conflicting prescribed value.
    pub fn roller_x(&mut self, node: NodeHandle) -> Result<(), FemError> {
        let i = self.check_node(node)?;
        self.inner.try_fix_dof(i, Dof::Ux.index(), 0.0)
    }

    /// Prescribe a DOF to an arbitrary finite value (e.g. a support settlement).
    ///
    /// Applied through the existing static condensation
    /// (`K_ff u_f = f_f - K_fc u_c`); no penalty constraints are used.
    ///
    /// If the DOF was previously constrained (e.g. by [`fix`](Self::fix)),
    /// the existing prescription is **replaced** — this allows the common
    /// workflow of fixing a support node and then applying a settlement.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if the handle is not valid.
    /// [`FemError::InvalidInput`] if `value` is not finite.
    pub fn restrain(&mut self, node: NodeHandle, dof: Dof, value: f64) -> Result<(), FemError> {
        let i = self.check_node(node)?;
        self.inner.try_override(i, dof, value)
    }

    /// Add a spring support at a node DOF.
    ///
    /// The spring stiffness is added to the global stiffness matrix diagonal;
    /// the DOF remains free and the spring provides finite restraint.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if the handle is not valid.
    /// [`FemError::InvalidInput`] if `stiffness` is not finite and positive.
    pub fn spring(&mut self, node: NodeHandle, dof: Dof, stiffness: f64) -> Result<(), FemError> {
        let i = self.check_node(node)?;
        self.inner.spring(i, dof, stiffness)
    }

    /// Add an inclined roller at a node.
    ///
    /// Constrains displacement in direction `(nx, ny)` (auto-normalized);
    /// the orthogonal direction is free. `value` is a prescribed displacement
    /// (default 0.0).
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if the handle is not valid.
    /// [`FemError::InvalidInput`] if `(nx, ny)` is zero or `value` is non-finite.
    pub fn inclined_roller(
        &mut self,
        node: NodeHandle,
        nx: f64,
        ny: f64,
        value: f64,
    ) -> Result<(), FemError> {
        let i = self.check_node(node)?;
        self.inner.inclined_roller(i, nx, ny, value)
    }

    /// Apply a **global** nodal force `(fx, fy)`.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if the handle is not valid, or
    /// [`FemError::InvalidInput`] if either component is non-finite. Both
    /// components are validated **before** either is recorded, so a rejected
    /// call leaves the model unchanged (no half-applied load).
    pub fn nodal_load(&mut self, node: NodeHandle, fx: f64, fy: f64) -> Result<(), FemError> {
        let i = self.check_node(node)?;
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
        self.inner.try_add_nodal_force(i, Dof::Ux.index(), fx)?;
        self.inner.try_add_nodal_force(i, Dof::Uy.index(), fy)
    }

    /// Apply a **global** nodal moment `mz` (counter-clockwise positive).
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if the handle is not valid.
    pub fn nodal_moment(&mut self, node: NodeHandle, mz: f64) -> Result<(), FemError> {
        let i = self.check_node(node)?;
        self.inner.add_applied_moment(i, mz)
    }

    /// Apply a uniform distributed load in the member's **local** axes.
    ///
    /// `qy > 0` is upward in local +y; `qx > 0` is tensile (towards `node_j`).
    /// Repeated calls accumulate on the same member.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidMember`] if the handle is not valid.
    pub fn member_udl(&mut self, member: MemberHandle, qx: f64, qy: f64) -> Result<(), FemError> {
        let i = self.check_member(member)?;
        self.inner.add_distributed_load(i, qx, qy)
    }

    /// Apply a trapezoidal distributed load in the member's **local** axes.
    ///
    /// `qx` / `qy` are the intensities at `node_i`; `qx_end` / `qy_end` at
    /// `node_j`. The load varies linearly along the member. Repeated calls
    /// accumulate on the same member.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidMember`] if the handle is not valid.
    pub fn member_trapezoidal(
        &mut self,
        member: MemberHandle,
        qx: f64,
        qy: f64,
        qx_end: f64,
        qy_end: f64,
    ) -> Result<(), FemError> {
        let i = self.check_member(member)?;
        self.inner.add_trapezoidal_load(i, qx, qy, qx_end, qy_end)
    }

    /// Apply a point load in the member's **local** axes at normalised position
    /// `xi in [0, 1]`.
    ///
    /// `xi = 0` / `xi = 1` places the load at a node; it is applied exactly once
    /// (no double counting).
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidMember`] if the handle is not valid.
    pub fn member_point_load(
        &mut self,
        member: MemberHandle,
        xi: f64,
        fx: f64,
        fy: f64,
        mz: f64,
    ) -> Result<(), FemError> {
        let i = self.check_member(member)?;
        self.inner.add_point_load(i, xi, fx, fy, mz)
    }

    /// Solve with automatic solver selection.
    ///
    /// Equivalent to `self.solve_with(SolverSelection::Auto)`. The registry
    /// inspects the condensed stiffness matrix and picks a backend (dense for
    /// small systems, skyline LDLᵀ for symmetric positive-definite, sparse LU
    /// otherwise). The selected solver is used exactly once; if it fails the
    /// error is returned directly — there is no automatic retry with a
    /// different backend.
    ///
    /// # Errors
    ///
    /// Returns [`FemError`] if the model is invalid (see [`Self::validate`]),
    /// the stiffness matrix is singular/near-singular, or the solver backend
    /// fails.
    pub fn solve(&self) -> Result<FrameAnalysisResult, FemError> {
        self.solve_with(SolverSelection::Auto)
    }

    /// Solve with an explicit backend selection.
    ///
    /// Validates the model (see [`Self::validate`]) before assembling, so
    /// structural modelling errors are reported as targeted diagnostics rather
    /// than as a matrix singularity.
    ///
    /// **Solver semantics**:
    /// - [`SolverSelection::Auto`] — the registry inspects the matrix and
    ///   selects a backend. The chosen solver is used exactly once; if it
    ///   fails, the error is returned directly (no automatic retry with a
    ///   different backend).
    /// - [`SolverSelection::Named`] — the requested solver is used **only**;
    ///   no silent fallback. If the solver is unavailable or fails, the error
    ///   is returned directly. Use the convenience constructors
    ///   [`SolverSelection::dense`], [`SolverSelection::skyline_ldlt`],
    ///   [`SolverSelection::sparse_lu`], [`SolverSelection::cg`],
    ///   [`SolverSelection::iccg`] for typed selection.
    ///
    /// After solving, [`FrameAnalysisResult::solver_name`] reports which
    /// backend was actually used.
    ///
    /// # Errors
    ///
    /// Returns [`FemError`] if the model is invalid, the requested solver is
    /// unavailable, or the solve fails (singular/near-singular matrix).
    pub fn solve_with(&self, selection: SolverSelection) -> Result<FrameAnalysisResult, FemError> {
        FrameSolver::new(self).with_selection(selection).solve()
    }

    /// Solve the frame under a single [`LoadCase`].
    ///
    /// The stiffness matrix and boundary conditions come from `self`; the load
    /// vector is assembled entirely from `case`. Any loads added directly to the
    /// model (via [`nodal_load`](Self::nodal_load), [`member_udl`](Self::member_udl),
    /// etc.) are **ignored** — only the load case's loads are used.
    ///
    /// The result's [`load_source`](FrameAnalysisResult::load_source) is
    /// `"case:{name}"`.
    ///
    /// # Errors
    ///
    /// Same as [`solve`](Self::solve), plus [`FemError::InvalidModel`] if a load
    /// in the case references a node or member that does not exist.
    pub fn solve_case(&self, case: &LoadCase) -> Result<FrameAnalysisResult, FemError> {
        self.validate()?;
        let model = self.inner_with_loads(
            case.nodal_forces(),
            case.distributed_loads(),
            case.point_loads(),
            case.applied_moments(),
        );
        let mut beam = BeamSolver::from_model(&model)?;
        if let Err(e) = beam.solve_configured() {
            let diagnosis = beam.reduced_system().map(|rs| self.classify(rs));
            return Err(with_structural_diagnosis(e, diagnosis));
        }
        Ok(FrameAnalysisResult {
            beam,
            model: FrameModel { inner: model },
            load_source: Some(format!("case:{}", case.name())),
        })
    }

    /// Solve the frame under a [`LoadCombination`].
    ///
    /// The right-hand side is merged: `f = Σ factor_i × f_case_i`, then
    /// `K u = f` is solved **once**. This is mathematically equivalent to
    /// solving each case separately and superposing the results (by linearity
    /// of the solve), but more efficient (a single factorisation).
    ///
    /// The result's [`load_source`](FrameAnalysisResult::load_source) is
    /// `"combination:{name}"`.
    ///
    /// # Errors
    ///
    /// Same as [`solve`](Self::solve), plus [`FemError::InvalidModel`] if a load
    /// in any case references a node or member that does not exist.
    pub fn solve_combination(
        &self,
        combo: &LoadCombination,
    ) -> Result<FrameAnalysisResult, FemError> {
        self.validate()?;
        let mut merged = self.inner.clone();
        merged.nodal_forces.clear();
        merged.distributed_loads.clear();
        merged.point_loads.clear();
        merged.applied_moments.clear();
        for (case, factor) in combo.terms() {
            for &(node, dof, value) in case.nodal_forces() {
                merged.nodal_forces.push((node, dof, factor * value));
            }
            for dl in case.distributed_loads() {
                merged.distributed_loads.push(DistributedLoad::new(
                    dl.element_idx,
                    factor * dl.qx,
                    factor * dl.qy,
                ));
            }
            for pl in case.point_loads() {
                merged.point_loads.push(PointLoad::new(
                    pl.element_idx,
                    pl.position,
                    factor * pl.fx,
                    factor * pl.fy,
                    factor * pl.mz,
                ));
            }
            for am in case.applied_moments() {
                merged
                    .applied_moments
                    .push(AppliedMoment::new(am.node_idx, factor * am.value));
            }
        }
        let mut beam = BeamSolver::from_model(&merged)?;
        if let Err(e) = beam.solve_configured() {
            let diagnosis = beam.reduced_system().map(|rs| self.classify(rs));
            return Err(with_structural_diagnosis(e, diagnosis));
        }
        Ok(FrameAnalysisResult {
            beam,
            model: FrameModel { inner: merged },
            load_source: Some(format!("combination:{}", combo.name())),
        })
    }

    /// A solver builder for this frame (configure a backend, then solve).
    pub fn solver(&self) -> FrameSolver<'_> {
        FrameSolver::new(self)
    }

    /// Validate the structural model.
    ///
    /// Checks, in order:
    ///
    /// 1. the model is not empty (at least one node and one member);
    /// 2. no **orphan nodes** - every node is used by at least one member;
    /// 3. the members form a **single connected component**.
    ///
    /// Per-element checks (zero length, duplicate connectivity, invalid
    /// material/section) are enforced when the member is added.
    ///
    /// Insufficient restraint is *not* diagnosed here: [`Self::diagnostic`]
    /// classifies the boundary-conditioned system separately, without
    /// rejecting the model, and [`FrameSolver::solve`] attaches that
    /// classification to the solver error when the solve fails.
    pub fn validate(&self) -> Result<(), FemError> {
        let inner = &self.inner;
        if inner.nodes.is_empty() {
            return Err(FemError::InvalidModel(
                "frame has no nodes; add at least one node and one member".to_string(),
            ));
        }
        if inner.elements.is_empty() {
            return Err(FemError::InvalidModel(
                "frame has no members; add at least one member".to_string(),
            ));
        }

        let mut used = vec![false; inner.nodes.len()];
        for e in &inner.elements {
            used[e.node_i] = true;
            used[e.node_j] = true;
        }
        if let Some(i) = used.iter().position(|u| !u) {
            let p = inner.nodes[i].point();
            return Err(FemError::OrphanNode(format!(
                "node {i} at ({}, {}) is not connected to any member",
                p.x, p.y
            )));
        }

        let components = count_components(inner.nodes.len(), &inner.elements);
        if components > 1 {
            return Err(FemError::DisconnectedStructure(format!(
                "frame has {components} disconnected components; a single connected \
                 structural system is required"
            )));
        }
        Ok(())
    }

    /// Diagnose the **boundary-conditioned** (reduced) system without solving
    /// it: is this structure stable, under-restrained (rigid-body mechanism),
    /// internally rank deficient (mechanism), or merely numerically
    /// ill-conditioned?
    ///
    /// The diagnosis reads the reduced free-free stiffness block `K_ff` that
    /// the **actual solve path** condenses (`BeamSolver::condense`) — the exact
    /// matrix a solve factorises, not a second assembly — and classifies it with
    /// relative, scale-invariant probes (see [`crate::mechanism`] for the
    /// criterion and its limits). It never assembles with penalties, never
    /// perturbs the matrix and never fabricates displacements, and it does not
    /// run as part of a successful [`Self::solve`].
    ///
    /// The verdict is a property of the **structure**, not of the loads: the
    /// load vector is not consulted, so scaling or omitting the loads cannot
    /// change the result.
    ///
    /// # Errors
    ///
    /// The same model errors as [`Self::solve`]: [`Self::validate`] plus the
    /// construction checks [`BeamSolver::from_model`] performs (its error
    /// surface is slightly wider than `validate` alone).
    pub fn diagnostic(&self) -> Result<StructuralDiagnostic, FemError> {
        self.validate()?;
        let mut beam = BeamSolver::from_model(&self.inner)?;
        Ok(self.classify(beam.condense()))
    }

    /// Classify a reduced (boundary-conditioned) stiffness system using this
    /// model's geometry: the global rigid-body motions are built in *reduced*
    /// order from the solver's own free-DOF map, so no independent reassembly
    /// of `K_ff` is involved.
    fn classify(&self, reduced: &ReducedSystem) -> StructuralDiagnostic {
        let rigid = self.rigid_candidates(reduced.free_to_global());
        diagnose_reduced(reduced.k_ff(), &rigid)
    }

    /// The three global rigid-body motions (Tx, Ty, Rz) restricted to the free
    /// DOFs, laid out in reduced order.
    ///
    /// `free_to_global[r]` is the global DOF (`3 * node + dof`) of reduced index
    /// `r`; constrained entries are dropped, not displaced. Rotation by `theta`
    /// about the global origin is `ux = -theta*y`, `uy = theta*x`, `rz = theta`.
    fn rigid_candidates(&self, free_to_global: &[usize]) -> Vec<Vec<f64>> {
        let n_free = free_to_global.len();
        let mut tx = vec![0.0; n_free];
        let mut ty = vec![0.0; n_free];
        let mut rz = vec![0.0; n_free];
        for (r, &global) in free_to_global.iter().enumerate() {
            let point = self.inner.nodes[global / 3].point();
            match global % 3 {
                0 => {
                    tx[r] = 1.0;
                    rz[r] = -point.y;
                }
                1 => {
                    ty[r] = 1.0;
                    rz[r] = point.x;
                }
                _ => rz[r] = 1.0,
            }
        }
        vec![tx, ty, rz]
    }

    /// Handle of the node with the given index.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if the index is out of range.
    pub fn node_handle(&self, index: usize) -> Result<NodeHandle, FemError> {
        if index >= self.inner.nodes.len() {
            return Err(FemError::InvalidNode(format!(
                "node index {index} is out of range (max: {})",
                self.inner.nodes.len().saturating_sub(1)
            )));
        }
        Ok(NodeHandle(index))
    }

    /// Handle of the member with the given index.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidMember`] if the index is out of range.
    pub fn member_handle(&self, index: usize) -> Result<MemberHandle, FemError> {
        if index >= self.inner.elements.len() {
            return Err(FemError::InvalidMember(format!(
                "member index {index} is out of range (max: {})",
                self.inner.elements.len().saturating_sub(1)
            )));
        }
        Ok(MemberHandle(index))
    }

    fn check_node(&self, node: NodeHandle) -> Result<usize, FemError> {
        if node.0 >= self.inner.nodes.len() {
            return Err(FemError::InvalidNode(format!(
                "node handle {} is not valid for this frame (max: {})",
                node.0,
                self.inner.nodes.len().saturating_sub(1)
            )));
        }
        Ok(node.0)
    }

    fn check_member(&self, member: MemberHandle) -> Result<usize, FemError> {
        if member.0 >= self.inner.elements.len() {
            return Err(FemError::InvalidMember(format!(
                "member handle {} is not valid for this frame (max: {})",
                member.0,
                self.inner.elements.len().saturating_sub(1)
            )));
        }
        Ok(member.0)
    }

    /// Clone the inner BeamModel, replacing its four load collections with the
    /// provided loads. Geometry, elements, constraints and end releases are
    /// preserved. This is the bridge between [`LoadCase`] and [`BeamSolver::from_model`]:
    /// the solver's stored model must carry the correct loads for end-force
    /// recovery (`f_end = f_equiv − K_e u_e`).
    fn inner_with_loads(
        &self,
        nodal_forces: &[(usize, usize, f64)],
        distributed_loads: &[DistributedLoad],
        point_loads: &[PointLoad],
        applied_moments: &[AppliedMoment],
    ) -> BeamModel {
        let mut m = self.inner.clone();
        m.nodal_forces = nodal_forces.to_vec();
        m.distributed_loads = distributed_loads.to_vec();
        m.point_loads = point_loads.to_vec();
        m.applied_moments = applied_moments.to_vec();
        m
    }
}

/// Number of connected components of the member connectivity graph.
fn count_components(n_nodes: usize, elements: &[BeamElement]) -> usize {
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n_nodes];
    for e in elements {
        adj[e.node_i].push(e.node_j);
        adj[e.node_j].push(e.node_i);
    }
    let mut seen = vec![false; n_nodes];
    let mut components = 0;
    for start in 0..n_nodes {
        if seen[start] || adj[start].is_empty() {
            continue;
        }
        components += 1;
        seen[start] = true;
        let mut queue = VecDeque::from([start]);
        while let Some(u) = queue.pop_front() {
            for &v in &adj[u] {
                if !seen[v] {
                    seen[v] = true;
                    queue.push_back(v);
                }
            }
        }
    }
    components
}

// ---------------------------------------------------------------------------
// FrameSolver
// ---------------------------------------------------------------------------

/// Solves a [`FrameModel`]; configure the backend, then [`FrameSolver::solve`].
pub struct FrameSolver<'a> {
    model: &'a FrameModel,
    selection: SolverSelection,
}

impl<'a> FrameSolver<'a> {
    /// Create a solver for `model` using automatic backend selection.
    pub fn new(model: &'a FrameModel) -> Self {
        Self {
            model,
            selection: SolverSelection::Auto,
        }
    }

    /// Choose an explicit backend (or [`SolverSelection::Auto`]).
    pub fn with_selection(mut self, selection: SolverSelection) -> Self {
        self.selection = selection;
        self
    }

    /// Set the backend (builder-style alternative to [`Self::with_selection`]).
    pub fn set_solver(&mut self, selection: SolverSelection) {
        self.selection = selection;
    }

    /// Validate, assemble and solve the frame.
    ///
    /// A successful solve runs **no** diagnostic and its numerical path is
    /// unchanged (see [`FrameModel::diagnostic`] for the on-demand API). Only a
    /// *failed* solve additionally classifies the reduced system, and appends
    /// that classification to the error message so an under-restrained or
    /// rank-deficient frame is not reported as an opaque "singular matrix".
    ///
    /// # Errors
    ///
    /// Model-structure errors from [`FrameModel::validate`] plus any
    /// [`FemError::SolverError`] from the linear solver (e.g. insufficient
    /// restraint, or a conditioning limit at extreme scales). The variant is
    /// unchanged; the message carries the diagnosis.
    pub fn solve(&self) -> Result<FrameAnalysisResult, FemError> {
        self.model.validate()?;
        let mut beam = BeamSolver::from_model(&self.model.inner)?;
        beam.set_solver(self.selection.clone());
        if let Err(e) = beam.solve_configured() {
            // The solver condensed the boundary conditions before attempting any
            // backend, so its retained reduced system is the exact matrix the
            // failed factorisation saw - no second assembly.
            let diagnosis = beam.reduced_system().map(|rs| self.model.classify(rs));
            return Err(with_structural_diagnosis(e, diagnosis));
        }
        Ok(FrameAnalysisResult {
            beam,
            model: self.model.clone(),
            load_source: None,
        })
    }
}

/// Record the structural diagnosis of a failed solve in the error message.
///
/// The error **variant** is deliberately preserved (`SolverError` stays
/// `SolverError`): the diagnosis is extra information, not a re-typing of the
/// failure, so no existing caller or exhaustive match changes meaning. A
/// `Stable` diagnosis adds nothing - the failure was not structural - and a
/// missing diagnosis (`None`, i.e. no reduced system was available) leaves the
/// error untouched.
fn with_structural_diagnosis(err: FemError, diagnosis: Option<StructuralDiagnostic>) -> FemError {
    match (err, diagnosis) {
        (FemError::SolverError { source, message }, Some(d))
            if !matches!(d, StructuralDiagnostic::Stable) =>
        {
            FemError::SolverError {
                source,
                message: format!("{message}; structural diagnosis: {d}"),
            }
        }
        (other, _) => other,
    }
}

// ---------------------------------------------------------------------------
// FrameAnalysisResult
// ---------------------------------------------------------------------------

/// Solved frame: frame-level access to displacements, reactions, member end
/// forces and global equilibrium.
///
/// All values come from the existing [`BeamSolver`]; nothing is recomputed with
/// a second formulation.
pub struct FrameAnalysisResult {
    beam: BeamSolver,
    model: FrameModel,
    load_source: Option<String>,
}

impl std::fmt::Debug for FrameAnalysisResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FrameAnalysisResult")
            .field("nodes", &self.model.n_nodes())
            .field("members", &self.model.n_members())
            .field("solver", &self.beam.solver_name())
            .field("load_source", &self.load_source)
            .finish_non_exhaustive()
    }
}

impl FrameAnalysisResult {
    /// Name of the backend used by the successful solve.
    pub fn solver_name(&self) -> Option<&str> {
        self.beam.solver_name()
    }

    /// Provenance of the load that produced this result.
    ///
    /// `None` for a direct [`FrameModel::solve`] / [`FrameModel::solve_with`];
    /// `"case:{name}"` for [`FrameModel::solve_case`];
    /// `"combination:{name}"` for [`FrameModel::solve_combination`].
    pub fn load_source(&self) -> Option<&str> {
        self.load_source.as_deref()
    }

    /// Number of nodes in the solved frame.
    pub fn n_nodes(&self) -> usize {
        self.model.n_nodes()
    }

    /// Number of members in the solved frame.
    pub fn n_members(&self) -> usize {
        self.model.n_members()
    }

    /// Full global displacement vector, laid out `[ux, uy, rz]` per node.
    pub fn displacements(&self) -> &[f64] {
        self.beam.displacements()
    }

    /// Global support reactions, laid out `[Rx, Ry, Mz]` per node.
    pub fn reactions(&self) -> Vec<f64> {
        self.beam.reactions()
    }

    /// Displacement of a DOF at a node (global).
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if the handle is not valid for this frame.
    pub fn displacement(&self, node: NodeHandle, dof: Dof) -> Result<f64, FemError> {
        self.check_node(node)?;
        self.beam.displacement_dof(node.index(), dof)
    }

    /// Support reaction of a DOF at a node (global).
    ///
    /// Free DOFs report the raw (round-off) residual, exactly as
    /// [`BeamSolver::reaction_dof`] does.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidNode`] if the handle is not valid for this frame.
    pub fn reaction(&self, node: NodeHandle, dof: Dof) -> Result<f64, FemError> {
        self.check_node(node)?;
        self.beam.reaction_dof(node.index(), dof)
    }

    /// Member end forces in **local** axes:
    /// `[N_i, V_i, M_i, N_j, V_j, M_j]`.
    ///
    /// These are element-on-node forces, `f_end = f_equiv - K_e u_e`, exactly as
    /// documented in `docs/beam_fem.md`; the sign convention is **not** inverted
    /// for the frame API.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidMember`] if the handle is not valid for this frame.
    pub fn member_end_forces(&self, member: MemberHandle) -> Result<[f64; 6], FemError> {
        let i = self.check_member(member)?;
        let all = self.beam.element_end_forces()?;
        all.get(i)
            .copied()
            .ok_or_else(|| FemError::InvalidMember(format!("member handle {} is out of range", i)))
    }

    /// Member end forces transformed to **global** axes
    /// (`f_global = Tᵀ f_local`).
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidMember`] if the handle is not valid for this frame.
    pub fn member_end_forces_global(&self, member: MemberHandle) -> Result<[f64; 6], FemError> {
        let i = self.check_member(member)?;
        let all = self.beam.element_end_forces_global()?;
        all.get(i)
            .copied()
            .ok_or_else(|| FemError::InvalidMember(format!("member handle {} is out of range", i)))
    }

    /// Section internal forces `N`, `V`, `M` (LOCAL) at `(member, xi)`.
    ///
    /// Delegates to [`BeamSolver::element_section_forces`]. For end-released
    /// members, the released end moment is exactly zero.
    ///
    /// # Errors
    ///
    /// [`FemError::InvalidMember`] if the handle is not valid.
    /// [`FemError::InvalidInput`] if `xi` is not in `[0, 1]`.
    pub fn section_forces(
        &self,
        member: MemberHandle,
        xi: f64,
    ) -> Result<crate::beam_fem::SectionForces, FemError> {
        let i = self.check_member(member)?;
        self.beam.element_section_forces(i, xi)
    }

    /// Global equilibrium check about the origin `(0, 0)`.
    ///
    /// Combines support reactions, global nodal loads and moments, and the
    /// global resultants of member distributed and point loads (transformed from
    /// member-local axes and applied at their true global locations).
    /// See [`EquilibriumReport`].
    pub fn equilibrium(&self) -> EquilibriumReport {
        let m = &self.model.inner;
        let mut applied_fx = 0.0;
        let mut applied_fy = 0.0;
        let mut applied_mz = 0.0;
        // Absolute per-term magnitudes (not resultants): a load pair that cancels
        // in the sum must still contribute to the equilibrium scale.
        let mut applied_f_mag = 0.0;
        let mut applied_m_mag = 0.0;

        // Global nodal loads and moments.
        for (node, dof, v) in &m.nodal_forces {
            let p = m.nodes[*node].point();
            match *dof {
                0 => {
                    applied_fx += v;
                    applied_mz += -p.y * v;
                    applied_f_mag += v.abs();
                }
                1 => {
                    applied_fy += v;
                    applied_mz += p.x * v;
                    applied_f_mag += v.abs();
                }
                _ => {
                    applied_mz += v;
                    applied_m_mag += v.abs();
                }
            }
        }
        for am in &m.applied_moments {
            applied_mz += am.value;
            applied_m_mag += am.value.abs();
        }

        // Member load resultants, rotated from local to global.
        for (elem_idx, dl) in m.distributed_loads.iter().enumerate() {
            let _ = elem_idx;
            let Some((pi, pj)) = member_points(m, dl.element_idx) else {
                continue;
            };
            let (dx, dy) = (pj.x - pi.x, pj.y - pi.y);
            let len = (dx * dx + dy * dy).sqrt();
            let (c, s) = (dx / len, dy / len);
            let (lfx, lfy) = (dl.qx * len, dl.qy * len);
            let (gx, gy) = (c * lfx - s * lfy, s * lfx + c * lfy);
            let (xm, ym) = (pi.x + 0.5 * dx, pi.y + 0.5 * dy);
            applied_fx += gx;
            applied_fy += gy;
            applied_mz += xm * gy - ym * gx;
            applied_f_mag += gx.abs() + gy.abs();
        }
        for pl in &m.point_loads {
            let Some((pi, _)) = member_points(m, pl.element_idx) else {
                continue;
            };
            let el = &m.elements[pl.element_idx];
            let pj = m.nodes[el.node_j].point();
            let (dx, dy) = (pj.x - pi.x, pj.y - pi.y);
            let len = (dx * dx + dy * dy).sqrt();
            let (c, s) = (dx / len, dy / len);
            let (gx, gy) = (c * pl.fx - s * pl.fy, s * pl.fx + c * pl.fy);
            let (xp, yp) = (pi.x + pl.position * dx, pi.y + pl.position * dy);
            applied_fx += gx;
            applied_fy += gy;
            applied_mz += xp * gy - yp * gx + pl.mz;
            applied_f_mag += gx.abs() + gy.abs();
            applied_m_mag += pl.mz.abs();
        }

        // Support reactions, per node.
        let reactions = self.beam.reactions();
        let mut reaction_fx = 0.0;
        let mut reaction_fy = 0.0;
        let mut reaction_mz = 0.0;
        let mut reaction_f_mag = 0.0;
        let mut reaction_m_mag = 0.0;
        for (idx, node) in m.nodes.iter().enumerate() {
            let p = node.point();
            let rx = reactions.get(3 * idx).copied().unwrap_or(0.0);
            let ry = reactions.get(3 * idx + 1).copied().unwrap_or(0.0);
            let rz = reactions.get(3 * idx + 2).copied().unwrap_or(0.0);
            reaction_fx += rx;
            reaction_fy += ry;
            reaction_mz += p.x * ry - p.y * rx + rz;
            reaction_f_mag += rx.abs() + ry.abs();
            reaction_m_mag += rz.abs();
        }

        // Characteristic length of the structure: the largest absolute nodal
        // coordinate. Positive for any solvable model (a zero-length member is
        // rejected). It is what makes the moment scale `Σ|F|·l_char` reflect the
        // structure rather than a constant.
        let l_char = characteristic_length(m);
        let f_mag = applied_f_mag + reaction_f_mag;
        let m_mag = applied_m_mag + reaction_m_mag;
        let cond_rel_floor = EQUILIBRIUM_COND_FACTOR * max_element_slenderness_sq(m) * f64::EPSILON;
        let rel = EQUILIBRIUM_REL_TOL.max(cond_rel_floor);
        let (f_scale, _m_scale) = equilibrium_scales(f_mag, m_mag, l_char);

        EquilibriumReport {
            fx_residual: applied_fx + reaction_fx,
            fy_residual: applied_fy + reaction_fy,
            mz_residual: applied_mz + reaction_mz,
            applied_fx,
            applied_fy,
            applied_mz,
            reaction_fx,
            reaction_fy,
            reaction_mz,
            tolerance: rel * f_scale,
            f_mag,
            m_mag,
            l_char,
            cond_rel_floor,
        }
    }

    fn check_node(&self, node: NodeHandle) -> Result<(), FemError> {
        if node.index() >= self.model.n_nodes() {
            return Err(FemError::InvalidNode(format!(
                "node handle {} is not valid for this result (max: {})",
                node.index(),
                self.model.n_nodes().saturating_sub(1)
            )));
        }
        Ok(())
    }

    fn check_member(&self, member: MemberHandle) -> Result<usize, FemError> {
        if member.index() >= self.model.n_members() {
            return Err(FemError::InvalidMember(format!(
                "member handle {} is not valid for this result (max: {})",
                member.index(),
                self.model.n_members().saturating_sub(1)
            )));
        }
        Ok(member.index())
    }
}

/// End-point coordinates of a member, or `None` if the index is out of range.
fn member_points(
    model: &BeamModel,
    member: usize,
) -> Option<(
    section_properties::geometry::Point,
    section_properties::geometry::Point,
)> {
    let el = model.elements.get(member)?;
    Some((
        model.nodes[el.node_i].point(),
        model.nodes[el.node_j].point(),
    ))
}

/// Characteristic length of the model: the largest absolute nodal coordinate.
///
/// The equilibrium moments are summed **about the origin**, so the length that
/// relates the force and moment scales must be measured from the origin too —
/// not a translation-invariant span. `0.0` for an empty model.
fn characteristic_length(model: &BeamModel) -> f64 {
    let mut l_char: f64 = 0.0;
    for node in &model.nodes {
        let p = node.point();
        l_char = l_char.max(p.x.abs()).max(p.y.abs());
    }
    l_char
}

/// Maximum element slenderness squared `λ² = L²·A/I` over all members.
///
/// This is a dimensionless element-level conditioning proxy for an
/// Euler–Bernoulli stiffness matrix: the forward error of a direct solve is
/// `O(λ²·ε)`, and the recovered reactions inherit that error, so equilibrium
/// residuals of a physically balanced frame can legitimately reach the same
/// relative magnitude. It is not a strict bound on the full assembled frame
/// stiffness matrix condition number.
///
/// Only elements with `I > 0` contribute to the maximum; elements with
/// non-positive `second_moment` are skipped (defensive guard, since
/// `BeamSection::new` does not validate `I`). Returns `0.0` for an empty model
/// or when no element has `I > 0`.
///
/// **Dimensionless and unit-invariant**: `L²·A/I` has units `m²·m²/m⁴ = 1`, and
/// a consistent change of length/area/inertia units multiplies numerator and
/// denominator by the same power, leaving the ratio unchanged. This is what
/// lets the conditioning floor feed a *relative* tolerance without breaking
/// unit invariance of the verdict.
fn max_element_slenderness_sq(model: &BeamModel) -> f64 {
    let mut lambda_sq: f64 = 0.0;
    for el in &model.elements {
        let p_i = model.nodes[el.node_i].point();
        let p_j = model.nodes[el.node_j].point();
        let l = el.length(p_i, p_j);
        let i = el.section.second_moment;
        if i > 0.0 {
            lambda_sq = lambda_sq.max(l * l * el.section.area / i);
        }
    }
    lambda_sq
}

// ---------------------------------------------------------------------------
// Single-source guarantee: the diagnostic reads the solver's condensed system
// ---------------------------------------------------------------------------
//
// These are in-crate tests on purpose: `tests/*.rs` are integration tests and
// cannot see `pub(crate)` items such as `BeamSolver::condense` /
// `FrameModel::classify`, so this guarantee is not testable from there.
#[cfg(test)]
mod reduced_system_single_source_tests {
    use super::*;
    use section_properties::fea::SparseMatrix;

    fn steel() -> Material {
        Material::new(200e9, 0.3, 7850.0, "Steel")
    }

    fn sec() -> BeamSection {
        BeamSection::new(5e-3, 2e-5)
    }

    /// The diagnostic must classify exactly the system the solver's own
    /// condensation produced: reading the live solver and the public API must
    /// agree on every structure in the battery.
    fn assert_agrees(f: &FrameModel, label: &str) -> Result<(), FemError> {
        let mut beam = BeamSolver::from_model(&f.inner)?;
        let from_solver = f.classify(beam.condense());
        assert_eq!(
            from_solver,
            f.diagnostic()?,
            "{label}: diagnostic disagrees with the solver's condensed system"
        );
        Ok(())
    }

    /// Two-column portal: `base1 - top1 - top2 - base2`, with the given supports
    /// on the two base nodes.
    fn portal(base_1: &[Dof], base_2: &[Dof]) -> Result<FrameModel, FemError> {
        let mut f = FrameModel::new();
        let b1 = f.add_node(0.0, 0.0)?;
        let t1 = f.add_node(0.0, 3.0)?;
        let t2 = f.add_node(4.0, 3.0)?;
        let b2 = f.add_node(4.0, 0.0)?;
        f.add_member(b1, t1, steel(), sec())?;
        f.add_member(t1, t2, steel(), sec())?;
        f.add_member(b2, t2, steel(), sec())?;
        for dof in base_1 {
            f.restrain(b1, *dof, 0.0)?;
        }
        for dof in base_2 {
            f.restrain(b2, *dof, 0.0)?;
        }
        Ok(f)
    }

    #[test]
    fn diagnostic_classifies_the_solver_condensed_system() -> Result<(), FemError> {
        // Completely free beam - three rigid-body modes.
        let mut free = FrameModel::new();
        let a = free.add_node(0.0, 0.0)?;
        let b = free.add_node(2.0, 0.0)?;
        free.add_member(a, b, steel(), sec())?;
        assert_agrees(&free, "free beam")?;

        // One DOF restrained - still under-constrained.
        let mut one = FrameModel::new();
        let a = one.add_node(0.0, 0.0)?;
        let b = one.add_node(2.0, 0.0)?;
        one.add_member(a, b, steel(), sec())?;
        one.restrain(a, Dof::Ux, 0.0)?;
        assert_agrees(&one, "one-DOF-restrained beam")?;

        // Stable cantilever.
        let mut cant = FrameModel::new();
        let base = cant.add_node(0.0, 0.0)?;
        let tip = cant.add_node(2.0, 0.0)?;
        cant.add_member(base, tip, steel(), sec())?;
        cant.fix(base)?;
        assert_agrees(&cant, "stable cantilever")?;

        // Portal with several support sets, stable and deficient.
        let fixed = portal(&[Dof::Ux, Dof::Uy, Dof::Rz], &[Dof::Ux, Dof::Uy, Dof::Rz])?;
        assert_agrees(&fixed, "portal, both bases fixed")?;
        let pinned = portal(&[Dof::Ux, Dof::Uy], &[Dof::Ux, Dof::Uy])?;
        assert_agrees(&pinned, "portal, both bases pinned")?;
        let roller = portal(&[Dof::Ux, Dof::Uy, Dof::Rz], &[Dof::Uy])?;
        assert_agrees(&roller, "portal, fixed base + roller")?;
        let mechanism = portal(&[Dof::Ux, Dof::Uy], &[])?;
        assert_agrees(&mechanism, "portal, single pin (mechanism)")?;

        Ok(())
    }

    #[test]
    fn diagnostic_uses_the_supplied_matrix_not_a_copy() -> Result<(), FemError> {
        let mut f = FrameModel::new();
        let base = f.add_node(0.0, 0.0)?;
        let tip = f.add_node(2.0, 0.0)?;
        f.add_member(base, tip, steel(), sec())?;
        f.fix(base)?;

        let mut beam = BeamSolver::from_model(&f.inner)?;
        let real = beam.condense().clone();
        let stable = f.classify(&real);
        assert_eq!(
            stable,
            StructuralDiagnostic::Stable,
            "control must be stable"
        );

        // Forge a reduced system with the first free DOF (tip `ux`) removed. If
        // `classify` reassembled its own matrix instead of reading its argument,
        // this forgery would have no effect at all.
        let n = real.k_ff().n;
        let mut forged_k = SparseMatrix::new(n);
        for i in 0..n {
            for j in 0..n {
                let v = real.k_ff().get(i, j);
                if i != 0 && j != 0 && v != 0.0 {
                    forged_k.add(i, j, v);
                }
            }
        }
        forged_k.compress();
        let forged = ReducedSystem::new(forged_k, real.free_to_global().to_vec());

        let verdict = f.classify(&forged);
        assert_ne!(
            verdict, stable,
            "classify must read the supplied matrix: zeroing a free DOF changed nothing"
        );
        // The public diagnostic still reads the untouched, real system.
        assert_eq!(f.diagnostic()?, stable);
        Ok(())
    }

    /// The failed-solve diagnostic must be driven by the solver's **retained**
    /// `K_ff`, not by a reassembly: the frame solve's error carries exactly the
    /// verdict that classifying the retained system produces.
    #[test]
    fn failed_solve_diagnosis_reads_the_solver_retained_system() -> Result<(), FemError> {
        // A free beam has the three rigid-body modes and cannot be solved.
        let mut f = FrameModel::new();
        let a = f.add_node(0.0, 0.0)?;
        let b = f.add_node(2.0, 0.0)?;
        f.add_member(a, b, steel(), sec())?;

        let expected = StructuralDiagnostic::RigidBodyMode {
            n_free: 6,
            rank: 3,
            rigid_modes: 3,
        };

        // (a) the frame solve fails as a `SolverError` that carries the verdict.
        let err = f.solve().expect_err("a free beam must not solve");
        match &err {
            FemError::SolverError { message, .. } => assert!(
                message.contains("structural diagnosis") && message.contains(&expected.to_string()),
                "expected the retained-system diagnosis in the error, got {message:?}"
            ),
            other => panic!("expected SolverError, got {other:?}"),
        }

        // (b) the verdict is exactly what the solver's retained reduced system
        // classifies to: run the identical solve path on a `BeamSolver` built the
        // same way, and classify the matrix that failed solve retained.
        let mut beam = BeamSolver::from_model(&f.inner)?;
        assert!(
            beam.solve_configured().is_err(),
            "the identical solve must also fail"
        );
        let retained = beam
            .reduced_system()
            .expect("a failed solve still retains the condensed system it factorised");
        assert_eq!(
            f.classify(retained),
            expected,
            "the diagnosis must come from the solver's retained K_ff"
        );
        Ok(())
    }

    /// A diagnosis that adds nothing (`Stable`) or is absent leaves the solver
    /// error byte-for-byte unchanged; only a mechanism annotates it. This covers
    /// the "failure before condensation leaves the error untouched" case: a
    /// missing diagnosis (`None`) is a no-op.
    #[test]
    fn diagnosis_only_annotates_mechanism_errors() {
        use section_properties::fea::solver::SolverError;
        let err = || FemError::from(SolverError::SingularMatrix("singular matrix".to_string()));
        let untouched = err().to_string();
        assert_eq!(
            with_structural_diagnosis(err(), None).to_string(),
            untouched
        );
        assert_eq!(
            with_structural_diagnosis(err(), Some(StructuralDiagnostic::Stable)).to_string(),
            untouched
        );
        let mechanism = StructuralDiagnostic::Mechanism { n_free: 4, rank: 3 };
        let annotated = with_structural_diagnosis(err(), Some(mechanism));
        assert!(
            matches!(annotated, FemError::SolverError { .. }),
            "the error variant must be preserved"
        );
        assert_eq!(
            annotated.to_string(),
            format!("{untouched}; structural diagnosis: {mechanism}")
        );
    }

    /// A model with **every** DOF restrained condenses to a 0x0 reduced system.
    /// The verdict is `Stable` (no free DOF means no mechanism) and the probe
    /// must not panic on the empty matrix / zero scale.
    #[test]
    fn fully_restrained_model_is_stable_and_does_not_panic() -> Result<(), FemError> {
        let mut f = FrameModel::new();
        let a = f.add_node(0.0, 0.0)?;
        let b = f.add_node(2.0, 0.0)?;
        f.add_member(a, b, steel(), sec())?;
        f.fix(a)?;
        f.fix(b)?;

        let mut beam = BeamSolver::from_model(&f.inner)?;
        let reduced = beam.condense();
        assert_eq!(reduced.k_ff().n, 0, "every DOF is restrained");
        assert!(reduced.free_to_global().is_empty());
        assert_eq!(f.classify(reduced), StructuralDiagnostic::Stable);

        // The public diagnostic agrees, and the fully prescribed solve is a
        // clean success - no panic, no mechanism claim.
        assert_eq!(f.diagnostic()?, StructuralDiagnostic::Stable);
        let result = f.solve()?;
        assert_eq!(result.displacement(a, Dof::Ux)?, 0.0);
        assert_eq!(result.displacement(b, Dof::Rz)?, 0.0);
        Ok(())
    }

    /// `condense()` is idempotent: repeated calls return the already-retained
    /// system (the same object, no re-condensation). `BeamSolver` has no mutator
    /// that can change the assembled system, so the cache cannot go stale.
    #[test]
    fn condense_is_idempotent_and_retained() -> Result<(), FemError> {
        let mut f = FrameModel::new();
        let base = f.add_node(0.0, 0.0)?;
        let tip = f.add_node(2.0, 0.0)?;
        f.add_member(base, tip, steel(), sec())?;
        f.fix(base)?;

        let mut beam = BeamSolver::from_model(&f.inner)?;
        assert!(beam.reduced_system().is_none(), "nothing condensed yet");

        let first = beam.condense() as *const ReducedSystem;
        let second = beam.condense() as *const ReducedSystem;
        assert_eq!(
            first, second,
            "repeated condense() must return the retained system, not rebuild it"
        );

        // A solve refreshes the retained system in place; after it, `condense()`
        // returns that retained object rather than re-condensing.
        beam.solve_configured()?;
        let after_k = beam
            .reduced_system()
            .expect("retained after a successful solve")
            .k_ff()
            .clone();
        let after_ptr = beam.reduced_system().unwrap() as *const ReducedSystem;
        assert_eq!(after_k.n, 3, "cantilever tip has 3 free DOFs");
        assert_eq!(
            beam.condense() as *const ReducedSystem,
            after_ptr,
            "condense() after a solve still returns the retained system"
        );

        // `set_solver` only chooses a backend; it cannot invalidate `K_ff`. The
        // retained content is unchanged across it.
        beam.set_solver(SolverSelection::Auto);
        let again = beam.condense();
        assert_eq!(again.k_ff().n, after_k.n);
        for i in 0..after_k.n {
            for j in 0..after_k.n {
                assert_eq!(
                    again.k_ff().get(i, j),
                    after_k.get(i, j),
                    "K_ff entry ({i},{j}) changed without a model change"
                );
            }
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Equilibrium tolerance: relative, scale/unit invariant, never absolute
// ---------------------------------------------------------------------------
#[cfg(test)]
mod equilibrium_tolerance_tests {
    use super::*;

    /// Build a report with the given residuals and raw magnitudes. The
    /// conditioning floor is left at `0.0`, so the effective tolerance is
    /// exactly `EQUILIBRIUM_REL_TOL`: these tests exercise the scale/unit
    /// behaviour, not the conditioning relief (which has its own module below).
    fn report(
        fx_residual: f64,
        fy_residual: f64,
        mz_residual: f64,
        f_mag: f64,
        m_mag: f64,
        l_char: f64,
    ) -> EquilibriumReport {
        let (f_scale, _) = equilibrium_scales(f_mag, m_mag, l_char);
        EquilibriumReport {
            fx_residual,
            fy_residual,
            mz_residual,
            applied_fx: 0.0,
            applied_fy: 0.0,
            applied_mz: 0.0,
            reaction_fx: 0.0,
            reaction_fy: 0.0,
            reaction_mz: 0.0,
            tolerance: EQUILIBRIUM_REL_TOL * f_scale,
            f_mag,
            m_mag,
            l_char,
            cond_rel_floor: 0.0,
        }
    }

    #[test]
    fn force_and_moment_scales_are_unit_invariant() {
        // Same physical state: 2e6 N of force, 5e2 m lever arm, zero net moment.
        let (f_si, m_si) = equilibrium_scales(2.0e6, 0.0, 5.0e2);
        assert_eq!(f_si, 2.0e6);
        assert_eq!(m_si, 1.0e9);
        // kN / mm: force 2e3 kN, lever arm 5e5 mm.
        let (f_kmm, m_kmm) = equilibrium_scales(2.0e3, 0.0, 5.0e5);
        assert_eq!(f_kmm, 2.0e3);
        // 1e9 kN·mm == 1e9 N·m: the moment scale is the same physical quantity.
        assert_eq!(m_kmm, m_si);
    }

    #[test]
    fn moment_scale_is_non_degenerate_with_zero_applied_moment() {
        // Forces present but `applied_mz == 0`: the moment scale must not
        // collapse to a constant (the old `|applied_mz|`-only definition did).
        let (_, m_scale) = equilibrium_scales(2.0e6, 0.0, 5.0e2);
        assert_eq!(m_scale, 2.0e6 * 5.0e2);
        assert!(m_scale > 0.0);
    }

    #[test]
    fn scales_vanish_only_when_nothing_is_present() {
        assert_eq!(equilibrium_scales(0.0, 0.0, 5.0e2), (0.0, 0.0));
        assert_eq!(equilibrium_scales(0.0, 0.0, 0.0), (0.0, 0.0));
    }

    #[test]
    fn unloaded_report_is_balanced_only_with_exact_zero_residual() {
        assert!(report(0.0, 0.0, 0.0, 0.0, 0.0, 5.0e2).is_balanced());
        assert!(!report(0.0, 0.0, f64::MIN_POSITIVE, 0.0, 0.0, 5.0e2).is_balanced());
        assert!(!report(f64::MIN_POSITIVE, 0.0, 0.0, 0.0, 0.0, 5.0e2).is_balanced());
    }

    #[test]
    fn corrupted_reaction_is_still_rejected() {
        // No over-relaxation: the relative bound must still reject a genuine
        // imbalance, while accepting round-off.
        let (f_scale, m_scale) = equilibrium_scales(1.0e6, 0.0, 5.0e2);
        assert!(
            report(
                1e-15 * f_scale,
                1e-15 * f_scale,
                1e-15 * m_scale,
                1.0e6,
                0.0,
                5.0e2
            )
            .is_balanced()
        );
        assert!(
            !report(
                1e-3 * f_scale,
                1e-3 * f_scale,
                1e-3 * m_scale,
                1.0e6,
                0.0,
                5.0e2
            )
            .is_balanced()
        );
    }

    /// Build a report with an explicit conditioning floor, so the anti-masking
    /// test can exercise `cond_rel_floor > 0` directly.
    fn report_with_cond(
        fx_residual: f64,
        fy_residual: f64,
        mz_residual: f64,
        f_mag: f64,
        m_mag: f64,
        l_char: f64,
        cond_rel_floor: f64,
    ) -> EquilibriumReport {
        let (f_scale, _) = equilibrium_scales(f_mag, m_mag, l_char);
        let rel = EQUILIBRIUM_REL_TOL.max(cond_rel_floor);
        EquilibriumReport {
            fx_residual,
            fy_residual,
            mz_residual,
            applied_fx: 0.0,
            applied_fy: 0.0,
            applied_mz: 0.0,
            reaction_fx: 0.0,
            reaction_fy: 0.0,
            reaction_mz: 0.0,
            tolerance: rel * f_scale,
            f_mag,
            m_mag,
            l_char,
            cond_rel_floor,
        }
    }

    #[test]
    fn conditioning_floor_accepts_roundoff_rejects_genuine_imbalance() {
        // A slender frame: λ² ≈ 2.5e14, so the conditioning floor dominates.
        // cond_floor = 1e-2 · 2.5e14 · ε ≈ 5.55e-4.
        let lambda_sq = 2.5e14;
        let cond_floor = EQUILIBRIUM_COND_FACTOR * lambda_sq * f64::EPSILON;
        assert!(
            cond_floor > EQUILIBRIUM_REL_TOL,
            "this test needs a floor above the base tolerance"
        );

        let (f_scale, m_scale) = equilibrium_scales(1.0e12, 0.0, 5.0e5);

        // (a) Residual at 10% of the conditioning floor: pure round-off,
        //     must be accepted.
        assert!(
            report_with_cond(
                0.1 * cond_floor * f_scale,
                0.1 * cond_floor * f_scale,
                0.1 * cond_floor * m_scale,
                1.0e12,
                0.0,
                5.0e5,
                cond_floor
            )
            .is_balanced(),
            "round-off below the conditioning floor must be accepted"
        );

        // (b) Residual at 10× the conditioning floor: a genuine imbalance,
        //     must be rejected — the floor must not mask it.
        assert!(
            !report_with_cond(
                10.0 * cond_floor * f_scale,
                10.0 * cond_floor * f_scale,
                10.0 * cond_floor * m_scale,
                1.0e12,
                0.0,
                5.0e5,
                cond_floor
            )
            .is_balanced(),
            "imbalance above the conditioning floor must be rejected"
        );
    }

    #[test]
    fn conditioning_floor_is_dimensionless_and_unit_invariant() {
        // λ² = L²·A/I is dimensionless: the same physical element expressed in
        // SI (m, m², m⁴) and kN/mm (mm, mm², mm⁴) gives the same λ².
        let l_si = 1.0e6_f64;
        let a_si = 5.0e-3_f64;
        let i_si = 2.0e-5_f64;
        let lambda_sq_si = l_si * l_si * a_si / i_si;

        let l_kmm = 1.0e9_f64; // 1e6 m = 1e9 mm
        let a_kmm = 5.0e3_f64; // 5e-3 m² = 5e3 mm²
        let i_kmm = 2.0e7_f64; // 2e-5 m⁴ = 2e7 mm⁴
        let lambda_sq_kmm = l_kmm * l_kmm * a_kmm / i_kmm;

        // The two values differ only by round-off in the multiplications.
        let avg = 0.5 * (lambda_sq_si + lambda_sq_kmm);
        assert!(
            (lambda_sq_si - lambda_sq_kmm).abs() <= 10.0 * f64::EPSILON * avg,
            "λ² must be unit-invariant (SI={}, kN/mm={})",
            lambda_sq_si,
            lambda_sq_kmm
        );
    }
}
