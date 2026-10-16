//! Editable structural model with stable IDs.
//!
//! This module provides an editable representation of a structural model
//! that uses stable identifiers (`NodeId`, `MemberId`) independent of Vec
//! indices.  It converts to [`FrameModel3D`] for analysis via
//! [`EditModel::to_frame_model`].
//!
//! All mutation operations validate the complete operation before applying
//! any change — a failed operation leaves the model untouched.

use std::collections::HashMap;

use structural_analysis::{Dof3D, EndReleases3D, FrameModel3D, FrameSection3D};

/// Tolerance for detecting near-parallel reference vectors.
///
/// Matches `frame3d::PARALLEL_TOL` — `sin(angle) < 1e-8` means the
/// reference vector is effectively parallel to the member axis.
const PARALLEL_TOL: f64 = 1e-8;

// ── Identifiers ────────────────────────────────────────────────────────

/// Stable node identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct NodeId(pub u32);

/// Stable member identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct MemberId(pub u32);

/// Stable load-case / load-combination identifier.
///
/// Load combinations live in their own ID space (`u32` values starting at 0)
/// but share this type: exactly one of them can be the *active result
/// selection*, never both.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct LoadCaseId(pub u32);

/// Standard gravity (m/s²) used by the self-weight option.
pub const GRAVITY: f64 = 9.806_65;

/// Default mass density (kg/m³, structural steel) for new members.
pub const DEFAULT_DENSITY: f64 = 7850.0;

// ── Entities ───────────────────────────────────────────────────────────

/// Editable node.
#[derive(Debug, Clone)]
pub struct EditNode {
    pub id: NodeId,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Editable member.
#[derive(Debug, Clone)]
pub struct EditMember {
    pub id: MemberId,
    pub node_i: NodeId,
    pub node_j: NodeId,
    pub ref_vec: [f64; 3],
    pub section: FrameSection3D,
    /// Mass density (kg/m³) used by the self-weight load case option.
    pub density: f64,
    /// Rotational end releases: `(rx_i, ry_i, rz_i, rx_j, ry_j, rz_j)`.
    ///
    /// A released rotation transmits no moment at that end (a pin) — see
    /// [`EndReleases3D`].  Default: fully rigid.
    pub releases: [bool; 6],
}

/// A named static load case (one independent pattern of loads).
///
/// Loads are stored *in* the case they were created in, so the model can
/// hold any number of mutually-exclusive patterns (dead, live, wind, …)
/// exactly like MIDAS Gen's static load cases.  A case optionally carries
/// the members' **self weight** (converted to global `-Y` line loads), the
/// usual home for which is the dead-load case.
#[derive(Debug, Clone, PartialEq)]
pub struct EditLoadCase {
    pub id: LoadCaseId,
    pub name: String,
    /// Include member self weight (`density × g × A`) as global `-Y` loads.
    pub self_weight: bool,
}

/// One term of a load combination: a case scaled by its factor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ComboTerm {
    /// Source load case.
    pub case: LoadCaseId,
    /// Linear superposition factor (1.4, 0.5, …).
    pub factor: f64,
}

/// A named load combination: a linear sum of load cases.
///
/// Because the analysis is linear-static, the combination's results are the
/// factor-weighted sum of the participating cases' results (superposition) —
/// no second solve is ever needed for a combination.
#[derive(Debug, Clone, PartialEq)]
pub struct EditLoadCombination {
    pub id: LoadCaseId,
    pub name: String,
    /// Zero or more `(case, factor)` terms; at least one is required to solve.
    pub terms: Vec<ComboTerm>,
}

/// The default first load case's name (dead load).
pub const DEFAULT_CASE_NAME: &str = "DL";

impl Default for EditLoadCase {
    fn default() -> Self {
        Self {
            id: LoadCaseId(0),
            name: DEFAULT_CASE_NAME.to_string(),
            self_weight: false,
        }
    }
}

// ── Support specification ─────────────────────────────────────────────

/// Boundary-condition specification for a node.
///
/// Each of the 6 DOFs (Ux, Uy, Uz, Rx, Ry, Rz) can be:
/// - `None`     — free (unconstrained)
/// - `Some(v)`  — constrained to prescribed value `v`
///
/// Common presets: [`SupportSpec::fixed`], [`SupportSpec::pinned`],
/// [`SupportSpec::free`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SupportSpec {
    dofs: [Option<f64>; 6],
}

impl Default for SupportSpec {
    fn default() -> Self {
        Self::free()
    }
}

impl SupportSpec {
    /// Fully fixed: all 6 DOFs constrained to 0.
    pub fn fixed() -> Self {
        Self {
            dofs: [Some(0.0); 6],
        }
    }

    /// Pinned: translations (Ux, Uy, Uz) constrained to 0, rotations free.
    pub fn pinned() -> Self {
        Self {
            dofs: [Some(0.0), Some(0.0), Some(0.0), None, None, None],
        }
    }

    /// Free: no constraints.
    pub fn free() -> Self {
        Self { dofs: [None; 6] }
    }

    /// Custom from an explicit DOF array.
    pub fn custom(dofs: [Option<f64>; 6]) -> Self {
        Self { dofs }
    }

    /// Query a specific DOF.
    pub fn dof(&self, dof: Dof3D) -> Option<f64> {
        self.dofs[dof.index()]
    }

    /// Set a specific DOF: `None` = free, `Some(v)` = constrained to v.
    pub fn set_dof(&mut self, dof: Dof3D, value: Option<f64>) {
        self.dofs[dof.index()] = value;
    }

    /// Is this a fully fixed support (all 6 DOFs constrained)?
    pub fn is_fixed(&self) -> bool {
        self.dofs.iter().all(|d| d.is_some())
    }

    /// Is this a pinned support (Ux, Uy, Uz constrained, Rx, Ry, Rz free)?
    pub fn is_pinned(&self) -> bool {
        self.dofs[0].is_some()
            && self.dofs[1].is_some()
            && self.dofs[2].is_some()
            && self.dofs[3].is_none()
            && self.dofs[4].is_none()
            && self.dofs[5].is_none()
    }

    /// Is this a free support (no constraints)?
    pub fn is_free(&self) -> bool {
        self.dofs.iter().all(|d| d.is_none())
    }

    /// Number of constrained DOFs.
    pub fn n_constrained(&self) -> usize {
        self.dofs.iter().filter(|d| d.is_some()).count()
    }

    /// Short human-readable label.
    pub fn label(&self) -> &'static str {
        if self.is_fixed() {
            "Fixed"
        } else if self.is_pinned() {
            "Pinned"
        } else if self.is_free() {
            "Free"
        } else {
            "Custom"
        }
    }

    /// Raw DOF array (read-only copy).
    pub fn dofs(&self) -> [Option<f64>; 6] {
        self.dofs
    }
}

// ── Errors ──────────────────────────────────────────────────────────────

/// Error from a model editing operation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EditError {
    /// Node ID not found in the model.
    NodeNotFound(NodeId),
    /// Member ID not found in the model.
    MemberNotFound(MemberId),
    /// Coordinate value is NaN or infinity.
    NonFiniteCoordinate,
    /// Node is referenced by these members; cannot delete.
    NodeInUse(Vec<MemberId>),
    /// Start and end nodes are the same.
    SelfConnection,
    /// Member has zero or near-zero length.
    ZeroLengthMember,
    /// A member between these two nodes already exists (either direction).
    DuplicateMember,
    /// Conversion to FrameModel3D failed.
    FrameModelBuild(String),
    /// Invalid section/material parameter (non-finite, zero, or negative).
    InvalidSection(String),
    /// Invalid reference vector (non-finite, zero, or parallel to member axis).
    InvalidRefVec(String),
    /// Load component is NaN or infinity.
    NonFiniteLoad,
    /// Load position `xi` is outside the valid range [0, 1].
    InvalidLoadPosition,
    /// Load case ID not found in the model.
    LoadCaseNotFound(LoadCaseId),
    /// Requested to delete the last remaining load case.
    LastLoadCase,
    /// Load combination references a load case that no longer exists.
    CombinationCaseMissing(LoadCaseId),
    /// Mass density is not a positive finite number.
    InvalidDensity,
}

impl EditError {
    /// Human-readable diagnostic message.
    pub fn message(&self) -> String {
        match self {
            EditError::NodeNotFound(id) => format!("Node N{} not found", id.0),
            EditError::MemberNotFound(id) => format!("Member M{} not found", id.0),
            EditError::NonFiniteCoordinate => "Coordinates must be finite numbers".into(),
            EditError::NodeInUse(members) => {
                let ids: Vec<String> = members.iter().map(|m| format!("M{}", m.0)).collect();
                format!("Node is referenced by member(s): {}", ids.join(", "))
            }
            EditError::SelfConnection => "Start and end nodes must be different".into(),
            EditError::ZeroLengthMember => "Member has zero length".into(),
            EditError::DuplicateMember => "A member between these nodes already exists".into(),
            EditError::FrameModelBuild(s) => format!("Model build error: {s}"),
            EditError::InvalidSection(s) => format!("Invalid section: {s}"),
            EditError::InvalidRefVec(s) => format!("Invalid reference vector: {s}"),
            EditError::NonFiniteLoad => "Load components must be finite numbers".into(),
            EditError::InvalidLoadPosition => "Load position xi must be in [0, 1]".into(),
            EditError::LoadCaseNotFound(id) => format!("Load case #{} not found", id.0),
            EditError::LastLoadCase => "A model must keep at least one load case".into(),
            EditError::CombinationCaseMissing(id) => {
                format!("Load combination references missing load case #{}", id.0)
            }
            EditError::InvalidDensity => "Mass density must be a positive finite number".into(),
        }
    }
}

// ── Nodal load specification ──────────────────────────────────────────

/// Six-component nodal load in global coordinates.
///
/// `fx`, `fy`, `fz` are forces along global X, Y, Z.
/// `mx`, `my`, `mz` are moments about global X, Y, Z.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NodalLoadSpec {
    pub fx: f64,
    pub fy: f64,
    pub fz: f64,
    pub mx: f64,
    pub my: f64,
    pub mz: f64,
}

impl Default for NodalLoadSpec {
    fn default() -> Self {
        Self::zero()
    }
}

impl NodalLoadSpec {
    /// Create a nodal load from six components.
    pub fn new(fx: f64, fy: f64, fz: f64, mx: f64, my: f64, mz: f64) -> Self {
        Self {
            fx,
            fy,
            fz,
            mx,
            my,
            mz,
        }
    }

    /// Zero load (all components zero).
    pub fn zero() -> Self {
        Self {
            fx: 0.0,
            fy: 0.0,
            fz: 0.0,
            mx: 0.0,
            my: 0.0,
            mz: 0.0,
        }
    }

    /// Are all components zero?
    pub fn is_zero(&self) -> bool {
        self.fx == 0.0
            && self.fy == 0.0
            && self.fz == 0.0
            && self.mx == 0.0
            && self.my == 0.0
            && self.mz == 0.0
    }

    /// Are all components finite (not NaN, not infinity)?
    pub fn is_finite(&self) -> bool {
        self.fx.is_finite()
            && self.fy.is_finite()
            && self.fz.is_finite()
            && self.mx.is_finite()
            && self.my.is_finite()
            && self.mz.is_finite()
    }

    /// Force magnitude (Euclidean norm of fx, fy, fz).
    pub fn force_magnitude(&self) -> f64 {
        (self.fx * self.fx + self.fy * self.fy + self.fz * self.fz).sqrt()
    }

    /// Moment magnitude (Euclidean norm of mx, my, mz).
    pub fn moment_magnitude(&self) -> f64 {
        (self.mx * self.mx + self.my * self.my + self.mz * self.mz).sqrt()
    }

    /// As a `[fx, fy, fz, mx, my, mz]` array.
    pub fn as_array(&self) -> [f64; 6] {
        [self.fx, self.fy, self.fz, self.mx, self.my, self.mz]
    }
}

// ── Member uniform load specification ─────────────────────────────────

/// Uniform distributed load on a member in local coordinates.
///
/// `qx` is the axial load (along local x, force per unit length).
/// `qy` is the transverse load (along local y, force per unit length).
/// `qz` is the transverse load (along local z, force per unit length).
///
/// Positive values act in the positive local axis direction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MemberUniformLoad3D {
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
}

impl Default for MemberUniformLoad3D {
    fn default() -> Self {
        Self::zero()
    }
}

impl MemberUniformLoad3D {
    pub fn new(qx: f64, qy: f64, qz: f64) -> Self {
        Self { qx, qy, qz }
    }

    pub fn zero() -> Self {
        Self {
            qx: 0.0,
            qy: 0.0,
            qz: 0.0,
        }
    }

    pub fn is_zero(&self) -> bool {
        self.qx == 0.0 && self.qy == 0.0 && self.qz == 0.0
    }

    pub fn is_finite(&self) -> bool {
        self.qx.is_finite() && self.qy.is_finite() && self.qz.is_finite()
    }

    pub fn magnitude(&self) -> f64 {
        (self.qx * self.qx + self.qy * self.qy + self.qz * self.qz).sqrt()
    }
}

// ── Member point load specification ──────────────────────────────────

/// Concentrated force at a position along a member in local coordinates.
///
/// `xi` is the nondimensional position ∈ [0, 1]: 0 at node i, 1 at node j.
/// `fx` is the axial force (along local x, in N).
/// `fy` is the transverse force (along local y, in N).
/// `fz` is the transverse force (along local z, in N).
///
/// Positive values act in the positive local axis direction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MemberPointLoad3D {
    pub xi: f64,
    pub fx: f64,
    pub fy: f64,
    pub fz: f64,
}

impl MemberPointLoad3D {
    pub fn new(xi: f64, fx: f64, fy: f64, fz: f64) -> Self {
        Self { xi, fx, fy, fz }
    }

    pub fn zero() -> Self {
        Self {
            xi: 0.5,
            fx: 0.0,
            fy: 0.0,
            fz: 0.0,
        }
    }

    pub fn is_zero(&self) -> bool {
        self.fx == 0.0 && self.fy == 0.0 && self.fz == 0.0
    }

    pub fn is_finite(&self) -> bool {
        self.xi.is_finite() && self.fx.is_finite() && self.fy.is_finite() && self.fz.is_finite()
    }

    pub fn magnitude(&self) -> f64 {
        (self.fx * self.fx + self.fy * self.fy + self.fz * self.fz).sqrt()
    }
}

// ── Edit model ─────────────────────────────────────────────────────────

/// Editable structural model with stable IDs.
///
/// Stores nodes and members with stable identifiers that survive deletion,
/// supports (boundary conditions), **named load cases** (each carrying its
/// own nodal / member loads), load combinations, and per-member densities
/// for the self-weight option.
///
/// Every load accessor reads and writes the **active** load case
/// ([`active_case`](Self::active_case)); cases are selected from the Load
/// module's inspector or the command palette, so the same "select node →
/// type a load" workflow operates on whichever case is current.  Views
/// that must see everything (persistence, the case list itself) use the
/// case-tagged accessors.
#[derive(Debug, Clone)]
pub struct EditModel {
    nodes: Vec<EditNode>,
    members: Vec<EditMember>,
    supports: Vec<(NodeId, SupportSpec)>,
    /// Named static load cases.  Always non-empty (a fresh model starts
    /// with the default dead-load case `DL`).
    load_cases: Vec<EditLoadCase>,
    /// Named load combinations (linear sums of load cases).
    combinations: Vec<EditLoadCombination>,
    /// Loads currently being edited by the per-entity accessors.
    active_case: LoadCaseId,
    /// Case-tagged nodal loads: `(node, load, case)`.
    nodal_loads: Vec<(NodeId, NodalLoadSpec, LoadCaseId)>,
    /// Case-tagged member uniform loads: `(member, load, case)`.
    member_uniform_loads: Vec<(MemberId, MemberUniformLoad3D, LoadCaseId)>,
    /// Case-tagged member point loads: `(member, load, case)`.
    member_point_loads: Vec<(MemberId, MemberPointLoad3D, LoadCaseId)>,
    next_case_id: u32,
    next_combo_id: u32,
    next_node_id: u32,
    next_member_id: u32,
    revision: u64,
}

impl Default for EditModel {
    fn default() -> Self {
        Self::new()
    }
}

impl EditModel {
    /// Create an empty model with a single default load case (`DL`).
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            members: Vec::new(),
            supports: Vec::new(),
            load_cases: vec![EditLoadCase::default()],
            combinations: Vec::new(),
            active_case: LoadCaseId(0),
            nodal_loads: Vec::new(),
            member_uniform_loads: Vec::new(),
            member_point_loads: Vec::new(),
            next_case_id: 1,
            next_combo_id: 0,
            next_node_id: 0,
            next_member_id: 0,
            revision: 0,
        }
    }

    /// Construct an EditModel from pre-validated parts (case-tagged loads).
    ///
    /// Intended for deserialization: the caller is responsible for ensuring
    /// the data is consistent (unique IDs, valid references, at least one
    /// load case, and an `active_case` that exists).
    #[allow(clippy::too_many_arguments)]
    pub fn from_parts(
        nodes: Vec<EditNode>,
        members: Vec<EditMember>,
        supports: Vec<(NodeId, SupportSpec)>,
        load_cases: Vec<EditLoadCase>,
        combinations: Vec<EditLoadCombination>,
        active_case: LoadCaseId,
        nodal_loads: Vec<(NodeId, NodalLoadSpec, LoadCaseId)>,
        member_uniform_loads: Vec<(MemberId, MemberUniformLoad3D, LoadCaseId)>,
        member_point_loads: Vec<(MemberId, MemberPointLoad3D, LoadCaseId)>,
        next_case_id: u32,
        next_combo_id: u32,
        next_node_id: u32,
        next_member_id: u32,
    ) -> Self {
        Self {
            nodes,
            members,
            supports,
            load_cases,
            combinations,
            active_case,
            nodal_loads,
            member_uniform_loads,
            member_point_loads,
            next_case_id,
            next_combo_id,
            next_node_id,
            next_member_id,
            revision: 0,
        }
    }

    /// Construct an EditModel from **legacy single-case** parts.
    ///
    /// Every load is placed in load case 0 (which is created if `load_cases`
    /// is empty), keeping the pre-load-cases file formats (versions 1–3)
    /// loading without silent data loss.  New members default to structural
    /// steel density.
    pub fn from_parts_single_case(
        nodes: Vec<EditNode>,
        members: Vec<EditMember>,
        supports: Vec<(NodeId, SupportSpec)>,
        nodal_loads: Vec<(NodeId, NodalLoadSpec)>,
        member_uniform_loads: Vec<(MemberId, MemberUniformLoad3D)>,
        member_point_loads: Vec<(MemberId, MemberPointLoad3D)>,
        next_node_id: u32,
        next_member_id: u32,
    ) -> Self {
        let load_cases = vec![EditLoadCase::default()];
        let members: Vec<EditMember> = members
            .into_iter()
            .map(|m| EditMember {
                id: m.id,
                node_i: m.node_i,
                node_j: m.node_j,
                ref_vec: m.ref_vec,
                section: m.section,
                density: DEFAULT_DENSITY,
                releases: m.releases,
            })
            .collect();
        Self {
            nodes,
            members,
            supports,
            load_cases,
            combinations: Vec::new(),
            active_case: LoadCaseId(0),
            nodal_loads: nodal_loads
                .into_iter()
                .map(|(n, l)| (n, l, LoadCaseId(0)))
                .collect(),
            member_uniform_loads: member_uniform_loads
                .into_iter()
                .map(|(m, l)| (m, l, LoadCaseId(0)))
                .collect(),
            member_point_loads: member_point_loads
                .into_iter()
                .map(|(m, l)| (m, l, LoadCaseId(0)))
                .collect(),
            next_case_id: 1,
            next_combo_id: 0,
            next_node_id,
            next_member_id,
            revision: 0,
        }
    }

    /// Current model revision count.
    ///
    /// Incremented on every effective mutation.  Used by [`Project`]
    /// to detect stale analysis results without manual stale-flag bookkeeping.
    pub fn revision(&self) -> u64 {
        self.revision
    }

    fn bump_revision(&mut self) {
        self.revision = self.revision.saturating_add(1);
    }

    /// Ensure the revision is at least `min_rev`.
    ///
    /// Used by undo/redo to maintain the monotonic revision invariant:
    /// a restored snapshot must not have a revision that matches any
    /// prior solved revision, otherwise stale results could appear current.
    pub fn ensure_revision_at_least(&mut self, min_rev: u64) {
        if self.revision < min_rev {
            self.revision = min_rev;
        }
    }

    /// Compute a canonical fingerprint of the engineering data.
    ///
    /// Two models with the same fingerprint have identical engineering
    /// content: nodes (IDs + coordinates), members (IDs + connectivity +
    /// section + ref vec), supports, nodal loads, and ID allocation state.
    /// The revision counter is intentionally excluded — dirty-state
    /// comparison must reflect engineering content, not edit history.
    pub fn fingerprint(&self) -> u64 {
        use std::hash::Hash;
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        // Nodes
        self.nodes.len().hash(&mut hasher);
        for n in &self.nodes {
            n.id.0.hash(&mut hasher);
            n.x.to_bits().hash(&mut hasher);
            n.y.to_bits().hash(&mut hasher);
            n.z.to_bits().hash(&mut hasher);
        }
        // Members
        self.members.len().hash(&mut hasher);
        for m in &self.members {
            m.id.0.hash(&mut hasher);
            m.node_i.0.hash(&mut hasher);
            m.node_j.0.hash(&mut hasher);
            m.ref_vec[0].to_bits().hash(&mut hasher);
            m.ref_vec[1].to_bits().hash(&mut hasher);
            m.ref_vec[2].to_bits().hash(&mut hasher);
            m.section.E().to_bits().hash(&mut hasher);
            m.section.G().to_bits().hash(&mut hasher);
            m.section.area().to_bits().hash(&mut hasher);
            m.section.iy().to_bits().hash(&mut hasher);
            m.section.iz().to_bits().hash(&mut hasher);
            m.section.j().to_bits().hash(&mut hasher);
            m.density.to_bits().hash(&mut hasher);
        }
        // Load cases (IDs, names, self-weight flags).
        self.load_cases.len().hash(&mut hasher);
        for c in &self.load_cases {
            c.id.0.hash(&mut hasher);
            c.name.hash(&mut hasher);
            c.self_weight.hash(&mut hasher);
        }
        // Load combinations.
        self.combinations.len().hash(&mut hasher);
        for combo in &self.combinations {
            combo.id.0.hash(&mut hasher);
            combo.name.hash(&mut hasher);
            combo.terms.len().hash(&mut hasher);
            for t in &combo.terms {
                t.case.0.hash(&mut hasher);
                t.factor.to_bits().hash(&mut hasher);
            }
        }
        self.next_case_id.hash(&mut hasher);
        self.next_combo_id.hash(&mut hasher);
        // Supports
        self.supports.len().hash(&mut hasher);
        for (nid, spec) in &self.supports {
            nid.0.hash(&mut hasher);
            for dof in Dof3D::ALL {
                match spec.dof(dof) {
                    Some(v) => {
                        1u8.hash(&mut hasher);
                        v.to_bits().hash(&mut hasher);
                    }
                    None => {
                        0u8.hash(&mut hasher);
                    }
                }
            }
        }
        // Nodal loads (with the case they belong to).
        self.nodal_loads.len().hash(&mut hasher);
        for (nid, spec, case) in &self.nodal_loads {
            nid.0.hash(&mut hasher);
            case.0.hash(&mut hasher);
            spec.fx.to_bits().hash(&mut hasher);
            spec.fy.to_bits().hash(&mut hasher);
            spec.fz.to_bits().hash(&mut hasher);
            spec.mx.to_bits().hash(&mut hasher);
            spec.my.to_bits().hash(&mut hasher);
            spec.mz.to_bits().hash(&mut hasher);
        }
        // Member uniform loads.
        self.member_uniform_loads.len().hash(&mut hasher);
        for (mid, load, case) in &self.member_uniform_loads {
            mid.0.hash(&mut hasher);
            case.0.hash(&mut hasher);
            load.qx.to_bits().hash(&mut hasher);
            load.qy.to_bits().hash(&mut hasher);
            load.qz.to_bits().hash(&mut hasher);
        }
        // Member point loads.
        self.member_point_loads.len().hash(&mut hasher);
        for (mid, load, case) in &self.member_point_loads {
            mid.0.hash(&mut hasher);
            case.0.hash(&mut hasher);
            load.xi.to_bits().hash(&mut hasher);
            load.fx.to_bits().hash(&mut hasher);
            load.fy.to_bits().hash(&mut hasher);
            load.fz.to_bits().hash(&mut hasher);
        }
        // ID allocation state
        self.next_node_id.hash(&mut hasher);
        self.next_member_id.hash(&mut hasher);
        // Member end releases (pins) are engineering data.
        for m in &self.members {
            for r in &m.releases {
                r.hash(&mut hasher);
            }
        }
        std::hash::Hasher::finish(&hasher)
    }

    // ── Node operations ───────────────────────────────────────────────

    /// Add a node with the given coordinates.
    ///
    /// Returns the new node's stable ID.  Coordinates must be finite.
    pub fn add_node(&mut self, x: f64, y: f64, z: f64) -> Result<NodeId, EditError> {
        if !x.is_finite() || !y.is_finite() || !z.is_finite() {
            return Err(EditError::NonFiniteCoordinate);
        }
        let id = NodeId(self.next_node_id);
        self.next_node_id += 1;
        self.nodes.push(EditNode { id, x, y, z });
        self.bump_revision();
        Ok(id)
    }

    /// Update a node's coordinates.
    ///
    /// The node must exist and the new coordinates must be finite.
    pub fn update_node(&mut self, id: NodeId, x: f64, y: f64, z: f64) -> Result<(), EditError> {
        if !x.is_finite() || !y.is_finite() || !z.is_finite() {
            return Err(EditError::NonFiniteCoordinate);
        }
        let node = self
            .nodes
            .iter_mut()
            .find(|n| n.id == id)
            .ok_or(EditError::NodeNotFound(id))?;
        if node.x == x && node.y == y && node.z == z {
            return Ok(());
        }
        node.x = x;
        node.y = y;
        node.z = z;
        self.bump_revision();
        Ok(())
    }

    /// Delete a node.
    ///
    /// Fails if the node is referenced by any member.  The caller receives
    /// the list of referencing member IDs in the error so the user can be
    /// told which members block deletion.
    pub fn delete_node(&mut self, id: NodeId) -> Result<(), EditError> {
        if self.nodes.iter().all(|n| n.id != id) {
            return Err(EditError::NodeNotFound(id));
        }
        let referencing: Vec<MemberId> = self
            .members
            .iter()
            .filter(|m| m.node_i == id || m.node_j == id)
            .map(|m| m.id)
            .collect();
        if !referencing.is_empty() {
            return Err(EditError::NodeInUse(referencing));
        }
        self.nodes.retain(|n| n.id != id);
        self.supports.retain(|(nid, _)| *nid != id);
        self.nodal_loads.retain(|(nid, _, _)| *nid != id);
        self.bump_revision();
        Ok(())
    }

    /// Delete a node **and every member attached to it** (cascade delete).
    ///
    /// Returns the stable IDs of the members that were removed.  The node's
    /// support and nodal load are removed with it, and each removed member's
    /// uniform and point loads are removed too.
    ///
    /// Unlike [`delete_node`](Self::delete_node) — which refuses to remove a
    /// node still referenced by a member and reports the blockers — this is
    /// the "delete the selection" variant used by the viewport context menu
    /// and the ribbon Delete command.  Remaining nodes and members keep their
    /// stable IDs; only their storage order can change.
    pub fn delete_node_cascade(&mut self, id: NodeId) -> Result<Vec<MemberId>, EditError> {
        if self.nodes.iter().all(|n| n.id != id) {
            return Err(EditError::NodeNotFound(id));
        }
        let removed: Vec<MemberId> = self
            .members
            .iter()
            .filter(|m| m.node_i == id || m.node_j == id)
            .map(|m| m.id)
            .collect();
        self.members.retain(|m| m.node_i != id && m.node_j != id);
        self.member_uniform_loads
            .retain(|(mid, _, _)| !removed.contains(mid));
        self.member_point_loads
            .retain(|(mid, _, _)| !removed.contains(mid));
        self.nodes.retain(|n| n.id != id);
        self.supports.retain(|(nid, _)| *nid != id);
        self.nodal_loads.retain(|(nid, _, _)| *nid != id);
        self.bump_revision();
        Ok(removed)
    }

    // ── Member operations ─────────────────────────────────────────────

    /// Add a member between two existing nodes.
    ///
    /// Validates: both nodes exist, nodes are different, member has non-zero
    /// length, and no duplicate member exists between these nodes.
    pub fn add_member(&mut self, node_i: NodeId, node_j: NodeId) -> Result<MemberId, EditError> {
        if node_i == node_j {
            return Err(EditError::SelfConnection);
        }
        if self.nodes.iter().all(|n| n.id != node_i) {
            return Err(EditError::NodeNotFound(node_i));
        }
        if self.nodes.iter().all(|n| n.id != node_j) {
            return Err(EditError::NodeNotFound(node_j));
        }
        let pi = self.node(node_i).unwrap();
        let pj = self.node(node_j).unwrap();
        let dx = pj.x - pi.x;
        let dy = pj.y - pi.y;
        let dz = pj.z - pi.z;
        let len = dx.hypot(dy).hypot(dz);
        if len < 1e-12 {
            return Err(EditError::ZeroLengthMember);
        }
        let dup = self.members.iter().any(|m| {
            (m.node_i == node_i && m.node_j == node_j) || (m.node_i == node_j && m.node_j == node_i)
        });
        if dup {
            return Err(EditError::DuplicateMember);
        }
        let id = MemberId(self.next_member_id);
        self.next_member_id += 1;
        self.members.push(EditMember {
            id,
            node_i,
            node_j,
            ref_vec: [0.0, 0.0, 1.0],
            section: default_section(),
            density: DEFAULT_DENSITY,
            releases: [false; 6],
        });
        self.bump_revision();
        Ok(id)
    }

    /// Delete a member.
    pub fn delete_member(&mut self, id: MemberId) -> Result<(), EditError> {
        if self.members.iter().all(|m| m.id != id) {
            return Err(EditError::MemberNotFound(id));
        }
        self.members.retain(|m| m.id != id);
        self.member_uniform_loads.retain(|(mid, _, _)| *mid != id);
        self.member_point_loads.retain(|(mid, _, _)| *mid != id);
        self.bump_revision();
        Ok(())
    }

    /// Change a member's connectivity (its `i` / `j` nodes).
    ///
    /// Validates like [`add_member`](Self::add_member): both nodes must exist,
    /// differ, and not create a zero-length or duplicate member (a duplicate
    /// between the *same* pair in either direction is rejected; this member's
    /// own current connectivity is excluded from the check).  A no-op call
    /// (same endpoints) leaves the model unchanged.
    pub fn set_member_nodes(
        &mut self,
        id: MemberId,
        node_i: NodeId,
        node_j: NodeId,
    ) -> Result<(), EditError> {
        if node_i == node_j {
            return Err(EditError::SelfConnection);
        }
        if self.nodes.iter().all(|n| n.id != node_i) {
            return Err(EditError::NodeNotFound(node_i));
        }
        if self.nodes.iter().all(|n| n.id != node_j) {
            return Err(EditError::NodeNotFound(node_j));
        }
        let (old_i, old_j) = {
            let m = self
                .members
                .iter()
                .find(|m| m.id == id)
                .ok_or(EditError::MemberNotFound(id))?;
            (m.node_i, m.node_j)
        };
        if (old_i, old_j) == (node_i, node_j) {
            return Ok(());
        }
        let pi = self.node(node_i).unwrap();
        let pj = self.node(node_j).unwrap();
        let len = (pj.x - pi.x).hypot(pj.y - pi.y).hypot(pj.z - pi.z);
        if len < 1e-12 {
            return Err(EditError::ZeroLengthMember);
        }
        let dup = self.members.iter().any(|m| {
            m.id != id
                && ((m.node_i == node_i && m.node_j == node_j)
                    || (m.node_i == node_j && m.node_j == node_i))
        });
        if dup {
            return Err(EditError::DuplicateMember);
        }
        let m = self.members.iter_mut().find(|m| m.id == id).unwrap();
        m.node_i = node_i;
        m.node_j = node_j;
        self.bump_revision();
        Ok(())
    }

    /// Set the reference vector for a member.
    ///
    /// Validates that the vector is finite, nonzero, and not parallel
    /// (or nearly parallel) to the member axis, using the same tolerance
    /// as the analysis engine (`sin(angle) < 1e-8`).
    pub fn set_member_ref_vec(&mut self, id: MemberId, ref_vec: [f64; 3]) -> Result<(), EditError> {
        if !ref_vec[0].is_finite() || !ref_vec[1].is_finite() || !ref_vec[2].is_finite() {
            return Err(EditError::InvalidRefVec(
                "reference vector must be finite".to_string(),
            ));
        }
        let ref_norm = ref_vec[0].hypot(ref_vec[1]).hypot(ref_vec[2]);
        if ref_norm == 0.0 {
            return Err(EditError::InvalidRefVec(
                "reference vector must be nonzero".to_string(),
            ));
        }
        let member = self
            .members
            .iter()
            .find(|m| m.id == id)
            .ok_or(EditError::MemberNotFound(id))?;
        let pi = self.node(member.node_i).unwrap();
        let pj = self.node(member.node_j).unwrap();
        let dx = pj.x - pi.x;
        let dy = pj.y - pi.y;
        let dz = pj.z - pi.z;
        let len = dx.hypot(dy).hypot(dz);
        let x_hat = [dx / len, dy / len, dz / len];
        let cross = [
            x_hat[1] * ref_vec[2] - x_hat[2] * ref_vec[1],
            x_hat[2] * ref_vec[0] - x_hat[0] * ref_vec[2],
            x_hat[0] * ref_vec[1] - x_hat[1] * ref_vec[0],
        ];
        let cross_norm = cross[0].hypot(cross[1]).hypot(cross[2]);
        let sin_angle = cross_norm / ref_norm;
        if sin_angle < PARALLEL_TOL {
            return Err(EditError::InvalidRefVec(format!(
                "parallel to member axis; sin(angle) = {sin_angle:.3e} < tol {PARALLEL_TOL:.3e}"
            )));
        }
        let member = self
            .members
            .iter_mut()
            .find(|m| m.id == id)
            .ok_or(EditError::MemberNotFound(id))?;
        if member.ref_vec == ref_vec {
            return Ok(());
        }
        member.ref_vec = ref_vec;
        self.bump_revision();
        Ok(())
    }

    /// Query the reference vector of a member.
    pub fn member_ref_vec(&self, id: MemberId) -> Option<[f64; 3]> {
        self.members.iter().find(|m| m.id == id).map(|m| m.ref_vec)
    }

    /// Set the section/material properties for a member.
    ///
    /// All six parameters must be finite and strictly positive.
    /// On validation failure the member is left unchanged.
    #[allow(clippy::too_many_arguments)]
    pub fn set_member_section(
        &mut self,
        id: MemberId,
        e: f64,
        g: f64,
        area: f64,
        iy: f64,
        iz: f64,
        j: f64,
    ) -> Result<(), EditError> {
        for (label, val) in [
            ("E", e),
            ("G", g),
            ("A", area),
            ("Iy", iy),
            ("Iz", iz),
            ("J", j),
        ] {
            if !val.is_finite() {
                return Err(EditError::InvalidSection(format!(
                    "{label} must be finite, got {val}"
                )));
            }
            if val <= 0.0 {
                return Err(EditError::InvalidSection(format!(
                    "{label} must be positive, got {val}"
                )));
            }
        }
        let section = FrameSection3D::new(e, g, area, iy, iz, j)
            .map_err(|e| EditError::InvalidSection(format!("{e:?}")))?;
        let member = self
            .members
            .iter_mut()
            .find(|m| m.id == id)
            .ok_or(EditError::MemberNotFound(id))?;
        let old = &member.section;
        if old.E() == e
            && old.G() == g
            && old.area() == area
            && old.iy() == iy
            && old.iz() == iz
            && old.j() == j
        {
            return Ok(());
        }
        member.section = section;
        self.bump_revision();
        Ok(())
    }

    /// Query the section of a member.
    pub fn member_section(&self, id: MemberId) -> Option<&FrameSection3D> {
        self.members.iter().find(|m| m.id == id).map(|m| &m.section)
    }

    // ── Support / constraint operations ───────────────────────────────

    /// Mark a node as fully fixed.  Delegates to
    /// [`set_support`](Self::set_support).
    pub fn fix_node(&mut self, id: NodeId) -> Result<(), EditError> {
        self.set_support(id, SupportSpec::fixed())
    }

    /// Constrain a single DOF at a node to 0.  Merges into any existing
    /// support at that node.
    pub fn fix_dof(&mut self, id: NodeId, dof: Dof3D) -> Result<(), EditError> {
        if self.nodes.iter().all(|n| n.id != id) {
            return Err(EditError::NodeNotFound(id));
        }
        if let Some((_, spec)) = self.supports.iter_mut().find(|(nid, _)| *nid == id) {
            if spec.dof(dof) == Some(0.0) {
                return Ok(());
            }
            spec.set_dof(dof, Some(0.0));
        } else {
            let mut spec = SupportSpec::free();
            spec.set_dof(dof, Some(0.0));
            self.supports.push((id, spec));
        }
        self.bump_revision();
        Ok(())
    }

    /// Set the support specification for a node, replacing any existing
    /// support.
    pub fn set_support(&mut self, id: NodeId, spec: SupportSpec) -> Result<(), EditError> {
        if self.nodes.iter().all(|n| n.id != id) {
            return Err(EditError::NodeNotFound(id));
        }
        if let Some((_, existing)) = self.supports.iter_mut().find(|(nid, _)| *nid == id) {
            if *existing == spec {
                return Ok(());
            }
            *existing = spec;
        } else {
            self.supports.push((id, spec));
        }
        self.bump_revision();
        Ok(())
    }

    /// Clear the support at a node, making all DOFs free.
    pub fn clear_support(&mut self, id: NodeId) -> Result<(), EditError> {
        if self.nodes.iter().all(|n| n.id != id) {
            return Err(EditError::NodeNotFound(id));
        }
        let had_support = self.supports.iter().any(|(nid, _)| *nid == id);
        if !had_support {
            return Ok(());
        }
        self.supports.retain(|(nid, _)| *nid != id);
        self.bump_revision();
        Ok(())
    }

    /// Get the support specification for a node, if any.
    pub fn support(&self, id: NodeId) -> Option<&SupportSpec> {
        self.supports
            .iter()
            .find(|(nid, _)| *nid == id)
            .map(|(_, spec)| spec)
    }

    /// Does the node have any support?
    pub fn has_support(&self, id: NodeId) -> bool {
        self.supports.iter().any(|(nid, _)| *nid == id)
    }

    /// All supports (node ID + spec), in insertion order.
    pub fn supports(&self) -> &[(NodeId, SupportSpec)] {
        &self.supports
    }

    // ── Load cases ─────────────────────────────────────────────────────

    /// Load case that per-entity load accessors read and write.
    pub fn active_case(&self) -> LoadCaseId {
        self.active_case
    }

    /// Select the active load case (must exist).
    ///
    /// This is **view state**: it changes which case the load editors read
    /// and write, not the engineering data, so it deliberately leaves the
    /// revision (and therefore any analysis result) untouched.
    pub fn set_active_case(&mut self, id: LoadCaseId) -> Result<(), EditError> {
        if self.load_case(id).is_none() {
            return Err(EditError::LoadCaseNotFound(id));
        }
        if self.active_case == id {
            return Ok(());
        }
        self.active_case = id;
        Ok(())
    }

    /// All load cases in display order.
    pub fn load_cases(&self) -> &[EditLoadCase] {
        &self.load_cases
    }

    /// Look up a load case by ID.
    pub fn load_case(&self, id: LoadCaseId) -> Option<&EditLoadCase> {
        self.load_cases.iter().find(|c| c.id == id)
    }

    /// Number of load cases.
    pub fn n_load_cases(&self) -> usize {
        self.load_cases.len()
    }

    /// Add a new load case and make it active.
    ///
    /// The name must be non-empty and finite-length; MIDAS-style short codes
    /// ("DL", "LL", "WX") are conventional.  The new case starts empty.
    pub fn add_load_case(&mut self, name: &str) -> Result<LoadCaseId, EditError> {
        let name = name.trim();
        if name.is_empty() {
            return Err(EditError::InvalidSection(
                "load case name must not be empty".to_string(),
            ));
        }
        let id = LoadCaseId(self.next_case_id);
        self.next_case_id += 1;
        self.load_cases.push(EditLoadCase {
            id,
            name: name.to_string(),
            self_weight: false,
        });
        self.active_case = id;
        self.bump_revision();
        Ok(id)
    }

    /// Rename an existing load case.
    pub fn rename_load_case(&mut self, id: LoadCaseId, name: &str) -> Result<(), EditError> {
        let name = name.trim();
        if name.is_empty() {
            return Err(EditError::InvalidSection(
                "load case name must not be empty".to_string(),
            ));
        }
        let case = self
            .load_cases
            .iter_mut()
            .find(|c| c.id == id)
            .ok_or(EditError::LoadCaseNotFound(id))?;
        if case.name == name {
            return Ok(());
        }
        case.name = name.to_string();
        self.bump_revision();
        Ok(())
    }

    /// Delete a load case **and its loads**.
    ///
    /// Refuses to remove the last remaining case; deleting the active case
    /// selects the first remaining one.  Combinations referencing the case
    /// keep the (now dangling) term so the user can see it in the editor —
    /// solving such a combination fails with a clear diagnostic.
    pub fn delete_load_case(&mut self, id: LoadCaseId) -> Result<(), EditError> {
        if self.load_case(id).is_none() {
            return Err(EditError::LoadCaseNotFound(id));
        }
        if self.load_cases.len() <= 1 {
            return Err(EditError::LastLoadCase);
        }
        self.load_cases.retain(|c| c.id != id);
        self.nodal_loads.retain(|(_, _, c)| *c != id);
        self.member_uniform_loads.retain(|(_, _, c)| *c != id);
        self.member_point_loads.retain(|(_, _, c)| *c != id);
        if self.active_case == id {
            self.active_case = self.load_cases[0].id;
        }
        self.bump_revision();
        Ok(())
    }

    /// Toggle the self-weight option on a load case.
    ///
    /// When enabled, solving that case adds `density × g × A` global `-Y`
    /// line loads on every member (converted exactly into the member's local
    /// qx/qy/qz components).
    pub fn set_case_self_weight(&mut self, id: LoadCaseId, on: bool) -> Result<(), EditError> {
        let case = self
            .load_cases
            .iter_mut()
            .find(|c| c.id == id)
            .ok_or(EditError::LoadCaseNotFound(id))?;
        if case.self_weight == on {
            return Ok(());
        }
        case.self_weight = on;
        self.bump_revision();
        Ok(())
    }

    /// Set a member's mass density (kg/m³), used by the self-weight option.
    pub fn set_member_density(&mut self, id: MemberId, density: f64) -> Result<(), EditError> {
        if !density.is_finite() || density <= 0.0 {
            return Err(EditError::InvalidDensity);
        }
        let m = self
            .members
            .iter_mut()
            .find(|m| m.id == id)
            .ok_or(EditError::MemberNotFound(id))?;
        if m.density == density {
            return Ok(());
        }
        m.density = density;
        self.bump_revision();
        Ok(())
    }

    /// Member mass density (kg/m³), if the member exists.
    pub fn member_density(&self, id: MemberId) -> Option<f64> {
        self.member(id).map(|m| m.density)
    }

    /// Member self weight (N, `density × g × A × L`), or `None` if missing.
    pub fn member_self_weight(&self, id: MemberId) -> Option<f64> {
        let len = self.member_length(id)?;
        let m = self.member(id)?;
        Some(m.density * GRAVITY * m.section.area() * len)
    }

    // ── Member end releases (pins) ────────────────────────────────────

    /// Set a member's rotational end releases.
    ///
    /// `releases` is `(rx_i, ry_i, rz_i, rx_j, ry_j, rz_j)`: a released
    /// rotation transmits no moment at that end, i.e. the member is pinned
    /// there (applied by static condensation in the solver).  Default for a
    /// new member is fully rigid.
    ///
    /// # Errors
    ///
    /// [`EditError::MemberNotFound`] if the member does not exist.
    pub fn set_member_releases(
        &mut self,
        id: MemberId,
        releases: [bool; 6],
    ) -> Result<(), EditError> {
        let m = self
            .members
            .iter_mut()
            .find(|m| m.id == id)
            .ok_or(EditError::MemberNotFound(id))?;
        if m.releases == releases {
            return Ok(());
        }
        m.releases = releases;
        self.bump_revision();
        Ok(())
    }

    /// A member's rotational end releases, or `None` if it does not exist.
    pub fn member_releases(&self, id: MemberId) -> Option<[bool; 6]> {
        self.member(id).map(|m| m.releases)
    }

    /// Toggle one rotational release at one end (`end`: `false` = i,
    /// `true` = j; `axis`: `0` = rx, `1` = ry, `2` = rz).
    ///
    /// # Errors
    ///
    /// [`EditError::MemberNotFound`], [`EditError::InvalidSection`] for an
    /// out-of-range `axis`.
    pub fn set_member_release(
        &mut self,
        id: MemberId,
        end: bool,
        axis: usize,
        on: bool,
    ) -> Result<(), EditError> {
        if axis > 2 {
            return Err(EditError::InvalidSection(
                "release axis must be 0 (rx), 1 (ry) or 2 (rz)".to_string(),
            ));
        }
        let mut r = self
            .member_releases(id)
            .ok_or(EditError::MemberNotFound(id))?;
        let idx = if end { 3 + axis } else { axis };
        if r[idx] == on {
            return Ok(());
        }
        r[idx] = on;
        self.set_member_releases(id, r)
    }

    /// Short description of a member's releases, e.g. `"Rigid"` /
    /// `"Pin i"` / `"Pin i+j"` — for the tree and inspector labels.
    pub fn release_label(releases: [bool; 6]) -> Option<&'static str> {
        let pi = releases[0] || releases[1] || releases[2];
        let pj = releases[3] || releases[4] || releases[5];
        match (pi, pj) {
            (false, false) => None,
            (true, false) => Some("Pin i"),
            (false, true) => Some("Pin j"),
            (true, true) => Some("Pin i+j"),
        }
    }

    /// Solver-facing release flag set for a member.
    pub fn member_releases_solver(&self, id: MemberId) -> Option<EndReleases3D> {
        self.member_releases(id).map(EndReleases3D::from_array)
    }

    // ── Load combinations ──────────────────────────────────────────────

    /// All load combinations in display order.
    pub fn combinations(&self) -> &[EditLoadCombination] {
        &self.combinations
    }

    /// Look up a load combination by ID.
    pub fn combination(&self, id: LoadCaseId) -> Option<&EditLoadCombination> {
        self.combinations.iter().find(|c| c.id == id)
    }

    /// Number of load combinations.
    pub fn n_combinations(&self) -> usize {
        self.combinations.len()
    }

    /// Add a new (empty) load combination.
    pub fn add_load_combination(&mut self, name: &str) -> Result<LoadCaseId, EditError> {
        let name = name.trim();
        if name.is_empty() {
            return Err(EditError::InvalidSection(
                "load combination name must not be empty".to_string(),
            ));
        }
        let id = LoadCaseId(self.next_combo_id);
        self.next_combo_id += 1;
        self.combinations.push(EditLoadCombination {
            id,
            name: name.to_string(),
            terms: Vec::new(),
        });
        self.bump_revision();
        Ok(id)
    }

    /// Rename a load combination.
    pub fn rename_load_combination(&mut self, id: LoadCaseId, name: &str) -> Result<(), EditError> {
        let name = name.trim();
        if name.is_empty() {
            return Err(EditError::InvalidSection(
                "load combination name must not be empty".to_string(),
            ));
        }
        let combo = self
            .combinations
            .iter_mut()
            .find(|c| c.id == id)
            .ok_or(EditError::LoadCaseNotFound(id))?;
        if combo.name == name {
            return Ok(());
        }
        combo.name = name.to_string();
        self.bump_revision();
        Ok(())
    }

    /// Replace a load combination's terms.
    ///
    /// Factors must be finite; duplicate cases are merged by summing their
    /// factors.  Every term must reference an existing load case.
    pub fn set_combination_terms(
        &mut self,
        id: LoadCaseId,
        terms: Vec<ComboTerm>,
    ) -> Result<(), EditError> {
        let mut merged: Vec<ComboTerm> = Vec::new();
        for t in terms {
            if !t.factor.is_finite() {
                return Err(EditError::NonFiniteLoad);
            }
            if self.load_case(t.case).is_none() {
                return Err(EditError::LoadCaseNotFound(t.case));
            }
            if let Some(existing) = merged.iter_mut().find(|e| e.case == t.case) {
                existing.factor += t.factor;
            } else {
                merged.push(t);
            }
        }
        let combo = self
            .combinations
            .iter_mut()
            .find(|c| c.id == id)
            .ok_or(EditError::LoadCaseNotFound(id))?;
        if combo.terms == merged {
            return Ok(());
        }
        combo.terms = merged;
        self.bump_revision();
        Ok(())
    }

    /// Delete a load combination (the referenced cases are untouched).
    pub fn delete_load_combination(&mut self, id: LoadCaseId) -> Result<(), EditError> {
        if self.combinations.iter().all(|c| c.id != id) {
            return Err(EditError::LoadCaseNotFound(id));
        }
        self.combinations.retain(|c| c.id != id);
        self.bump_revision();
        Ok(())
    }

    /// All load-case IDs referenced by a combination.
    ///
    /// The solver uses this to validate a combination's case references
    /// before solving it.
    pub fn combination_cases(&self, id: LoadCaseId) -> Vec<LoadCaseId> {
        self.combination(id)
            .map(|c| c.terms.iter().map(|t| t.case).collect())
            .unwrap_or_default()
    }

    /// Load cases a combination references that no longer exist.
    ///
    /// A non-empty list means the combination cannot be solved; the UI
    /// surfaces it next to the combination so the user can repair it.
    pub fn dangling_combination_cases(&self, id: LoadCaseId) -> Vec<LoadCaseId> {
        self.combination(id)
            .map(|c| {
                c.terms
                    .iter()
                    .filter(|t| self.load_case(t.case).is_none())
                    .map(|t| t.case)
                    .collect()
            })
            .unwrap_or_default()
    }

    // ── Case-tagged nodal load operations ──────────────────────────────

    /// Set the nodal load at a node in the **active** case.
    ///
    /// One load record per (node, case).  All components must be finite.
    pub fn set_nodal_load(&mut self, id: NodeId, spec: NodalLoadSpec) -> Result<(), EditError> {
        if self.nodes.iter().all(|n| n.id != id) {
            return Err(EditError::NodeNotFound(id));
        }
        if !spec.is_finite() {
            return Err(EditError::NonFiniteCoordinate);
        }
        let case = self.active_case;
        if let Some((_, existing, _)) = self
            .nodal_loads
            .iter_mut()
            .find(|(nid, _, c)| *nid == id && *c == case)
        {
            if *existing == spec {
                return Ok(());
            }
            *existing = spec;
        } else {
            self.nodal_loads.push((id, spec, case));
        }
        self.bump_revision();
        Ok(())
    }

    /// Clear the nodal load at a node in the **active** case.
    pub fn clear_nodal_load(&mut self, id: NodeId) -> Result<(), EditError> {
        if self.nodes.iter().all(|n| n.id != id) {
            return Err(EditError::NodeNotFound(id));
        }
        let case = self.active_case;
        let had_load = self
            .nodal_loads
            .iter()
            .any(|(nid, _, c)| *nid == id && *c == case);
        if !had_load {
            return Ok(());
        }
        self.nodal_loads
            .retain(|(nid, _, c)| !(*nid == id && *c == case));
        self.bump_revision();
        Ok(())
    }

    /// Nodal load at a node in the **active** case, if any.
    pub fn nodal_load(&self, id: NodeId) -> Option<&NodalLoadSpec> {
        let case = self.active_case;
        self.nodal_loads
            .iter()
            .find(|(nid, _, c)| *nid == id && *c == case)
            .map(|(_, spec, _)| spec)
    }

    /// Does the node have a nodal load in the active case?
    pub fn has_nodal_load(&self, id: NodeId) -> bool {
        self.nodal_loads
            .iter()
            .any(|(nid, _, c)| *nid == id && *c == self.active_case)
    }

    /// Nodal loads of the **active** case (node ID + spec), in insertion order.
    pub fn nodal_loads(&self) -> Vec<(NodeId, NodalLoadSpec)> {
        self.nodal_loads
            .iter()
            .filter(|(_, _, c)| *c == self.active_case)
            .map(|(nid, spec, _)| (*nid, *spec))
            .collect()
    }

    /// Nodal loads of a **specific** case (node ID + spec).
    pub fn nodal_loads_in(&self, case: LoadCaseId) -> Vec<(NodeId, NodalLoadSpec)> {
        self.nodal_loads
            .iter()
            .filter(|(_, _, c)| *c == case)
            .map(|(nid, spec, _)| (*nid, *spec))
            .collect()
    }

    /// Nodal loads of **every** case, with their case tag.
    pub fn nodal_loads_all(&self) -> &[(NodeId, NodalLoadSpec, LoadCaseId)] {
        &self.nodal_loads
    }

    /// Number of nodal load records in the active case.
    pub fn n_nodal_loads(&self) -> usize {
        self.nodal_loads
            .iter()
            .filter(|(_, _, c)| *c == self.active_case)
            .count()
    }

    // ── Case-tagged member uniform load operations ─────────────────────

    /// Set the uniform distributed load on a member in the **active** case,
    /// replacing any existing load for that (member, case).
    pub fn set_member_uniform_load(
        &mut self,
        id: MemberId,
        load: MemberUniformLoad3D,
    ) -> Result<(), EditError> {
        if self.members.iter().all(|m| m.id != id) {
            return Err(EditError::MemberNotFound(id));
        }
        if !load.is_finite() {
            return Err(EditError::NonFiniteLoad);
        }
        let case = self.active_case;
        if let Some((_, existing, _)) = self
            .member_uniform_loads
            .iter_mut()
            .find(|(mid, _, c)| *mid == id && *c == case)
        {
            if *existing == load {
                return Ok(());
            }
            *existing = load;
        } else {
            self.member_uniform_loads.push((id, load, case));
        }
        self.bump_revision();
        Ok(())
    }

    /// Clear the uniform distributed load on a member in the **active** case.
    pub fn clear_member_uniform_load(&mut self, id: MemberId) -> Result<(), EditError> {
        if self.members.iter().all(|m| m.id != id) {
            return Err(EditError::MemberNotFound(id));
        }
        let case = self.active_case;
        let had_load = self
            .member_uniform_loads
            .iter()
            .any(|(mid, _, c)| *mid == id && *c == case);
        if !had_load {
            return Ok(());
        }
        self.member_uniform_loads
            .retain(|(mid, _, c)| !(*mid == id && *c == case));
        self.bump_revision();
        Ok(())
    }

    /// Uniform distributed load on a member in the **active** case, if any.
    pub fn member_uniform_load(&self, id: MemberId) -> Option<&MemberUniformLoad3D> {
        let case = self.active_case;
        self.member_uniform_loads
            .iter()
            .find(|(mid, _, c)| *mid == id && *c == case)
            .map(|(_, load, _)| load)
    }

    /// Does the member have a uniform load in the active case?
    pub fn has_member_uniform_load(&self, id: MemberId) -> bool {
        self.member_uniform_loads
            .iter()
            .any(|(mid, _, c)| *mid == id && *c == self.active_case)
    }

    /// Member uniform loads of the **active** case (member ID + spec).
    pub fn member_uniform_loads(&self) -> Vec<(MemberId, MemberUniformLoad3D)> {
        self.member_uniform_loads
            .iter()
            .filter(|(_, _, c)| *c == self.active_case)
            .map(|(mid, load, _)| (*mid, *load))
            .collect()
    }

    /// Member uniform loads of a **specific** case (member ID + spec).
    pub fn member_uniform_loads_in(
        &self,
        case: LoadCaseId,
    ) -> Vec<(MemberId, MemberUniformLoad3D)> {
        self.member_uniform_loads
            .iter()
            .filter(|(_, _, c)| *c == case)
            .map(|(mid, load, _)| (*mid, *load))
            .collect()
    }

    /// Member uniform loads of **every** case, with their case tag.
    pub fn member_uniform_loads_all(&self) -> &[(MemberId, MemberUniformLoad3D, LoadCaseId)] {
        &self.member_uniform_loads
    }

    /// Number of member uniform loads in the active case.
    pub fn n_member_uniform_loads(&self) -> usize {
        self.member_uniform_loads
            .iter()
            .filter(|(_, _, c)| *c == self.active_case)
            .count()
    }

    // ── Case-tagged member point load operations ───────────────────────

    /// Add a concentrated force at a position along a member in the
    /// **active** case.
    ///
    /// Multiple point loads per member are allowed — each call appends
    /// a new entry.  `xi` must be in [0, 1] and all force components
    /// must be finite.
    pub fn add_member_point_load(
        &mut self,
        id: MemberId,
        load: MemberPointLoad3D,
    ) -> Result<(), EditError> {
        if self.members.iter().all(|m| m.id != id) {
            return Err(EditError::MemberNotFound(id));
        }
        if !load.is_finite() {
            return Err(EditError::NonFiniteLoad);
        }
        if !(0.0..=1.0).contains(&load.xi) {
            return Err(EditError::InvalidLoadPosition);
        }
        self.member_point_loads.push((id, load, self.active_case));
        self.bump_revision();
        Ok(())
    }

    /// Update the point load at the given index within a member's loads
    /// (active case only).
    pub fn update_member_point_load(
        &mut self,
        id: MemberId,
        index: usize,
        load: MemberPointLoad3D,
    ) -> Result<(), EditError> {
        if self.members.iter().all(|m| m.id != id) {
            return Err(EditError::MemberNotFound(id));
        }
        if !load.is_finite() {
            return Err(EditError::NonFiniteLoad);
        }
        if !(0.0..=1.0).contains(&load.xi) {
            return Err(EditError::InvalidLoadPosition);
        }
        let mut count = 0;
        let mut found = false;
        for (mid, existing, c) in &mut self.member_point_loads {
            if *mid == id && *c == self.active_case {
                if count == index {
                    if *existing == load {
                        return Ok(());
                    }
                    *existing = load;
                    found = true;
                    break;
                }
                count += 1;
            }
        }
        if !found {
            return Err(EditError::MemberNotFound(id));
        }
        self.bump_revision();
        Ok(())
    }

    /// Remove the point load at the given index within a member's loads
    /// (active case only).
    pub fn remove_member_point_load(
        &mut self,
        id: MemberId,
        index: usize,
    ) -> Result<(), EditError> {
        if self.members.iter().all(|m| m.id != id) {
            return Err(EditError::MemberNotFound(id));
        }
        let mut count = 0;
        let mut global_idx = None;
        for (i, (mid, _, c)) in self.member_point_loads.iter().enumerate() {
            if *mid == id && *c == self.active_case {
                if count == index {
                    global_idx = Some(i);
                    break;
                }
                count += 1;
            }
        }
        let Some(gi) = global_idx else {
            return Err(EditError::MemberNotFound(id));
        };
        self.member_point_loads.remove(gi);
        self.bump_revision();
        Ok(())
    }

    /// Remove all point loads on a member in the **active** case.
    pub fn clear_member_point_loads(&mut self, id: MemberId) -> Result<(), EditError> {
        if self.members.iter().all(|m| m.id != id) {
            return Err(EditError::MemberNotFound(id));
        }
        let case = self.active_case;
        let before = self.member_point_loads.len();
        self.member_point_loads
            .retain(|(mid, _, c)| !(*mid == id && *c == case));
        if self.member_point_loads.len() != before {
            self.bump_revision();
        }
        Ok(())
    }

    /// Get all point loads for a member in the **active** case.
    pub fn member_point_loads_for(&self, id: MemberId) -> Vec<MemberPointLoad3D> {
        self.member_point_loads
            .iter()
            .filter(|(mid, _, c)| *mid == id && *c == self.active_case)
            .map(|(_, load, _)| *load)
            .collect()
    }

    /// Point loads of a **specific** case (member ID + spec).
    pub fn member_point_loads_in(&self, case: LoadCaseId) -> Vec<(MemberId, MemberPointLoad3D)> {
        self.member_point_loads
            .iter()
            .filter(|(_, _, c)| *c == case)
            .map(|(mid, load, _)| (*mid, *load))
            .collect()
    }

    /// Member point loads of **every** case, with their case tag.
    pub fn member_point_loads_all(&self) -> &[(MemberId, MemberPointLoad3D, LoadCaseId)] {
        &self.member_point_loads
    }

    /// Does the member have any point loads in the active case?
    pub fn has_member_point_loads(&self, id: MemberId) -> bool {
        self.member_point_loads
            .iter()
            .any(|(mid, _, c)| *mid == id && *c == self.active_case)
    }

    /// Member point loads of the **active** case (member ID + spec).
    pub fn member_point_loads(&self) -> Vec<(MemberId, MemberPointLoad3D)> {
        self.member_point_loads
            .iter()
            .filter(|(_, _, c)| *c == self.active_case)
            .map(|(mid, load, _)| (*mid, *load))
            .collect()
    }

    // ── Queries ───────────────────────────────────────────────────────

    /// Number of nodes.
    pub fn n_nodes(&self) -> usize {
        self.nodes.len()
    }

    /// Number of members.
    pub fn n_members(&self) -> usize {
        self.members.len()
    }

    /// Get a node by ID.
    pub fn node(&self, id: NodeId) -> Option<&EditNode> {
        self.nodes.iter().find(|n| n.id == id)
    }

    /// Get a member by ID.
    pub fn member(&self, id: MemberId) -> Option<&EditMember> {
        self.members.iter().find(|m| m.id == id)
    }

    /// All nodes.
    pub fn nodes(&self) -> &[EditNode] {
        &self.nodes
    }

    /// All members.
    pub fn members(&self) -> &[EditMember] {
        &self.members
    }

    /// Next node ID that will be assigned by `add_node`.
    pub fn next_node_id(&self) -> u32 {
        self.next_node_id
    }

    /// Next member ID that will be assigned by `add_member`.
    pub fn next_member_id(&self) -> u32 {
        self.next_member_id
    }

    /// Next load case ID that will be assigned.
    pub fn next_load_case_id(&self) -> u32 {
        self.next_case_id
    }

    /// Next load combination ID that will be assigned.
    pub fn next_combination_id(&self) -> u32 {
        self.next_combo_id
    }

    /// Vec index of a node ID, for viewport rendering.
    pub fn node_index(&self, id: NodeId) -> Option<usize> {
        self.nodes.iter().position(|n| n.id == id)
    }

    /// Vec index of a member ID, for viewport rendering.
    pub fn member_index(&self, id: MemberId) -> Option<usize> {
        self.members.iter().position(|m| m.id == id)
    }

    /// Member IDs that reference the given node.
    pub fn members_referencing_node(&self, id: NodeId) -> Vec<MemberId> {
        self.members
            .iter()
            .filter(|m| m.node_i == id || m.node_j == id)
            .map(|m| m.id)
            .collect()
    }

    /// Length of a member.
    pub fn member_length(&self, id: MemberId) -> Option<f64> {
        let m = self.member(id)?;
        let ni = self.node(m.node_i)?;
        let nj = self.node(m.node_j)?;
        let dx = nj.x - ni.x;
        let dy = nj.y - ni.y;
        let dz = nj.z - ni.z;
        Some(dx.hypot(dy).hypot(dz))
    }

    /// Is the model empty (no nodes and no members)?
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty() && self.members.is_empty()
    }

    // ── Conversion to FrameModel3D ────────────────────────────────────

    /// Convert the **active** load case to a `FrameModel3D` for analysis.
    ///
    /// For the case-aware form (solving every case in turn) see
    /// [`to_frame_model_for_case`](Self::to_frame_model_for_case).
    pub fn to_frame_model(&self) -> Result<FrameModel3D, EditError> {
        self.to_frame_model_for_case(self.active_case)
    }

    /// Convert one load case to a `FrameModel3D`.
    ///
    /// Nodes and members are added in storage order; supports transfer with
    /// their prescribed values / spring stiffnesses.  Only the loads tagged
    /// with `case` are applied, plus the members' self weight when the case
    /// has [`self_weight`](EditLoadCase::self_weight) enabled.
    pub fn to_frame_model_for_case(&self, case: LoadCaseId) -> Result<FrameModel3D, EditError> {
        let case_def = self
            .load_case(case)
            .ok_or(EditError::LoadCaseNotFound(case))?;
        let mut model = FrameModel3D::new();
        let mut id_to_idx: HashMap<NodeId, usize> = HashMap::new();

        for node in &self.nodes {
            let idx = model
                .add_node(node.x, node.y, node.z)
                .map_err(|e| EditError::FrameModelBuild(format!("{e}")))?;
            id_to_idx.insert(node.id, idx);
        }

        for member in &self.members {
            let ni = *id_to_idx
                .get(&member.node_i)
                .ok_or(EditError::NodeNotFound(member.node_i))?;
            let nj = *id_to_idx
                .get(&member.node_j)
                .ok_or(EditError::NodeNotFound(member.node_j))?;
            let idx = model
                .add_member(ni, nj, member.section, member.ref_vec)
                .map_err(|e| EditError::FrameModelBuild(format!("{e}")))?;
            model
                .set_member_releases(idx, EndReleases3D::from_array(member.releases))
                .map_err(|e| EditError::FrameModelBuild(format!("{e}")))?;
        }

        for (nid, spec) in &self.supports {
            let idx = *id_to_idx.get(nid).ok_or(EditError::NodeNotFound(*nid))?;
            if spec.is_fixed() {
                model
                    .fix_node(idx)
                    .map_err(|e| EditError::FrameModelBuild(format!("{e}")))?;
            } else if spec.is_pinned() {
                model
                    .pin_node(idx)
                    .map_err(|e| EditError::FrameModelBuild(format!("{e}")))?;
            } else {
                for dof in Dof3D::ALL {
                    if let Some(value) = spec.dof(dof) {
                        model
                            .fix_dof(idx, dof, value)
                            .map_err(|e| EditError::FrameModelBuild(format!("{e}")))?;
                    }
                }
            }
        }

        for (nid, spec, c) in &self.nodal_loads {
            if *c != case {
                continue;
            }
            let idx = *id_to_idx.get(nid).ok_or(EditError::NodeNotFound(*nid))?;
            model
                .add_nodal_load(idx, spec.fx, spec.fy, spec.fz, spec.mx, spec.my, spec.mz)
                .map_err(|e| EditError::FrameModelBuild(format!("{e}")))?;
        }

        let member_id_to_idx: HashMap<MemberId, usize> = self
            .members
            .iter()
            .enumerate()
            .map(|(i, m)| (m.id, i))
            .collect();

        for (mid, load, c) in &self.member_uniform_loads {
            if *c != case {
                continue;
            }
            let idx = *member_id_to_idx
                .get(mid)
                .ok_or(EditError::MemberNotFound(*mid))?;
            if load.qx != 0.0 {
                model
                    .add_uniform_axial(idx, load.qx)
                    .map_err(|e| EditError::FrameModelBuild(format!("{e}")))?;
            }
            if load.qy != 0.0 {
                model
                    .add_uniform_y(idx, load.qy)
                    .map_err(|e| EditError::FrameModelBuild(format!("{e}")))?;
            }
            if load.qz != 0.0 {
                model
                    .add_uniform_z(idx, load.qz)
                    .map_err(|e| EditError::FrameModelBuild(format!("{e}")))?;
            }
        }

        for (mid, load, c) in &self.member_point_loads {
            if *c != case {
                continue;
            }
            let idx = *member_id_to_idx
                .get(mid)
                .ok_or(EditError::MemberNotFound(*mid))?;
            model
                .add_point_load(idx, load.xi, load.fx, load.fy, load.fz)
                .map_err(|e| EditError::FrameModelBuild(format!("{e}")))?;
        }

        // Self weight: a global −Y line load of `w = ρ·g·A` per member length,
        // resolved into the member's (constant) local axes.  For a vertical
        // member this degenerates to a pure axial load, which is correct.
        if case_def.self_weight {
            for member in &self.members {
                let node_i = self
                    .node(member.node_i)
                    .ok_or(EditError::NodeNotFound(member.node_i))?;
                let idx = *member_id_to_idx
                    .get(&member.id)
                    .ok_or(EditError::MemberNotFound(member.id))?;
                let gravity = [0.0, -1.0, 0.0];
                self.add_self_weight(&mut model, idx, member, node_i, gravity)?;
            }
        }

        Ok(model)
    }

    /// Append one member's self-weight line load to a `FrameModel3D`.
    ///
    /// Private helper for [`to_frame_model_for_case`](Self::to_frame_model_for_case):
    /// mirrors `FrameElement3D::local_axes` to resolve the global `−Y` load
    /// into local `q` components, so the magnitude a solver sees is exactly
    /// `ρ·g·A` per unit length.  Returns silently when the member's local
    /// axes are degenerate (mirrored ref vec); the analysis itself would
    /// reject the element later with a clear error anyway.
    fn add_self_weight(
        &self,
        model: &mut FrameModel3D,
        member_idx: usize,
        member: &EditMember,
        node_i: &EditNode,
        gravity_dir: [f64; 3],
    ) -> Result<(), EditError> {
        let build = |e: structural_analysis::FemError| EditError::FrameModelBuild(format!("{e}"));
        let nj = self
            .node(member.node_j)
            .ok_or(EditError::NodeNotFound(member.node_j))?;
        let dx = nj.x - node_i.x;
        let dy = nj.y - node_i.y;
        let dz = nj.z - node_i.z;
        let len = dx.hypot(dy).hypot(dz);
        if len < 1e-12 {
            return Err(EditError::ZeroLengthMember);
        }
        let x_hat = [dx / len, dy / len, dz / len];
        let rv = member.ref_vec;
        let ref_norm = rv[0].hypot(rv[1]).hypot(rv[2]);
        if ref_norm < 1e-12 {
            return Err(EditError::InvalidRefVec(
                "reference vector must be nonzero".to_string(),
            ));
        }
        let cross = [
            x_hat[1] * rv[2] - x_hat[2] * rv[1],
            x_hat[2] * rv[0] - x_hat[0] * rv[2],
            x_hat[0] * rv[1] - x_hat[1] * rv[0],
        ];
        let cross_norm = cross[0].hypot(cross[1]).hypot(cross[2]);
        if cross_norm / ref_norm < PARALLEL_TOL {
            return Err(EditError::InvalidRefVec(
                "reference vector parallel to member axis".to_string(),
            ));
        }
        let z_hat = [
            cross[0] / cross_norm,
            cross[1] / cross_norm,
            cross[2] / cross_norm,
        ];
        let y_hat = [
            z_hat[1] * x_hat[2] - z_hat[2] * x_hat[1],
            z_hat[2] * x_hat[0] - z_hat[0] * x_hat[2],
            z_hat[0] * x_hat[1] - z_hat[1] * x_hat[0],
        ];
        let w = member.density * GRAVITY * member.section.area();
        let q = [gravity_dir[0] * w, gravity_dir[1] * w, gravity_dir[2] * w];
        let qx = q[0] * x_hat[0] + q[1] * x_hat[1] + q[2] * x_hat[2];
        let qy = q[0] * y_hat[0] + q[1] * y_hat[1] + q[2] * y_hat[2];
        let qz = q[0] * z_hat[0] + q[1] * z_hat[1] + q[2] * z_hat[2];
        if qx != 0.0 {
            model.add_uniform_axial(member_idx, qx).map_err(build)?;
        }
        if qy != 0.0 {
            model.add_uniform_y(member_idx, qy).map_err(build)?;
        }
        if qz != 0.0 {
            model.add_uniform_z(member_idx, qz).map_err(build)?;
        }
        Ok(())
    }
}

/// Default steel I-section for MVP.
fn default_section() -> FrameSection3D {
    FrameSection3D::new(200e9, 80e9, 1e-3, 2e-6, 1e-6, 3e-6).unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Node tests ───────────────────────────────────────────────────

    #[test]
    fn add_node_success() {
        let mut m = EditModel::new();
        let id = m.add_node(1.0, 2.0, 3.0).unwrap();
        assert_eq!(m.n_nodes(), 1);
        assert_eq!(m.node(id).unwrap().x, 1.0);
    }

    #[test]
    fn add_node_rejects_nan() {
        let mut m = EditModel::new();
        assert_eq!(
            m.add_node(f64::NAN, 0.0, 0.0),
            Err(EditError::NonFiniteCoordinate)
        );
        assert_eq!(m.n_nodes(), 0);
    }

    #[test]
    fn add_node_rejects_infinity() {
        let mut m = EditModel::new();
        assert_eq!(
            m.add_node(f64::INFINITY, 0.0, 0.0),
            Err(EditError::NonFiniteCoordinate)
        );
        assert_eq!(m.n_nodes(), 0);
    }

    #[test]
    fn update_node_success() {
        let mut m = EditModel::new();
        let id = m.add_node(0.0, 0.0, 0.0).unwrap();
        m.update_node(id, 5.0, 6.0, 7.0).unwrap();
        let n = m.node(id).unwrap();
        assert_eq!(n.x, 5.0);
        assert_eq!(n.y, 6.0);
        assert_eq!(n.z, 7.0);
    }

    #[test]
    fn update_node_rejects_non_finite() {
        let mut m = EditModel::new();
        let id = m.add_node(0.0, 0.0, 0.0).unwrap();
        let err = m.update_node(id, f64::NAN, 0.0, 0.0);
        assert_eq!(err, Err(EditError::NonFiniteCoordinate));
        assert_eq!(m.node(id).unwrap().x, 0.0);
    }

    #[test]
    fn update_node_not_found() {
        let mut m = EditModel::new();
        assert_eq!(
            m.update_node(NodeId(99), 1.0, 2.0, 3.0),
            Err(EditError::NodeNotFound(NodeId(99)))
        );
    }

    #[test]
    fn delete_node_success() {
        let mut m = EditModel::new();
        let id = m.add_node(0.0, 0.0, 0.0).unwrap();
        m.delete_node(id).unwrap();
        assert_eq!(m.n_nodes(), 0);
    }

    #[test]
    fn delete_node_in_use_fails() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let _mid = m.add_member(a, b).unwrap();
        let err = m.delete_node(a).unwrap_err();
        assert!(matches!(err, EditError::NodeInUse(_)));
        assert_eq!(m.n_nodes(), 2);
    }

    #[test]
    fn delete_node_not_found() {
        let mut m = EditModel::new();
        assert_eq!(
            m.delete_node(NodeId(99)),
            Err(EditError::NodeNotFound(NodeId(99)))
        );
    }

    #[test]
    fn failed_add_does_not_change_model() {
        let mut m = EditModel::new();
        let _ = m.add_node(1.0, 0.0, 0.0).unwrap();
        let err = m.add_node(f64::NAN, 0.0, 0.0);
        assert!(err.is_err());
        assert_eq!(m.n_nodes(), 1);
    }

    #[test]
    fn node_ids_are_unique() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(1.0, 0.0, 0.0).unwrap();
        let c = m.add_node(2.0, 0.0, 0.0).unwrap();
        assert_ne!(a, b);
        assert_ne!(b, c);
        assert_ne!(a, c);
    }

    #[test]
    fn deleted_node_id_not_reused() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        m.delete_node(a).unwrap();
        let b = m.add_node(1.0, 0.0, 0.0).unwrap();
        assert_ne!(a, b);
    }

    // ── Member tests ──────────────────────────────────────────────────

    #[test]
    fn add_member_success() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        assert_eq!(m.n_members(), 1);
        assert_eq!(m.member(mid).unwrap().node_i, a);
        assert_eq!(m.member(mid).unwrap().node_j, b);
    }

    #[test]
    fn add_member_rejects_nonexistent_node() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        assert_eq!(
            m.add_member(a, NodeId(99)),
            Err(EditError::NodeNotFound(NodeId(99)))
        );
        assert_eq!(m.n_members(), 0);
    }

    #[test]
    fn add_member_rejects_self_connection() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        assert_eq!(m.add_member(a, a), Err(EditError::SelfConnection));
        assert_eq!(m.n_members(), 0);
    }

    #[test]
    fn add_member_rejects_zero_length() {
        let mut m = EditModel::new();
        let a = m.add_node(1.0, 2.0, 3.0).unwrap();
        let b = m.add_node(1.0, 2.0, 3.0).unwrap();
        assert_eq!(m.add_member(a, b), Err(EditError::ZeroLengthMember));
        assert_eq!(m.n_members(), 0);
    }

    #[test]
    fn add_member_rejects_near_zero_length() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(1e-13, 0.0, 0.0).unwrap();
        assert_eq!(m.add_member(a, b), Err(EditError::ZeroLengthMember));
    }

    #[test]
    fn add_member_rejects_duplicate() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        m.add_member(a, b).unwrap();
        assert_eq!(m.add_member(a, b), Err(EditError::DuplicateMember));
        assert_eq!(m.n_members(), 1);
    }

    #[test]
    fn add_member_rejects_reverse_duplicate() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        m.add_member(a, b).unwrap();
        assert_eq!(m.add_member(b, a), Err(EditError::DuplicateMember));
    }

    #[test]
    fn delete_member_success() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        m.delete_member(mid).unwrap();
        assert_eq!(m.n_members(), 0);
        assert_eq!(m.n_nodes(), 2);
    }

    #[test]
    fn delete_member_not_found() {
        let mut m = EditModel::new();
        assert_eq!(
            m.delete_member(MemberId(99)),
            Err(EditError::MemberNotFound(MemberId(99)))
        );
    }

    #[test]
    fn failed_member_add_does_not_change_model() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let err = m.add_member(a, NodeId(99));
        assert!(err.is_err());
        assert_eq!(m.n_members(), 0);
    }

    // ── Query tests ───────────────────────────────────────────────────

    #[test]
    fn members_referencing_node() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let c = m.add_node(0.0, 3.0, 0.0).unwrap();
        let _m1 = m.add_member(a, b).unwrap();
        let _m2 = m.add_member(a, c).unwrap();
        let refs = m.members_referencing_node(a);
        assert_eq!(refs.len(), 2);
    }

    #[test]
    fn member_length_correct() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 4.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        assert_eq!(m.member_length(mid).unwrap(), 5.0);
    }

    // ── Conversion tests ──────────────────────────────────────────────

    #[test]
    fn to_frame_model_empty() {
        let m = EditModel::new();
        let fm = m.to_frame_model().unwrap();
        assert_eq!(fm.n_nodes(), 0);
        assert_eq!(fm.n_members(), 0);
    }

    #[test]
    fn to_frame_model_with_nodes_and_members() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(4.0, 0.0, 0.0).unwrap();
        let c = m.add_node(0.0, 3.0, 0.0).unwrap();
        m.add_member(a, c).unwrap();
        m.add_member(c, b).unwrap();
        m.add_member(b, a).unwrap();
        let fm = m.to_frame_model().unwrap();
        assert_eq!(fm.n_nodes(), 3);
        assert_eq!(fm.n_members(), 3);
    }

    #[test]
    fn to_frame_model_with_fixes_and_loads() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(4.0, 0.0, 0.0).unwrap();
        let c = m.add_node(0.0, 3.0, 0.0).unwrap();
        m.add_member(a, c).unwrap();
        m.add_member(c, b).unwrap();
        m.add_member(b, a).unwrap();
        m.fix_node(a).unwrap();
        m.fix_node(b).unwrap();
        m.set_nodal_load(c, NodalLoadSpec::new(5e3, 0.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        let fm = m.to_frame_model().unwrap();
        assert_eq!(fm.n_nodes(), 3);
        assert_eq!(fm.n_members(), 3);
        let result = fm.solve();
        assert!(
            result.is_ok(),
            "portal frame should solve: {:?}",
            result.err()
        );
    }

    // ── Error message tests ───────────────────────────────────────────

    #[test]
    fn error_messages() {
        assert!(EditError::NonFiniteCoordinate.message().contains("finite"));
        assert!(EditError::SelfConnection.message().contains("different"));
        assert!(EditError::ZeroLengthMember.message().contains("zero"));
        assert!(
            EditError::DuplicateMember
                .message()
                .contains("already exists")
        );
        let e = EditError::NodeNotFound(NodeId(5));
        assert!(e.message().contains("N5"));
        let e = EditError::NodeInUse(vec![MemberId(3)]);
        assert!(e.message().contains("M3"));
    }

    // ── SupportSpec tests ─────────────────────────────────────────────

    #[test]
    fn support_fixed_constrains_all() {
        let s = SupportSpec::fixed();
        assert!(s.is_fixed());
        assert!(!s.is_pinned());
        assert!(!s.is_free());
        assert_eq!(s.n_constrained(), 6);
        assert_eq!(s.label(), "Fixed");
        for dof in Dof3D::ALL {
            assert_eq!(s.dof(dof), Some(0.0));
        }
    }

    #[test]
    fn support_pinned_constrains_translations_only() {
        let s = SupportSpec::pinned();
        assert!(!s.is_fixed());
        assert!(s.is_pinned());
        assert!(!s.is_free());
        assert_eq!(s.n_constrained(), 3);
        assert_eq!(s.label(), "Pinned");
        assert_eq!(s.dof(Dof3D::Ux), Some(0.0));
        assert_eq!(s.dof(Dof3D::Uy), Some(0.0));
        assert_eq!(s.dof(Dof3D::Uz), Some(0.0));
        assert_eq!(s.dof(Dof3D::Rx), None);
        assert_eq!(s.dof(Dof3D::Ry), None);
        assert_eq!(s.dof(Dof3D::Rz), None);
    }

    #[test]
    fn support_free_constrains_none() {
        let s = SupportSpec::free();
        assert!(!s.is_fixed());
        assert!(!s.is_pinned());
        assert!(s.is_free());
        assert_eq!(s.n_constrained(), 0);
        assert_eq!(s.label(), "Free");
    }

    #[test]
    fn support_custom() {
        let dofs = [Some(0.0), None, Some(0.0), None, None, Some(0.0)];
        let s = SupportSpec::custom(dofs);
        assert!(!s.is_fixed());
        assert!(!s.is_pinned());
        assert!(!s.is_free());
        assert_eq!(s.n_constrained(), 3);
        assert_eq!(s.label(), "Custom");
    }

    #[test]
    fn support_set_dof() {
        let mut s = SupportSpec::free();
        s.set_dof(Dof3D::Ux, Some(0.0));
        s.set_dof(Dof3D::Rz, Some(0.0));
        assert_eq!(s.n_constrained(), 2);
        assert_eq!(s.dof(Dof3D::Ux), Some(0.0));
        assert_eq!(s.dof(Dof3D::Rz), Some(0.0));
        assert_eq!(s.dof(Dof3D::Uy), None);
        assert_eq!(s.label(), "Custom");
    }

    #[test]
    fn support_default_is_free() {
        assert_eq!(SupportSpec::default(), SupportSpec::free());
    }

    #[test]
    fn support_dofs_roundtrip() {
        let s = SupportSpec::fixed();
        let dofs = s.dofs();
        let s2 = SupportSpec::custom(dofs);
        assert_eq!(s, s2);
    }

    // ── Support operation tests ───────────────────────────────────────

    #[test]
    fn set_support_fixed() {
        let mut m = EditModel::new();
        let id = m.add_node(0.0, 0.0, 0.0).unwrap();
        m.set_support(id, SupportSpec::fixed()).unwrap();
        assert!(m.has_support(id));
        assert!(m.support(id).unwrap().is_fixed());
    }

    #[test]
    fn set_support_pinned() {
        let mut m = EditModel::new();
        let id = m.add_node(0.0, 0.0, 0.0).unwrap();
        m.set_support(id, SupportSpec::pinned()).unwrap();
        assert!(m.support(id).unwrap().is_pinned());
    }

    #[test]
    fn set_support_replaces_existing() {
        let mut m = EditModel::new();
        let id = m.add_node(0.0, 0.0, 0.0).unwrap();
        m.set_support(id, SupportSpec::fixed()).unwrap();
        m.set_support(id, SupportSpec::pinned()).unwrap();
        assert!(m.support(id).unwrap().is_pinned());
        assert!(!m.support(id).unwrap().is_fixed());
    }

    #[test]
    fn clear_support() {
        let mut m = EditModel::new();
        let id = m.add_node(0.0, 0.0, 0.0).unwrap();
        m.set_support(id, SupportSpec::fixed()).unwrap();
        assert!(m.has_support(id));
        m.clear_support(id).unwrap();
        assert!(!m.has_support(id));
        assert!(m.support(id).is_none());
    }

    #[test]
    fn clear_support_nonexistent_is_ok() {
        let mut m = EditModel::new();
        let id = m.add_node(0.0, 0.0, 0.0).unwrap();
        m.clear_support(id).unwrap();
    }

    #[test]
    fn set_support_nonexistent_node_fails() {
        let mut m = EditModel::new();
        assert_eq!(
            m.set_support(NodeId(99), SupportSpec::fixed()),
            Err(EditError::NodeNotFound(NodeId(99)))
        );
    }

    #[test]
    fn fix_node_delegates_to_set_support() {
        let mut m = EditModel::new();
        let id = m.add_node(0.0, 0.0, 0.0).unwrap();
        m.fix_node(id).unwrap();
        assert!(m.support(id).unwrap().is_fixed());
    }

    #[test]
    fn fix_dof_creates_custom_support() {
        let mut m = EditModel::new();
        let id = m.add_node(0.0, 0.0, 0.0).unwrap();
        m.fix_dof(id, Dof3D::Ux).unwrap();
        let s = m.support(id).unwrap();
        assert_eq!(s.n_constrained(), 1);
        assert_eq!(s.dof(Dof3D::Ux), Some(0.0));
        assert_eq!(s.dof(Dof3D::Uy), None);
        assert_eq!(s.label(), "Custom");
    }

    #[test]
    fn fix_dof_merges_into_existing() {
        let mut m = EditModel::new();
        let id = m.add_node(0.0, 0.0, 0.0).unwrap();
        m.fix_dof(id, Dof3D::Ux).unwrap();
        m.fix_dof(id, Dof3D::Uy).unwrap();
        m.fix_dof(id, Dof3D::Uz).unwrap();
        let s = m.support(id).unwrap();
        assert_eq!(s.n_constrained(), 3);
        assert!(s.is_pinned());
    }

    #[test]
    fn fix_dof_merges_into_fixed() {
        let mut m = EditModel::new();
        let id = m.add_node(0.0, 0.0, 0.0).unwrap();
        m.fix_node(id).unwrap();
        m.fix_dof(id, Dof3D::Ux).unwrap();
        let s = m.support(id).unwrap();
        assert!(s.is_fixed());
        assert_eq!(s.n_constrained(), 6);
    }

    #[test]
    fn delete_node_removes_support() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        m.add_member(a, b).unwrap();
        m.set_support(b, SupportSpec::fixed()).unwrap();
        assert!(m.has_support(b));
        m.delete_member(m.members()[0].id).unwrap();
        m.delete_node(b).unwrap();
        assert!(!m.has_support(b));
    }

    #[test]
    fn supports_list() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        m.set_support(a, SupportSpec::fixed()).unwrap();
        m.set_support(b, SupportSpec::pinned()).unwrap();
        let sups = m.supports();
        assert_eq!(sups.len(), 2);
    }

    // ── Conversion with supports ─────────────────────────────────────

    #[test]
    fn to_frame_model_with_pinned_support_solves() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(4.0, 0.0, 0.0).unwrap();
        let c = m.add_node(0.0, 3.0, 0.0).unwrap();
        let d = m.add_node(4.0, 3.0, 0.0).unwrap();
        m.add_member(a, c).unwrap();
        m.add_member(c, d).unwrap();
        m.add_member(b, d).unwrap();
        m.set_support(a, SupportSpec::fixed()).unwrap();
        m.set_support(b, SupportSpec::fixed()).unwrap();
        m.set_nodal_load(c, NodalLoadSpec::new(5e3, 0.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        let fm = m.to_frame_model().unwrap();
        assert!(fm.solve().is_ok());
    }

    #[test]
    fn to_frame_model_custom_support_solves() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(4.0, 0.0, 0.0).unwrap();
        let c = m.add_node(0.0, 3.0, 0.0).unwrap();
        let d = m.add_node(4.0, 3.0, 0.0).unwrap();
        m.add_member(a, c).unwrap();
        m.add_member(c, d).unwrap();
        m.add_member(b, d).unwrap();
        let mut spec = SupportSpec::fixed();
        spec.set_dof(Dof3D::Rz, None);
        m.set_support(a, spec).unwrap();
        m.set_support(b, SupportSpec::fixed()).unwrap();
        m.set_nodal_load(c, NodalLoadSpec::new(5e3, 0.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        let fm = m.to_frame_model().unwrap();
        assert!(fm.solve().is_ok());
    }

    // ── NodalLoadSpec tests ───────────────────────────────────────────

    #[test]
    fn nodal_load_default_is_zero() {
        let s = NodalLoadSpec::default();
        assert!(s.is_zero());
        assert!(s.is_finite());
    }

    #[test]
    fn nodal_load_zero() {
        let s = NodalLoadSpec::zero();
        assert!(s.is_zero());
        assert_eq!(s.fx, 0.0);
        assert_eq!(s.mz, 0.0);
    }

    #[test]
    fn nodal_load_new() {
        let s = NodalLoadSpec::new(1e3, -2e3, 3e3, 4e2, -5e2, 6e2);
        assert!(!s.is_zero());
        assert!(s.is_finite());
        assert_eq!(s.fx, 1e3);
        assert_eq!(s.fy, -2e3);
        assert_eq!(s.fz, 3e3);
        assert_eq!(s.mx, 4e2);
        assert_eq!(s.my, -5e2);
        assert_eq!(s.mz, 6e2);
    }

    #[test]
    fn nodal_load_is_finite_rejects_nan() {
        let s = NodalLoadSpec::new(f64::NAN, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert!(!s.is_finite());
    }

    #[test]
    fn nodal_load_is_finite_rejects_infinity() {
        let s = NodalLoadSpec::new(0.0, f64::INFINITY, 0.0, 0.0, 0.0, 0.0);
        assert!(!s.is_finite());
    }

    #[test]
    fn nodal_load_force_magnitude() {
        let s = NodalLoadSpec::new(3.0, 4.0, 0.0, 0.0, 0.0, 0.0);
        assert!((s.force_magnitude() - 5.0).abs() < 1e-15);
    }

    #[test]
    fn nodal_load_moment_magnitude() {
        let s = NodalLoadSpec::new(0.0, 0.0, 0.0, 0.0, 3.0, 4.0);
        assert!((s.moment_magnitude() - 5.0).abs() < 1e-15);
    }

    #[test]
    fn nodal_load_as_array() {
        let s = NodalLoadSpec::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        assert_eq!(s.as_array(), [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    }

    // ── Nodal load operation tests ───────────────────────────────────

    #[test]
    fn set_nodal_load_success() {
        let mut m = EditModel::new();
        let id = m.add_node(0.0, 0.0, 0.0).unwrap();
        m.set_nodal_load(id, NodalLoadSpec::new(1e3, 0.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        assert!(m.has_nodal_load(id));
        let load = m.nodal_load(id).unwrap();
        assert_eq!(load.fx, 1e3);
    }

    #[test]
    fn set_nodal_load_replaces_existing() {
        let mut m = EditModel::new();
        let id = m.add_node(0.0, 0.0, 0.0).unwrap();
        m.set_nodal_load(id, NodalLoadSpec::new(1e3, 0.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        m.set_nodal_load(id, NodalLoadSpec::new(2e3, 1e3, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        let load = m.nodal_load(id).unwrap();
        assert_eq!(load.fx, 2e3);
        assert_eq!(load.fy, 1e3);
    }

    #[test]
    fn set_nodal_load_rejects_nonexistent_node() {
        let mut m = EditModel::new();
        assert_eq!(
            m.set_nodal_load(NodeId(99), NodalLoadSpec::zero()),
            Err(EditError::NodeNotFound(NodeId(99)))
        );
    }

    #[test]
    fn set_nodal_load_rejects_nan() {
        let mut m = EditModel::new();
        let id = m.add_node(0.0, 0.0, 0.0).unwrap();
        assert_eq!(
            m.set_nodal_load(id, NodalLoadSpec::new(f64::NAN, 0.0, 0.0, 0.0, 0.0, 0.0)),
            Err(EditError::NonFiniteCoordinate)
        );
    }

    #[test]
    fn set_nodal_load_rejects_infinity() {
        let mut m = EditModel::new();
        let id = m.add_node(0.0, 0.0, 0.0).unwrap();
        assert_eq!(
            m.set_nodal_load(
                id,
                NodalLoadSpec::new(0.0, 0.0, 0.0, 0.0, f64::INFINITY, 0.0)
            ),
            Err(EditError::NonFiniteCoordinate)
        );
    }

    #[test]
    fn clear_nodal_load() {
        let mut m = EditModel::new();
        let id = m.add_node(0.0, 0.0, 0.0).unwrap();
        m.set_nodal_load(id, NodalLoadSpec::new(1e3, 0.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        assert!(m.has_nodal_load(id));
        m.clear_nodal_load(id).unwrap();
        assert!(!m.has_nodal_load(id));
        assert!(m.nodal_load(id).is_none());
    }

    #[test]
    fn clear_nodal_load_nonexistent_is_ok() {
        let mut m = EditModel::new();
        let id = m.add_node(0.0, 0.0, 0.0).unwrap();
        m.clear_nodal_load(id).unwrap();
    }

    #[test]
    fn clear_nodal_load_nonexistent_node_fails() {
        let mut m = EditModel::new();
        assert_eq!(
            m.clear_nodal_load(NodeId(99)),
            Err(EditError::NodeNotFound(NodeId(99)))
        );
    }

    #[test]
    fn delete_node_removes_load() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        m.add_member(a, b).unwrap();
        m.set_nodal_load(b, NodalLoadSpec::new(1e3, 0.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        assert!(m.has_nodal_load(b));
        m.delete_member(m.members()[0].id).unwrap();
        m.delete_node(b).unwrap();
        assert!(!m.has_nodal_load(b));
    }

    #[test]
    fn nodal_loads_list() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        m.set_nodal_load(a, NodalLoadSpec::new(1e3, 0.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        m.set_nodal_load(b, NodalLoadSpec::new(0.0, 2e3, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        let loads = m.nodal_loads();
        assert_eq!(loads.len(), 2);
    }

    #[test]
    fn zero_load_is_valid() {
        let mut m = EditModel::new();
        let id = m.add_node(0.0, 0.0, 0.0).unwrap();
        m.set_nodal_load(id, NodalLoadSpec::zero()).unwrap();
        assert!(m.has_nodal_load(id));
        assert!(m.nodal_load(id).unwrap().is_zero());
    }

    #[test]
    fn support_and_load_coexist() {
        let mut m = EditModel::new();
        let id = m.add_node(0.0, 0.0, 0.0).unwrap();
        m.set_support(id, SupportSpec::fixed()).unwrap();
        m.set_nodal_load(id, NodalLoadSpec::new(1e3, 0.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        assert!(m.has_support(id));
        assert!(m.has_nodal_load(id));
    }

    #[test]
    fn to_frame_model_with_edited_load_solves() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(4.0, 0.0, 0.0).unwrap();
        let c = m.add_node(0.0, 3.0, 0.0).unwrap();
        let d = m.add_node(4.0, 3.0, 0.0).unwrap();
        m.add_member(a, c).unwrap();
        m.add_member(c, d).unwrap();
        m.add_member(b, d).unwrap();
        m.set_support(a, SupportSpec::fixed()).unwrap();
        m.set_support(b, SupportSpec::fixed()).unwrap();
        m.set_nodal_load(c, NodalLoadSpec::new(0.0, -1e4, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        let fm = m.to_frame_model().unwrap();
        assert!(fm.solve().is_ok());
    }

    // ── Member section / ref_vec tests ───────────────────────────────

    #[test]
    fn set_member_section_success() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        m.set_member_section(mid, 210e9, 81e9, 2e-3, 4e-6, 3e-6, 5e-6)
            .unwrap();
        let s = m.member_section(mid).unwrap();
        assert!((s.E() - 210e9).abs() < 1e-3);
        assert!((s.G() - 81e9).abs() < 1e-3);
        assert!((s.area() - 2e-3).abs() < 1e-15);
        assert!((s.iy() - 4e-6).abs() < 1e-15);
        assert!((s.iz() - 3e-6).abs() < 1e-15);
        assert!((s.j() - 5e-6).abs() < 1e-15);
    }

    #[test]
    fn set_member_section_rejects_nonexistent() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        m.add_member(a, b).unwrap();
        let err = m
            .set_member_section(MemberId(99), 200e9, 80e9, 1e-3, 2e-6, 1e-6, 3e-6)
            .unwrap_err();
        assert!(matches!(err, EditError::MemberNotFound(MemberId(99))));
    }

    #[test]
    fn set_member_section_rejects_zero() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        let err = m
            .set_member_section(mid, 0.0, 80e9, 1e-3, 2e-6, 1e-6, 3e-6)
            .unwrap_err();
        assert!(matches!(err, EditError::InvalidSection(_)));
    }

    #[test]
    fn set_member_section_rejects_negative() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        let err = m
            .set_member_section(mid, 200e9, 80e9, -1e-3, 2e-6, 1e-6, 3e-6)
            .unwrap_err();
        assert!(matches!(err, EditError::InvalidSection(_)));
    }

    #[test]
    fn set_member_section_rejects_nan() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        let err = m
            .set_member_section(mid, f64::NAN, 80e9, 1e-3, 2e-6, 1e-6, 3e-6)
            .unwrap_err();
        assert!(matches!(err, EditError::InvalidSection(_)));
    }

    #[test]
    fn set_member_section_rejects_infinity() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        let err = m
            .set_member_section(mid, 200e9, f64::INFINITY, 1e-3, 2e-6, 1e-6, 3e-6)
            .unwrap_err();
        assert!(matches!(err, EditError::InvalidSection(_)));
    }

    #[test]
    fn set_member_section_atomic_on_failure() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        let original = m.member_section(mid).unwrap().E();
        let _ = m.set_member_section(mid, -1.0, 80e9, 1e-3, 2e-6, 1e-6, 3e-6);
        assert!((m.member_section(mid).unwrap().E() - original).abs() < 1e-15);
    }

    #[test]
    fn set_member_section_does_not_affect_others() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let c = m.add_node(6.0, 0.0, 0.0).unwrap();
        let m1 = m.add_member(a, b).unwrap();
        let m2 = m.add_member(b, c).unwrap();
        m.set_member_section(m1, 210e9, 81e9, 2e-3, 4e-6, 3e-6, 5e-6)
            .unwrap();
        let s2 = m.member_section(m2).unwrap();
        assert!((s2.E() - 200e9).abs() < 1e-3);
    }

    #[test]
    fn member_section_query_none_for_nonexistent() {
        let m = EditModel::new();
        assert!(m.member_section(MemberId(99)).is_none());
    }

    #[test]
    fn set_member_ref_vec_success() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        m.set_member_ref_vec(mid, [0.0, 1.0, 0.0]).unwrap();
        assert_eq!(m.member_ref_vec(mid).unwrap(), [0.0, 1.0, 0.0]);
    }

    #[test]
    fn set_member_ref_vec_rejects_nonexistent() {
        let mut m = EditModel::new();
        let err = m
            .set_member_ref_vec(MemberId(99), [0.0, 0.0, 1.0])
            .unwrap_err();
        assert!(matches!(err, EditError::MemberNotFound(MemberId(99))));
    }

    #[test]
    fn set_member_ref_vec_rejects_zero() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        let err = m.set_member_ref_vec(mid, [0.0, 0.0, 0.0]).unwrap_err();
        assert!(matches!(err, EditError::InvalidRefVec(_)));
    }

    #[test]
    fn set_member_ref_vec_rejects_nan() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        let err = m.set_member_ref_vec(mid, [f64::NAN, 0.0, 1.0]).unwrap_err();
        assert!(matches!(err, EditError::InvalidRefVec(_)));
    }

    #[test]
    fn set_member_ref_vec_rejects_parallel() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        let err = m.set_member_ref_vec(mid, [1.0, 0.0, 0.0]).unwrap_err();
        assert!(matches!(err, EditError::InvalidRefVec(_)));
    }

    #[test]
    fn set_member_ref_vec_rejects_near_parallel() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(1.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        let err = m.set_member_ref_vec(mid, [1.0, 1e-10, 0.0]).unwrap_err();
        assert!(matches!(err, EditError::InvalidRefVec(_)));
    }

    #[test]
    fn set_member_ref_vec_atomic_on_failure() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        let original = m.member_ref_vec(mid).unwrap();
        let _ = m.set_member_ref_vec(mid, [1.0, 0.0, 0.0]);
        assert_eq!(m.member_ref_vec(mid).unwrap(), original);
    }

    #[test]
    fn member_ref_vec_query_none_for_nonexistent() {
        let m = EditModel::new();
        assert!(m.member_ref_vec(MemberId(99)).is_none());
    }

    #[test]
    fn section_edit_preserves_connectivity() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        m.set_member_section(mid, 210e9, 81e9, 2e-3, 4e-6, 3e-6, 5e-6)
            .unwrap();
        let member = m.member(mid).unwrap();
        assert_eq!(member.node_i, a);
        assert_eq!(member.node_j, b);
    }

    #[test]
    fn section_edit_preserves_supports_and_loads() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        m.set_support(a, SupportSpec::fixed()).unwrap();
        m.set_nodal_load(b, NodalLoadSpec::new(1e3, 0.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        m.set_member_section(mid, 210e9, 81e9, 2e-3, 4e-6, 3e-6, 5e-6)
            .unwrap();
        assert!(m.has_support(a));
        assert!(m.has_nodal_load(b));
    }

    #[test]
    fn to_frame_model_uses_edited_section() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        m.set_member_section(mid, 210e9, 81e9, 2e-3, 4e-6, 3e-6, 5e-6)
            .unwrap();
        let fm = m.to_frame_model().unwrap();
        let sec = fm.member(0).unwrap().section();
        assert!((sec.E() - 210e9).abs() < 1e-3);
        assert!((sec.area() - 2e-3).abs() < 1e-15);
    }

    #[test]
    fn to_frame_model_uses_edited_ref_vec() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        m.set_member_ref_vec(mid, [0.0, 1.0, 0.0]).unwrap();
        let fm = m.to_frame_model().unwrap();
        let rv = fm.member(0).unwrap().ref_vec();
        assert!((rv[1] - 1.0).abs() < 1e-15);
    }

    #[test]
    fn default_section_is_steel() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        let s = m.member_section(mid).unwrap();
        assert!((s.E() - 200e9).abs() < 1e-3);
        assert!((s.G() - 80e9).abs() < 1e-3);
        assert!((s.area() - 1e-3).abs() < 1e-15);
        assert!((s.iy() - 2e-6).abs() < 1e-15);
        assert!((s.iz() - 1e-6).abs() < 1e-15);
        assert!((s.j() - 3e-6).abs() < 1e-15);
    }

    // ── Cascade delete ────────────────────────────────────────────────

    #[test]
    fn cascade_delete_removes_attached_members() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let c = m.add_node(0.0, 3.0, 0.0).unwrap();
        let m1 = m.add_member(a, b).unwrap();
        let m2 = m.add_member(a, c).unwrap();
        let m3 = m.add_member(b, c).unwrap();

        let removed = m.delete_node_cascade(a).unwrap();
        assert_eq!(removed.len(), 2);
        assert!(removed.contains(&m1));
        assert!(removed.contains(&m2));
        assert!(m.member(m3).is_some(), "unrelated member must survive");
        assert_eq!(m.n_nodes(), 2);
        assert_eq!(m.n_members(), 1);
        // The surviving member still references existing nodes.
        let surv = m.member(m3).unwrap();
        assert!(m.node(surv.node_i).is_some());
        assert!(m.node(surv.node_j).is_some());
    }

    #[test]
    fn cascade_delete_removes_member_loads_and_node_support_load() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        m.set_member_uniform_load(mid, MemberUniformLoad3D::new(0.0, -1.0, 0.0))
            .unwrap();
        m.add_member_point_load(mid, MemberPointLoad3D::new(0.5, 0.0, -1.0, 0.0))
            .unwrap();
        m.set_support(b, SupportSpec::fixed()).unwrap();
        m.set_nodal_load(b, NodalLoadSpec::new(1e3, 0.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();

        m.delete_node_cascade(b).unwrap();
        assert_eq!(m.n_nodes(), 1);
        assert_eq!(m.n_members(), 0);
        assert!(m.member_uniform_loads().is_empty());
        assert!(m.member_point_loads().is_empty());
        assert!(!m.has_support(b));
        assert!(!m.has_nodal_load(b));
    }

    #[test]
    fn cascade_delete_not_found() {
        let mut m = EditModel::new();
        assert_eq!(
            m.delete_node_cascade(NodeId(99)),
            Err(EditError::NodeNotFound(NodeId(99)))
        );
    }

    #[test]
    fn cascade_delete_keeps_ids_stable() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let c = m.add_node(0.0, 3.0, 0.0).unwrap();
        let d = m.add_node(0.0, 0.0, 3.0).unwrap();
        m.add_member(a, b).unwrap();
        let keep = m.add_member(c, d).unwrap();

        m.delete_node_cascade(a).unwrap();
        // Remaining nodes keep their IDs and the surviving member its ID.
        let ids: Vec<NodeId> = m.nodes().iter().map(|n| n.id).collect();
        assert_eq!(ids, vec![b, c, d]);
        assert!(m.member(keep).is_some());
        // Rebuild a frame model to prove the surviving structure is consistent.
        assert!(m.to_frame_model().is_ok());
    }

    // ── Connectivity edit ─────────────────────────────────────────────

    #[test]
    fn set_member_nodes_success() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let c = m.add_node(0.0, 3.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        m.set_member_nodes(mid, b, c).unwrap();
        let mm = m.member(mid).unwrap();
        assert_eq!(mm.node_i, b);
        assert_eq!(mm.node_j, c);
    }

    #[test]
    fn set_member_nodes_noop_is_ok() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        let rev = m.revision();
        m.set_member_nodes(mid, a, b).unwrap();
        assert_eq!(m.revision(), rev);
    }

    #[test]
    fn set_member_nodes_rejects_bad_inputs() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let c = m.add_node(6.0, 0.0, 0.0).unwrap();
        let mid = m.add_member(a, b).unwrap();
        m.add_member(b, c).unwrap();

        assert_eq!(
            m.set_member_nodes(mid, a, a),
            Err(EditError::SelfConnection)
        );
        assert_eq!(
            m.set_member_nodes(mid, a, NodeId(99)),
            Err(EditError::NodeNotFound(NodeId(99)))
        );
        // Re-wiring a—b onto b—c duplicates the existing b—c member.
        assert_eq!(
            m.set_member_nodes(mid, b, c),
            Err(EditError::DuplicateMember)
        );
        // Unchanged after every rejected attempt.
        assert_eq!(m.member(mid).unwrap().node_i, a);
        assert_eq!(m.member(mid).unwrap().node_j, b);
    }

    #[test]
    fn set_member_nodes_rejects_zero_length() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let c = m.add_node(0.0, 3.0, 0.0).unwrap();
        let d = m.add_node(0.0, 3.0, 0.0).unwrap(); // coincident with c
        let mid = m.add_member(a, b).unwrap();
        assert_eq!(
            m.set_member_nodes(mid, c, d),
            Err(EditError::ZeroLengthMember)
        );
    }

    // ── Load cases (MIDAS-style multi-case workflows) ─────────────────

    #[test]
    fn new_model_has_one_default_case() {
        let m = EditModel::new();
        assert_eq!(m.n_load_cases(), 1);
        let case = &m.load_cases()[0];
        assert_eq!(case.id, LoadCaseId(0));
        assert_eq!(case.name, DEFAULT_CASE_NAME);
        assert!(!case.self_weight);
        assert_eq!(m.active_case(), LoadCaseId(0));
        assert_eq!(m.next_load_case_id(), 1);
        assert_eq!(m.n_combinations(), 0);
    }

    #[test]
    fn add_load_case_becomes_active() {
        let mut m = EditModel::new();
        let ll = m.add_load_case("LL").unwrap();
        assert_eq!(m.active_case(), ll);
        assert_eq!(m.n_load_cases(), 2);
        assert!(m.load_case(ll).is_some());
        assert_eq!(m.load_case(ll).unwrap().name, "LL");
        assert_eq!(m.next_load_case_id(), 2);
    }

    #[test]
    fn add_load_case_rejects_empty_name() {
        let mut m = EditModel::new();
        assert!(m.add_load_case("   ").is_err());
        assert_eq!(m.n_load_cases(), 1);
    }

    #[test]
    fn loads_are_scoped_to_their_case() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        m.add_member(a, b).unwrap();
        m.set_nodal_load(b, NodalLoadSpec::new(1e3, 0.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        assert_eq!(m.n_nodal_loads(), 1);

        // A new case starts empty — the DL load is not visible in LL.
        let ll = m.add_load_case("LL").unwrap();
        assert_eq!(m.n_nodal_loads(), 0);
        assert!(!m.has_nodal_load(b));
        assert!(m.nodal_load(b).is_none());
        assert!(m.nodal_loads().is_empty());

        // The load is still there in its own case.
        assert_eq!(m.nodal_loads_in(LoadCaseId(0)).len(), 1);
        assert_eq!(m.nodal_loads_all().len(), 1);

        // Two loads of the same node in different cases are independent.
        m.set_nodal_load(b, NodalLoadSpec::new(0.0, -5e3, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        assert_eq!(m.nodal_loads_all().len(), 2);
        assert_eq!(m.nodal_loads_in(ll).len(), 1);

        // Switching back restores the DL view.
        m.set_active_case(LoadCaseId(0)).unwrap();
        assert_eq!(m.nodal_load(b).unwrap().fx, 1e3);
    }

    #[test]
    fn set_active_case_rejects_unknown_case() {
        let mut m = EditModel::new();
        assert!(m.set_active_case(LoadCaseId(42)).is_err());
        assert_eq!(m.active_case(), LoadCaseId(0));
    }

    #[test]
    fn delete_load_case_removes_only_its_loads() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mem = m.add_member(a, b).unwrap();
        m.set_nodal_load(b, NodalLoadSpec::new(1e3, 0.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        let ll = m.add_load_case("LL").unwrap();
        m.set_member_uniform_load(mem, MemberUniformLoad3D::new(0.0, -1e3, 0.0))
            .unwrap();
        m.add_member_point_load(mem, MemberPointLoad3D::new(0.5, 0.0, -2e3, 0.0))
            .unwrap();

        m.delete_load_case(ll).unwrap();
        assert_eq!(m.n_load_cases(), 1);
        // Deleting the active case selects the first remaining case.
        assert_eq!(m.active_case(), LoadCaseId(0));
        // The surviving case's load is untouched.
        assert_eq!(m.n_nodal_loads(), 1);
        m.set_active_case(LoadCaseId(0)).unwrap();
        assert_eq!(m.nodal_load(b).unwrap().fx, 1e3);
        assert_eq!(m.nodal_loads_all().len(), 1);
        assert!(m.member_uniform_loads_all().is_empty());
        assert!(m.member_point_loads_all().is_empty());
    }

    #[test]
    fn cannot_delete_the_last_load_case() {
        let mut m = EditModel::new();
        assert_eq!(
            m.delete_load_case(LoadCaseId(0)),
            Err(EditError::LastLoadCase)
        );
        assert_eq!(m.n_load_cases(), 1);
    }

    #[test]
    fn delete_unknown_load_case_errors() {
        let mut m = EditModel::new();
        assert!(matches!(
            m.delete_load_case(LoadCaseId(9)),
            Err(EditError::LoadCaseNotFound(_))
        ));
    }

    #[test]
    fn rename_load_case_updates_name() {
        let mut m = EditModel::new();
        m.rename_load_case(LoadCaseId(0), "Dead").unwrap();
        assert_eq!(m.load_case(LoadCaseId(0)).unwrap().name, "Dead");
        m.rename_load_case(LoadCaseId(0), "  Padded  ").unwrap();
        assert_eq!(m.load_case(LoadCaseId(0)).unwrap().name, "Padded");
        assert!(m.rename_load_case(LoadCaseId(0), "").is_err());
        assert!(m.rename_load_case(LoadCaseId(7), "Nope").is_err());
    }

    #[test]
    fn self_weight_flag_toggles() {
        let mut m = EditModel::new();
        m.set_case_self_weight(LoadCaseId(0), true).unwrap();
        assert!(m.load_case(LoadCaseId(0)).unwrap().self_weight);
        // Idempotent — no revision bump on a no-op.
        let rev = m.revision();
        m.set_case_self_weight(LoadCaseId(0), true).unwrap();
        assert_eq!(m.revision(), rev);
        assert!(m.set_case_self_weight(LoadCaseId(5), true).is_err());
    }

    #[test]
    fn member_density_setters_and_self_weight() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(4.0, 0.0, 0.0).unwrap();
        let mem = m.add_member(a, b).unwrap();
        // Default is structural steel.
        assert_eq!(m.member_density(mem), Some(DEFAULT_DENSITY));
        // Weight of a 4 m steel member with A = 1e-3 m².
        let expected = DEFAULT_DENSITY * GRAVITY * m.member(mem).unwrap().section.area() * 4.0;
        assert!((m.member_self_weight(mem).unwrap() - expected).abs() < 1e-9);

        m.set_member_density(mem, 2700.0).unwrap();
        assert_eq!(m.member_density(mem), Some(2700.0));
        assert!(m.set_member_density(mem, 0.0).is_err());
        assert!(m.set_member_density(mem, f64::NAN).is_err());
        assert!(m.set_member_density(MemberId(42), 2700.0).is_err());
    }

    #[test]
    fn to_frame_model_for_case_includes_only_that_case() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        m.add_member(a, b).unwrap();
        m.set_support(a, SupportSpec::fixed()).unwrap();
        m.set_nodal_load(b, NodalLoadSpec::new(1e3, 0.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        let ll = m.add_load_case("LL").unwrap();
        m.set_nodal_load(b, NodalLoadSpec::new(0.0, -5e3, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        // Solve each case through the to_frame_model boundary: the DL
        // FrameModel must carry the DL load only, LL the LL load only.
        let fm_dl = m.to_frame_model_for_case(LoadCaseId(0)).unwrap();
        let fm_ll = m.to_frame_model_for_case(ll).unwrap();
        // A fixed-free model under a single load: the tip displacement must
        // differ between cases (different loads → different response).
        let res_dl = fm_dl.solve().unwrap();
        let res_ll = fm_ll.solve().unwrap();
        assert_ne!(res_dl.displacements, res_ll.displacements);

        // An unknown case is rejected.
        assert!(m.to_frame_model_for_case(LoadCaseId(99)).is_err());
    }

    #[test]
    fn self_weight_matches_equivalent_uniform_load() {
        // Cantilever along +X, area 1e-3, steel 7850 kg/m³.
        let mut sw = EditModel::new();
        let a = sw.add_node(0.0, 0.0, 0.0).unwrap();
        let b = sw.add_node(2.0, 0.0, 0.0).unwrap();
        let mem = sw.add_member(a, b).unwrap();
        sw.set_support(a, SupportSpec::fixed()).unwrap();
        sw.set_case_self_weight(LoadCaseId(0), true).unwrap();

        // Self weight of the member: ρ·g·A·L (N) pointing −Y.
        let weight = DEFAULT_DENSITY * GRAVITY * sw.member(mem).unwrap().section.area() * 2.0;
        let sw_res = sw.to_frame_model().unwrap().solve().unwrap();
        let fy_reaction = sw_res.reactions[1];
        assert!(
            (fy_reaction - weight).abs() / weight < 1e-12,
            "vertical reaction {fy_reaction} should equal the weight {weight}"
        );

        // The same deflected shape results from the equivalent explicit
        // load: a uniform −Y line load of w = ρ·g·A (N/m).
        let w = DEFAULT_DENSITY * GRAVITY * sw.member(mem).unwrap().section.area();
        let mut manual = sw.clone();
        manual.set_case_self_weight(LoadCaseId(0), false).unwrap();
        manual
            .set_member_uniform_load(mem, MemberUniformLoad3D::new(0.0, 0.0, w))
            .unwrap();
        let manual_res = manual.to_frame_model().unwrap().solve().unwrap();
        for (x, y) in sw_res
            .displacements
            .iter()
            .zip(manual_res.displacements.iter())
        {
            assert!((x - y).abs() < 1e-12 * (1.0 + x.abs()), "disp {x} vs {y}");
        }

        // Analytic tip deflection: δ = w·L⁴ / (8·E·I) — the load resolves
        // onto the strong axis (local y = global Z, local z = −Y) so the
        // governing inertia is `iy`.
        let iy = sw.member(mem).unwrap().section.iy();
        let e = sw.member(mem).unwrap().section.E();
        let analytic = w * (2.0_f64).powi(4) / (8.0 * e * iy);
        // Node B (index 1) is the free tip; its global-Y displacement sits
        // at DOF index 6·1 + 1.
        let tip_uy = sw_res.displacements[7];
        assert!(
            (tip_uy.abs() - analytic).abs() / analytic < 1e-9,
            "tip deflection {tip_uy} vs analytic {analytic}"
        );
        // Gravity points down.
        assert!(tip_uy < 0.0, "self weight must push the tip down");
    }

    #[test]
    fn self_weight_of_inclined_member_balances() {
        // A 45° rafter: self weight is still global −Y, so the vertical
        // components of the two support reactions must add up to the weight.
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 3.0, 0.0).unwrap();
        let mem = m.add_member(a, b).unwrap();
        m.set_support(a, SupportSpec::fixed()).unwrap();
        m.set_support(b, SupportSpec::fixed()).unwrap();
        m.set_case_self_weight(LoadCaseId(0), true).unwrap();

        let weight = m.member_self_weight(mem).unwrap();
        let res = m.to_frame_model().unwrap().solve().unwrap();
        let ry = res.reactions[1] + res.reactions[6 + 1];
        assert!(
            (ry - weight).abs() / weight < 1e-12,
            "ΣRy {ry} should equal weight {weight}"
        );
    }

    #[test]
    fn combinations_crud_and_validation() {
        let mut m = EditModel::new();
        let ll = m.add_load_case("LL").unwrap();
        let combo = m.add_load_combination("1.35DL+1.4LL").unwrap();

        // Unknown cases and non-finite factors are rejected.
        assert!(
            m.set_combination_terms(
                combo,
                vec![ComboTerm {
                    case: LoadCaseId(99),
                    factor: 1.0
                }]
            )
            .is_err()
        );
        assert!(
            m.set_combination_terms(
                combo,
                vec![ComboTerm {
                    case: LoadCaseId(0),
                    factor: f64::NAN
                }]
            )
            .is_err()
        );

        // Duplicate cases merge into a single summed factor.
        m.set_combination_terms(
            combo,
            vec![
                ComboTerm {
                    case: LoadCaseId(0),
                    factor: 1.35,
                },
                ComboTerm {
                    case: ll,
                    factor: 1.4,
                },
                ComboTerm {
                    case: LoadCaseId(0),
                    factor: 0.0,
                },
            ],
        )
        .unwrap();
        let terms = &m.combination(combo).unwrap().terms;
        assert_eq!(terms.len(), 2, "duplicate cases should merge");
        assert_eq!(
            m.combination_cases(combo),
            vec![LoadCaseId(0), ll],
            "reference list follows the merged terms"
        );
        assert!(m.dangling_combination_cases(combo).is_empty());

        // Rename and delete.
        m.rename_load_combination(combo, "Combo1").unwrap();
        assert_eq!(m.combination(combo).unwrap().name, "Combo1");
        m.delete_load_combination(combo).unwrap();
        assert_eq!(m.n_combinations(), 0);
        assert!(m.delete_load_combination(combo).is_err());
    }

    #[test]
    fn dangling_combination_cases_are_reported() {
        let mut m = EditModel::new();
        let combo = m.add_load_combination("1.4LL").unwrap();
        let ll = m.add_load_case("LL").unwrap();
        m.set_combination_terms(
            combo,
            vec![ComboTerm {
                case: ll,
                factor: 1.4,
            }],
        )
        .unwrap();
        m.delete_load_case(ll).unwrap();
        // The combination keeps its term (visible in the editor), but the
        // model reports the missing reference so the solve can fail loudly.
        assert_eq!(m.dangling_combination_cases(combo), vec![ll]);
    }

    #[test]
    fn fingerprint_changes_with_case_data() {
        let mut m = EditModel::new();
        let base = m.fingerprint();
        m.add_load_case("LL").unwrap();
        let with_case = m.fingerprint();
        assert_ne!(base, with_case);

        let combo = m.add_load_combination("1.4LL").unwrap();
        m.set_combination_terms(
            combo,
            vec![ComboTerm {
                case: m.active_case(),
                factor: 1.4,
            }],
        )
        .unwrap();
        assert_ne!(with_case, m.fingerprint());

        m.set_member_density(MemberId(99), 100.0).ok();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(1.0, 0.0, 0.0).unwrap();
        let mem = m.add_member(a, b).unwrap();
        m.set_member_density(mem, 2700.0).unwrap();
        assert_ne!(m.fingerprint(), with_case);
    }
    // ── Member id stability after edits ───────────────────────────────

    #[test]
    fn member_indices_track_after_delete() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let c = m.add_node(0.0, 3.0, 0.0).unwrap();
        let m1 = m.add_member(a, b).unwrap();
        let m2 = m.add_member(b, c).unwrap();
        m.delete_member(m1).unwrap();
        // m2 is now at index 0 but keeps its stable ID.
        assert_eq!(m.member_index(m2), Some(0));
        assert_eq!(m.member_index(m1), None);
    }
    // ── Member end releases (pins) ─────────────────────────────────────

    #[test]
    fn new_members_are_rigid() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mem = m.add_member(a, b).unwrap();
        assert_eq!(m.member_releases(mem), Some([false; 6]));
        assert_eq!(EditModel::release_label([false; 6]), None);
    }

    #[test]
    fn set_member_releases_validates_and_labels() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mem = m.add_member(a, b).unwrap();

        m.set_member_releases(mem, [false, false, true, false, false, false])
            .unwrap();
        assert_eq!(
            m.member_releases(mem),
            Some([false, false, true, false, false, false])
        );
        assert_eq!(
            EditModel::release_label([false, false, true, false, false, false]),
            Some("Pin i")
        );

        m.set_member_releases(mem, [true, true, true, true, true, true])
            .unwrap();
        assert_eq!(
            EditModel::release_label(m.member_releases(mem).unwrap()),
            Some("Pin i+j")
        );

        // Unknown member.
        assert!(matches!(
            m.set_member_releases(MemberId(42), [true; 6]),
            Err(EditError::MemberNotFound(_))
        ));
        // No-op writes leave the revision alone.
        let rev = m.revision();
        m.set_member_releases(mem, [true; 6]).unwrap();
        assert_eq!(m.revision(), rev);
    }

    #[test]
    fn set_member_release_toggles_one_axis() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mem = m.add_member(a, b).unwrap();

        m.set_member_release(mem, false, 2, true).unwrap();
        assert_eq!(
            m.member_releases(mem),
            Some([false, false, true, false, false, false])
        );
        m.set_member_release(mem, true, 0, true).unwrap();
        assert_eq!(
            m.member_releases(mem),
            Some([false, false, true, true, false, false])
        );

        // Out-of-range axis.
        assert!(m.set_member_release(mem, false, 3, true).is_err());
        assert_eq!(
            m.member_releases(mem),
            Some([false, false, true, true, false, false])
        );
    }

    #[test]
    fn to_frame_model_carries_releases_into_the_solver() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(4.0, 0.0, 0.0).unwrap();
        m.add_member(a, b).unwrap();
        m.set_member_releases(MemberId(0), [false, false, false, false, false, true])
            .unwrap();

        let fm = m.to_frame_model().unwrap();
        let releases = fm.member_releases(0).unwrap();
        assert!(releases.end_rz, "the release must reach FrameModel3D");
        assert!(!releases.start_rz);
        assert_eq!(fm.member_releases(1), None);
    }

    #[test]
    fn releases_change_the_fingerprint() {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        m.add_member(a, b).unwrap();
        let before = m.fingerprint();
        m.set_member_release(MemberId(0), true, 1, true).unwrap();
        assert_ne!(m.fingerprint(), before);
    }

    #[test]
    fn simply_supported_beam_via_release_matches_closed_form() {
        // End-to-end through EditModel → FrameModel3D: a beam whose far end
        // is pinned by the member release and rotationally restrained by a
        // sibling member reproduces the classic simply-supported rotation
        // qL³/(24 E I) at the free support.
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(0.0, 4.0, 0.0).unwrap();
        let c = m.add_node(6.0, 4.0, 0.0).unwrap();
        m.add_member(a, b).unwrap(); // column
        let beam = m.add_member(b, c).unwrap();
        // Pin the beam at the joint B (all three rotations released); the
        // sibling column restrains the joint's rotation, and C stays a
        // roller (translations only, rotations free through the rigid end).
        m.set_member_releases(beam, [true, true, true, false, false, false])
            .unwrap();
        m.set_support(a, SupportSpec::fixed()).unwrap();
        // Roller + twist restraint at C: releasing the beam's torsion at B
        // decouples the member from the joint twist, so C must carry it.
        m.set_support(
            c,
            SupportSpec::custom([Some(0.0), Some(0.0), Some(0.0), Some(0.0), None, None]),
        )
        .unwrap();
        m.set_member_uniform_load(beam, MemberUniformLoad3D::new(0.0, 0.0, 800.0))
            .unwrap();

        let res = m.to_frame_model().unwrap().solve().unwrap();
        let iy = m.member(beam).unwrap().section.iy();
        let e = m.member(beam).unwrap().section.E();
        let theta = 800.0 * 6.0_f64.powi(3) / (24.0 * e * iy);
        // Node C is the second node (index 2).
        let rz_c = res.displacements[2 * 6 + Dof3D::Rz.index()];
        assert!(
            (rz_c - theta).abs() / theta < 5e-4,
            "theta_C = {rz_c} vs {theta}"
        );
        // The pinned end carries no moment.
        let f = res.member_end_forces(1).unwrap();
        assert!(
            f[3].abs() < 1e-9 && f[4].abs() < 1e-9,
            "pin i: {:?}",
            &f[..5]
        );
    }
}
