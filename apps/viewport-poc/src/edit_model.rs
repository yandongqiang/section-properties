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

use structural_analysis::{Dof3D, FrameModel3D, FrameSection3D};

// ── Identifiers ────────────────────────────────────────────────────────

/// Stable node identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct NodeId(pub u32);

/// Stable member identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct MemberId(pub u32);

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

// ── Edit model ─────────────────────────────────────────────────────────

/// Editable structural model with stable IDs.
///
/// Stores nodes and members with stable identifiers that survive deletion.
/// Also stores supports (boundary conditions) and nodal loads.
#[derive(Debug, Clone)]
pub struct EditModel {
    nodes: Vec<EditNode>,
    members: Vec<EditMember>,
    supports: Vec<(NodeId, SupportSpec)>,
    nodal_loads: Vec<(NodeId, NodalLoadSpec)>,
    next_node_id: u32,
    next_member_id: u32,
}

impl Default for EditModel {
    fn default() -> Self {
        Self::new()
    }
}

impl EditModel {
    /// Create an empty model.
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            members: Vec::new(),
            supports: Vec::new(),
            nodal_loads: Vec::new(),
            next_node_id: 0,
            next_member_id: 0,
        }
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
        node.x = x;
        node.y = y;
        node.z = z;
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
        self.nodal_loads.retain(|(nid, _)| *nid != id);
        Ok(())
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
        });
        Ok(id)
    }

    /// Delete a member.
    pub fn delete_member(&mut self, id: MemberId) -> Result<(), EditError> {
        if self.members.iter().all(|m| m.id != id) {
            return Err(EditError::MemberNotFound(id));
        }
        self.members.retain(|m| m.id != id);
        Ok(())
    }

    /// Set the reference vector for a member.  Used by preset models.
    pub fn set_member_ref_vec(&mut self, id: MemberId, ref_vec: [f64; 3]) -> Result<(), EditError> {
        let member = self
            .members
            .iter_mut()
            .find(|m| m.id == id)
            .ok_or(EditError::MemberNotFound(id))?;
        member.ref_vec = ref_vec;
        Ok(())
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
            spec.set_dof(dof, Some(0.0));
        } else {
            let mut spec = SupportSpec::free();
            spec.set_dof(dof, Some(0.0));
            self.supports.push((id, spec));
        }
        Ok(())
    }

    /// Set the support specification for a node, replacing any existing
    /// support.
    pub fn set_support(&mut self, id: NodeId, spec: SupportSpec) -> Result<(), EditError> {
        if self.nodes.iter().all(|n| n.id != id) {
            return Err(EditError::NodeNotFound(id));
        }
        if let Some((_, existing)) = self.supports.iter_mut().find(|(nid, _)| *nid == id) {
            *existing = spec;
        } else {
            self.supports.push((id, spec));
        }
        Ok(())
    }

    /// Clear the support at a node, making all DOFs free.
    pub fn clear_support(&mut self, id: NodeId) -> Result<(), EditError> {
        if self.nodes.iter().all(|n| n.id != id) {
            return Err(EditError::NodeNotFound(id));
        }
        self.supports.retain(|(nid, _)| *nid != id);
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

    /// Set the nodal load at a node, replacing any existing load.
    ///
    /// One load record per node.  All components must be finite.
    pub fn set_nodal_load(&mut self, id: NodeId, spec: NodalLoadSpec) -> Result<(), EditError> {
        if self.nodes.iter().all(|n| n.id != id) {
            return Err(EditError::NodeNotFound(id));
        }
        if !spec.is_finite() {
            return Err(EditError::NonFiniteCoordinate);
        }
        if let Some((_, existing)) = self.nodal_loads.iter_mut().find(|(nid, _)| *nid == id) {
            *existing = spec;
        } else {
            self.nodal_loads.push((id, spec));
        }
        Ok(())
    }

    /// Clear the nodal load at a node.
    pub fn clear_nodal_load(&mut self, id: NodeId) -> Result<(), EditError> {
        if self.nodes.iter().all(|n| n.id != id) {
            return Err(EditError::NodeNotFound(id));
        }
        self.nodal_loads.retain(|(nid, _)| *nid != id);
        Ok(())
    }

    /// Get the nodal load at a node, if any.
    pub fn nodal_load(&self, id: NodeId) -> Option<&NodalLoadSpec> {
        self.nodal_loads
            .iter()
            .find(|(nid, _)| *nid == id)
            .map(|(_, spec)| spec)
    }

    /// Does the node have a nodal load?
    pub fn has_nodal_load(&self, id: NodeId) -> bool {
        self.nodal_loads.iter().any(|(nid, _)| *nid == id)
    }

    /// All nodal loads (node ID + spec), in insertion order.
    pub fn nodal_loads(&self) -> &[(NodeId, NodalLoadSpec)] {
        &self.nodal_loads
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

    /// Convert to a `FrameModel3D` for analysis.
    ///
    /// Nodes and members are added in storage order.  Fixed nodes and nodal
    /// loads are transferred.  Members use a default steel section and
    /// `[0, 0, 1]` reference vector.
    pub fn to_frame_model(&self) -> Result<FrameModel3D, EditError> {
        let mut model = FrameModel3D::new();
        let mut id_to_idx: HashMap<NodeId, usize> = HashMap::new();

        for node in &self.nodes {
            let idx = model
                .add_node(node.x, node.y, node.z)
                .map_err(|e| EditError::FrameModelBuild(format!("{e:?}")))?;
            id_to_idx.insert(node.id, idx);
        }

        let section = default_section();
        for member in &self.members {
            let ni = *id_to_idx
                .get(&member.node_i)
                .ok_or(EditError::NodeNotFound(member.node_i))?;
            let nj = *id_to_idx
                .get(&member.node_j)
                .ok_or(EditError::NodeNotFound(member.node_j))?;
            model
                .add_member(ni, nj, section, member.ref_vec)
                .map_err(|e| EditError::FrameModelBuild(format!("{e:?}")))?;
        }

        for (nid, spec) in &self.supports {
            let idx = *id_to_idx.get(nid).ok_or(EditError::NodeNotFound(*nid))?;
            if spec.is_fixed() {
                model
                    .fix_node(idx)
                    .map_err(|e| EditError::FrameModelBuild(format!("{e:?}")))?;
            } else if spec.is_pinned() {
                model
                    .pin_node(idx)
                    .map_err(|e| EditError::FrameModelBuild(format!("{e:?}")))?;
            } else {
                for dof in Dof3D::ALL {
                    if let Some(value) = spec.dof(dof) {
                        model
                            .fix_dof(idx, dof, value)
                            .map_err(|e| EditError::FrameModelBuild(format!("{e:?}")))?;
                    }
                }
            }
        }

        for (nid, spec) in &self.nodal_loads {
            let idx = *id_to_idx.get(nid).ok_or(EditError::NodeNotFound(*nid))?;
            model
                .add_nodal_load(idx, spec.fx, spec.fy, spec.fz, spec.mx, spec.my, spec.mz)
                .map_err(|e| EditError::FrameModelBuild(format!("{e:?}")))?;
        }

        Ok(model)
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
}
