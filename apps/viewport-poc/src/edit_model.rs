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

// ── Edit model ─────────────────────────────────────────────────────────

/// Editable structural model with stable IDs.
///
/// Stores nodes and members with stable identifiers that survive deletion.
/// Also stores fixed nodes and nodal loads (not editable in this phase, but
/// needed for preset model compatibility).
#[derive(Debug, Clone)]
pub struct EditModel {
    nodes: Vec<EditNode>,
    members: Vec<EditMember>,
    fixed_nodes: Vec<NodeId>,
    fixed_dofs: Vec<(NodeId, Dof3D)>,
    nodal_loads: Vec<(NodeId, [f64; 6])>,
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
            fixed_nodes: Vec::new(),
            fixed_dofs: Vec::new(),
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
        self.fixed_nodes.retain(|&nid| nid != id);
        self.fixed_dofs.retain(|(nid, _)| *nid != id);
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

    // ── Constraint / load storage (not editable in this phase) ────────

    /// Mark a node as fully fixed.  Used by preset models.
    pub fn fix_node(&mut self, id: NodeId) -> Result<(), EditError> {
        if self.nodes.iter().all(|n| n.id != id) {
            return Err(EditError::NodeNotFound(id));
        }
        if !self.fixed_nodes.contains(&id) {
            self.fixed_nodes.push(id);
        }
        Ok(())
    }

    /// Fix a single DOF at a node.  Used by preset models.
    pub fn fix_dof(&mut self, id: NodeId, dof: Dof3D) -> Result<(), EditError> {
        if self.nodes.iter().all(|n| n.id != id) {
            return Err(EditError::NodeNotFound(id));
        }
        if !self.fixed_dofs.contains(&(id, dof)) {
            self.fixed_dofs.push((id, dof));
        }
        Ok(())
    }

    /// Add a nodal load.  Used by preset models.
    pub fn add_nodal_load(&mut self, id: NodeId, loads: [f64; 6]) -> Result<(), EditError> {
        if self.nodes.iter().all(|n| n.id != id) {
            return Err(EditError::NodeNotFound(id));
        }
        self.nodal_loads.push((id, loads));
        Ok(())
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

        for &nid in &self.fixed_nodes {
            let idx = *id_to_idx.get(&nid).ok_or(EditError::NodeNotFound(nid))?;
            model
                .fix_node(idx)
                .map_err(|e| EditError::FrameModelBuild(format!("{e:?}")))?;
        }

        for (nid, dof) in &self.fixed_dofs {
            let idx = *id_to_idx.get(nid).ok_or(EditError::NodeNotFound(*nid))?;
            model
                .fix_dof(idx, *dof, 0.0)
                .map_err(|e| EditError::FrameModelBuild(format!("{e:?}")))?;
        }

        for (nid, loads) in &self.nodal_loads {
            let idx = *id_to_idx.get(nid).ok_or(EditError::NodeNotFound(*nid))?;
            model
                .add_nodal_load(
                    idx, loads[0], loads[1], loads[2], loads[3], loads[4], loads[5],
                )
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
        m.add_nodal_load(c, [5e3, 0.0, 0.0, 0.0, 0.0, 0.0]).unwrap();
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
}
