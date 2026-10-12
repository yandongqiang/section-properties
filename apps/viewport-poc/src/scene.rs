//! Display geometry extracted from the editable model.
//!
//! The scene stores only what the renderer needs: node positions, member
//! connectivity, and stable ID mappings for selection.  It never holds a
//! reference to the solver or model, so camera and selection operations
//! cannot mutate the analysis data.

use crate::edit_model::{
    EditModel, MemberId, MemberUniformLoad3D, NodalLoadSpec, NodeId, SupportSpec,
};
use crate::project::Vec3;

/// Stable display geometry.
#[derive(Debug, Clone)]
pub struct ViewportScene {
    /// Node positions in world coordinates.
    pub node_positions: Vec<Vec3>,
    /// Stable node ID for each entry (parallel to `node_positions`).
    pub node_ids: Vec<NodeId>,
    /// Member connectivity as `(node_i_idx, node_j_idx)`.
    pub member_nodes: Vec<(usize, usize)>,
    /// Stable member ID for each entry (parallel to `member_nodes`).
    pub member_ids: Vec<MemberId>,
    /// Member reference vectors (one per member).
    pub member_ref_vecs: Vec<Vec3>,
    /// Support spec for each node (parallel to `node_positions`).
    /// `None` means no support (free).
    pub node_supports: Vec<Option<SupportSpec>>,
    /// Nodal load spec for each node (parallel to `node_positions`).
    /// `None` means no load.
    pub node_loads: Vec<Option<NodalLoadSpec>>,
    /// Member uniform load spec for each member (parallel to `member_ids`).
    /// `None` means no load.
    pub member_loads: Vec<Option<MemberUniformLoad3D>>,
}

impl ViewportScene {
    /// Build from an [`EditModel`].
    pub fn from_edit_model(model: &EditModel) -> Self {
        let node_positions: Vec<Vec3> = model.nodes().iter().map(|n| [n.x, n.y, n.z]).collect();
        let node_ids: Vec<NodeId> = model.nodes().iter().map(|n| n.id).collect();

        let member_nodes: Vec<(usize, usize)> = model
            .members()
            .iter()
            .map(|m| {
                (
                    model.node_index(m.node_i).unwrap_or(usize::MAX),
                    model.node_index(m.node_j).unwrap_or(usize::MAX),
                )
            })
            .collect();
        let member_ids: Vec<MemberId> = model.members().iter().map(|m| m.id).collect();
        let member_ref_vecs: Vec<Vec3> = model.members().iter().map(|m| m.ref_vec).collect();

        let node_supports: Vec<Option<SupportSpec>> = model
            .nodes()
            .iter()
            .map(|n| model.support(n.id).copied())
            .collect();

        let node_loads: Vec<Option<NodalLoadSpec>> = model
            .nodes()
            .iter()
            .map(|n| model.nodal_load(n.id).copied())
            .collect();

        let member_loads: Vec<Option<MemberUniformLoad3D>> = model
            .members()
            .iter()
            .map(|m| model.member_uniform_load(m.id).copied())
            .collect();

        Self {
            node_positions,
            node_ids,
            member_nodes,
            member_ids,
            member_ref_vecs,
            node_supports,
            node_loads,
            member_loads,
        }
    }

    /// Number of nodes.
    pub fn n_nodes(&self) -> usize {
        self.node_positions.len()
    }

    /// Number of members.
    pub fn n_members(&self) -> usize {
        self.member_nodes.len()
    }

    /// Look up the Vec index for a stable node ID.
    pub fn node_index(&self, id: NodeId) -> Option<usize> {
        self.node_ids.iter().position(|&nid| nid == id)
    }

    /// Look up the Vec index for a stable member ID.
    pub fn member_index(&self, id: MemberId) -> Option<usize> {
        self.member_ids.iter().position(|&mid| mid == id)
    }

    /// Axis-aligned bounding box of the node positions.
    ///
    /// Returns `([0;3],[0;3])` for an empty scene.
    pub fn bounding_box(&self) -> (Vec3, Vec3) {
        if self.node_positions.is_empty() {
            return ([0.0; 3], [0.0; 3]);
        }
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        for &p in &self.node_positions {
            for i in 0..3 {
                if p[i] < min[i] {
                    min[i] = p[i];
                }
                if p[i] > max[i] {
                    max[i] = p[i];
                }
            }
        }
        (min, max)
    }

    /// Compute deformed node positions: `original + scale * displacement`.
    ///
    /// This is a **read-only** operation — it does not modify any stored data.
    /// Non-finite displacements produce the original position.
    pub fn deformed_positions(&self, displacements: &[f64], scale: f64) -> Vec<Vec3> {
        self.node_positions
            .iter()
            .enumerate()
            .map(|(i, &pos)| {
                let base = i * 6;
                if base + 2 >= displacements.len() {
                    return pos;
                }
                let dx = displacements[base];
                let dy = displacements[base + 1];
                let dz = displacements[base + 2];
                if !dx.is_finite() || !dy.is_finite() || !dz.is_finite() {
                    return pos;
                }
                [
                    pos[0] + scale * dx,
                    pos[1] + scale * dy,
                    pos[2] + scale * dz,
                ]
            })
            .collect()
    }

    /// Member midpoint (world coordinates).
    pub fn member_midpoint(&self, member_idx: usize) -> Option<Vec3> {
        let (ni, nj) = self.member_nodes.get(member_idx).copied()?;
        let pi = self.node_positions.get(ni)?;
        let pj = self.node_positions.get(nj)?;
        Some([
            (pi[0] + pj[0]) * 0.5,
            (pi[1] + pj[1]) * 0.5,
            (pi[2] + pj[2]) * 0.5,
        ])
    }

    /// Member length.
    pub fn member_length(&self, member_idx: usize) -> Option<f64> {
        let (ni, nj) = self.member_nodes.get(member_idx).copied()?;
        let pi = self.node_positions.get(ni)?;
        let pj = self.node_positions.get(nj)?;
        let d = [pj[0] - pi[0], pj[1] - pi[1], pj[2] - pi[2]];
        Some(d[0].hypot(d[1]).hypot(d[2]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dummy_scene() -> ViewportScene {
        ViewportScene {
            node_positions: vec![[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 3.0, 0.0]],
            node_ids: vec![NodeId(0), NodeId(1), NodeId(2)],
            member_nodes: vec![(0, 1), (0, 2)],
            member_ids: vec![MemberId(0), MemberId(1)],
            member_ref_vecs: vec![[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]],
            node_supports: vec![None, None, None],
            node_loads: vec![None, None, None],
            member_loads: vec![None, None],
        }
    }

    #[test]
    fn bounding_box() {
        let s = dummy_scene();
        let (min, max) = s.bounding_box();
        assert_eq!(min, [0.0, 0.0, 0.0]);
        assert_eq!(max, [2.0, 3.0, 0.0]);
    }

    #[test]
    fn empty_bounding_box() {
        let s = ViewportScene {
            node_positions: vec![],
            node_ids: vec![],
            member_nodes: vec![],
            member_ids: vec![],
            member_ref_vecs: vec![],
            node_supports: vec![],
            node_loads: vec![],
            member_loads: vec![],
        };
        let (min, max) = s.bounding_box();
        assert_eq!(min, [0.0; 3]);
        assert_eq!(max, [0.0; 3]);
    }

    #[test]
    fn deformed_zero_scale_is_original() {
        let s = dummy_scene();
        let disp = vec![
            0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0,
        ];
        let deformed = s.deformed_positions(&disp, 0.0);
        assert_eq!(deformed, s.node_positions);
    }

    #[test]
    fn deformed_nonzero_scale() {
        let s = dummy_scene();
        let disp = vec![
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0,
        ];
        let deformed = s.deformed_positions(&disp, 10.0);
        assert_eq!(deformed[0], [10.0, 0.0, 0.0]);
        assert_eq!(deformed[1], [2.0, 0.0, 0.0]);
    }

    #[test]
    fn deformed_does_not_modify_original() {
        let s = dummy_scene();
        let original = s.node_positions.clone();
        let disp = vec![
            1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0,
        ];
        let _ = s.deformed_positions(&disp, 100.0);
        assert_eq!(s.node_positions, original);
    }

    #[test]
    fn non_finite_displacement_returns_original() {
        let s = dummy_scene();
        let disp = vec![
            f64::NAN,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ];
        let deformed = s.deformed_positions(&disp, 1.0);
        assert_eq!(deformed[0], [0.0, 0.0, 0.0]);
    }

    #[test]
    fn member_length_correct() {
        let s = dummy_scene();
        assert!((s.member_length(0).unwrap() - 2.0).abs() < 1e-15);
        assert!((s.member_length(1).unwrap() - 3.0).abs() < 1e-15);
    }

    #[test]
    fn node_index_lookup() {
        let s = dummy_scene();
        assert_eq!(s.node_index(NodeId(0)), Some(0));
        assert_eq!(s.node_index(NodeId(2)), Some(2));
        assert_eq!(s.node_index(NodeId(99)), None);
    }

    #[test]
    fn member_index_lookup() {
        let s = dummy_scene();
        assert_eq!(s.member_index(MemberId(0)), Some(0));
        assert_eq!(s.member_index(MemberId(99)), None);
    }
}
