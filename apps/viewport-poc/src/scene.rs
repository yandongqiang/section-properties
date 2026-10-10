//! Display geometry extracted from the solver result.
//!
//! The scene stores only what the renderer needs: node positions and member
//! connectivity.  It never holds a reference to the solver or model, so camera
//! and selection operations cannot mutate the analysis data.

use crate::project::Vec3;
use structural_analysis::FrameAnalysisResult3D;

/// Stable display geometry.
#[derive(Debug, Clone)]
pub struct ViewportScene {
    /// Node positions in world coordinates.
    pub node_positions: Vec<Vec3>,
    /// Member connectivity as `(node_i_idx, node_j_idx)`.
    pub member_nodes: Vec<(usize, usize)>,
    /// Member reference vectors (one per member).
    pub member_ref_vecs: Vec<Vec3>,
}

impl ViewportScene {
    /// Build from an analysis result.
    pub fn from_result(result: &FrameAnalysisResult3D) -> Self {
        Self {
            node_positions: result
                .node_coords
                .iter()
                .map(|(x, y, z)| [*x, *y, *z])
                .collect(),
            member_nodes: result.member_nodes.clone(),
            member_ref_vecs: result.member_ref_vecs.clone(),
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
            member_nodes: vec![(0, 1), (0, 2)],
            member_ref_vecs: vec![[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]],
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
            member_nodes: vec![],
            member_ref_vecs: vec![],
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
}
