//! Display geometry extracted from the editable model.
//!
//! The scene stores only what the renderer needs: node positions, member
//! connectivity, and stable ID mappings for selection.  It never holds a
//! reference to the solver or model, so camera and selection operations
//! cannot mutate the analysis data.

use crate::edit_model::{
    EditModel, MemberId, MemberPointLoad3D, MemberUniformLoad3D, NodalLoadSpec, NodeId, SupportSpec,
};
use crate::project::Vec3;
use structural_analysis::FrameAnalysisResult3D;

/// Internal section forces along a 3D frame member, in the member's **local**
/// axes (SI units: N and N·m).
///
/// # Sign convention
///
/// The local axes follow the solver convention (`x` along `i → j`,
/// `z = x × ref`, `y = z × x`; right-handed).  The reported values mirror
/// [`structural_analysis::beam_fem::SectionForces`] for the local `x–y`
/// plane and extend the same right-hand convention to the `x–z` plane:
///
/// * [`n`](Self::n) — axial force, **tension positive**.
/// * [`vy`](Self::vy) — shear along local `+y`; satisfies `d(mz)/dx = vy`.
/// * [`vz`](Self::vz) — shear along local `+z`; satisfies `d(my)/dx = vz`.
/// * [`t`](Self::t) — torsion about local `+x`.
/// * [`my`](Self::my) — bending moment about local `+y` (curvature in the
///   local `x–z` plane).
/// * [`mz`](Self::mz) — bending moment about local `+z`, **sagging positive**
///   in the local `x–y` plane.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SectionForces3D {
    pub n: f64,
    pub vy: f64,
    pub vz: f64,
    pub t: f64,
    pub my: f64,
    pub mz: f64,
}

impl SectionForces3D {
    /// Largest absolute value across all six components (0.0 if all empty).
    pub fn max_abs(&self) -> f64 {
        [self.n, self.vy, self.vz, self.t, self.my, self.mz]
            .iter()
            .fold(0.0_f64, |a, &v| a.max(v.abs()))
    }
}

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
    /// Member point loads for each member (parallel to `member_ids`).
    /// Empty Vec means no point loads.
    pub member_point_loads: Vec<Vec<MemberPointLoad3D>>,
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

        let member_point_loads: Vec<Vec<MemberPointLoad3D>> = model
            .members()
            .iter()
            .map(|m| model.member_point_loads_for(m.id))
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
            member_point_loads,
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

    /// Local-axis triple `(x_hat, y_hat, z_hat)` for a member.
    ///
    /// Uses the identical convention to
    /// [`FrameAnalysisResult3D::member_local_axes`] so that diagram offsets
    /// and deformation directions agree with the solver's local end forces:
    /// `x_hat = (p_j − p_i)/L`, `z_hat = (x_hat × ref)/|·|`, `y_hat = z_hat × x_hat`.
    ///
    /// Returns `None` for an invalid member, zero length, or a reference vector
    /// parallel to the member axis.
    #[allow(clippy::type_complexity)]
    pub fn member_local_axes(
        &self,
        member_idx: usize,
    ) -> Option<([f64; 3], [f64; 3], [f64; 3])> {
        let (ni, nj) = *self.member_nodes.get(member_idx)?;
        let pi = *self.node_positions.get(ni)?;
        let pj = *self.node_positions.get(nj)?;
        let ref_vec = *self.member_ref_vecs.get(member_idx)?;

        let dx = pj[0] - pi[0];
        let dy = pj[1] - pi[1];
        let dz = pj[2] - pi[2];
        let l = dx.hypot(dy).hypot(dz);
        if !l.is_finite() || l <= 1e-12 {
            return None;
        }
        let x_hat = [dx / l, dy / l, dz / l];

        let cross = [
            x_hat[1] * ref_vec[2] - x_hat[2] * ref_vec[1],
            x_hat[2] * ref_vec[0] - x_hat[0] * ref_vec[2],
            x_hat[0] * ref_vec[1] - x_hat[1] * ref_vec[0],
        ];
        let cross_norm = cross[0].hypot(cross[1]).hypot(cross[2]);
        let ref_norm = ref_vec[0].hypot(ref_vec[1]).hypot(ref_vec[2]);
        let tol = 1e-12 * ref_norm.max(1.0);
        if !cross_norm.is_finite() || cross_norm <= tol {
            return None;
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
        Some((x_hat, y_hat, z_hat))
    }

    /// Sample the internal section forces `N / V / M` at normalized position
    /// `xi ∈ [0, 1]` along member `member_idx`.
    ///
    /// # Method
    ///
    /// The 3D solver exposes only **member-local end forces** (12 values) and
    /// member loads, not a section-force sampler (the crate's
    /// `section_forces` lives on the 2D `FrameAnalysisResult`).  This routine
    /// therefore recovers the internal forces from statics of the left segment
    /// `[0, x]`, using the local end force `g` at end `i` from
    /// [`FrameAnalysisResult3D::member_end_forces`] plus the distributed and
    /// point loads stored in this scene.  It reproduces
    /// `beam_fem::BeamSolver::element_section_forces` exactly for the local
    /// `x–y` plane and extends it to the `x–z` plane via the right-hand rule.
    ///
    /// # Errors / edge cases
    ///
    /// Returns `None` if the member index is invalid, has zero length, or has
    /// no end forces in `result`.  Non-finite load components are ignored.
    pub fn member_section_forces(
        &self,
        member_idx: usize,
        xi: f64,
        result: &FrameAnalysisResult3D,
    ) -> Option<SectionForces3D> {
        let length = self.member_length(member_idx)?;
        if !length.is_finite() || length <= 1e-12 {
            return None;
        }
        let g = result.member_end_forces(member_idx).ok()?;
        if !g.iter().all(|v| v.is_finite()) {
            return None;
        }

        let xi = xi.clamp(0.0, 1.0);
        let x = xi * length;

        // Uniform distributed load in local axes (force per unit length).
        let (qx, qy, qz) = match self.member_loads.get(member_idx).copied().flatten() {
            Some(l) if l.is_finite() => (l.qx, l.qy, l.qz),
            _ => (0.0, 0.0, 0.0),
        };

        // Point loads strictly to the left of the cut (left-limit convention),
        // matching the crate's `element_section_forces`.
        let mut sum_fx = 0.0;
        let mut sum_fy = 0.0;
        let mut sum_fz = 0.0;
        let mut mom_y = 0.0; // Σ (x_p − x)·fz
        let mut mom_z = 0.0; // Σ (x_p − x)·fy
        if let Some(loads) = self.member_point_loads.get(member_idx) {
            for pl in loads {
                if !pl.is_finite() {
                    continue;
                }
                let x_p = pl.xi * length;
                if x_p < x {
                    sum_fx += pl.fx;
                    sum_fy += pl.fy;
                    sum_fz += pl.fz;
                    mom_y += (x_p - x) * pl.fz;
                    mom_z += (x_p - x) * pl.fy;
                }
            }
        }

        // Integrals of the uniform load over [0, x].
        let qx_t = qx * x;
        let qy_t = qy * x;
        let qz_t = qz * x;

        // Local end force at end i: `[fx, fy, fz, mx, my, mz]`.
        let n = -(g[0] + qx_t + sum_fx);
        let vy = g[1] + qy_t + sum_fy;
        let vz = g[2] + qz_t + sum_fz;
        let t = -g[3];
        let my = -g[4] - x * g[2] - 0.5 * qz * x * x + mom_y;
        let mz = -g[5] + x * g[1] + 0.5 * qy * x * x - mom_z;

        Some(SectionForces3D {
            n,
            vy,
            vz,
            t,
            my,
            mz,
        })
    }

    /// Sample a member's **deformed** shape at normalized position
    /// `xi ∈ [0, 1]`.
    ///
    /// Interpolates the member's nodal translations and rotations using cubic
    /// Hermite shape functions in each bending plane and linear interpolation
    /// axially, then transforms the local displacement back to world axes and
    /// adds `scale ×` displacement to the undeformed point:
    ///
    /// ```text
    /// d(ξ) = (1−ξ)·u_i + ξ·u_j                              (axial, linear)
    /// v(ξ) = H₁ v_i + H₂·L·θz_i + H₃ v_j + H₄·L·θz_j        (x–y plane)
    /// w(ξ) = H₁ w_i − H₂·L·θy_i + H₃ w_j − H₄·L·θy_j        (x–z plane,
    ///                                                        θy = −dw/dx)
    /// ```
    ///
    /// `θz = +dv/dx` and `θy = −dw/dx` are the solver's rotation conventions.
    /// Returns `None` for an invalid member, degenerate axes, or a
    /// displacement vector too short to cover the member's end nodes.
    /// Non-finite nodal DOFs are treated as zero.
    pub fn member_deformed_point(
        &self,
        member_idx: usize,
        xi: f64,
        displacements: &[f64],
        scale: f64,
    ) -> Option<Vec3> {
        let (ni, nj) = *self.member_nodes.get(member_idx)?;
        let pi = *self.node_positions.get(ni)?;
        let pj = *self.node_positions.get(nj)?;
        let length = self.member_length(member_idx)?;
        if !length.is_finite() || length <= 1e-12 {
            return None;
        }
        let (x_hat, y_hat, z_hat) = self.member_local_axes(member_idx)?;

        // Nodal translations and rotations at ends i and j (global).
        let get = |idx: usize| -> Option<([f64; 3], [f64; 3])> {
            let base = idx.checked_mul(6)?;
            if base + 5 >= displacements.len() {
                return None;
            }
            let fin = |v: f64| if v.is_finite() { v } else { 0.0 };
            let u = [
                fin(displacements[base]),
                fin(displacements[base + 1]),
                fin(displacements[base + 2]),
            ];
            let r = [
                fin(displacements[base + 3]),
                fin(displacements[base + 4]),
                fin(displacements[base + 5]),
            ];
            Some((u, r))
        };
        let (u_i, r_i) = get(ni)?;
        let (u_j, r_j) = get(nj)?;

        let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

        // Local nodal components.
        let (au_i, av_i, aw_i) = (dot(u_i, x_hat), dot(u_i, y_hat), dot(u_i, z_hat));
        let (au_j, av_j, aw_j) = (dot(u_j, x_hat), dot(u_j, y_hat), dot(u_j, z_hat));
        let thz_i = dot(r_i, z_hat);
        let thz_j = dot(r_j, z_hat);
        let thy_i = dot(r_i, y_hat);
        let thy_j = dot(r_j, y_hat);

        let xi = xi.clamp(0.0, 1.0);
        let (x2, x3) = (xi * xi, xi * xi * xi);
        let h1 = 1.0 - 3.0 * x2 + 2.0 * x3;
        let h2 = xi - 2.0 * x2 + x3;
        let h3 = 3.0 * x2 - 2.0 * x3;
        let h4 = -x2 + x3;

        let axial = (1.0 - xi) * au_i + xi * au_j;
        let v = h1 * av_i + h2 * (length * thz_i) + h3 * av_j + h4 * (length * thz_j);
        let w = h1 * aw_i - h2 * (length * thy_i) + h3 * aw_j - h4 * (length * thy_j);

        // Local displacement → world vector.
        let disp = [
            axial * x_hat[0] + v * y_hat[0] + w * z_hat[0],
            axial * x_hat[1] + v * y_hat[1] + w * z_hat[1],
            axial * x_hat[2] + v * y_hat[2] + w * z_hat[2],
        ];
        let base = [
            pi[0] + xi * (pj[0] - pi[0]),
            pi[1] + xi * (pj[1] - pi[1]),
            pi[2] + xi * (pj[2] - pi[2]),
        ];
        Some([
            base[0] + scale * disp[0],
            base[1] + scale * disp[1],
            base[2] + scale * disp[2],
        ])
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
            member_point_loads: vec![vec![], vec![]],
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
            member_point_loads: vec![],
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

    // ── Section-force sampler ──────────────────────────────────────────

    /// Cantilever along +X (so local y = +Y for `ref = (0, 1, 0)`).
    fn cantilever(l: f64) -> (EditModel, crate::edit_model::MemberId) {
        let mut m = EditModel::new();
        let n0 = m.add_node(0.0, 0.0, 0.0).unwrap();
        let n1 = m.add_node(l, 0.0, 0.0).unwrap();
        let mem = m.add_member(n0, n1).unwrap();
        m.set_member_ref_vec(mem, [0.0, 1.0, 0.0]).unwrap();
        m.fix_node(n0).unwrap();
        (m, mem)
    }

    /// Uniform local `-y` load (downward): analytical `V_y(x) = q(L−x)`,
    /// `M_z(x) = −q(L−x)²/2`, `N = 0` — the same case as the crate's
    /// `beam_section_forces::test_section_forces_cantilever_udl`.
    #[test]
    fn section_forces_cantilever_udl() {
        let l = 2.0;
        let q = 1000.0;
        let (mut m, mem) = cantilever(l);
        m.set_member_uniform_load(mem, MemberUniformLoad3D::new(0.0, -q, 0.0))
            .unwrap();

        let scene = ViewportScene::from_edit_model(&m);
        let res = m.to_frame_model().unwrap().solve().unwrap();

        for &xi in &[0.0, 0.25, 0.5, 0.75, 1.0] {
            let x = xi * l;
            let f = scene.member_section_forces(0, xi, &res).unwrap();
            assert!(f.n.abs() < 1e-9, "N at xi={xi}: {}", f.n);
            assert!(
                (f.vy - q * (l - x)).abs() < 1e-7 * q * l,
                "Vy at xi={xi}: {} vs {}",
                f.vy,
                q * (l - x)
            );
            assert!(
                (f.mz + q * (l - x).powi(2) / 2.0).abs() < 1e-7 * q * l * l,
                "Mz at xi={xi}: {} vs {}",
                f.mz,
                -q * (l - x).powi(2) / 2.0
            );
            assert!(f.vz.abs() < 1e-9);
            assert!(f.my.abs() < 1e-9);
        }
    }

    /// Axial tip tension: `N(x) = +P` (tension positive) everywhere.
    #[test]
    fn section_forces_cantilever_axial_tension() {
        let l = 2.0;
        let p = 500.0;
        let (mut m, _mem) = cantilever(l);
        let n1 = m.nodes()[1].id;
        m.set_nodal_load(n1, NodalLoadSpec::new(p, 0.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();

        let scene = ViewportScene::from_edit_model(&m);
        let res = m.to_frame_model().unwrap().solve().unwrap();

        for &xi in &[0.0, 0.5, 1.0] {
            let f = scene.member_section_forces(0, xi, &res).unwrap();
            assert!((f.n - p).abs() < 1e-9 * p.max(1.0), "N at xi={xi}: {}", f.n);
            assert!(f.vy.abs() < 1e-9);
            assert!(f.mz.abs() < 1e-9);
        }
    }

    /// Interior point load: shear jumps by the applied load, moment is
    /// continuous. Mirrors the crate's `test_section_forces_interior_point_force`.
    #[test]
    fn section_forces_interior_point_load_jump() {
        let l = 2.0;
        let p = 1000.0;
        let (mut m, _mem) = cantilever(l);
        let mem = m.members()[0].id;
        m.add_member_point_load(mem, MemberPointLoad3D::new(0.5, 0.0, -p, 0.0))
            .unwrap();

        let scene = ViewportScene::from_edit_model(&m);
        let res = m.to_frame_model().unwrap().solve().unwrap();

        // Left of the load: V = +p, M = −p(L/2 − x).
        for &xi in &[0.0, 0.25] {
            let x = xi * l;
            let f = scene.member_section_forces(0, xi, &res).unwrap();
            assert!((f.vy - p).abs() < 1e-6 * p, "Vy left at xi={xi}: {}", f.vy);
            assert!(
                (f.mz + p * (l / 2.0 - x)).abs() < 1e-6 * p * l,
                "Mz left at xi={xi}: {}",
                f.mz
            );
        }
        // Right of the load: the force has been shed; V and M are zero.
        for &xi in &[0.75, 1.0] {
            let f = scene.member_section_forces(0, xi, &res).unwrap();
            assert!(f.vy.abs() < 1e-6 * p, "Vy right at xi={xi}: {}", f.vy);
            assert!(f.mz.abs() < 1e-6 * p * l, "Mz right at xi={xi}: {}", f.mz);
        }
    }

    // ── Deformed-shape (Hermite) sampler ───────────────────────────────

    /// A single-element cantilever with a tip load is exactly cubic, so the
    /// Hermite interpolation must reproduce the analytical mid-span deflection
    /// `v(L/2) = 5·P·L³ / (48·E·I_z)`.
    #[test]
    fn deformed_curve_matches_euler_bernoulli_midspan() {
        let l = 2.0;
        let p = 1000.0;
        let e = 200e9;
        let iz = 1e-6; // default section's Iz (x–y plane bending)
        let (mut m, _mem) = cantilever(l);
        let n1 = m.nodes()[1].id;
        m.set_nodal_load(n1, NodalLoadSpec::new(0.0, p, 0.0, 0.0, 0.0, 0.0))
            .unwrap();

        let scene = ViewportScene::from_edit_model(&m);
        let res = m.to_frame_model().unwrap().solve().unwrap();

        let expected = 5.0 * p * l.powi(3) / (48.0 * e * iz);
        let mid = scene
            .member_deformed_point(0, 0.5, &res.displacements, 1.0)
            .unwrap();
        assert!((mid[0] - 1.0).abs() < 1e-12, "x: {}", mid[0]);
        assert!(
            (mid[1] - expected).abs() < 1e-9,
            "midspan deflection: {} vs {}",
            mid[1],
            expected
        );
    }

    /// The curve endpoints must coincide with the nodal deformed positions.
    #[test]
    fn deformed_curve_endpoints_match_nodes() {
        let (mut m, _mem) = cantilever(2.0);
        let n1 = m.nodes()[1].id;
        m.set_nodal_load(n1, NodalLoadSpec::new(1000.0, 500.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();

        let scene = ViewportScene::from_edit_model(&m);
        let res = m.to_frame_model().unwrap().solve().unwrap();
        let scale = 100.0;
        let nodes = scene.deformed_positions(&res.displacements, scale);

        let at0 = scene
            .member_deformed_point(0, 0.0, &res.displacements, scale)
            .unwrap();
        let at1 = scene
            .member_deformed_point(0, 1.0, &res.displacements, scale)
            .unwrap();
        for i in 0..3 {
            assert!((at0[i] - nodes[0][i]).abs() < 1e-12, "end i axis {i}");
            assert!((at1[i] - nodes[1][i]).abs() < 1e-12, "end j axis {i}");
        }
    }

    #[test]
    fn section_forces_rejects_bad_inputs() {
        let (m, _mem) = cantilever(2.0);
        let scene = ViewportScene::from_edit_model(&m);
        let res = m.to_frame_model().unwrap().solve().unwrap();
        assert!(scene.member_section_forces(99, 0.5, &res).is_none());
        assert!(scene.member_deformed_point(99, 0.5, &res.displacements, 1.0).is_none());
        // Too-short displacement vector.
        assert!(scene.member_deformed_point(0, 0.5, &[0.0; 3], 1.0).is_none());
    }
}
