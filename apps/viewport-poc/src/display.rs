//! Display-mode state and result visualization helpers.
//!
//! [`ResultViewMode`] is the authoritative enum that determines the
//! primary result visualization in the viewport.  Independent overlays
//! (undeformed reference, grid, axes, labels) remain as separate booleans
//! in [`ResultDisplayState`].

use std::collections::HashSet;

use crate::edit_model::{MemberId, NodeId};

// ── Display mode ──────────────────────────────────────────────────────

/// Primary result visualization mode.
///
/// Exactly one mode is active at a time — this replaces the former
/// `show_deformed` / `show_reactions` / `show_forces` booleans that could
/// produce contradictory display states.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ResultViewMode {
    /// Undeformed model geometry only (no result overlay).
    #[default]
    Model,
    /// Deformed shape using solved nodal translations.
    Deformed,
    /// Nodal displacement magnitude color-coded on the undeformed shape.
    Displacement,
    /// Nodal reaction force/moment indicators at supported nodes.
    Reaction,
    /// Member end-force indicators for the selected member.
    MemberEndForces,
}

impl ResultViewMode {
    /// All modes in display order.
    pub const ALL: [ResultViewMode; 5] = [
        ResultViewMode::Model,
        ResultViewMode::Deformed,
        ResultViewMode::Displacement,
        ResultViewMode::Reaction,
        ResultViewMode::MemberEndForces,
    ];

    /// Short label for GUI buttons.
    pub const fn label(self) -> &'static str {
        match self {
            ResultViewMode::Model => "Model",
            ResultViewMode::Deformed => "Deformed",
            ResultViewMode::Displacement => "Displacement",
            ResultViewMode::Reaction => "Reaction",
            ResultViewMode::MemberEndForces => "End Forces",
        }
    }

    /// `true` if this mode requires a current analysis result.
    pub const fn requires_result(self) -> bool {
        !matches!(self, ResultViewMode::Model)
    }
}

// ── Section cut plane (Phase P5) ──────────────────────────────────────

/// Primary axis of the section-cut plane.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CutAxis {
    #[default]
    X,
    Y,
    Z,
}

impl CutAxis {
    /// All axes in display order.
    pub const ALL: [CutAxis; 3] = [CutAxis::X, CutAxis::Y, CutAxis::Z];

    /// Short label for GUI buttons.
    pub const fn label(self) -> &'static str {
        match self {
            CutAxis::X => "X",
            CutAxis::Y => "Y",
            CutAxis::Z => "Z",
        }
    }

    /// Outward unit normal (world) for this axis.
    pub const fn normal(self) -> [f64; 3] {
        match self {
            CutAxis::X => [1.0, 0.0, 0.0],
            CutAxis::Y => [0.0, 1.0, 0.0],
            CutAxis::Z => [0.0, 0.0, 1.0],
        }
    }
}

/// **View-only** section-cut plane (Phase P5).  Never serialized — it lives
/// alongside the other display toggles and is rebuilt from defaults on load.
///
/// # Representation
///
/// The plane is `dot(n, p) = offset` with a **unit** normal `n` (from
/// [`axis`](Self::axis), unless an explicit [`normal`](Self::normal) is set).
/// The signed distance of a point is `dot(n, p) − offset`; the side with a
/// positive distance is kept, unless [`flip`](Self::flip) reverses it.
///
/// `offset` is measured along `n` in **model units** and is clamped to the
/// scene's projection range (see [`sync_bounds`](Self::sync_bounds)) so the cut
/// works at any model scale.  [`positioned`](Self::positioned) tracks whether
/// the offset has been placed for the current axis/scene; while it is `false`
/// the offset re-centres on the scene the next time it is synced.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionCut {
    /// Whether the cut is active.  When `false` nothing is clipped.
    pub enabled: bool,
    /// Primary axis (used unless [`normal`](Self::normal) is `Some`).
    pub axis: CutAxis,
    /// Optional arbitrary plane normal (world).  `None` = derive from `axis`.
    pub normal: Option<[f64; 3]>,
    /// Plane offset along the (unit) normal, in model units.
    pub offset: f64,
    /// Reverse the kept half-space (keep the opposite side of the plane).
    pub flip: bool,
    /// Has `offset` been placed for the current axis/scene?  Reset to re-centre.
    pub positioned: bool,
}

impl Default for SectionCut {
    fn default() -> Self {
        Self {
            enabled: false,
            axis: CutAxis::X,
            normal: None,
            offset: 0.0,
            flip: false,
            positioned: false,
        }
    }
}

impl SectionCut {
    /// Unit plane normal: the axis normal, or the normalised custom normal.
    /// Returns `None` for a zero / non-finite custom normal.
    pub fn unit_normal(&self) -> Option<[f64; 3]> {
        match self.normal {
            None => Some(self.axis.normal()),
            Some(n) => {
                let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                if len.is_finite() && len > 1e-12 {
                    Some([n[0] / len, n[1] / len, n[2] / len])
                } else {
                    None
                }
            }
        }
    }

    /// Signed distance of `p` to the plane (`> 0` on the kept side when not
    /// flipped).  `None` when the normal is degenerate.
    pub fn signed_distance(&self, p: [f64; 3]) -> Option<f64> {
        let n = self.unit_normal()?;
        Some(n[0] * p[0] + n[1] * p[1] + n[2] * p[2] - self.offset)
    }

    /// The distance written to `gl_ClipDistance[0]`: `> 0` means keep.  `flip`
    /// reverses the sign.  `None` when the normal is degenerate.
    pub fn clip_distance(&self, p: [f64; 3]) -> Option<f64> {
        let d = self.signed_distance(p)?;
        Some(if self.flip { -d } else { d })
    }

    /// `true` if `p` is on the kept side.  A disabled or degenerate cut keeps
    /// everything.
    pub fn keeps(&self, p: [f64; 3]) -> bool {
        if !self.enabled {
            return true;
        }
        match self.clip_distance(p) {
            Some(d) => d >= 0.0,
            None => true,
        }
    }

    /// Clip a segment against the kept half-space.
    ///
    /// Returns the (possibly shortened) endpoints, or `None` when the whole
    /// segment lies outside.  A disabled / degenerate cut returns the segment
    /// unchanged.
    pub fn clip_segment(&self, a: [f64; 3], b: [f64; 3]) -> Option<([f64; 3], [f64; 3])> {
        if !self.enabled {
            return Some((a, b));
        }
        let (Some(da), Some(db)) = (self.clip_distance(a), self.clip_distance(b)) else {
            return Some((a, b));
        };
        let inside_a = da >= 0.0;
        let inside_b = db >= 0.0;
        if inside_a && inside_b {
            return Some((a, b));
        }
        if !inside_a && !inside_b {
            return None;
        }
        // Exactly one endpoint is kept: move the outside endpoint to the
        // crossing point (linear interpolation of the signed distance).
        let denom = da - db;
        if !denom.is_finite() || denom.abs() < 1e-30 {
            return Some((a, b)); // parallel / on-plane — keep as-is
        }
        let t = (da / denom).clamp(0.0, 1.0);
        let cross = [
            a[0] + t * (b[0] - a[0]),
            a[1] + t * (b[1] - a[1]),
            a[2] + t * (b[2] - a[2]),
        ];
        if inside_a {
            Some((a, cross))
        } else {
            Some((cross, b))
        }
    }

    /// Place / clamp the offset against the scene projection range `[min, max]`
    /// (the span of `dot(n, p)` over the model, from
    /// [`ViewportScene::projection_range`](crate::scene::ViewportScene::projection_range)).
    ///
    /// While [`positioned`](Self::positioned) is `false` the offset is centred
    /// at the range midpoint; afterwards it is clamped into the range so it
    /// tracks model changes without flying off.  A non-finite range centres at
    /// `0`.
    pub fn sync_bounds(&mut self, min: f64, max: f64) {
        let (lo, hi) = if min <= max { (min, max) } else { (max, min) };
        let (lo, hi) = if lo.is_finite() && hi.is_finite() {
            (lo, hi)
        } else {
            (0.0, 0.0)
        };
        if !self.positioned {
            self.offset = (lo + hi) * 0.5;
            self.positioned = true;
        } else {
            self.offset = self.offset.clamp(lo, hi);
        }
    }

    /// Force the plane to re-centre on the scene at the next
    /// [`sync_bounds`](Self::sync_bounds).
    pub fn recenter(&mut self) {
        self.positioned = false;
    }

    /// The plane as `[n.x, n.y, n.z, offset]`, already flip-applied so a
    /// positive `gl_ClipDistance` means "keep".  `None` when the normal is
    /// degenerate (the caller then leaves clipping off).
    pub fn gpu_plane(&self) -> Option<[f32; 4]> {
        let n = self.unit_normal()?;
        let s = if self.flip { -1.0 } else { 1.0 };
        Some([
            (s * n[0]) as f32,
            (s * n[1]) as f32,
            (s * n[2]) as f32,
            (s * self.offset) as f32,
        ])
    }
}

// ── Display state ─────────────────────────────────────────────────────

/// Viewport display state: primary mode + independent overlays.
///
/// The [`mode`](Self::mode) field determines the primary result
/// visualization.  The remaining fields are independent overlays that
/// can be toggled regardless of mode.
#[derive(Debug, Clone)]
pub struct ResultDisplayState {
    /// Primary result visualization mode.
    pub mode: ResultViewMode,
    /// Scale factor for deformed-shape display (1.0 = actual scale).
    pub deformation_scale: f64,
    /// Show the undeformed structure as a reference overlay.
    pub show_undeformed: bool,
    /// Show local axes for the selected member.
    pub show_local_axes: bool,
    /// Show the ground grid.
    pub show_grid: bool,
    /// Show global axes.
    pub show_global_axes: bool,
    /// Show node labels.
    pub show_node_labels: bool,
    /// Show member internal-force diagrams (N / V / M) as an overlay.
    pub show_diagrams: bool,
    /// Include the axial-force diagram (N).
    pub diagram_axial: bool,
    /// Include the shear-force diagrams (Vy / Vz).
    pub diagram_shear: bool,
    /// Include the bending-moment diagrams (Mz / My).
    pub diagram_moment: bool,
    /// Diagram amplitude multiplier (1.0 = auto-scaled to member length).
    pub diagram_scale: f64,
    /// `true` = draw diagrams for every member; `false` = selected member only.
    pub diagram_all_members: bool,
    /// **View-only** hidden node IDs.  Never touches the model, the solve, or
    /// the saved file — default is empty (nothing hidden).
    ///
    /// Hiding a node hides only its marker and label; members attached to it
    /// stay visible (see `ResultDisplayState::is_node_hidden` callers).  This
    /// keeps a member whose end nodes are hidden from visually collapsing.
    pub hidden_nodes: HashSet<NodeId>,
    /// **View-only** hidden member IDs (see [`hidden_nodes`](Self::hidden_nodes)).
    pub hidden_members: HashSet<MemberId>,
    /// **View-only** section-cut plane (Phase P5).  Never serialized.
    pub section_cut: SectionCut,
}

impl Default for ResultDisplayState {
    fn default() -> Self {
        Self {
            mode: ResultViewMode::Model,
            deformation_scale: 100.0,
            show_undeformed: true,
            show_local_axes: true,
            show_grid: true,
            show_global_axes: true,
            show_node_labels: true,
            show_diagrams: false,
            diagram_axial: true,
            diagram_shear: true,
            diagram_moment: true,
            diagram_scale: 1.0,
            diagram_all_members: false,
            hidden_nodes: HashSet::new(),
            hidden_members: HashSet::new(),
            section_cut: SectionCut::default(),
        }
    }
}

impl ResultDisplayState {
    /// Reset to defaults.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// `true` if the current mode requires a result and the result is
    /// available (non-`None`).
    pub fn needs_result(&self) -> bool {
        self.mode.requires_result()
    }

    // ── View-only visibility ───────────────────────────────────────────

    /// Is this node hidden in the viewport?
    pub fn is_node_hidden(&self, id: NodeId) -> bool {
        self.hidden_nodes.contains(&id)
    }

    /// Is this member hidden in the viewport?
    pub fn is_member_hidden(&self, id: MemberId) -> bool {
        self.hidden_members.contains(&id)
    }

    /// Hide a single node (view-only).
    pub fn hide_node(&mut self, id: NodeId) {
        self.hidden_nodes.insert(id);
    }

    /// Hide a single member (view-only).
    pub fn hide_member(&mut self, id: MemberId) {
        self.hidden_members.insert(id);
    }

    /// Unhide everything (`Show all`).
    pub fn show_all(&mut self) {
        self.hidden_nodes.clear();
        self.hidden_members.clear();
    }

    /// `true` if anything is currently hidden.
    pub fn has_hidden(&self) -> bool {
        !self.hidden_nodes.is_empty() || !self.hidden_members.is_empty()
    }

    /// Isolate a node: hide every other node, plus every member not attached to
    /// `keep` (`Hide others` on a node).  `all_*` are the full ID sets; the
    /// caller derives `keep_members` from the member connectivity.
    pub fn isolate_node(
        &mut self,
        all_nodes: &[NodeId],
        all_members: &[MemberId],
        keep: NodeId,
        keep_members: &[MemberId],
    ) {
        self.hidden_nodes = all_nodes.iter().copied().filter(|&n| n != keep).collect();
        self.hidden_members = all_members
            .iter()
            .copied()
            .filter(|m| !keep_members.contains(m))
            .collect();
    }

    /// Isolate a member: hide every other member, plus every node not on the
    /// kept member (`Hide others` on a member).
    pub fn isolate_member(
        &mut self,
        all_nodes: &[NodeId],
        all_members: &[MemberId],
        keep: MemberId,
        keep_nodes: &[NodeId],
    ) {
        self.hidden_members = all_members.iter().copied().filter(|&m| m != keep).collect();
        self.hidden_nodes = all_nodes
            .iter()
            .copied()
            .filter(|n| !keep_nodes.contains(n))
            .collect();
    }
}

// ── Displacement magnitude ────────────────────────────────────────────

/// Translational displacement magnitude `|u| = sqrt(ux² + uy² + uz²)`.
///
/// Rotational DOFs are **not** included — they have different units and
/// combining them with translations would be physically meaningless.
///
/// Returns 0.0 for non-finite inputs (defensive).
pub fn displacement_magnitude(ux: f64, uy: f64, uz: f64) -> f64 {
    if !ux.is_finite() || !uy.is_finite() || !uz.is_finite() {
        return 0.0;
    }
    (ux * ux + uy * uy + uz * uz).sqrt()
}

/// Compute per-node translational displacement magnitudes from a flat
/// displacement vector (6 DOFs per node: ux, uy, uz, rx, ry, rz).
///
/// Only the first 3 components (translations) are used.
pub fn displacement_magnitudes(displacements: &[f64], n_nodes: usize) -> Vec<f64> {
    (0..n_nodes)
        .map(|i| {
            let base = i * 6;
            if base + 2 >= displacements.len() {
                return 0.0;
            }
            displacement_magnitude(
                displacements[base],
                displacements[base + 1],
                displacements[base + 2],
            )
        })
        .collect()
}

/// Maximum absolute value in a slice, or 0.0 if empty / all non-finite.
pub fn max_abs(values: &[f64]) -> f64 {
    values
        .iter()
        .filter(|v| v.is_finite())
        .map(|v| v.abs())
        .fold(0.0_f64, f64::max)
}

/// Normalize a value to `[0, 1]` relative to `max_val`.
///
/// Returns 0.0 if `max_val <= 0` (avoids division by zero).
pub fn normalize(value: f64, max_val: f64) -> f64 {
    if max_val <= 0.0 || !max_val.is_finite() {
        return 0.0;
    }
    let r = value.abs() / max_val;
    if r.is_finite() { r.min(1.0) } else { 0.0 }
}

/// Map a normalized value `[0, 1]` to a blue→red color.
///
/// * 0.0 → blue  `(80, 80, 255)`
/// * 0.5 → green `(80, 255, 80)`
/// * 1.0 → red   `(255, 80, 80)`
///
/// Clamps input to `[0, 1]`.
pub fn magnitude_color(t: f64) -> [u8; 3] {
    let t = t.clamp(0.0, 1.0);
    let r = if t <= 0.5 {
        80.0 + (t / 0.5) * (255.0 - 80.0)
    } else {
        255.0
    };
    let g = if t <= 0.5 {
        255.0
    } else {
        255.0 - ((t - 0.5) / 0.5) * (255.0 - 80.0)
    };
    let b = if t <= 0.5 {
        255.0 - (t / 0.5) * (255.0 - 80.0)
    } else {
        80.0
    };
    [r as u8, g as u8, b as u8]
}

/// Format an engineering value with adaptive precision.
///
/// * Non-finite → `"NaN"`
/// * `|v| < 1e-12` → `"0"` (exact zero, not misleading tiny value)
/// * Otherwise → 4 significant figures in scientific notation
pub fn fmt_eng(v: f64) -> String {
    if !v.is_finite() {
        return "NaN".into();
    }
    if v.abs() < 1e-12 {
        return "0".into();
    }
    format!("{:.4e}", v)
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── ResultViewMode ──

    #[test]
    fn mode_default_is_model() {
        assert_eq!(ResultViewMode::default(), ResultViewMode::Model);
    }

    #[test]
    fn model_does_not_require_result() {
        assert!(!ResultViewMode::Model.requires_result());
    }

    #[test]
    fn all_result_modes_require_result() {
        assert!(ResultViewMode::Deformed.requires_result());
        assert!(ResultViewMode::Displacement.requires_result());
        assert!(ResultViewMode::Reaction.requires_result());
        assert!(ResultViewMode::MemberEndForces.requires_result());
    }

    #[test]
    fn mode_labels() {
        assert_eq!(ResultViewMode::Model.label(), "Model");
        assert_eq!(ResultViewMode::Deformed.label(), "Deformed");
        assert_eq!(ResultViewMode::Displacement.label(), "Displacement");
        assert_eq!(ResultViewMode::Reaction.label(), "Reaction");
        assert_eq!(ResultViewMode::MemberEndForces.label(), "End Forces");
    }

    // ── ResultDisplayState ──

    #[test]
    fn display_default() {
        let d = ResultDisplayState::default();
        assert_eq!(d.mode, ResultViewMode::Model);
        assert!(d.show_undeformed);
        assert!(d.show_grid);
        assert!((d.deformation_scale - 100.0).abs() < 1e-15);
    }

    #[test]
    fn display_reset() {
        let mut d = ResultDisplayState::default();
        d.mode = ResultViewMode::Deformed;
        d.deformation_scale = 500.0;
        d.show_undeformed = false;
        d.reset();
        assert_eq!(d.mode, ResultViewMode::Model);
        assert!((d.deformation_scale - 100.0).abs() < 1e-15);
        assert!(d.show_undeformed);
    }

    #[test]
    fn display_diagram_defaults() {
        let d = ResultDisplayState::default();
        assert!(!d.show_diagrams);
        assert!(d.diagram_axial);
        assert!(d.diagram_shear);
        assert!(d.diagram_moment);
        assert!((d.diagram_scale - 1.0).abs() < 1e-15);
        assert!(!d.diagram_all_members);
    }

    #[test]
    fn display_reset_clears_diagrams() {
        let mut d = ResultDisplayState::default();
        d.show_diagrams = true;
        d.diagram_scale = 5.0;
        d.diagram_all_members = true;
        d.reset();
        assert!(!d.show_diagrams);
        assert!((d.diagram_scale - 1.0).abs() < 1e-15);
        assert!(!d.diagram_all_members);
    }

    #[test]
    fn needs_result_reflects_mode() {
        let mut d = ResultDisplayState::default();
        assert!(!d.needs_result());
        d.mode = ResultViewMode::Deformed;
        assert!(d.needs_result());
        d.mode = ResultViewMode::Displacement;
        assert!(d.needs_result());
    }

    // ── View-only visibility ──

    #[test]
    fn visibility_defaults_to_nothing_hidden() {
        let d = ResultDisplayState::default();
        assert!(!d.has_hidden());
        assert!(!d.is_node_hidden(NodeId(0)));
        assert!(!d.is_member_hidden(MemberId(0)));
    }

    #[test]
    fn hide_and_show_all_roundtrip() {
        let mut d = ResultDisplayState::default();
        d.hide_node(NodeId(3));
        d.hide_member(MemberId(7));
        assert!(d.is_node_hidden(NodeId(3)));
        assert!(d.is_member_hidden(MemberId(7)));
        assert!(d.has_hidden());
        d.show_all();
        assert!(!d.has_hidden());
        assert!(!d.is_node_hidden(NodeId(3)));
    }

    #[test]
    fn reset_clears_visibility() {
        let mut d = ResultDisplayState::default();
        d.hide_node(NodeId(1));
        d.reset();
        assert!(!d.has_hidden());
    }

    #[test]
    fn isolate_node_keeps_node_and_its_members() {
        let all_nodes = [NodeId(0), NodeId(1), NodeId(2)];
        let all_members = [MemberId(0), MemberId(1)];
        let mut d = ResultDisplayState::default();
        // Node 1 belongs to member 1 only.
        d.isolate_node(&all_nodes, &all_members, NodeId(1), &[MemberId(1)]);
        assert!(!d.is_node_hidden(NodeId(1)));
        assert!(d.is_node_hidden(NodeId(0)));
        assert!(d.is_node_hidden(NodeId(2)));
        assert!(!d.is_member_hidden(MemberId(1)));
        assert!(d.is_member_hidden(MemberId(0)));
    }

    #[test]
    fn isolate_member_keeps_member_and_its_nodes() {
        let all_nodes = [NodeId(0), NodeId(1), NodeId(2)];
        let all_members = [MemberId(0), MemberId(1)];
        let mut d = ResultDisplayState::default();
        d.isolate_member(&all_nodes, &all_members, MemberId(0), &[NodeId(0), NodeId(1)]);
        assert!(!d.is_member_hidden(MemberId(0)));
        assert!(d.is_member_hidden(MemberId(1)));
        assert!(!d.is_node_hidden(NodeId(0)));
        assert!(!d.is_node_hidden(NodeId(1)));
        assert!(d.is_node_hidden(NodeId(2)));
    }

    // ── Displacement magnitude ──

    #[test]
    fn displacement_magnitude_zero() {
        assert_eq!(displacement_magnitude(0.0, 0.0, 0.0), 0.0);
    }

    #[test]
    fn displacement_magnitude_unit() {
        assert!((displacement_magnitude(1.0, 0.0, 0.0) - 1.0).abs() < 1e-15);
        assert!((displacement_magnitude(0.0, 1.0, 0.0) - 1.0).abs() < 1e-15);
        assert!((displacement_magnitude(0.0, 0.0, 1.0) - 1.0).abs() < 1e-15);
    }

    #[test]
    fn displacement_magnitude_3d() {
        let m = displacement_magnitude(3.0, 4.0, 0.0);
        assert!((m - 5.0).abs() < 1e-15);
    }

    #[test]
    fn displacement_magnitude_excludes_rotation() {
        let m = displacement_magnitude(1.0, 0.0, 0.0);
        let m_with_rot = displacement_magnitude(1.0, 0.0, 0.0);
        assert_eq!(m, m_with_rot);
    }

    #[test]
    fn displacement_magnitude_non_finite() {
        assert_eq!(displacement_magnitude(f64::NAN, 0.0, 0.0), 0.0);
        assert_eq!(displacement_magnitude(0.0, f64::INFINITY, 0.0), 0.0);
    }

    #[test]
    fn displacement_magnitudes_flat() {
        let disp = vec![1.0, 2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mags = displacement_magnitudes(&disp, 2);
        assert!((mags[0] - 3.0).abs() < 1e-15);
        assert!((mags[1] - 0.0).abs() < 1e-15);
    }

    #[test]
    fn displacement_magnitudes_short_vector() {
        let disp = vec![1.0, 0.0, 0.0];
        let mags = displacement_magnitudes(&disp, 2);
        assert!((mags[0] - 1.0).abs() < 1e-15);
        assert_eq!(mags[1], 0.0);
    }

    // ── max_abs / normalize ──

    #[test]
    fn max_abs_empty() {
        assert_eq!(max_abs(&[]), 0.0);
    }

    #[test]
    fn max_abs_mixed() {
        assert_eq!(max_abs(&[-3.0, 1.0, 5.0, -2.0]), 5.0);
    }

    #[test]
    fn max_abs_non_finite() {
        assert_eq!(max_abs(&[f64::NAN, f64::INFINITY]), 0.0);
    }

    #[test]
    fn normalize_zero_max() {
        assert_eq!(normalize(1.0, 0.0), 0.0);
    }

    #[test]
    fn normalize_negative_max() {
        assert_eq!(normalize(1.0, -1.0), 0.0);
    }

    #[test]
    fn normalize_half() {
        assert!((normalize(0.5, 1.0) - 0.5).abs() < 1e-15);
    }

    #[test]
    fn normalize_clamps_to_one() {
        assert_eq!(normalize(2.0, 1.0), 1.0);
    }

    // ── magnitude_color ──

    #[test]
    fn color_blue_at_zero() {
        let [r, g, b] = magnitude_color(0.0);
        assert_eq!(r, 80);
        assert_eq!(g, 255);
        assert_eq!(b, 255);
    }

    #[test]
    fn color_green_at_half() {
        let [r, g, b] = magnitude_color(0.5);
        assert_eq!(r, 255);
        assert_eq!(g, 255);
        assert_eq!(b, 80);
    }

    #[test]
    fn color_red_at_one() {
        let [r, g, b] = magnitude_color(1.0);
        assert_eq!(r, 255);
        assert_eq!(g, 80);
        assert_eq!(b, 80);
    }

    #[test]
    fn color_clamps_input() {
        let c_below = magnitude_color(-1.0);
        let c_zero = magnitude_color(0.0);
        assert_eq!(c_below, c_zero);

        let c_above = magnitude_color(2.0);
        let c_one = magnitude_color(1.0);
        assert_eq!(c_above, c_one);
    }

    // ── fmt_eng ──

    #[test]
    fn fmt_eng_zero() {
        assert_eq!(fmt_eng(0.0), "0");
        assert_eq!(fmt_eng(1e-13), "0");
        assert_eq!(fmt_eng(-1e-13), "0");
    }

    #[test]
    fn fmt_eng_nan() {
        assert_eq!(fmt_eng(f64::NAN), "NaN");
        assert_eq!(fmt_eng(f64::INFINITY), "NaN");
    }

    #[test]
    fn fmt_eng_value() {
        assert_eq!(fmt_eng(1.23456e-3), "1.2346e-3");
        assert_eq!(fmt_eng(-5.0), "-5.0000e0");
    }

    // ── Section cut plane ───────────────────────────────────────────────

    fn cut_x(offset: f64) -> SectionCut {
        SectionCut {
            enabled: true,
            axis: CutAxis::X,
            offset,
            ..SectionCut::default()
        }
    }

    #[test]
    fn cut_default_is_disabled() {
        let c = SectionCut::default();
        assert!(!c.enabled);
        assert_eq!(c.axis, CutAxis::X);
        assert!(c.normal.is_none());
        assert_eq!(c.offset, 0.0);
        assert!(!c.flip);
        assert!(!c.positioned);
        // A disabled cut keeps everything and never clips a segment.
        assert!(c.keeps([-100.0, 0.0, 0.0]));
        assert_eq!(
            c.clip_segment([-1.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            Some(([-1.0, 0.0, 0.0], [1.0, 0.0, 0.0]))
        );
    }

    #[test]
    fn cut_axis_labels_and_normals() {
        assert_eq!(CutAxis::ALL.len(), 3);
        assert_eq!(CutAxis::X.label(), "X");
        assert_eq!(CutAxis::Y.label(), "Y");
        assert_eq!(CutAxis::Z.label(), "Z");
        assert_eq!(CutAxis::X.normal(), [1.0, 0.0, 0.0]);
        assert_eq!(CutAxis::Y.normal(), [0.0, 1.0, 0.0]);
        assert_eq!(CutAxis::Z.normal(), [0.0, 0.0, 1.0]);
        assert_eq!(CutAxis::default(), CutAxis::X);
    }

    #[test]
    fn cut_signed_distance_and_keeps() {
        let c = cut_x(2.0);
        assert_eq!(c.signed_distance([5.0, 0.0, 0.0]), Some(3.0));
        assert_eq!(c.signed_distance([0.0, 0.0, 0.0]), Some(-2.0));
        assert!(c.keeps([5.0, 0.0, 0.0]));
        assert!(!c.keeps([0.0, 0.0, 0.0]));
        // On-plane is kept.
        assert!(c.keeps([2.0, 0.0, 0.0]));
    }

    #[test]
    fn cut_flip_reverses_kept_side() {
        let mut c = cut_x(2.0);
        c.flip = true;
        assert!(c.keeps([0.0, 0.0, 0.0]));
        assert!(!c.keeps([5.0, 0.0, 0.0]));
        // The GPU plane is pre-flipped so a positive distance still means keep.
        let p = c.gpu_plane().unwrap();
        assert_eq!(&p[..3], &[-1.0, 0.0, 0.0]);
        assert_eq!(p[3], -2.0);
    }

    #[test]
    fn cut_custom_normal_is_normalised() {
        let c = SectionCut {
            enabled: true,
            normal: Some([0.0, 0.0, 10.0]),
            offset: 1.0,
            ..SectionCut::default()
        };
        let n = c.unit_normal().unwrap();
        assert!((n[2] - 1.0).abs() < 1e-15);
        assert!(c.keeps([0.0, 0.0, 2.0]));
        assert!(!c.keeps([0.0, 0.0, 0.0]));
        // Degenerate custom normal disables clipping entirely.
        let bad = SectionCut {
            enabled: true,
            normal: Some([0.0, 0.0, 0.0]),
            ..SectionCut::default()
        };
        assert!(bad.unit_normal().is_none());
        assert!(bad.keeps([1.0, 2.0, 3.0]));
        assert!(bad.gpu_plane().is_none());
    }

    #[test]
    fn cut_clip_segment_cases() {
        let c = cut_x(0.0);
        // Both inside → unchanged.
        assert_eq!(
            c.clip_segment([1.0, 0.0, 0.0], [2.0, 0.0, 0.0]),
            Some(([1.0, 0.0, 0.0], [2.0, 0.0, 0.0]))
        );
        // Both outside → clipped away.
        assert_eq!(c.clip_segment([-2.0, 0.0, 0.0], [-1.0, 0.0, 0.0]), None);
        // Straddling: the outside end is moved to the crossing point.
        let (a, b) = c.clip_segment([-1.0, 0.0, 0.0], [1.0, 0.0, 0.0]).unwrap();
        assert!((a[0]).abs() < 1e-15, "crossing x = {}", a[0]);
        assert_eq!(b, [1.0, 0.0, 0.0]);
        let (a2, b2) = c.clip_segment([1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]).unwrap();
        assert_eq!(a2, [1.0, 0.0, 0.0]);
        assert!((b2[0]).abs() < 1e-15);
    }

    #[test]
    fn cut_sync_bounds_centres_then_clamps() {
        let mut c = SectionCut {
            enabled: true,
            ..SectionCut::default()
        };
        // First sync centres at the midpoint and marks positioned.
        c.sync_bounds(-4.0, 8.0);
        assert!((c.offset - 2.0).abs() < 1e-15);
        assert!(c.positioned);
        // Further syncs clamp instead of re-centring.
        c.offset = 100.0;
        c.sync_bounds(-4.0, 8.0);
        assert_eq!(c.offset, 8.0);
        c.offset = -100.0;
        c.sync_bounds(-4.0, 8.0);
        assert_eq!(c.offset, -4.0);
        // recenter() forces the midpoint again.
        c.recenter();
        c.sync_bounds(-4.0, 8.0);
        assert!((c.offset - 2.0).abs() < 1e-15);
    }

    #[test]
    fn cut_sync_bounds_handles_degenerate_and_non_finite() {
        let mut c = SectionCut::default();
        c.sync_bounds(3.0, 3.0); // flat model along the axis
        assert!((c.offset - 3.0).abs() < 1e-15);
        let mut c2 = SectionCut::default();
        c2.sync_bounds(f64::NAN, f64::INFINITY);
        assert_eq!(c2.offset, 0.0);
        // Reversed range is ordered.
        let mut c3 = SectionCut::default();
        c3.sync_bounds(8.0, -4.0);
        assert!((c3.offset - 2.0).abs() < 1e-15);
    }

    #[test]
    fn display_section_cut_defaults_off_and_resets() {
        let mut d = ResultDisplayState::default();
        assert!(!d.section_cut.enabled);
        d.section_cut.enabled = true;
        d.section_cut.offset = 42.0;
        d.reset();
        assert!(!d.section_cut.enabled);
        assert_eq!(d.section_cut.offset, 0.0);
    }
}
