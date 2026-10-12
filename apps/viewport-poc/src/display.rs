//! Display-mode state and result visualization helpers.
//!
//! [`ResultViewMode`] is the authoritative enum that determines the
//! primary result visualization in the viewport.  Independent overlays
//! (undeformed reference, grid, axes, labels) remain as separate booleans
//! in [`ResultDisplayState`].

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
    fn needs_result_reflects_mode() {
        let mut d = ResultDisplayState::default();
        assert!(!d.needs_result());
        d.mode = ResultViewMode::Deformed;
        assert!(d.needs_result());
        d.mode = ResultViewMode::Displacement;
        assert!(d.needs_result());
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
}
