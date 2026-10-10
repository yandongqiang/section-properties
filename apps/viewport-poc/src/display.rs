//! Display-mode state for analysis results.

/// What to show in the viewport.
#[derive(Debug, Clone)]
pub struct ResultDisplayState {
    /// Scale factor for deformed-shape display (1.0 = actual scale).
    pub deformation_scale: f64,
    /// Show the undeformed structure.
    pub show_undeformed: bool,
    /// Show the deformed structure.
    pub show_deformed: bool,
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
            deformation_scale: 100.0,
            show_undeformed: true,
            show_deformed: true,
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
}
