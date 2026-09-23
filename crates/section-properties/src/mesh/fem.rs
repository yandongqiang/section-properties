//! Stress result type for FEM-based section analysis.

/// Stress/strain result at a point.
#[derive(Debug, Clone, Copy)]
pub struct StressResult {
    /// Normal stress in x direction
    pub sigma_x: f64,
    /// Normal stress in y direction
    pub sigma_y: f64,
    /// Shear stress xy
    pub tau_xy: f64,
    /// Strain in x direction
    pub epsilon_x: f64,
    /// Strain in y direction
    pub epsilon_y: f64,
    /// Shear strain xy
    pub gamma_xy: f64,
    /// Von Mises stress
    pub von_mises: f64,
    /// Principal stress 1 (major)
    pub sigma_1: f64,
    /// Principal stress 2 (minor)
    pub sigma_2: f64,
    /// Principal angle (radians)
    pub principal_angle: f64,
}

impl StressResult {
    /// Create from stress components.
    pub fn from_stress(sigma_x: f64, sigma_y: f64, tau_xy: f64) -> Self {
        let sigma_avg = (sigma_x + sigma_y) * 0.5;
        let r = ((sigma_x - sigma_y) * 0.5).powi(2) + tau_xy * tau_xy;
        let r = r.sqrt();
        let sigma_1 = sigma_avg + r;
        let sigma_2 = sigma_avg - r;
        let principal_angle = 0.5 * (2.0 * tau_xy).atan2(sigma_x - sigma_y);
        let von_mises =
            (sigma_x * sigma_x - sigma_x * sigma_y + sigma_y * sigma_y + 3.0 * tau_xy * tau_xy)
                .sqrt();

        Self {
            sigma_x,
            sigma_y,
            tau_xy,
            epsilon_x: 0.0,
            epsilon_y: 0.0,
            gamma_xy: 0.0,
            von_mises,
            sigma_1,
            sigma_2,
            principal_angle,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stress_result_von_mises() {
        let stress = StressResult::from_stress(100.0, 50.0, 30.0);
        let expected =
            (100.0_f64.powi(2) - 100.0 * 50.0 + 50.0_f64.powi(2) + 3.0 * 30.0_f64.powi(2)).sqrt();
        assert!((stress.von_mises - expected).abs() < 1e-6);
    }
}
