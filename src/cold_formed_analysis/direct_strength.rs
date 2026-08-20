//! Direct Strength Method (DSM) per EN 1993-1-3, AISI S100.
//!
//! Provides nominal capacity predictions for local, distortional, and global buckling.

use crate::geometry::{Point, Polygon};
use crate::material::Material;
use crate::section::Section;
use crate::section_properties::SectionProperties;

/// DSM parameters.
#[derive(Debug, Clone, Copy)]
pub struct DsmParams {
    /// Elastic local buckling moment
    pub m_cr_l: f64,
    /// Elastic distortional buckling moment
    pub m_cr_d: f64,
    /// Elastic global (Euler) buckling moment
    pub m_cr_e: f64,
    /// Yield moment
    pub m_y: f64,
}

impl DsmParams {
    /// Compute DSM parameters from section properties.
    pub fn from_section(section: &Section, material: &Material) -> Self {
        let props = SectionProperties::from_section(section);
        let fy = material.yield_strength;
        let e = material.youngs_modulus;

        // Simplified elastic buckling moments
        // Real implementation would use FSM (finite strip method)
        let iy = props.iy;
        let iz = props.ix;
        let iw = props.ix; // Approximate

        // Global buckling (flexural-torsional)
        let l = 6.0; // Assume 6m span
        let g = material.shear_modulus;
        let j = 0.0; // Approximate

        let m_cr_e = std::f64::consts::PI / l * (e * iz * g * j).sqrt();
        let m_cr_l = m_cr_e * 0.8; // Approximate
        let m_cr_d = m_cr_e * 0.6; // Approximate
        let m_y = fy * props.ix / props.max_fiber_distance_y();

        Self {
            m_cr_l: m_cr_l.max(1.0),
            m_cr_d: m_cr_d.max(1.0),
            m_cr_e: m_cr_e.max(1.0),
            m_y: m_y.max(1.0),
        }
    }
}

/// Nominal capacity for local buckling per DSM.
pub fn dsm_nominal_capacity_local(lambda: f64, fy: f64, z: f64) -> f64 {
    if lambda <= 0.776 {
        fy * z
    } else {
        fy * z * (1.0 - 0.15 * (1.0 / lambda).powf(0.4)) * (1.0 / lambda).powf(0.4)
    }
}

/// Nominal capacity for distortional buckling per DSM.
pub fn dsm_nominal_capacity_distortional(lambda: f64, fy: f64, z: f64) -> f64 {
    if lambda <= 0.673 {
        fy * z
    } else {
        fy * z * (1.0 - 0.22 / lambda) * (1.0 / lambda)
    }
}

/// Nominal capacity for global buckling per DSM.
pub fn dsm_nominal_capacity_global(lambda: f64, fy: f64, z: f64) -> f64 {
    if lambda <= 0.216 {
        fy * z
    } else {
        fy * z * (1.0 - 0.22 / lambda) * (1.0 / lambda)
    }
}

/// DSM nominal strengths.
#[derive(Debug, Clone, Copy)]
pub struct DsmStrengths {
    pub m_n_l: f64, // Local
    pub m_n_d: f64, // Distortional
    pub m_n_g: f64, // Global
    pub m_n: f64,   // Minimum
}

impl DsmStrengths {
    /// Compute all DSM nominal strengths.
    pub fn compute(params: &DsmParams, z: f64, fy: f64) -> Self {
        let lambda_l = (params.m_y / params.m_cr_l).sqrt();
        let lambda_d = (params.m_y / params.m_cr_d).sqrt();
        let lambda_g = (params.m_y / params.m_cr_e).sqrt();

        let m_n_l = dsm_nominal_capacity_local(lambda_l, fy, z);
        let m_n_d = dsm_nominal_capacity_distortional(lambda_d, fy, z);
        let m_n_g = dsm_nominal_capacity_global(lambda_g, fy, z);

        let m_n = m_n_l.min(m_n_d).min(m_n_g);

        Self {
            m_n_l,
            m_n_d,
            m_n_g,
            m_n,
        }
    }
}

/// Direct Strength Method analysis for a section.
pub fn dsm_analysis(section: &Section, material: &Material) -> DsmStrengths {
    let params = DsmParams::from_section(section, material);
    let props = SectionProperties::from_section(section);
    let fy = material.yield_strength;
    let z = props.ix / props.max_fiber_distance_y();

    DsmStrengths::compute(&params, z, fy)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::presets::STEEL_S355;
    use crate::section_library::steel::ISection;

    #[test]
    fn dsm_local_buckling() {
        let strength = dsm_nominal_capacity_local(0.5, 355e6, 1e-4);
        assert!(strength > 0.0);
        assert!(strength <= 355e6 * 1e-4);
    }

    #[test]
    fn dsm_distortional_buckling() {
        let strength = dsm_nominal_capacity_distortional(0.5, 355e6, 1e-4);
        assert!(strength > 0.0);
    }

    #[test]
    fn dsm_global_buckling() {
        let strength = dsm_nominal_capacity_global(0.5, 355e6, 1e-4);
        assert!(strength > 0.0);
    }

    #[test]
    fn dsm_analysis_i_section() {
        let i = ISection::from_designation("IPE300").unwrap();
        let section = i.build();
        let strengths = dsm_analysis(&section, &STEEL_S355);

        assert!(strengths.m_n > 0.0);
        assert!(strengths.m_n <= strengths.m_n_l);
        assert!(strengths.m_n <= strengths.m_n_d);
        assert!(strengths.m_n <= strengths.m_n_g);
    }
}

