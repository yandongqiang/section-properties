//! Direct Strength Method (DSM) per EN 1993-1-3, AISI S100.
//!
//! Provides nominal capacity predictions for local, distortional, and global buckling.

use crate::cold_formed_analysis::distortional::{DistortionalBuckling, DistortionalParams};
use crate::material::Material;
use crate::plastic::warping::WarpingProperties;
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
    /// Compute DSM parameters from section properties with a given span.
    ///
    /// `span` is the unbraced length [m] for lateral-torsional buckling.
    pub fn from_section_with_span(section: &Section, material: &Material, span: f64) -> Self {
        let props = SectionProperties::from_section(section);
        let warping = WarpingProperties::from_section(section);
        let fy = material.yield_strength;
        let e = material.youngs_modulus;
        let g = material.shear_modulus;
        let nu = material.poissons_ratio;

        let iy = props.iy;
        let j = warping.j;
        let iw = warping.iw;
        let z_x = props.section_modulus_x();

        let m_y = fy * z_x;

        // Elastic global (lateral-torsional) buckling moment:
        // M_cr = (π/L) * sqrt(E * Iy * (G*J + (π/L)² * E * Iw))
        let pi_over_l = std::f64::consts::PI / span;
        let m_cr_e = pi_over_l * (e * iy * (g * j + pi_over_l * pi_over_l * e * iw)).sqrt();

        // Elastic distortional buckling moment from distortional analysis.
        let dist_params = DistortionalParams::default();
        let dist = DistortionalBuckling::analyze(section, material, &dist_params);
        let m_cr_d = dist.sigma_crd * z_x;

        // Elastic local buckling moment: simplified plate buckling estimate.
        // Plate buckling stress sigma_cr = k * pi² * E * t² / (12*(1-ν²)*b²)
        // for a simply supported plate of width b. The plate width b and
        // thickness t are rough estimates from the bounding box and area, so
        // this is only an approximation until a proper FSM/strip analysis.
        let (min_x, max_x, min_y, max_y) = section.bounds();
        let h = max_y - min_y;
        let b = max_x - min_x;
        // Effective thickness estimate from area / perimeter (rough)
        let t_est = props.area / (2.0 * (h + b));
        let k_local = 4.0; // simply supported plate buckling coefficient
        let m_cr_l = if t_est > 1e-10 && h > 1e-10 {
            let sigma_cr_l = k_local * std::f64::consts::PI.powi(2) * e * t_est.powi(2)
                / (12.0 * (1.0 - nu * nu) * h * h);
            sigma_cr_l * z_x
        } else {
            m_cr_e // fallback: assume local buckling doesn't control
        };

        Self {
            m_cr_l: m_cr_l.max(1e-6),
            m_cr_d: m_cr_d.max(1e-6),
            m_cr_e: m_cr_e.max(1e-6),
            m_y: m_y.max(1e-6),
        }
    }

    /// Compute DSM parameters from section properties (default span = 6 m).
    pub fn from_section(section: &Section, material: &Material) -> Self {
        Self::from_section_with_span(section, material, 6.0)
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
    use crate::section_library::ParametricSection;
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
