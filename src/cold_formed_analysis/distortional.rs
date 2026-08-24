//! Distortional buckling analysis for cold-formed steel.
//!
//! Per EN 1993-1-3 Annex B, AS/NZS 4600, AISI S100 Appendix 1.

use crate::material::Material;
use crate::section::Section;
use crate::section_properties::SectionProperties;

/// Distortional buckling parameters.
#[derive(Debug, Clone)]
pub struct DistortionalParams {
    /// Yield strength
    pub fy: f64,
    /// Young's modulus
    pub e: f64,
    /// Shear modulus
    pub g: f64,
    /// Poisson's ratio
    pub nu: f64,
    /// Length for distortional buckling (buckling half-wavelength)
    pub length: f64,
    /// Warping constant
    pub iw: f64,
    /// Torsion constant
    pub j: f64,
}

impl Default for DistortionalParams {
    fn default() -> Self {
        Self {
            fy: 350e6,
            e: 200e9,
            g: 76.9e9,
            nu: 0.3,
            length: 1.0,
            iw: 0.0,
            j: 0.0,
        }
    }
}

/// Distortional buckling results.
#[derive(Debug, Clone)]
pub struct DistortionalBuckling {
    /// Elastic distortional buckling stress
    pub sigma_crd: f64,
    /// Distortional slenderness
    pub lambda_d: f64,
    /// Reduction factor (chi_d)
    pub chi_d: f64,
    /// Nominal distortional buckling capacity
    pub n_crd: f64,
}

impl DistortionalBuckling {
    /// Compute distortional buckling stress for a section.
    pub fn analyze(section: &Section, material: &Material, params: &DistortionalParams) -> Self {
        let props = SectionProperties::from_section(section);
        let area = props.area;

        // Elastic distortional buckling stress per EN 1993-1-3 Annex B
        // σ_crd = k_fe * E * (I_w / (A * L^2)) + G * J / A
        // Simplified for common sections

        let (sigma_crd, lambda_d, chi_d) = if section.holes.is_empty() {
            // Open section (C, Z, hat, etc.)
            compute_open_section_distortional(section, material, params)
        } else {
            // Closed section (box, tube)
            compute_closed_section_distortional(section, material, params)
        };

        let n_crd = chi_d * area * material.yield_strength;

        Self {
            sigma_crd,
            lambda_d,
            chi_d,
            n_crd,
        }
    }

    /// Distortional buckling capacity.
    pub fn capacity(&self) -> f64 {
        self.n_crd
    }
}

/// Compute distortional buckling for open sections.
fn compute_open_section_distortional(
    section: &Section,
    material: &Material,
    params: &DistortionalParams,
) -> (f64, f64, f64) {
    let props = SectionProperties::from_section(section);
    let _area = props.area;
    let bounds = section.bounds();
    let h = bounds.3 - bounds.2; // depth
    let _b = bounds.1 - bounds.0; // width

    // For C/Z sections, use simplified formula from Hancock et al.
    // σ_crd = (k_φ * E * I_w / (A * L^2) + G * J / A) / (1 + ...)

    // Get warping and torsion constants
    let _iw = if params.iw > 0.0 {
        params.iw
    } else {
        estimate_warping_constant(section, &props)
    };
    let _j = if params.j > 0.0 {
        params.j
    } else {
        estimate_torsion_constant(section)
    };

    // Elastic distortional buckling stress
    // Simplified: σ_crd = k_d * E * (t/h)^2 for lips
    // More accurate: use finite strip or GBT
    let k_d = distortional_buckling_coefficient(section);
    let t = estimate_thickness(section);

    let sigma_crd = k_d * material.youngs_modulus * (t / h).powi(2);

    // Slenderness
    let lambda_d = (material.yield_strength / sigma_crd).sqrt();

    // Reduction factor per EN 1993-1-3 Eq B.3
    let chi_d = if lambda_d <= 0.561 {
        1.0
    } else {
        1.0 / (lambda_d * lambda_d) * (1.0 - 0.25 / (lambda_d * lambda_d))
    }
    .min(1.0);

    (sigma_crd, lambda_d, chi_d)
}

/// Compute distortional buckling for closed sections.
fn compute_closed_section_distortional(
    section: &Section,
    material: &Material,
    params: &DistortionalParams,
) -> (f64, f64, f64) {
    let props = SectionProperties::from_section(section);
    let area = props.area;

    // For closed sections, distortional buckling is usually not critical
    // But can occur in very slender box sections
    let iw = params.iw.max(estimate_warping_constant(section, &props));
    let j = params.j.max(estimate_torsion_constant(section));
    let length = params.length.max(1.0);

    // Simplified formula
    let sigma_crd =
        material.youngs_modulus * iw / (area * length * length) + material.shear_modulus * j / area;

    let lambda_d = (material.yield_strength / sigma_crd).sqrt();
    let chi_d = if lambda_d <= 0.561 {
        1.0
    } else {
        1.0 / (lambda_d * lambda_d)
    }
    .min(1.0);

    (sigma_crd, lambda_d, chi_d)
}

/// Distortional buckling coefficient for common sections.
fn distortional_buckling_coefficient(section: &Section) -> f64 {
    let bounds = section.bounds();
    let h = bounds.3 - bounds.2;
    let b = bounds.1 - bounds.0;
    let aspect = h / b;

    // Simplified coefficients based on section type
    // C-section: ~0.5-1.0
    // Z-section: ~0.3-0.7
    // Hat section: ~0.8-1.5

    if aspect > 2.0 {
        0.5 // Deep section (C/Z-like)
    } else if aspect > 1.0 {
        0.8
    } else {
        1.2 // Shallow (hat-like)
    }
}

/// Estimate section thickness from area/perimeter.
fn estimate_thickness(section: &Section) -> f64 {
    let props = SectionProperties::from_section(section);
    let peri = section.perimeter();
    if peri > 0.0 { props.area / peri } else { 0.001 }
}

/// Estimate warping constant for open section.
fn estimate_warping_constant(section: &Section, props: &SectionProperties) -> f64 {
    let bounds = section.bounds();
    let h = bounds.3 - bounds.2;
    let _b = bounds.1 - bounds.0;

    // For C-section: I_w ≈ I_y * (h - t_f)^2 / 4
    props.iy * h.powi(2) / 4.0
}

/// Estimate St. Venant torsion constant.
fn estimate_torsion_constant(section: &Section) -> f64 {
    let _props = SectionProperties::from_section(section);
    let bounds = section.bounds();
    let h = bounds.3 - bounds.2;
    let b = bounds.1 - bounds.0;

    // Thin-walled open: J = Σ(1/3 * b * t^3)
    // Rough estimate
    let t = estimate_thickness(section);
    (h + 2.0 * b) * t.powi(3) / 3.0
}

/// Combined local-distortional interaction (EN 1993-1-3 Section 5.5.2).
pub fn local_distortional_interaction(local_capacity: f64, distortional_capacity: f64) -> f64 {
    // EN 1993-1-3 Eq 5.10
    // For intermediate members with both local and distortional buckling
    let n_cr = local_capacity.min(distortional_capacity);
    let n_c = local_capacity;
    let n_cd = distortional_capacity;

    if n_cr == n_c && n_cr == n_cd {
        n_cr
    } else if n_c < n_cd {
        // Local governs
        n_c * (1.0 - 0.5 * (1.0 - n_c / n_cd))
    } else {
        // Distortional governs
        n_cd * (1.0 - 0.5 * (1.0 - n_cd / n_c))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::presets::STEEL_S355;
    use crate::section_library::ParametricSection;

    #[test]
    fn distortional_c_section() {
        // Create a C-section
        let c =
            crate::section_library::steel::ChannelSection::new(0.2, 0.07, 0.002, 0.002, 0.0, 0.015);
        let section = c.build();

        let params = DistortionalParams {
            fy: STEEL_S355.yield_strength,
            e: STEEL_S355.youngs_modulus,
            g: STEEL_S355.shear_modulus,
            nu: STEEL_S355.poissons_ratio,
            length: 3.0,
            iw: 0.0,
            j: 0.0,
        };

        let db = DistortionalBuckling::analyze(&section, &STEEL_S355, &params);

        assert!(db.sigma_crd > 0.0);
        assert!(db.lambda_d > 0.0);
        assert!(db.chi_d <= 1.0);
        assert!(db.n_crd > 0.0);
    }

    #[test]
    fn distortional_box_section() {
        // Box section (closed)
        let rhs = crate::section_library::steel::RectangularHollowSection::new(
            0.1, 0.05, 0.002, 0.003, 0.001,
        );
        let section = rhs.build();

        let params = DistortionalParams::default();
        let db = DistortionalBuckling::analyze(&section, &STEEL_S355, &params);

        assert!(db.n_crd > 0.0);
    }

    #[test]
    fn local_distortional_interaction_test() {
        let local = 100e3;
        let distortional = 120e3;
        let combined = local_distortional_interaction(local, distortional);
        // Combined capacity must be less than the minimum individual capacity
        assert!(combined < local);
        assert!(combined < distortional);
        assert!(combined > 0.0);
    }

    #[test]
    fn distortional_c_section_depth_gt_width() {
        // Regression test: h (depth) and b (width) were swapped.
        // C-section: depth=0.2, width=0.07 -> h=0.2, b=0.07
        // sigma_crd = k_d * E * (t/h)^2
        // With correct h=0.2: lower sigma_crd (larger h)
        // With buggy h=0.07: higher sigma_crd (smaller h)
        let c =
            crate::section_library::steel::ChannelSection::new(0.2, 0.07, 0.002, 0.002, 0.0, 0.015);
        let section = c.build();

        let params = DistortionalParams {
            fy: STEEL_S355.yield_strength,
            e: STEEL_S355.youngs_modulus,
            g: STEEL_S355.shear_modulus,
            nu: STEEL_S355.poissons_ratio,
            length: 3.0,
            iw: 0.0,
            j: 0.0,
        };

        let db = DistortionalBuckling::analyze(&section, &STEEL_S355, &params);

        // Also compute with a section that has swapped dimensions
        let c2 =
            crate::section_library::steel::ChannelSection::new(0.07, 0.2, 0.002, 0.002, 0.0, 0.015);
        let section2 = c2.build();
        let db2 = DistortionalBuckling::analyze(&section2, &STEEL_S355, &params);

        // With correct code: section1 (depth=0.2) uses h=0.2 -> lower sigma_crd
        //                    section2 (depth=0.07) uses h=0.07 -> higher sigma_crd
        // So sigma_crd1 < sigma_crd2
        // With buggy code: section1 uses h=0.07 (width) -> higher sigma_crd
        //                  section2 uses h=0.2 (width) -> lower sigma_crd
        // So sigma_crd1 > sigma_crd2 (inverted!)
        assert!(
            db.sigma_crd < db2.sigma_crd,
            "sigma_crd for depth=0.2 should be < sigma_crd for depth=0.07; got {} vs {}",
            db.sigma_crd,
            db2.sigma_crd
        );
    }
}
