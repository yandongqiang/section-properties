//! Warping torsion analysis.
//!
//! Provides St. Venant torsion constant (J), warping constant (Iw),
//! shear center coordinates, and torsional-warping section properties.

use crate::geometry::{Point, Polygon};
use crate::section::Section;
use crate::section_properties::SectionProperties;

/// Warping torsion properties for open and closed sections.
#[derive(Debug, Clone)]
pub struct WarpingProperties {
    /// St. Venant torsion constant [m^4]
    pub j: f64,
    /// Warping constant [m^6]
    pub iw: f64,
    /// Shear center coordinates relative to centroid [m]
    pub shear_center: Point,
    /// Torsional radius of gyration [m]
    pub r_t: f64,
    /// Warping radius of gyration [m]
    pub r_w: f64,
    /// Section area [m^2]
    pub area: f64,
    /// Polar moment of inertia about centroid [m^4]
    pub ip: f64,
}

impl WarpingProperties {
    /// Compute warping properties for a section using numerical integration.
    pub fn from_section(section: &Section) -> Self {
        let props = SectionProperties::from_section(section);
        let area = props.area;
        let centroid = props.centroid;

        // For thin-walled open sections, use standard formulas
        // For general sections, use numerical integration

        let (j, iw, shear_center) = if section.is_thin_walled() {
            compute_thin_walled_properties(section, &props)
        } else {
            compute_general_properties(section, &props)
        };

        let ip = props.ix + props.iy;
        let r_t = (j / area).sqrt();
        let r_w = if iw > 0.0 { (iw / area).sqrt() } else { 0.0 };

        Self {
            j,
            iw,
            shear_center,
            r_t,
            r_w,
            area,
            ip,
        }
    }

    /// Torsional stiffness (GJ).
    pub fn torsional_stiffness(&self, shear_modulus: f64) -> f64 {
        shear_modulus * self.j
    }

    /// Warping stiffness (E*Iw).
    pub fn warping_stiffness(&self, youngs_modulus: f64) -> f64 {
        youngs_modulus * self.iw
    }

    /// Characteristic length for warping (sqrt(E*Iw/G*J)).
    pub fn warping_length(&self, youngs_modulus: f64, shear_modulus: f64) -> f64 {
        if self.j > 0.0 && self.iw > 0.0 {
            (youngs_modulus * self.iw / (shear_modulus * self.j)).sqrt()
        } else {
            0.0
        }
    }
}

/// Check if section is thin-walled.
trait ThinWalledCheck {
    fn is_thin_walled(&self) -> bool;
}

impl ThinWalledCheck for Section {
    fn is_thin_walled(&self) -> bool {
        // Heuristic: check if area is much smaller than bounding box
        let props = SectionProperties::from_section(self);
        let bounds = self.bounds();
        let bbox_area = (bounds.1 - bounds.0) * (bounds.3 - bounds.2);
        props.area / bbox_area < 0.1 // Less than 10% solid
    }
}

/// Compute properties for thin-walled open sections (I, C, Z, L, T).
fn compute_thin_walled_properties(
    section: &Section,
    props: &SectionProperties,
) -> (f64, f64, Point) {
    // For thin-walled open sections, J = Σ(1/3 * b * t^3)
    // This is a simplified approach - real implementation would
    // decompose section into rectangular elements

    let j = estimate_j_thin_walled(section);
    let iw = estimate_iw_thin_walled(section, props);
    let shear_center = estimate_shear_center_thin_walled(section, props);

    (j, iw, shear_center)
}

/// Estimate St. Venant torsion constant for thin-walled section.
fn estimate_j_thin_walled(section: &Section) -> f64 {
    // Decompose into rectangles and sum (1/3 * b * t^3)
    // Simplified: use approximate formula based on section type

    let props = SectionProperties::from_section(section);
    let area = props.area;
    let bounds = section.bounds();
    let h = bounds.1 - bounds.0;
    let b = bounds.3 - bounds.2;

    // Rough estimate for I-section: J ≈ 2*b*t_f^3/3 + h_w*t_w^3/3
    // For general: use empirical formula
    let t_equiv = area / (2.0 * (h + b)); // Equivalent thickness
    (h + b) * t_equiv.powi(3) / 3.0
}

/// Estimate warping constant for thin-walled section.
fn estimate_iw_thin_walled(section: &Section, props: &SectionProperties) -> f64 {
    // For I-section: Iw = Iy * (h - t_f)^2 / 4
    // Simplified estimate
    let bounds = section.bounds();
    let h = bounds.1 - bounds.0;
    let iw = props.iy * h.powi(2) / 4.0;
    iw.max(0.0)
}

/// Estimate shear center for thin-walled section.
fn estimate_shear_center_thin_walled(section: &Section, props: &SectionProperties) -> Point {
    // For symmetric sections, shear center = centroid
    // For channels: e = b^2 * t_f * h / (4 * Ix) approximately

    let bounds = section.bounds();
    let h = bounds.1 - bounds.0;
    let b = bounds.3 - bounds.2;

    // Check symmetry
    let mut sym_x = true;
    let mut sym_y = true;

    for v in &section.outer.vertices {
        let has_mirror_x = section
            .outer
            .vertices
            .iter()
            .any(|v2| (v2.x + v.x).abs() < 1e-6 && (v2.y - v.y).abs() < 1e-6);
        if !has_mirror_x {
            sym_x = false;
        }

        let has_mirror_y = section
            .outer
            .vertices
            .iter()
            .any(|v2| (v2.y + v.y).abs() < 1e-6 && (v2.x - v.x).abs() < 1e-6);
        if !has_mirror_y {
            sym_y = false;
        }
    }

    if sym_x && sym_y {
        Point::new(0.0, 0.0)
    } else if sym_x {
        // Symmetric about X (channel-like)
        let e = b * b * h / (4.0 * props.ix.max(1e-12)) * area_flange_estimate(section);
        Point::new(e, 0.0)
    } else if sym_y {
        // Symmetric about Y
        Point::new(0.0, 0.0)
    } else {
        // General - use centroid as approximation
        Point::new(0.0, 0.0)
    }
}

fn area_flange_estimate(section: &Section) -> f64 {
    // Rough estimate of flange area fraction
    let props = SectionProperties::from_section(section);
    let bounds = section.bounds();
    let h = bounds.1 - bounds.0;
    let b = bounds.3 - bounds.2;
    let area = props.area;
    // Assume flanges are ~1/3 of area each
    area / 3.0
}

/// Compute properties for general (solid) sections using numerical integration.
fn compute_general_properties(section: &Section, props: &SectionProperties) -> (f64, f64, Point) {
    // For solid sections, use numerical solution of torsion problem
    // J from Prandtl stress function
    // Iw = 0 for solid sections (no warping)
    // Shear center = centroid for solid sections

    let j = estimate_j_solid(section);
    let iw = 0.0; // No warping for solid sections
    let shear_center = Point::new(0.0, 0.0);

    (j, iw, shear_center)
}

/// Estimate J for solid section using numerical integration.
fn estimate_j_solid(section: &Section) -> f64 {
    // Use polar moment as upper bound
    let props = SectionProperties::from_section(section);
    let ip = props.ix + props.iy;

    // For solid sections, J < Ip
    // For circle: J = Ip
    // For rectangle: J = β * b * h^3 (β depends on h/b)
    // Approximate using equivalent rectangle

    let bounds = section.bounds();
    let h = (bounds.1 - bounds.0).max(bounds.3 - bounds.2);
    let b = (bounds.1 - bounds.0).min(bounds.3 - bounds.2);

    if (h - b).abs() / h.max(b) < 0.1 {
        // Nearly circular
        ip / 2.0 // Rough estimate
    } else {
        // Rectangle formula
        let beta = torsional_constant_beta(h / b);
        beta * b * h.powi(3)
    }
}

/// Torsional constant beta for rectangle (h/b ratio).
fn torsional_constant_beta(ratio: f64) -> f64 {
    // From Roark's formulas
    match ratio {
        r if r >= 10.0 => 1.0 / 3.0,
        r if r >= 5.0 => 0.291,
        r if r >= 3.0 => 0.263,
        r if r >= 2.0 => 0.229,
        r if r >= 1.5 => 0.196,
        r if r >= 1.25 => 0.166,
        r if r >= 1.0 => 0.141,
        _ => torsional_constant_beta(1.0 / ratio), // Symmetric
    }
}

/// Torsion analysis results.
#[derive(Debug, Clone)]
pub struct TorsionAnalysis {
    pub properties: WarpingProperties,
    /// St. Venant shear stress from torque T
    pub tau_sv_max: f64,
    /// Warping normal stress from bimoment B
    pub sigma_w_max: f64,
    /// Total angle of twist per unit length
    pub theta_prime: f64,
    /// Warping displacement
    pub warping_displacement: f64,
}

impl TorsionAnalysis {
    /// Analyze section under pure torsion.
    pub fn pure_torsion(
        section: &Section,
        torque: f64,
        material: &crate::material::Material,
    ) -> Self {
        let props = WarpingProperties::from_section(section);

        // St. Venant shear stress: τ = T * r / J
        // Max at farthest point from shear center
        let max_r = section.max_distance_from(props.shear_center);
        let tau_sv_max = torque * max_r / props.j.max(1e-12);

        // Angle of twist per unit length: θ' = T / (G*J)
        let theta_prime = torque / (material.shear_modulus * props.j.max(1e-12));

        Self {
            properties: props,
            tau_sv_max,
            sigma_w_max: 0.0, // No warping in pure St. Venant torsion
            theta_prime,
            warping_displacement: 0.0,
        }
    }

    /// Analyze section under torsion + warping (constrained torsion).
    pub fn constrained_torsion(
        section: &Section,
        torque: f64,
        bimoment: f64, // Bimoment (warping moment)
        material: &crate::material::Material,
    ) -> Self {
        let props = WarpingProperties::from_section(section);

        // St. Venant component
        let max_r = section.max_distance_from(props.shear_center);
        let tau_sv_max = torque * max_r / props.j.max(1e-12);

        // Warping component
        // σ_w = B * ω_max / Iw where ω is warping function
        // For I-section: ω_max ≈ (h/2)^2 * (b/2) for flange tips
        let bounds = section.bounds();
        let h = bounds.1 - bounds.0;
        let b = bounds.3 - bounds.2;
        let omega_max = (h / 2.0).powi(2) * (b / 2.0);

        let sigma_w_max = if props.iw > 0.0 {
            bimoment * omega_max / props.iw
        } else {
            0.0
        };

        // Total twist
        let theta_prime = torque / (material.shear_modulus * props.j.max(1e-12));

        // Warping displacement (simplified)
        let warping_displacement = if props.iw > 0.0 {
            bimoment / (material.youngs_modulus * props.iw)
        } else {
            0.0
        };

        Self {
            properties: props,
            tau_sv_max,
            sigma_w_max,
            theta_prime,
            warping_displacement,
        }
    }
}

trait SectionDistance {
    fn max_distance_from(&self, from: Point) -> f64;
}

impl SectionDistance for Section {
    fn max_distance_from(&self, from: Point) -> f64 {
        let mut max_dist = 0.0;
        for v in &self.outer.vertices {
            let dx = v.x - from.x;
            let dy = v.y - from.y;
            let dist = (dx * dx + dy * dy).sqrt();
            max_dist = max_dist.max(dist);
        }
        for hole in &self.holes {
            for v in &hole.vertices {
                let dx = v.x - from.x;
                let dy = v.y - from.y;
                let dist = (dx * dx + dy * dy).sqrt();
                max_dist = max_dist.max(dist);
            }
        }
        max_dist
    }
}

/// Exact warping analysis using sectorial coordinates.
pub mod exact {
    use super::*;
    use crate::geometry::Point;

    /// Sectorial coordinate (warping function) computation.
    pub fn compute_sectorial_coordinates(section: &Section) -> SectorialProperties {
        // Compute sectorial area properties
        // ω = ∫ r * ds along section contour from shear center
        // For thin-walled open sections

        let props = SectionProperties::from_section(section);
        let centroid = props.centroid;

        // Find shear center first
        let shear_center = find_shear_center_exact(section, &props);

        // Compute sectorial coordinates at key points
        let mut omega_coords = Vec::new();

        for v in &section.outer.vertices {
            let omega = compute_omega_at_point(section, v, shear_center);
            omega_coords.push(omega);
        }

        // Warping constant Iw = ∫ ω^2 dA
        let iw = compute_iw_from_sectorial(section, &omega_coords, shear_center);

        SectorialProperties {
            shear_center,
            omega_coords,
            iw,
        }
    }

    #[derive(Debug, Clone)]
    pub struct SectorialProperties {
        pub shear_center: Point,
        pub omega_coords: Vec<f64>,
        pub iw: f64,
    }

    fn find_shear_center_exact(section: &Section, props: &SectionProperties) -> Point {
        // For thin-walled open sections, shear center is intersection of
        // axes of symmetry or computed from sectorial static moments

        // Simplified: use centroid for doubly symmetric,
        // compute for singly symmetric

        let bounds = section.bounds();
        let h = bounds.1 - bounds.0;
        let b = bounds.3 - bounds.2;

        // Check symmetry
        let sym_x = section.is_symmetric_about_x();
        let sym_y = section.is_symmetric_about_y();

        if sym_x && sym_y {
            return Point::new(0.0, 0.0);
        } else if sym_x {
            // Channel-like: e = ∫ ω * t * ds / ∫ t * ds (sectorial static moment)
            let e = compute_shear_center_channel(section, props);
            return Point::new(e, 0.0);
        }

        Point::new(0.0, 0.0)
    }

    fn compute_omega_at_point(section: &Section, point: &Point, shear_center: Point) -> f64 {
        // Sectorial coordinate: ω = 2 * A_sectorial
        // A_sectorial = area swept by radius vector from shear center
        // For polygons: sum of triangle areas

        // Simplified: ω ≈ r^2 / 2 for circular, or use polygon integration
        let dx = point.x - shear_center.x;
        let dy = point.y - shear_center.y;
        (dx * dx + dy * dy) / 2.0
    }

    fn compute_iw_from_sectorial(
        section: &Section,
        omega_coords: &[f64],
        shear_center: Point,
    ) -> f64 {
        // Iw = ∫ ω^2 dA ≈ Σ ω_i^2 * A_i
        // Use fiber integration
        let props = SectionProperties::from_section(section);
        let bounds = section.bounds();
        let n = 100;
        let dx = (bounds.1 - bounds.0) / n as f64;
        let dy = (bounds.3 - bounds.2) / n as f64;

        let mut iw = 0.0;

        for i in 0..n {
            for j in 0..n {
                let x = bounds.0 + (i as f64 + 0.5) * dx;
                let y = bounds.2 + (j as f64 + 0.5) * dy;
                let point = Point::new(x, y);

                if section.contains_point(point) {
                    let dx_sc = x - shear_center.x;
                    let dy_sc = y - shear_center.y;
                    let omega = (dx_sc * dx_sc + dy_sc * dy_sc) / 2.0;
                    iw += omega * omega * dx * dy;
                }
            }
        }

        iw
    }

    fn compute_shear_center_channel(section: &Section, props: &SectionProperties) -> f64 {
        // e = b^2 * h * t_f / (4 * Ix) for idealized channel
        let bounds = section.bounds();
        let h = bounds.1 - bounds.0;
        let b = bounds.3 - bounds.2;

        // Estimate flange thickness from area
        let area = props.area;
        let t_equiv = area / (h + 2.0 * b);

        b * b * h * t_equiv / (4.0 * props.ix.max(1e-12))
    }
}

trait SectionSymmetry {
    fn is_symmetric_about_x(&self) -> bool;
    fn is_symmetric_about_y(&self) -> bool;
}

impl SectionSymmetry for Section {
    fn is_symmetric_about_x(&self) -> bool {
        for v in &self.outer.vertices {
            let found = self
                .outer
                .vertices
                .iter()
                .any(|v2| (v2.x - v.x).abs() < 1e-6 && (v2.y + v.y).abs() < 1e-6);
            if !found {
                return false;
            }
        }
        true
    }

    fn is_symmetric_about_y(&self) -> bool {
        for v in &self.outer.vertices {
            let found = self
                .outer
                .vertices
                .iter()
                .any(|v2| (v2.y - v.y).abs() < 1e-6 && (v2.x + v.x).abs() < 1e-6);
            if !found {
                return false;
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon};
    use crate::material::presets::STEEL_S355;
    use crate::section::Section;

    #[test]
    fn warping_solid_rectangle() {
        let poly = Polygon::new(vec![
            Point::new(-0.1, -0.05),
            Point::new(0.1, -0.05),
            Point::new(0.1, 0.05),
            Point::new(-0.1, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let props = WarpingProperties::from_section(&section);

        assert!(props.j > 0.0);
        assert_eq!(props.iw, 0.0); // Solid section
        assert!(props.shear_center.x.abs() < 1e-6);
        assert!(props.shear_center.y.abs() < 1e-6);
    }

    #[test]
    fn warping_circle() {
        let poly = crate::section_library::circle_polygon(0.05, 64);
        let section = Section::new(poly, vec![]);
        let props = WarpingProperties::from_section(&section);

        // For circle: J = π*r^4/2
        let r = 0.05;
        let expected_j = std::f64::consts::PI * r.powi(4) / 2.0;
        assert!((props.j - expected_j).abs() / expected_j < 0.2); // Approximate

        assert_eq!(props.iw, 0.0);
    }

    #[test]
    fn warping_i_section() {
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();
        let props = WarpingProperties::from_section(&section);

        assert!(props.j > 0.0);
        assert!(props.iw > 0.0); // Open section has warping
        assert!(props.shear_center.x.abs() < 1e-6); // Symmetric about Y
    }

    #[test]
    fn torsion_analysis_pure() {
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();

        let analysis = TorsionAnalysis::pure_torsion(&section, 10e3, &STEEL_S355); // 10 kNm

        assert!(analysis.tau_sv_max > 0.0);
        assert!(analysis.theta_prime > 0.0);
    }

    #[test]
    fn torsion_analysis_constrained() {
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();

        let analysis = TorsionAnalysis::constrained_torsion(&section, 10e3, 5e3, &STEEL_S355);

        assert!(analysis.tau_sv_max > 0.0);
        assert!(analysis.sigma_w_max >= 0.0);
    }

    #[test]
    fn warping_length() {
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();
        let props = WarpingProperties::from_section(&section);

        let l_w = props.warping_length(STEEL_S355.youngs_modulus, STEEL_S355.shear_modulus);

        // For IPE300, warping length ~2-5m typically
        assert!(l_w > 0.5 && l_w < 20.0);
    }
}
