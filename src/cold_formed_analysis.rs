//! Cold-formed steel design per EN 1993-1-3, AS 4600, AISI S100.
//!
//! Implements effective width method for slender elements,
//! distortional buckling, and combined local-distortional interaction.

pub mod direct_strength;
pub mod distortional;
pub mod effective_width;

pub use direct_strength::{
    DsmParams, DsmStrengths, dsm_analysis, dsm_nominal_capacity_distortional,
    dsm_nominal_capacity_global, dsm_nominal_capacity_local,
};
pub use distortional::{DistortionalBuckling, DistortionalParams, distortional_buckling_stress};
pub use effective_width::{
    BucklingCurve, EffectiveSectionProperties, EffectiveWidth, EffectiveWidthParams, ElementType,
    effective_width_corner, effective_width_flat, effective_width_stiffened,
    reduced_section_properties,
};

use crate::geometry::{Point, Polygon};
use crate::material::Material;
use crate::section::Section;
use crate::section_properties::SectionProperties;

/// Cold-formed steel section analysis.
#[derive(Debug, Clone)]
pub struct ColdFormedSection {
    pub section: Section,
    pub material: Material,
    /// Element properties for effective width calculation
    pub elements: Vec<ColdFormedElement>,
}

#[derive(Debug, Clone)]
pub struct ColdFormedElement {
    pub element_type: ElementType,
    pub width: f64, // Flat width (excluding corners)
    pub thickness: f64,
    pub yield_strength: f64,
    pub edge_support: EdgeSupport,
    pub stiffener: Option<Stiffener>,
    pub corner_radius: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EdgeSupport {
    /// Supported on both edges (internal element, e.g., web)
    DoubleSupported,
    /// Supported on one edge (outstand, e.g., flange)
    Outstand,
    /// Stiffened edge
    Stiffened,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ElementType {
    Flat,
    Corner,
    Stiffened,
    Lip,
    Web,
    Flange,
}

#[derive(Debug, Clone)]
pub struct Stiffener {
    pub type_: StiffenerType,
    pub width: f64,
    pub thickness: f64,
    pub corner_radius: f64,
    pub lip_length: Option<f64>, // For edge stiffener with lip
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StiffenerType {
    /// Intermediate stiffener
    Intermediate,
    /// Edge stiffener with lip
    EdgeWithLip,
    /// Edge stiffener without lip
    EdgeWithoutLip,
}

impl ColdFormedSection {
    pub fn new(section: Section, material: Material) -> Self {
        Self {
            section,
            material,
            elements: Vec::new(),
        }
    }

    pub fn add_element(mut self, element: ColdFormedElement) -> Self {
        self.elements.push(element);
        self
    }

    /// Compute effective section properties at a given stress level.
    pub fn effective_properties(&self, f_c: f64) -> EffectiveSectionProperties {
        reduced_section_properties(self, f_c)
    }

    /// Compute effective properties for pure compression.
    pub fn effective_properties_compression(&self) -> EffectiveSectionProperties {
        self.effective_properties(self.material.yield_strength)
    }

    /// Compute effective properties for bending (compression flange).
    pub fn effective_properties_bending(&self, m_ed: f64) -> EffectiveSectionProperties {
        // Estimate compression stress from moment
        let props = SectionProperties::from_section(&self.section);
        let max_fiber = props.max_fiber_distance_y();
        let f_c = m_ed / props.ix * max_fiber;
        self.effective_properties(f_c.min(self.material.yield_strength))
    }
}

#[derive(Debug, Clone)]
pub struct EffectiveSectionProperties {
    pub area_eff: f64,
    pub centroid_eff: Point,
    pub ix_eff: f64,
    pub iy_eff: f64,
    pub ixy_eff: f64,
    pub element_reductions: Vec<ElementReduction>,
}

#[derive(Debug, Clone)]
pub struct ElementReduction {
    pub element_index: usize,
    pub original_width: f64,
    pub effective_width: f64,
    pub reduction_factor: f64,
    pub buckling_curve: BucklingCurve,
}

impl EffectiveSectionProperties {
    /// Effective section modulus (compression).
    pub fn z_eff_compression(&self, section: &Section, axis: crate::plastic::PlasticAxis) -> f64 {
        let bounds = section.bounds();
        let max_dist = match axis {
            crate::plastic::PlasticAxis::X => (bounds.1 - self.centroid_eff.y).abs(),
            crate::plastic::PlasticAxis::Y => (bounds.3 - self.centroid_eff.x).abs(),
        };
        if max_dist > 1e-12 {
            match axis {
                crate::plastic::PlasticAxis::X => self.ix_eff / max_dist,
                crate::plastic::PlasticAxis::Y => self.iy_eff / max_dist,
            }
        } else {
            0.0
        }
    }

    /// Effective moment capacity.
    pub fn moment_capacity(
        &self,
        section: &Section,
        material: &Material,
        axis: crate::plastic::PlasticAxis,
    ) -> f64 {
        self.z_eff_compression(section, axis) * material.yield_strength
    }
}

trait SectionBoundsExt {
    fn bounds(&self) -> (f64, f64, f64, f64);
}

impl SectionBoundsExt for Section {
    fn bounds(&self) -> (f64, f64, f64, f64) {
        let mut min_x = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        for v in &self.outer.vertices {
            min_x = min_x.min(v.x);
            max_x = max_x.max(v.x);
            min_y = min_y.min(v.y);
            max_y = max_y.max(v.y);
        }
        for hole in &self.holes {
            for v in &hole.vertices {
                min_x = min_x.min(v.x);
                max_x = max_x.max(v.x);
                min_y = min_y.min(v.y);
                max_y = max_y.max(v.y);
            }
        }
        (min_x, max_x, min_y, max_y)
    }
}

/// Parameters for effective width calculation.
#[derive(Debug, Clone)]
pub struct EffectiveWidthParams {
    /// Material yield strength
    pub fy: f64,
    /// Young's modulus
    pub e: f64,
    /// Poisson's ratio
    pub nu: f64,
    /// Safety factor (gamma_m0)
    pub gamma_m0: f64,
    /// Whether to use EN 1993-1-3 (true) or AISI S100 (false)
    pub use_eurocode: bool,
}

impl Default for EffectiveWidthParams {
    fn default() -> Self {
        Self {
            fy: 350e6,
            e: 200e9,
            nu: 0.3,
            gamma_m0: 1.0,
            use_eurocode: true,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BucklingCurve {
    /// Internal element (flat)
    Internal,
    /// Outstand element
    Outstand,
    /// Stiffened element
    Stiffened,
    /// Corner element
    Corner,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::presets::STEEL_S355;

    #[test]
    fn cold_formed_section_creation() {
        let poly = crate::section_library::rectangle_polygon(0.2, 0.1);
        let section = Section::new(poly, vec![]);
        let cfs = ColdFormedSection::new(section, STEEL_S355);

        assert!(cfs.elements.is_empty());
    }

    #[test]
    fn element_types() {
        assert_eq!(ElementType::Flat as u8, 0);
    }
}
