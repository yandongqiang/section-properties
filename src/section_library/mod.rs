//! Pre-defined section library (mirrors Python sectionproperties.pre.library)
//!
//! Provides common hot-rolled and cold-formed steel sections, concrete sections,
//! and primitive shapes. All dimensions in meters (SI base units).

pub mod composite;
pub mod concrete;
pub mod primitive;
pub mod steel;
pub mod timber;

pub use composite::*;
pub use concrete::*;
pub use primitive::*;
pub use steel::*;
pub use timber::*;

use crate::geometry::{Point, Polygon};
use crate::material::{Material, MaterialGroup};
use crate::section::Section;

/// Trait for all parametric sections that can build their geometry.
pub trait ParametricSection {
    /// Build the section geometry (outer boundary + holes).
    fn build(&self) -> Section;

    /// Build the section as a composite section with material groups.
    fn build_composite(&self, material: Material) -> CompositeSection {
        CompositeSection::single_material(self.build(), material)
    }

    /// Get the section name/designation.
    fn designation(&self) -> String;

    /// Get approximate mass per unit length [kg/m].
    fn mass_per_length(&self, material: &Material) -> f64 {
        let sec = self.build();
        sec.area() * material.density
    }
}

/// Composite section with multiple materials (transformed section method).
#[derive(Debug, Clone)]
pub struct CompositeSection {
    pub outer: Polygon,
    pub holes: Vec<Polygon>,
    pub material_groups: Vec<MaterialGroup>,
}

impl CompositeSection {
    /// Create a composite section from a single-material Section.
    pub fn single_material(section: Section, material: Material) -> Self {
        Self {
            outer: section.outer,
            holes: section.holes,
            material_groups: vec![MaterialGroup::new(material, vec![0])],
        }
    }

    /// Create from outer polygon, holes, and material groups.
    pub fn new(outer: Polygon, holes: Vec<Polygon>, material_groups: Vec<MaterialGroup>) -> Self {
        Self {
            outer,
            holes,
            material_groups,
        }
    }

    /// Get the reference material (first group's material).
    pub fn reference_material(&self) -> &Material {
        &self.material_groups[0].material
    }

    /// Get modular ratios for all groups relative to reference material.
    pub fn modular_ratios(&self) -> Vec<f64> {
        let ref_mat = self.reference_material();
        self.material_groups
            .iter()
            .map(|g| g.material.modular_ratio(ref_mat))
            .collect()
    }

    /// Compute transformed section properties using the reference material.
    pub fn transformed_properties(&self) -> crate::section_properties::SectionProperties {
        // TODO: Implement full transformed section analysis
        // For now, fall back to reference material homogeneous properties
        let section = Section::new(self.outer.clone(), self.holes.clone());
        crate::section_properties::SectionProperties::from_section(&section)
    }
}

/// Helper: create a rectangle centered at origin.
pub fn rectangle_polygon(width: f64, height: f64) -> Polygon {
    let hw = width / 2.0;
    let hh = height / 2.0;
    Polygon::new(vec![
        Point::new(-hw, -hh),
        Point::new(hw, -hh),
        Point::new(hw, hh),
        Point::new(-hw, hh),
    ])
}

/// Helper: create a circle approximated by n vertices, centered at origin.
pub fn circle_polygon(radius: f64, n: usize) -> Polygon {
    let mut vertices = Vec::with_capacity(n);
    for i in 0..n {
        let theta = 2.0 * std::f64::consts::PI * i as f64 / n as f64;
        vertices.push(Point::new(radius * theta.cos(), radius * theta.sin()));
    }
    Polygon::new(vertices)
}

/// Helper: create a hollow circle (tube) approximated by n vertices.
pub fn hollow_circle_polygon(outer_radius: f64, inner_radius: f64, n: usize) -> (Polygon, Polygon) {
    let outer = circle_polygon(outer_radius, n);
    let mut inner_vertices = Vec::with_capacity(n);
    // CW for hole
    for i in (0..n).rev() {
        let theta = 2.0 * std::f64::consts::PI * i as f64 / n as f64;
        inner_vertices.push(Point::new(
            inner_radius * theta.cos(),
            inner_radius * theta.sin(),
        ));
    }
    let inner = Polygon::new(inner_vertices);
    (outer, inner)
}

/// Helper: round rectangle corners with fillet radius.
pub fn rounded_rectangle_polygon(
    width: f64,
    height: f64,
    radius: f64,
    n_per_corner: usize,
) -> Polygon {
    let hw = width / 2.0;
    let hh = height / 2.0;
    let r = radius.min(hw).min(hh);

    let mut vertices = Vec::new();

    // Start from bottom-left corner, going CCW
    // Bottom edge (left to right)
    vertices.push(Point::new(-hw + r, -hh));
    vertices.push(Point::new(hw - r, -hh));

    // Bottom-right corner (quarter circle, CW from -π/2 to 0 -> but we want CCW overall)
    for i in 0..=n_per_corner {
        let theta = -std::f64::consts::FRAC_PI_2
            + std::f64::consts::FRAC_PI_2 * i as f64 / n_per_corner as f64;
        vertices.push(Point::new(
            hw - r + r * theta.cos(),
            -hh + r + r * theta.sin(),
        ));
    }

    // Right edge (bottom to top)
    vertices.push(Point::new(hw, -hh + r));
    vertices.push(Point::new(hw, hh - r));

    // Top-right corner
    for i in 0..=n_per_corner {
        let theta = 0.0 + std::f64::consts::FRAC_PI_2 * i as f64 / n_per_corner as f64;
        vertices.push(Point::new(
            hw - r + r * theta.cos(),
            hh - r + r * theta.sin(),
        ));
    }

    // Top edge (right to left)
    vertices.push(Point::new(hw - r, hh));
    vertices.push(Point::new(-hw + r, hh));

    // Top-left corner
    for i in 0..=n_per_corner {
        let theta = std::f64::consts::FRAC_PI_2
            + std::f64::consts::FRAC_PI_2 * i as f64 / n_per_corner as f64;
        vertices.push(Point::new(
            -hw + r + r * theta.cos(),
            hh - r + r * theta.sin(),
        ));
    }

    // Left edge (top to bottom)
    vertices.push(Point::new(-hw, hh - r));
    vertices.push(Point::new(-hw, -hh + r));

    // Bottom-left corner
    for i in 0..=n_per_corner {
        let theta =
            std::f64::consts::PI + std::f64::consts::FRAC_PI_2 * i as f64 / n_per_corner as f64;
        vertices.push(Point::new(
            -hw + r + r * theta.cos(),
            -hh + r + r * theta.sin(),
        ));
    }

    Polygon::new(vertices)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::presets::STEEL_S355;

    #[test]
    fn rectangle_polygon_area() {
        let poly = rectangle_polygon(0.2, 0.1);
        assert!((poly.area() - 0.02).abs() < 1e-10);
    }

    #[test]
    fn circle_polygon_area() {
        let poly = circle_polygon(0.1, 100);
        assert!((poly.area() - std::f64::consts::PI * 0.01).abs() < 1e-4);
    }

    #[test]
    fn hollow_circle_area() {
        let (outer, inner) = hollow_circle_polygon(0.1, 0.05, 100);
        let outer_area = outer.area();
        let inner_area = inner.area();
        let expected = std::f64::consts::PI * (0.1_f64.powi(2) - 0.05_f64.powi(2));
        assert!((outer_area - inner_area - expected).abs() < 1e-4);
    }

    #[test]
    fn rounded_rectangle_area() {
        let poly = rounded_rectangle_polygon(0.2, 0.1, 0.01, 8);
        let expected = 0.02 - (4.0 - std::f64::consts::PI) * 0.01_f64.powi(2);
        assert!((poly.area() - expected).abs() < 1e-6);
    }
}

