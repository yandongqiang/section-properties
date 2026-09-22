//! Pre-defined section library (mirrors Python sectionproperties.pre.library)
//!
//! Provides common hot-rolled and cold-formed steel sections, concrete sections,
//! and primitive shapes. All dimensions in meters (SI base units).

pub mod cold_formed;
pub mod composite;
pub mod concrete;
pub mod primitive;
pub mod steel;
pub mod timber;

#[allow(ambiguous_glob_reexports)]
pub use cold_formed::*;
#[allow(ambiguous_glob_reexports)]
pub use composite::*;
#[allow(ambiguous_glob_reexports)]
pub use concrete::*;
#[allow(ambiguous_glob_reexports)]
pub use primitive::*;
#[allow(ambiguous_glob_reexports)]
pub use steel::*;
#[allow(ambiguous_glob_reexports)]
pub use timber::*;

use crate::geometry::{Point, Polygon};
use crate::material::Material;
use crate::section::Section;

/// Trait for all parametric sections that can build their geometry.
pub trait ParametricSection: core::fmt::Debug {
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

/// Generate points along an arc (mirrors Python `draw_radius`).
///
/// Centre at `pt`, radius `r`, starting angle `theta` (radians), `n` points.
/// `ccw` = counter-clockwise; `phi` = arc extent (radians, default π/2).
pub fn draw_radius(pt: Point, r: f64, theta: f64, n: usize, ccw: bool, phi: f64) -> Vec<Point> {
    if r == 0.0 {
        return vec![pt];
    }
    let mult = if ccw { 1.0 } else { -1.0 };
    let mut points = Vec::with_capacity(n);
    for i in 0..n {
        let t = theta + mult * i as f64 / (n.max(1) - 1).max(1) as f64 * phi;
        points.push(Point::new(pt.x + r * t.cos(), pt.y + r * t.sin()));
    }
    points
}

/// Rotate a point counter-clockwise about the origin by `angle` (radians).
pub fn rotate_point(point: Point, angle: f64) -> Point {
    let c = angle.cos();
    let s = angle.sin();
    Point::new(c * point.x - s * point.y, s * point.x + c * point.y)
}

#[cfg(test)]
mod tests {
    use super::*;

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
        let poly = rounded_rectangle_polygon(0.2, 0.1, 0.01, 16);
        let expected = 0.02 - (4.0 - std::f64::consts::PI) * 0.01_f64.powi(2);
        assert!((poly.area() - expected).abs() < 1e-6);
    }
}
