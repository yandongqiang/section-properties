//! Primitive section shapes (rectangles, circles, triangles, etc.)

use crate::geometry::{Point, Polygon};
use crate::material::Material;
use crate::section::Section;
use crate::section_library::{
    ParametricSection, circle_polygon, hollow_circle_polygon, rectangle_polygon,
    rounded_rectangle_polygon,
};
use std::f64::consts::PI;

/// Solid rectangular section.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RectangularSection {
    pub width: f64,  // b (x-direction)
    pub height: f64, // h (y-direction)
}

impl RectangularSection {
    pub fn new(width: f64, height: f64) -> Self {
        assert!(width > 0.0 && height > 0.0, "Dimensions must be positive");
        Self { width, height }
    }

    pub fn square(side: f64) -> Self {
        Self::new(side, side)
    }
}

impl ParametricSection for RectangularSection {
    fn build(&self) -> Section {
        Section::new(rectangle_polygon(self.width, self.height), Vec::new())
    }

    fn designation(&self) -> String {
        format!(
            "RECT {:.0}x{:.0}",
            self.width * 1000.0,
            self.height * 1000.0
        )
    }
}

/// Solid circular section.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CircularSection {
    pub radius: f64,
    pub n_vertices: usize,
}

impl CircularSection {
    pub fn new(radius: f64) -> Self {
        Self::with_vertices(radius, 64)
    }

    pub fn with_vertices(radius: f64, n_vertices: usize) -> Self {
        assert!(radius > 0.0, "Radius must be positive");
        assert!(n_vertices >= 8, "At least 8 vertices");
        Self { radius, n_vertices }
    }

    pub fn diameter(d: f64) -> Self {
        Self::new(d / 2.0)
    }
}

impl ParametricSection for CircularSection {
    fn build(&self) -> Section {
        Section::new(circle_polygon(self.radius, self.n_vertices), Vec::new())
    }

    fn designation(&self) -> String {
        format!("CIRC Ø{:.0}", self.radius * 2000.0)
    }
}

/// Circular hollow section (CHS / tube).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CircularHollowSection {
    pub outer_radius: f64,
    pub inner_radius: f64,
    pub n_vertices: usize,
}

impl CircularHollowSection {
    pub fn new(outer_radius: f64, inner_radius: f64) -> Self {
        Self::with_vertices(outer_radius, inner_radius, 64)
    }

    pub fn with_vertices(outer_radius: f64, inner_radius: f64, n_vertices: usize) -> Self {
        assert!(
            outer_radius > inner_radius,
            "Outer radius must exceed inner radius"
        );
        assert!(inner_radius > 0.0, "Inner radius must be positive");
        assert!(n_vertices >= 8, "At least 8 vertices");
        Self {
            outer_radius,
            inner_radius,
            n_vertices,
        }
    }

    pub fn from_dimensions(outer_diameter: f64, wall_thickness: f64) -> Self {
        let ro = outer_diameter / 2.0;
        let ri = ro - wall_thickness;
        Self::new(ro, ri)
    }
}

impl ParametricSection for CircularHollowSection {
    fn build(&self) -> Section {
        let (outer, inner) =
            hollow_circle_polygon(self.outer_radius, self.inner_radius, self.n_vertices);
        Section::new(outer, vec![inner])
    }

    fn designation(&self) -> String {
        format!(
            "CHS Ø{:.0}x{:.1}",
            self.outer_radius * 2000.0,
            (self.outer_radius - self.inner_radius) * 1000.0
        )
    }
}

/// Elliptical section.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EllipticalSection {
    pub semi_major: f64, // a (x-direction)
    pub semi_minor: f64, // b (y-direction)
    pub n_vertices: usize,
}

impl EllipticalSection {
    pub fn new(semi_major: f64, semi_minor: f64) -> Self {
        Self::with_vertices(semi_major, semi_minor, 64)
    }

    pub fn with_vertices(semi_major: f64, semi_minor: f64, n_vertices: usize) -> Self {
        assert!(
            semi_major > 0.0 && semi_minor > 0.0,
            "Axes must be positive"
        );
        assert!(n_vertices >= 8, "At least 8 vertices");
        Self {
            semi_major,
            semi_minor,
            n_vertices,
        }
    }
}

impl ParametricSection for EllipticalSection {
    fn build(&self) -> Section {
        let mut vertices = Vec::with_capacity(self.n_vertices);
        for i in 0..self.n_vertices {
            let theta = 2.0 * PI * i as f64 / self.n_vertices as f64;
            vertices.push(Point::new(
                self.semi_major * theta.cos(),
                self.semi_minor * theta.sin(),
            ));
        }
        Section::new(Polygon::new(vertices), Vec::new())
    }

    fn designation(&self) -> String {
        format!(
            "ELLIPSE {:.0}x{:.0}",
            self.semi_major * 2000.0,
            self.semi_minor * 2000.0
        )
    }
}

/// Solid triangular section.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TriangularSection {
    pub base: f64,
    pub height: f64,
    /// Apex position along base (0.0 = left, 0.5 = centered, 1.0 = right)
    pub apex_ratio: f64,
}

impl TriangularSection {
    pub fn new(base: f64, height: f64) -> Self {
        Self::with_apex(base, height, 0.5)
    }

    pub fn with_apex(base: f64, height: f64, apex_ratio: f64) -> Self {
        assert!(base > 0.0 && height > 0.0, "Dimensions must be positive");
        assert!(
            apex_ratio >= 0.0 && apex_ratio <= 1.0,
            "Apex ratio must be in [0,1]"
        );
        Self {
            base,
            height,
            apex_ratio,
        }
    }

    pub fn equilateral(side: f64) -> Self {
        let h = side * 3.0_f64.sqrt() / 2.0;
        Self::new(side, h)
    }

    pub fn right_angle(base: f64, height: f64) -> Self {
        Self::with_apex(base, height, 0.0)
    }
}

impl ParametricSection for TriangularSection {
    fn build(&self) -> Section {
        let apex_x = (self.apex_ratio - 0.5) * self.base;
        let poly = Polygon::new(vec![
            Point::new(-self.base / 2.0, -self.height / 3.0),
            Point::new(self.base / 2.0, -self.height / 3.0),
            Point::new(apex_x, 2.0 * self.height / 3.0),
        ]);
        Section::new(poly, Vec::new())
    }

    fn designation(&self) -> String {
        format!("TRI {:.0}x{:.0}", self.base * 1000.0, self.height * 1000.0)
    }
}

/// Rounded rectangle (rectangle with filleted corners).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RoundedRectangleSection {
    pub width: f64,
    pub height: f64,
    pub radius: f64,
    pub n_per_corner: usize,
}

impl RoundedRectangleSection {
    pub fn new(width: f64, height: f64, radius: f64) -> Self {
        Self::with_vertices(width, height, radius, 8)
    }

    pub fn with_vertices(width: f64, height: f64, radius: f64, n_per_corner: usize) -> Self {
        assert!(width > 0.0 && height > 0.0, "Dimensions must be positive");
        assert!(radius >= 0.0, "Radius must be non-negative");
        assert!(
            radius <= width / 2.0 && radius <= height / 2.0,
            "Radius too large for dimensions"
        );
        assert!(n_per_corner >= 1, "At least 1 vertex per corner");
        Self {
            width,
            height,
            radius,
            n_per_corner,
        }
    }
}

impl ParametricSection for RoundedRectangleSection {
    fn build(&self) -> Section {
        Section::new(
            rounded_rectangle_polygon(self.width, self.height, self.radius, self.n_per_corner),
            Vec::new(),
        )
    }

    fn designation(&self) -> String {
        format!(
            "RRECT {:.0}x{:.0} R{:.0}",
            self.width * 1000.0,
            self.height * 1000.0,
            self.radius * 1000.0
        )
    }
}

/// Regular polygon section (n-gon).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RegularPolygonSection {
    pub circumradius: f64, // Radius of circumscribed circle
    pub n_sides: usize,
}

impl RegularPolygonSection {
    pub fn new(circumradius: f64, n_sides: usize) -> Self {
        assert!(circumradius > 0.0, "Radius must be positive");
        assert!(n_sides >= 3, "At least 3 sides");
        Self {
            circumradius,
            n_sides,
        }
    }

    pub fn from_inscribed_radius(inradius: f64, n_sides: usize) -> Self {
        let circumradius = inradius / (PI / n_sides as f64).cos();
        Self::new(circumradius, n_sides)
    }
}

impl ParametricSection for RegularPolygonSection {
    fn build(&self) -> Section {
        let mut vertices = Vec::with_capacity(self.n_sides);
        for i in 0..self.n_sides {
            // Rotate so one vertex is at top (like a hexagon flat on top)
            let theta = -PI / 2.0 + 2.0 * PI * i as f64 / self.n_sides as f64;
            vertices.push(Point::new(
                self.circumradius * theta.cos(),
                self.circumradius * theta.sin(),
            ));
        }
        Section::new(Polygon::new(vertices), Vec::new())
    }

    fn designation(&self) -> String {
        format!("{}-GON R{:.0}", self.n_sides, self.circumradius * 1000.0)
    }
}

/// Cruciform (cross) section.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CruciformSection {
    pub width: f64,
    pub height: f64,
    pub flange_width: f64,
    pub flange_thickness: f64,
    pub web_thickness: f64,
}

impl CruciformSection {
    pub fn new(
        width: f64,
        height: f64,
        flange_width: f64,
        flange_thickness: f64,
        web_thickness: f64,
    ) -> Self {
        assert!(
            width > 0.0 && height > 0.0,
            "Overall dimensions must be positive"
        );
        assert!(
            flange_width > 0.0 && flange_thickness > 0.0,
            "Flange dims must be positive"
        );
        assert!(web_thickness > 0.0, "Web thickness must be positive");
        assert!(
            flange_width <= width,
            "Flange width cannot exceed overall width"
        );
        assert!(
            flange_thickness <= height / 2.0,
            "Flange thickness cannot exceed half height"
        );
        assert!(
            web_thickness <= width,
            "Web thickness cannot exceed overall width"
        );
        Self {
            width,
            height,
            flange_width,
            flange_thickness,
            web_thickness,
        }
    }

    pub fn symmetric(size: f64, flange: f64, web: f64) -> Self {
        Self::new(size, size, flange, flange, web)
    }
}

impl ParametricSection for CruciformSection {
    fn build(&self) -> Section {
        let hw = self.width / 2.0;
        let hh = self.height / 2.0;
        let fw = self.flange_width / 2.0;
        let ft = self.flange_thickness;
        let wt = self.web_thickness / 2.0;

        // Build as a polygon with cutouts (or as union of rectangles)
        // We'll build the cross shape directly
        let mut vertices = Vec::new();

        // Top flange - left to right
        vertices.push(Point::new(-fw, hh - ft));
        vertices.push(Point::new(fw, hh - ft));
        vertices.push(Point::new(fw, hh));
        vertices.push(Point::new(-fw, hh));

        // Right flange - top to bottom
        vertices.push(Point::new(hw - wt, hh));
        vertices.push(Point::new(hw, hh));
        vertices.push(Point::new(hw, -hh));
        vertices.push(Point::new(hw - wt, -hh));

        // Bottom flange - right to left
        vertices.push(Point::new(fw, -hh + ft));
        vertices.push(Point::new(-fw, -hh + ft));
        vertices.push(Point::new(-fw, -hh));
        vertices.push(Point::new(fw, -hh));

        // Left flange - bottom to top
        vertices.push(Point::new(-hw + wt, -hh));
        vertices.push(Point::new(-hw, -hh));
        vertices.push(Point::new(-hw, hh));
        vertices.push(Point::new(-hw + wt, hh));

        Section::new(Polygon::new(vertices), Vec::new())
    }

    fn designation(&self) -> String {
        format!(
            "CRUCIFORM {:.0}x{:.0}",
            self.width * 1000.0,
            self.height * 1000.0
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::presets::STEEL_S355;

    #[test]
    fn rectangular_section() {
        let rect = RectangularSection::new(0.2, 0.1);
        let sec = rect.build();
        assert!((sec.area() - 0.02).abs() < 1e-10);
    }

    #[test]
    fn circular_section() {
        let circ = CircularSection::new(0.1);
        let sec = circ.build();
        assert!((sec.area() - PI * 0.01).abs() < 1e-4);
    }

    #[test]
    fn chs_section() {
        let chs = CircularHollowSection::from_dimensions(0.2191, 0.0082); // 219.1x8.2
        let sec = chs.build();
        let expected_area = PI * (0.10955_f64.powi(2) - 0.10135_f64.powi(2));
        assert!((sec.area() - expected_area).abs() < 1e-4);
    }

    #[test]
    fn triangular_section() {
        let tri = TriangularSection::new(0.1, 0.1);
        let sec = tri.build();
        assert!((sec.area() - 0.005).abs() < 1e-10);
    }

    #[test]
    fn equilateral_triangle() {
        let tri = TriangularSection::equilateral(0.1);
        let sec = tri.build();
        let expected = 0.1_f64.powi(2) * 3.0_f64.sqrt() / 4.0;
        assert!((sec.area() - expected).abs() < 1e-10);
    }

    #[test]
    fn rounded_rectangle() {
        let rrect = RoundedRectangleSection::new(0.2, 0.1, 0.01);
        let sec = rrect.build();
        let expected = 0.02 - (4.0 - PI) * 0.0001;
        assert!((sec.area() - expected).abs() < 1e-6);
    }

    #[test]
    fn regular_polygon_hexagon() {
        let hex = RegularPolygonSection::new(0.1, 6);
        let sec = hex.build();
        // Area of regular hexagon: 3*sqrt(3)/2 * R^2
        let expected = 3.0 * 3.0_f64.sqrt() / 2.0 * 0.01;
        assert!((sec.area() - expected).abs() < 1e-6);
    }

    #[test]
    fn cruciform_section() {
        let cross = CruciformSection::new(0.2, 0.2, 0.15, 0.02, 0.01);
        let sec = cross.build();
        // Area = 2*flange*flange_t + web*web_t - overlap
        let expected = 2.0 * 0.15 * 0.02 + 0.2 * 0.01 - 0.01 * 0.02;
        assert!((sec.area() - expected).abs() < 1e-6);
    }
}
