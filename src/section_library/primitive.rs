//! Primitive section shapes (rectangles, circles, triangles, etc.)

use crate::geometry::{Point, Polygon};
use crate::section::Section;
use crate::section_library::{
    ParametricSection, circle_polygon, draw_radius, hollow_circle_polygon, rotate_point,
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
        // Match Python sectionproperties: bottom-left corner at the origin,
        // extending to (b, d).
        let b = self.width;
        let d = self.height;
        Section::new(
            Polygon::new(vec![
                Point::new(0.0, 0.0),
                Point::new(b, 0.0),
                Point::new(b, d),
                Point::new(0.0, d),
            ]),
            Vec::new(),
        )
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
        Self::with_vertices(width, height, radius, 16)
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
        let fw = self.flange_width / 2.0;
        let ft = self.flange_thickness;
        let wt = self.web_thickness / 2.0;
        let hh = self.height / 2.0;

        let vertices = vec![
            Point::new(-wt, hh),
            Point::new(wt, hh),
            Point::new(wt, ft),
            Point::new(fw, ft),
            Point::new(fw, -ft),
            Point::new(wt, -ft),
            Point::new(wt, -hh),
            Point::new(-wt, -hh),
            Point::new(-wt, -ft),
            Point::new(-fw, -ft),
            Point::new(-fw, ft),
            Point::new(-wt, ft),
        ];

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

/// Elliptical hollow section (EHS) centered at origin.
///
/// Mirrors Python `elliptical_hollow_section(d_x, d_y, t, n)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EllipticalHollowSection {
    pub d_x: f64,
    pub d_y: f64,
    pub t: f64,
    pub n: usize,
}

impl EllipticalHollowSection {
    pub fn new(d_x: f64, d_y: f64, t: f64, n: usize) -> Self {
        assert!(
            d_x > 0.0 && d_y > 0.0 && t > 0.0,
            "Dimensions must be positive"
        );
        assert!(2.0 * t < d_x && 2.0 * t < d_y, "Thickness too large");
        assert!(n >= 3, "Need at least 3 points");
        Self { d_x, d_y, t, n }
    }
}

impl ParametricSection for EllipticalHollowSection {
    fn build(&self) -> Section {
        let n = self.n;
        let mut outer_pts = Vec::with_capacity(n);
        let mut inner_pts = Vec::with_capacity(n);
        for i in 0..n {
            let theta = 2.0 * PI * i as f64 / n as f64;
            outer_pts.push(Point::new(
                0.5 * self.d_x * theta.cos(),
                0.5 * self.d_y * theta.sin(),
            ));
            inner_pts.push(Point::new(
                (0.5 * self.d_x - self.t) * theta.cos(),
                (0.5 * self.d_y - self.t) * theta.sin(),
            ));
        }
        // Inner polygon CW for hole
        inner_pts.reverse();
        Section::new(Polygon::new(outer_pts), vec![Polygon::new(inner_pts)])
    }

    fn designation(&self) -> String {
        format!(
            "EHS {:.0}x{:.0}x{:.0}",
            self.d_x * 1000.0,
            self.d_y * 1000.0,
            self.t * 1000.0
        )
    }
}

/// Regular hollow polygon section centered at origin.
///
/// Mirrors Python `polygon_hollow_section(d, t, n_sides, r_in, n_r, rot)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PolygonHollowSection {
    pub d: f64,
    pub t: f64,
    pub n_sides: usize,
    pub r_in: f64,
    pub n_r: usize,
    pub rot: f64,
}

impl PolygonHollowSection {
    pub fn new(d: f64, t: f64, n_sides: usize) -> Self {
        Self::with_radius(d, t, n_sides, 0.0, 1, 0.0)
    }

    pub fn with_radius(
        d: f64,
        t: f64,
        n_sides: usize,
        r_in: f64,
        n_r: usize,
        rot_deg: f64,
    ) -> Self {
        assert!(d > 0.0 && t > 0.0, "Dimensions must be positive");
        assert!(n_sides >= 3, "Need at least 3 sides");
        Self {
            d,
            t,
            n_sides,
            r_in,
            n_r,
            rot: rot_deg * PI / 180.0,
        }
    }
}

impl ParametricSection for PolygonHollowSection {
    fn build(&self) -> Section {
        let alpha = 2.0 * PI / self.n_sides as f64;
        let a_out = self.d / 2.0 * (alpha / 2.0).cos();
        let a_in = a_out - self.t;
        let side_length_out = self.d * (alpha / 2.0).sin();
        let side_length_in = a_in / a_out * side_length_out;

        let r_in = self.r_in.min(a_in);
        let (r_out, n_r) = if r_in == 0.0 {
            (0.0, 1)
        } else {
            (r_in + self.t, self.n_r)
        };

        let c_out = r_out * (side_length_out / 2.0) / a_out;
        let c_in = r_in * (side_length_in / 2.0) / a_in;
        let sl_straight_out = side_length_out - 2.0 * c_out;
        let sl_straight_in = side_length_in - 2.0 * c_in;

        // Build one corner radius, then rotate for each side
        let mut outer_base = Vec::new();
        let mut inner_base = Vec::new();
        for i in 0..n_r {
            let theta = 0.5 * PI + i as f64 / (n_r.max(1) - 1).max(1) as f64 * alpha;
            outer_base.push(Point::new(
                sl_straight_out / 2.0 - r_out * theta.cos(),
                -a_out + r_out - r_out * theta.sin(),
            ));
            inner_base.push(Point::new(
                sl_straight_in / 2.0 - r_in * theta.cos(),
                -a_in + r_in - r_in * theta.sin(),
            ));
        }

        let mut outer_pts = Vec::new();
        let mut inner_pts = Vec::new();
        for i in 0..self.n_sides {
            let angle = alpha * i as f64 + self.rot;
            for pt in &outer_base {
                outer_pts.push(rotate_point(*pt, angle));
            }
            for pt in &inner_base {
                inner_pts.push(rotate_point(*pt, angle));
            }
        }
        inner_pts.reverse();
        Section::new(Polygon::new(outer_pts), vec![Polygon::new(inner_pts)])
    }

    fn designation(&self) -> String {
        format!(
            "PHS {}-sides {:.0}x{:.0}",
            self.n_sides,
            self.d * 1000.0,
            self.t * 1000.0
        )
    }
}

/// Right-angled isosceles triangle with concave radius on hypotenuse.
///
/// Mirrors Python `triangular_radius_section(b, n_r)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TriangularRadiusSection {
    pub b: f64,
    pub n_r: usize,
}

impl TriangularRadiusSection {
    pub fn new(b: f64, n_r: usize) -> Self {
        assert!(b > 0.0, "Base must be positive");
        assert!(n_r >= 2, "Need at least 2 points for radius");
        Self { b, n_r }
    }
}

impl ParametricSection for TriangularRadiusSection {
    fn build(&self) -> Section {
        let mut points = vec![Point::new(0.0, 0.0)];
        let arc = draw_radius(
            Point::new(self.b, self.b),
            self.b,
            3.0 * PI / 2.0,
            self.n_r,
            false,
            PI / 2.0,
        );
        points.extend(arc);
        Section::new(Polygon::new(points), vec![])
    }

    fn designation(&self) -> String {
        format!("TriR {:.0}", self.b * 1000.0)
    }
}

/// Pentagon section.
///
/// Mirrors Python `pentagon_section(d, n, rot)`. `d` is the diameter of the
/// circumscribed circle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PentagonSection {
    pub d: f64,
    pub n: usize,
}

impl PentagonSection {
    pub fn new(d: f64) -> Self {
        Self::with_vertices(d, 64)
    }

    pub fn with_vertices(d: f64, n: usize) -> Self {
        assert!(d > 0.0, "Diameter must be positive");
        Self { d, n }
    }
}

impl ParametricSection for PentagonSection {
    fn build(&self) -> Section {
        RegularPolygonSection::new(self.d / 2.0, 5).build()
    }

    fn designation(&self) -> String {
        format!("PENTAGON Ø{:.0}", self.d * 1000.0)
    }
}

/// Hexagon section.
///
/// Mirrors Python `hexagon_section(d, n, rot)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HexagonSection {
    pub d: f64,
    pub n: usize,
}

impl HexagonSection {
    pub fn new(d: f64) -> Self {
        Self::with_vertices(d, 64)
    }

    pub fn with_vertices(d: f64, n: usize) -> Self {
        assert!(d > 0.0, "Diameter must be positive");
        Self { d, n }
    }
}

impl ParametricSection for HexagonSection {
    fn build(&self) -> Section {
        RegularPolygonSection::new(self.d / 2.0, 6).build()
    }

    fn designation(&self) -> String {
        format!("HEXAGON Ø{:.0}", self.d * 1000.0)
    }
}

/// Octagon section.
///
/// Mirrors Python `octagon_section(d, n, rot)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OctagonSection {
    pub d: f64,
    pub n: usize,
}

impl OctagonSection {
    pub fn new(d: f64) -> Self {
        Self::with_vertices(d, 64)
    }

    pub fn with_vertices(d: f64, n: usize) -> Self {
        assert!(d > 0.0, "Diameter must be positive");
        Self { d, n }
    }
}

impl ParametricSection for OctagonSection {
    fn build(&self) -> Section {
        RegularPolygonSection::new(self.d / 2.0, 8).build()
    }

    fn designation(&self) -> String {
        format!("OCTAGON Ø{:.0}", self.d * 1000.0)
    }
}

#[test]
fn pentagon_section() {
    let p = PentagonSection::new(0.1);
    let sec = p.build();
    // Area of regular pentagon with circumradius R: (5/2) R^2 sin(72°)
    let r = 0.05_f64;
    let expected = 2.5 * r * r * (2.0 * PI * 0.2).sin();
    assert!((sec.area() - expected).abs() < 1e-8);
}

#[test]
fn hexagon_section() {
    let h = HexagonSection::new(0.12);
    let sec = h.build();
    let expected = 3.0 * 3.0_f64.sqrt() / 2.0 * 0.06_f64.powi(2);
    assert!((sec.area() - expected).abs() < 1e-10);
}

#[test]
fn octagon_section() {
    let o = OctagonSection::new(0.2);
    let sec = o.build();
    let r = 0.1_f64;
    let expected = 2.0 * 2.0_f64.sqrt() * r * r;
    assert!((sec.area() - expected).abs() < 1e-8);
}

#[test]
fn geometry_align_center_and_mirror() {
    use crate::geometry::{Axis, Geometry};
    let rect = Geometry::new(
        Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(2.0, 0.0),
            Point::new(2.0, 1.0),
            Point::new(0.0, 1.0),
        ]),
        vec![],
    );
    let c = rect.align_center().apply_transforms();
    assert!((c.centroid().x - 0.0).abs() < 1e-12);
    assert!((c.centroid().y - 0.0).abs() < 1e-12);

    let m = rect.clone().mirror(Axis::Y).apply_transforms();
    assert!((m.centroid().x + 1.0).abs() < 1e-12);

    let s = rect.clone().shift(1.0, -3.0).apply_transforms();
    assert!((s.centroid().x - 2.0).abs() < 1e-12);
    assert!((s.centroid().y + 2.5).abs() < 1e-12);

    // Rotate about origin by 90 degrees: area preserved, centroid moved
    let r = rect
        .clone()
        .rotate_about(90.0, Point::new(0.0, 0.0))
        .apply_transforms();
    assert!((r.area() - rect.area()).abs() < 1e-12);
    assert!((r.centroid().x + 0.5).abs() < 1e-9);
    assert!((r.centroid().y - 1.0).abs() < 1e-9);
}

#[test]
fn geometry_align_to() {
    use crate::geometry::Geometry;
    let a = Geometry::new(
        Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(1.0, 1.0),
            Point::new(0.0, 1.0),
        ]),
        vec![],
    );
    let b = Geometry::new(
        Polygon::new(vec![
            Point::new(10.0, 10.0),
            Point::new(11.5, 10.0),
            Point::new(11.5, 11.0),
            Point::new(10.0, 11.0),
        ]),
        vec![],
    );
    let aligned = a.align_to(&b).apply_transforms();
    assert!((aligned.centroid().y - 10.5).abs() < 1e-12);
}

#[test]
fn geometry_offset_rectangle() {
    use crate::geometry::Geometry;
    let rect = Geometry::new(
        Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(2.0, 0.0),
            Point::new(2.0, 1.0),
            Point::new(0.0, 1.0),
        ]),
        vec![],
    );
    // Grow by 0.1: area = 2.2 * 1.2
    let grown = rect.offset(0.1).unwrap().apply_transforms();
    assert!((grown.area() - 2.64).abs() < 1e-9);
    // Shrink by 0.2: area = 1.6 * 0.6
    let shrunk = rect.offset(-0.2).unwrap().apply_transforms();
    assert!((shrunk.area() - 0.96).abs() < 1e-9);
    // Over-shrink degenerates -> None
    assert!(rect.offset(-0.6).is_none());
}

#[test]
fn geometry_offset_circle() {
    use crate::geometry::Geometry;
    use std::f64::consts::PI;
    let r = 0.5_f64;
    let circ = Geometry::from_section(&CircularSection::new(r).build());
    let grown = circ.offset(0.05).unwrap();
    let expected = PI * (r + 0.05_f64).powi(2);
    assert!((grown.area() - expected).abs() / expected < 5e-3);
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::f64::consts::PI;

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
        // Area = horizontal bar + vertical bar - overlap
        let expected = 0.15 * 2.0 * 0.02 + 0.2 * 0.01 - 0.01 * 2.0 * 0.02;
        assert!((sec.area() - expected).abs() < 1e-6);
    }

    #[test]
    fn elliptical_hollow_section() {
        let ehs = EllipticalHollowSection::new(0.05, 0.025, 0.002, 64);
        let sec = ehs.build();
        // Outer ellipse area = pi * a * b; inner = pi * (a-t) * (b-t)
        let a = 0.025_f64;
        let b = 0.0125_f64;
        let t = 0.002_f64;
        let expected = PI * (a * b - (a - t) * (b - t));
        assert!(
            (sec.area() - expected).abs() / expected < 0.02,
            "EHS area: got {}, expected {}",
            sec.area(),
            expected
        );
    }

    #[test]
    fn polygon_hollow_octagon() {
        let phs = PolygonHollowSection::new(0.2, 0.006, 8);
        let sec = phs.build();
        assert!(sec.area() > 0.0, "Polygon hollow area should be positive");
        // Octagon outer area = 2*(1+sqrt(2))*s^2 where s = side length
        // For circumscribed circle radius R=0.1, s = 2*R*sin(pi/8)
        let r = 0.1_f64;
        let s = 2.0 * r * (PI / 8.0).sin();
        let outer_area = 2.0 * (1.0 + 2.0_f64.sqrt()) * s * s;
        // Inner is scaled by (a_in/a_out)^2
        let a_out = r * (PI / 8.0).cos();
        let a_in = a_out - 0.006;
        let inner_area = outer_area * (a_in / a_out).powi(2);
        let expected = outer_area - inner_area;
        assert!(
            (sec.area() - expected).abs() / expected < 0.05,
            "PHS area: got {}, expected {}",
            sec.area(),
            expected
        );
    }

    #[test]
    fn polygon_hollow_with_radius() {
        let phs = PolygonHollowSection::with_radius(0.2, 0.006, 8, 0.02, 12, 0.0);
        let sec = phs.build();
        assert!(sec.area() > 0.0);
    }

    #[test]
    fn triangular_radius_section() {
        let tri = TriangularRadiusSection::new(0.06, 16);
        let sec = tri.build();
        // Area should be less than the full triangle (b*b/2) due to concave hypotenuse
        let full_triangle = 0.5 * 0.06 * 0.06;
        assert!(sec.area() < full_triangle);
        assert!(sec.area() > 0.0);
    }
}
