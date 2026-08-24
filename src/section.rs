use crate::geometry::{Point, Polygon};

/// A section can consist of an outer polygon and zero or more hole polygons.
#[derive(Debug, Clone)]
pub struct Section {
    /// Outer boundary (must be CCW for positive area)
    pub outer: Polygon,
    /// Hole boundaries (each must be CW so that their signed area is negative)
    pub holes: Vec<Polygon>,
}

impl Section {
    /// Create a new section from an outer polygon and optional holes.
    ///
    /// The outer boundary is normalised to counter-clockwise orientation
    /// and each hole to clockwise orientation.
    pub fn new(mut outer: Polygon, mut holes: Vec<Polygon>) -> Self {
        // Section owns the orientation convention:
        // outer boundary -> CCW
        // holes -> CW

        if outer.signed_area() < 0.0 {
            outer.vertices.reverse();
        }

        for hole in &mut holes {
            if hole.signed_area() > 0.0 {
                hole.vertices.reverse();
            }
        }

        Self { outer, holes }
    }

    /// Net area of the section (outer area minus hole areas).
    pub fn area(&self) -> f64 {
        let mut a = self.outer.area();
        for h in &self.holes {
            a -= h.area();
        }
        a
    }

    /// Centroid of the section using the composite area formula.
    pub fn centroid(&self) -> Point {
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut total_area = 0.0;

        // Outer contour (positive area)
        let outer_c = self.outer.centroid();
        let outer_a = self.outer.area();
        sum_x += outer_c.x * outer_a;
        sum_y += outer_c.y * outer_a;
        total_area += outer_a;

        // Holes (negative area)
        for h in &self.holes {
            let a = h.area(); // positive magnitude
            let c = h.centroid();
            sum_x -= c.x * a;
            sum_y -= c.y * a;
            total_area -= a;
        }

        assert!(
            total_area.abs() > f64::EPSILON,
            "Section area is too small to compute centroid"
        );

        Point::new(sum_x / total_area, sum_y / total_area)
    }

    /// Bounding box of the section: (min_x, max_x, min_y, max_y)
    pub fn bounds(&self) -> (f64, f64, f64, f64) {
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

    /// Width of the section (max_x - min_x)
    pub fn width(&self) -> f64 {
        let (min_x, max_x, _, _) = self.bounds();
        max_x - min_x
    }

    /// Height of the section (max_y - min_y)
    pub fn height(&self) -> f64 {
        let (_, _, min_y, max_y) = self.bounds();
        max_y - min_y
    }

    /// Check if a point is inside the section (including holes).
    pub fn contains_point(&self, point: Point) -> bool {
        // Check outer polygon
        if !self.outer.contains_point(point) {
            return false;
        }
        // Check holes
        for hole in &self.holes {
            if hole.contains_point(point) {
                return false;
            }
        }
        true
    }

    /// Perimeter of the section (outer + holes).
    pub fn perimeter(&self) -> f64 {
        let mut p = self.outer.perimeter();
        for hole in &self.holes {
            p += hole.perimeter();
        }
        p
    }

    /// Frame properties for beam/frame analysis (matches Python's calculate_frame_properties).
    ///
    /// Returns tuple: (area, ixx, iyy, ixy, j, phi)
    /// - area: cross-sectional area [m²]
    /// - ixx: second moment of area about x-axis [m⁴]
    /// - iyy: second moment of area about y-axis [m⁴]
    /// - ixy: product of inertia [m⁴]
    /// - j: St. Venant torsion constant [m⁴]
    /// - phi: principal axis angle (radians, CCW from x-axis) [rad]
    pub fn frame_properties(&self) -> (f64, f64, f64, f64, f64, f64) {
        use crate::plastic::warping::WarpingProperties;
        use crate::section_properties::SectionProperties;

        let props = SectionProperties::from_section(self);
        let warping = WarpingProperties::from_section(self);
        let principal = props.principal_properties();

        (
            props.area,
            props.ix,
            props.iy,
            props.ixy,
            warping.j,
            principal.phi,
        )
    }

    /// Frame properties with custom material (for composite sections).
    pub fn frame_properties_with_material(&self, _material: &crate::material::Material) -> (f64, f64, f64, f64, f64, f64) {
        // For composite sections, would use transformed section method
        // For now, return geometric properties
        self.frame_properties()
    }
}

impl crate::section_library::ParametricSection for Section {
    fn build(&self) -> Section {
        self.clone()
    }

    fn designation(&self) -> String {
        "Section".to_string()
    }
}
