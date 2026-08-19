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
    /// # Panics
    ///
    /// Panics if the outer polygon is not CCW or any hole is not CW.
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
}
