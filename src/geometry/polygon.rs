use super::Point;

#[derive(Debug, Clone)]
pub struct Polygon {
    pub vertices: Vec<Point>,
}

impl Polygon {
    pub fn new(vertices: Vec<Point>) -> Self {
        assert!(vertices.len() >= 3, "Polygon needs at least 3 vertices");

        for (i, v) in vertices.iter().enumerate() {
            assert!(
                v.x.is_finite() && v.y.is_finite(),
                "Polygon vertex {} is not finite: ({}, {})",
                i,
                v.x,
                v.y
            );
        }

        // Remove consecutive duplicate vertices (zero-length edges) and a
        // trailing vertex that closes back on the first (closed-ring convention).
        let mut dedup: Vec<Point> = Vec::with_capacity(vertices.len());
        for v in &vertices {
            if dedup.last().map_or(true, |p| *p != *v) {
                dedup.push(*v);
            }
        }
        if dedup.len() > 1 && dedup[0] == *dedup.last().unwrap() {
            dedup.pop();
        }

        assert!(
            dedup.len() >= 3,
            "Polygon needs at least 3 distinct vertices"
        );

        let poly = Self { vertices: dedup };
        assert!(
            poly.signed_area().abs() > f64::EPSILON,
            "Polygon has (near-)zero area"
        );

        poly
    }

    /// Offset the polygon by `distance` using mitred corners.
    ///
    /// Positive distances grow a CCW polygon outward; negative distances shrink
    /// it. Returns `None` if the offset polygon degenerates (e.g. an inward
    /// offset larger than the polygon size).
    ///
    /// Mirrors shapely's `buffer(distance, join_style=mitre)` used by Python
    /// `sectionproperties`' `Geometry.offset()`.
    pub fn offset(&self, distance: f64) -> Option<Polygon> {
        // Work with CCW orientation so that positive = grow.
        let ccw: Vec<Point> = if self.signed_area() < 0.0 {
            self.vertices.iter().rev().copied().collect()
        } else {
            self.vertices.clone()
        };
        let n = ccw.len();
        if n < 3 {
            return None;
        }

        let mut out: Vec<Point> = Vec::with_capacity(n);
        for i in 0..n {
            let p0 = ccw[(i + n - 1) % n];
            let p1 = ccw[i];
            let p2 = ccw[(i + 1) % n];

            let d1x = p1.x - p0.x;
            let d1y = p1.y - p0.y;
            let d2x = p2.x - p1.x;
            let d2y = p2.y - p1.y;

            let l1 = (d1x * d1x + d1y * d1y).sqrt();
            let l2 = (d2x * d2x + d2y * d2y).sqrt();
            if l1 < f64::EPSILON || l2 < f64::EPSILON {
                continue;
            }

            // Outward unit normals (right side of travel direction for CCW).
            let n1 = (d1y / l1, -d1x / l1);
            let n2 = (d2y / l2, -d2x / l2);

            // Offset lines: point + t * direction.
            // Line A through (p0 + d*n1) along (d1x,d1y); line B through
            // (p1 + d*n2) along (d2x,d2y).
            let ax = p0.x + distance * n1.0;
            let ay = p0.y + distance * n1.1;
            let bx = p1.x + distance * n2.0;
            let by = p1.y + distance * n2.1;

            let denom = d1x * d2y - d1y * d2x;
            if denom.abs() < 1e-14 {
                // Collinear edges: use the simple offset vertex.
                out.push(Point::new(p1.x + distance * n2.0, p1.y + distance * n2.1));
                continue;
            }

            let t = ((bx - ax) * d2y - (by - ay) * d2x) / denom;
            out.push(Point::new(ax + t * d1x, ay + t * d1y));
        }

        if out.len() < 3 {
            return None;
        }
        let poly_area = out
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let q = &out[(i + 1) % out.len()];
                p.x * q.y - q.x * p.y
            })
            .sum::<f64>()
            / 2.0;

        // Degenerate or self-intersecting after inward offset.
        if poly_area.abs() < f64::EPSILON || poly_area < 0.0 {
            return None;
        }
        Some(Polygon::new(out))
    }

    /// Calculate the signed area of the polygon.
    ///
    /// Counter-clockwise polygon -> positive
    /// Clockwise polygon -> negative
    pub fn signed_area(&self) -> f64 {
        let mut sum = 0.0;

        for i in 0..self.vertices.len() {
            let p1 = self.vertices[i];
            let p2 = self.vertices[(i + 1) % self.vertices.len()];

            sum += p1.x * p2.y - p2.x * p1.y;
        }

        0.5 * sum
    }

    /// Calculate the absolute area of the polygon.
    pub fn area(&self) -> f64 {
        self.signed_area().abs()
    }

    /// Calculate the centroid of the polygon.
    pub fn centroid(&self) -> Point {
        let signed_area = self.signed_area();

        assert!(
            signed_area.abs() > f64::EPSILON,
            "Cannot calculate centroid of a degenerate polygon"
        );

        let mut cx = 0.0;
        let mut cy = 0.0;

        for i in 0..self.vertices.len() {
            let p1 = self.vertices[i];
            let p2 = self.vertices[(i + 1) % self.vertices.len()];

            let cross = p1.x * p2.y - p2.x * p1.y;

            cx += (p1.x + p2.x) * cross;
            cy += (p1.y + p2.y) * cross;
        }

        Point::new(cx / (6.0 * signed_area), cy / (6.0 * signed_area))
    }

    /// Calculate the second moment of area about the global x-axis.
    pub fn moment_of_inertia_x(&self) -> f64 {
        let mut sum = 0.0;

        for i in 0..self.vertices.len() {
            let p1 = self.vertices[i];
            let p2 = self.vertices[(i + 1) % self.vertices.len()];

            let cross = p1.x * p2.y - p2.x * p1.y;

            sum += (p1.y.powi(2) + p1.y * p2.y + p2.y.powi(2)) * cross;
        }

        sum / 12.0
    }

    /// Calculate the second moment of area about the global y-axis.
    pub fn moment_of_inertia_y(&self) -> f64 {
        let mut sum = 0.0;

        for i in 0..self.vertices.len() {
            let p1 = self.vertices[i];
            let p2 = self.vertices[(i + 1) % self.vertices.len()];

            let cross = p1.x * p2.y - p2.x * p1.y;

            sum += (p1.x.powi(2) + p1.x * p2.x + p2.x.powi(2)) * cross;
        }

        sum / 12.0
    }

    /// Calculate the product of area about the global axes.
    pub fn product_of_inertia_xy(&self) -> f64 {
        let mut sum = 0.0;

        for i in 0..self.vertices.len() {
            let p1 = self.vertices[i];
            let p2 = self.vertices[(i + 1) % self.vertices.len()];

            let cross = p1.x * p2.y - p2.x * p1.y;

            sum += (p1.x * p2.y + 2.0 * p1.x * p1.y + 2.0 * p2.x * p2.y + p2.x * p1.y) * cross;
        }

        sum / 24.0
    }

    /// Calculate the second moment of area about the centroidal x-axis.
    pub fn centroidal_moment_of_inertia_x(&self) -> f64 {
        let area = self.area();
        let centroid = self.centroid();

        self.moment_of_inertia_x() - area * centroid.y.powi(2)
    }

    /// Calculate the second moment of area about the centroidal y-axis.
    pub fn centroidal_moment_of_inertia_y(&self) -> f64 {
        let area = self.area();
        let centroid = self.centroid();

        self.moment_of_inertia_y() - area * centroid.x.powi(2)
    }

    /// Calculate the product of area about the centroidal axes.
    pub fn centroidal_product_of_inertia_xy(&self) -> f64 {
        let area = self.area();
        let centroid = self.centroid();

        self.product_of_inertia_xy() - area * centroid.x * centroid.y
    }

    /// Rotate all vertices about the origin by `angle` radians (CCW positive).
    pub fn rotate(&self, angle: f64) -> Self {
        Polygon::new(
            self.vertices
                .iter()
                .map(|v| {
                    Point::new(
                        v.x * angle.cos() - v.y * angle.sin(),
                        v.x * angle.sin() + v.y * angle.cos(),
                    )
                })
                .collect(),
        )
    }

    /// Check if a point is inside the polygon (ray casting algorithm).
    pub fn contains_point(&self, point: Point) -> bool {
        let mut inside = false;
        let n = self.vertices.len();

        for i in 0..n {
            let p1 = self.vertices[i];
            let p2 = self.vertices[(i + 1) % n];

            // Check if point is on the edge
            if point_on_segment(point, p1, p2) {
                return true;
            }

            // Ray casting algorithm
            if ((p1.y > point.y) != (p2.y > point.y))
                && (point.x < (p2.x - p1.x) * (point.y - p1.y) / (p2.y - p1.y) + p1.x)
            {
                inside = !inside;
            }
        }

        inside
    }

    /// Calculate the perimeter of the polygon.
    pub fn perimeter(&self) -> f64 {
        let mut sum = 0.0;
        let n = self.vertices.len();

        for i in 0..n {
            let p1 = self.vertices[i];
            let p2 = self.vertices[(i + 1) % n];
            let dx = p2.x - p1.x;
            let dy = p2.y - p1.y;
            sum += (dx * dx + dy * dy).sqrt();
        }

        sum
    }

    /// Maximum fiber distance from centroid in Y direction (for section modulus).
    pub fn max_fiber_distance_y(&self) -> f64 {
        let centroid = self.centroid();
        self.vertices
            .iter()
            .map(|v| (v.y - centroid.y).abs())
            .fold(0.0, f64::max)
    }

    /// Maximum fiber distance from centroid in X direction (for section modulus).
    pub fn max_fiber_distance_x(&self) -> f64 {
        let centroid = self.centroid();
        self.vertices
            .iter()
            .map(|v| (v.x - centroid.x).abs())
            .fold(0.0, f64::max)
    }
}

/// Helper function to check if a point is on a line segment.
fn point_on_segment(p: Point, a: Point, b: Point) -> bool {
    let cross = (p.x - a.x) * (b.y - a.y) - (p.y - a.y) * (b.x - a.x);
    if cross.abs() > 1e-10 {
        return false;
    }
    let dot = (p.x - a.x) * (p.x - b.x) + (p.y - a.y) * (p.y - b.y);
    dot <= 1e-10
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rectangle_area() {
        let polygon = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 5.0),
            Point::new(0.0, 5.0),
        ]);

        assert_eq!(polygon.area(), 50.0);
    }

    #[test]
    fn triangle_area() {
        let polygon = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(4.0, 0.0),
            Point::new(0.0, 3.0),
        ]);

        assert_eq!(polygon.area(), 6.0);
    }

    #[test]
    fn rectangle_centroid() {
        let polygon = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 5.0),
            Point::new(0.0, 5.0),
        ]);

        let centroid = polygon.centroid();

        assert!((centroid.x - 5.0).abs() < 1e-12);
        assert!((centroid.y - 2.5).abs() < 1e-12);
    }

    #[test]
    fn triangle_centroid() {
        let polygon = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(4.0, 0.0),
            Point::new(0.0, 3.0),
        ]);

        let centroid = polygon.centroid();

        assert!((centroid.x - 4.0 / 3.0).abs() < 1e-12);
        assert!((centroid.y - 1.0).abs() < 1e-12);
    }

    #[test]
    fn clockwise_polygon_has_same_centroid() {
        let polygon = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(0.0, 5.0),
            Point::new(10.0, 5.0),
            Point::new(10.0, 0.0),
        ]);

        let centroid = polygon.centroid();

        assert!((centroid.x - 5.0).abs() < 1e-12);
        assert!((centroid.y - 2.5).abs() < 1e-12);
    }

    #[test]
    fn rectangle_global_moments_of_inertia() {
        let polygon = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 5.0),
            Point::new(0.0, 5.0),
        ]);

        let ix = polygon.moment_of_inertia_x();
        let iy = polygon.moment_of_inertia_y();
        let ixy = polygon.product_of_inertia_xy();

        assert!((ix - 416.6666666666667).abs() < 1e-10);
        assert!((iy - 1666.6666666666667).abs() < 1e-10);
        assert!((ixy - 625.0).abs() < 1e-10);
    }

    #[test]
    fn rectangle_centroidal_moments_of_inertia() {
        let polygon = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 5.0),
            Point::new(0.0, 5.0),
        ]);

        let ix = polygon.centroidal_moment_of_inertia_x();
        let iy = polygon.centroidal_moment_of_inertia_y();
        let ixy = polygon.centroidal_product_of_inertia_xy();

        assert!((ix - 104.16666666666667).abs() < 1e-10);
        assert!((iy - 416.6666666666667).abs() < 1e-10);
        assert!(ixy.abs() < 1e-10);
    }

    #[test]
    fn triangle_centroidal_moments_of_inertia() {
        let polygon = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(4.0, 0.0),
            Point::new(0.0, 3.0),
        ]);

        let ix = polygon.centroidal_moment_of_inertia_x();
        let iy = polygon.centroidal_moment_of_inertia_y();
        let ixy = polygon.centroidal_product_of_inertia_xy();

        assert!((ix - 3.0).abs() < 1e-10);
        assert!((iy - 5.333333333333333).abs() < 1e-10);
        assert!((ixy + 2.0).abs() < 1e-10);
    }
}
