use super::Point;

#[derive(Debug, Clone)]
pub struct Polygon {
    pub vertices: Vec<Point>,
}

impl Polygon {
    pub fn new(vertices: Vec<Point>) -> Self {
        assert!(vertices.len() >= 3, "Polygon needs at least 3 vertices");

        Self { vertices }
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

        Point::new(
            cx / (6.0 * signed_area),
            cy / (6.0 * signed_area),
        )
    }
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
}
