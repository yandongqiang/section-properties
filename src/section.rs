use crate::geometry::{Point, Polygon};

#[derive(Debug, Clone, Copy)]
pub struct SectionProperties {
    pub area: f64,

    pub centroid: Point,

    /// Second moment of area about the centroidal x-axis.
    pub ix: f64,

    /// Second moment of area about the centroidal y-axis.
    pub iy: f64,

    /// Product of inertia about the centroidal axes.
    pub ixy: f64,
}

impl SectionProperties {
    pub fn from_polygon(polygon: &Polygon) -> Self {
        Self {
            area: polygon.area(),
            centroid: polygon.centroid(),
            ix: polygon.centroidal_moment_of_inertia_x(),
            iy: polygon.centroidal_moment_of_inertia_y(),
            ixy: polygon.centroidal_product_of_inertia_xy(),
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rectangle_section_properties() {
        let polygon = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 5.0),
            Point::new(0.0, 5.0),
        ]);

        let section = SectionProperties::from_polygon(&polygon);

        assert!((section.area - 50.0).abs() < 1e-10);

        assert!((section.centroid.x - 5.0).abs() < 1e-10);
        assert!((section.centroid.y - 2.5).abs() < 1e-10);

        assert!((section.ix - 104.16666666666667).abs() < 1e-10);
        assert!((section.iy - 416.6666666666667).abs() < 1e-10);

        assert!(section.ixy.abs() < 1e-10);
    }
}
