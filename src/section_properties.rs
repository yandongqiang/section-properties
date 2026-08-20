use crate::geometry::Point;
use crate::section::Section;
#[derive(Debug, Clone, Copy)]
pub struct PrincipalProperties {
    /// Major principal moment of inertia.
    pub i1: f64,

    /// Minor principal moment of inertia.
    pub i2: f64,

    /// Principal axis angle in radians, measured CCW from the x-axis.
    pub angle: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct GyrationProperties {
    /// Radius of gyration about the centroidal x-axis.
    pub rx: f64,

    /// Radius of gyration about the centroidal y-axis.
    pub ry: f64,

    /// Polar radius of gyration.
    pub polar: f64,
}

/// Mechanical properties of a section (area, centroid, moments of inertia, etc.).
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
    /// Compute section properties from a `Section` (outer boundary + holes).
    pub fn from_section(section: &Section) -> Self {
        let mut area = 0.0;
        let mut first_x = 0.0;
        let mut first_y = 0.0;
        let mut ix = 0.0;
        let mut iy = 0.0;
        let mut ixy = 0.0;

        // Polygon orientation is normalized by Section:
        // outer -> CCW (positive signed area)
        // holes -> CW (negative signed area)
        //
        // Therefore we use signed geometric quantities directly.
        let mut add_polygon = |poly: &crate::geometry::Polygon| {
            let signed_area = poly.signed_area();
            let centroid = poly.centroid();

            area += signed_area;
            first_x += signed_area * centroid.x;
            first_y += signed_area * centroid.y;

            ix += poly.moment_of_inertia_x();
            iy += poly.moment_of_inertia_y();
            ixy += poly.product_of_inertia_xy();
        };

        add_polygon(&section.outer);

        for hole in &section.holes {
            add_polygon(hole);
        }

        assert!(
            area.abs() > f64::EPSILON,
            "Section area is too small to compute properties"
        );

        // Global centroid
        let centroid = Point::new(first_x / area, first_y / area);

        // Parallel-axis theorem:
        //
        // I_x,c = I_x,global - A * cy²
        // I_y,c = I_y,global - A * cx²
        // I_xy,c = I_xy,global - A * cx * cy
        let ix_c = ix - area * centroid.y.powi(2);
        let iy_c = iy - area * centroid.x.powi(2);
        let ixy_c = ixy - area * centroid.x * centroid.y;

        Self {
            area,
            centroid,
            ix: ix_c,
            iy: iy_c,
            ixy: ixy_c,
        }
    }

    /// Principal moments of inertia and principal angle (radians, CCW from x-axis).
    pub fn principal_moments(&self) -> (f64, f64, f64) {
        let avg = (self.ix + self.iy) * 0.5;
        let diff = (self.ix - self.iy) * 0.5;
        let rad = (diff * diff + self.ixy * self.ixy).sqrt();
        let i1 = avg + rad;
        let i2 = avg - rad;
        let theta = 0.5 * (2.0 * self.ixy).atan2(self.ix - self.iy);
        (i1, i2, theta)
    }
    pub fn principal_properties(&self) -> PrincipalProperties {
        let avg = (self.ix + self.iy) * 0.5;
        let diff = (self.ix - self.iy) * 0.5;

        let radius = (diff * diff + self.ixy * self.ixy).sqrt();

        let i1 = avg + radius;
        let i2 = avg - radius;

        let angle = 0.5 * (2.0 * self.ixy).atan2(self.ix - self.iy);

        PrincipalProperties { i1, i2, angle }
    }

    /// Radius of gyration about centroidal axes.
    pub fn radius_of_gyration(&self) -> (f64, f64, f64) {
        let rx = (self.ix / self.area).sqrt();
        let ry = (self.iy / self.area).sqrt();
        let rho = ((self.ix + self.iy) / (2.0 * self.area)).sqrt();
        (rx, ry, rho)
    }
    pub fn gyration_properties(&self) -> GyrationProperties {
        GyrationProperties {
            rx: (self.ix / self.area).sqrt(),
            ry: (self.iy / self.area).sqrt(),
            polar: ((self.ix + self.iy) / self.area).sqrt(),
        }
    }

    /// Maximum fiber distance from centroid in Y direction (for section modulus).
    pub fn max_fiber_distance_y(&self) -> f64 {
        (self.ix / self.area).sqrt() * 2.0
    }

    /// Maximum fiber distance from centroid in X direction (for section modulus).
    pub fn max_fiber_distance_x(&self) -> f64 {
        (self.iy / self.area).sqrt() * 2.0
    }
}
