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

    /// Maximum distance from the centroid to the section boundary along Y.
    pub max_fiber_distance_y: f64,

    /// Maximum distance from the centroid to the section boundary along X.
    pub max_fiber_distance_x: f64,

    /// Section modulus about x-axis for positive extreme fibre (Ix / y_max).
    pub zxx_plus: f64,
    /// Section modulus about x-axis for negative extreme fibre (Ix / |y_min|).
    pub zxx_minus: f64,
    /// Section modulus about y-axis for positive extreme fibre (Iy / x_max).
    pub zyy_plus: f64,
    /// Section modulus about y-axis for negative extreme fibre (Iy / |x_min|).
    pub zyy_minus: f64,
    /// Section modulus about 11-axis for positive extreme fibre.
    pub z11_plus: f64,
    /// Section modulus about 11-axis for negative extreme fibre.
    pub z11_minus: f64,
    /// Section modulus about 22-axis for positive extreme fibre.
    pub z22_plus: f64,
    /// Section modulus about 22-axis for negative extreme fibre.
    pub z22_minus: f64,
    /// Cross-sectional perimeter (outer + holes).
    pub perimeter: f64,
    /// First moment of area about the global x-axis (∫y dA).
    pub qx: f64,
    /// First moment of area about the global y-axis (∫x dA).
    pub qy: f64,
    /// Second moment of area about the global x-axis.
    pub ixx_g: f64,
    /// Second moment of area about the global y-axis.
    pub iyy_g: f64,
    /// Product of inertia about the global xy-axis.
    pub ixy_g: f64,
    /// Principal major second moment of area (centroidal).
    pub i11_c: f64,
    /// Principal minor second moment of area (centroidal).
    pub i22_c: f64,
    /// Principal axis angle in radians, CCW from x-axis.
    pub phi: f64,
    /// Radius of gyration about the centroidal x-axis.
    pub rx_c: f64,
    /// Radius of gyration about the centroidal y-axis.
    pub ry_c: f64,
    /// Radius of gyration about the principal 11-axis.
    pub r11_c: f64,
    /// Radius of gyration about the principal 22-axis.
    pub r22_c: f64,
}

impl Default for SectionProperties {
    fn default() -> Self {
        Self {
            area: 0.0,
            centroid: Point::new(0.0, 0.0),
            ix: 0.0,
            iy: 0.0,
            ixy: 0.0,
            max_fiber_distance_y: 0.0,
            max_fiber_distance_x: 0.0,
            zxx_plus: 0.0,
            zxx_minus: 0.0,
            zyy_plus: 0.0,
            zyy_minus: 0.0,
            z11_plus: 0.0,
            z11_minus: 0.0,
            z22_plus: 0.0,
            z22_minus: 0.0,
            perimeter: 0.0,
            qx: 0.0,
            qy: 0.0,
            ixx_g: 0.0,
            iyy_g: 0.0,
            ixy_g: 0.0,
            i11_c: 0.0,
            i22_c: 0.0,
            phi: 0.0,
            rx_c: 0.0,
            ry_c: 0.0,
            r11_c: 0.0,
            r22_c: 0.0,
        }
    }
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

        // Maximum fiber distances measure from the centroid to the extreme
        // boundary of the section (used for elastic section modulus).
        // Track positive and negative extremes separately (Python convention).
        let (y_min, y_max) = section
            .outer
            .vertices
            .iter()
            .chain(section.holes.iter().flat_map(|p| p.vertices.iter()))
            .map(|v| v.y - centroid.y)
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(mn, mx), d| {
                (mn.min(d), mx.max(d))
            });
        let (x_min, x_max) = section
            .outer
            .vertices
            .iter()
            .chain(section.holes.iter().flat_map(|p| p.vertices.iter()))
            .map(|v| v.x - centroid.x)
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(mn, mx), d| {
                (mn.min(d), mx.max(d))
            });

        let max_fiber_y = y_max.abs().max(y_min.abs());
        let max_fiber_x = x_max.abs().max(x_min.abs());

        // Section moduli about centroidal axes (positive/negative fibres)
        let zxx_plus = if y_max.abs() > 1e-15 { ix_c / y_max.abs() } else { 0.0 };
        let zxx_minus = if y_min.abs() > 1e-15 { ix_c / y_min.abs() } else { 0.0 };
        let zyy_plus = if x_max.abs() > 1e-15 { iy_c / x_max.abs() } else { 0.0 };
        let zyy_minus = if x_min.abs() > 1e-15 { iy_c / x_min.abs() } else { 0.0 };

        // Principal axis section moduli
        let principal = {
            let avg = (ix_c + iy_c) * 0.5;
            let diff = (ix_c - iy_c) * 0.5;
            let radius = (diff * diff + ixy_c * ixy_c).sqrt();
            let i11 = avg + radius;
            let i22 = avg - radius;
            let phi = 0.5 * (2.0 * ixy_c).atan2(ix_c - iy_c);
            (i11, i22, phi)
        };
        let (i11, i22, phi) = principal;
        let cos_phi = phi.cos();
        let sin_phi = phi.sin();

        let (x1_min, x1_max, y2_min, y2_max) = section
            .outer
            .vertices
            .iter()
            .chain(section.holes.iter().flat_map(|p| p.vertices.iter()))
            .map(|v| {
                let dx = v.x - centroid.x;
                let dy = v.y - centroid.y;
                let x1 = dx * cos_phi + dy * sin_phi;
                let y2 = -dx * sin_phi + dy * cos_phi;
                (x1, y2)
            })
            .fold(
                (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY),
                |(x1n, x1x, y2n, y2x), (x1, y2)| {
                    (x1n.min(x1), x1x.max(x1), y2n.min(y2), y2x.max(y2))
                },
            );

        let z11_plus = if y2_max.abs() > 1e-15 { i11 / y2_max.abs() } else { 0.0 };
        let z11_minus = if y2_min.abs() > 1e-15 { i11 / y2_min.abs() } else { 0.0 };
        let z22_plus = if x1_max.abs() > 1e-15 { i22 / x1_max.abs() } else { 0.0 };
        let z22_minus = if x1_min.abs() > 1e-15 { i22 / x1_min.abs() } else { 0.0 };

        // Perimeter (outer + holes)
        let perimeter = section.outer.perimeter()
            + section.holes.iter().map(|h| h.perimeter()).sum::<f64>();

        // Radii of gyration
        let rx_c = (ix_c / area).sqrt();
        let ry_c = (iy_c / area).sqrt();
        let r11_c = (i11 / area).sqrt();
        let r22_c = (i22 / area).sqrt();

        Self {
            area,
            centroid,
            ix: ix_c,
            iy: iy_c,
            ixy: ixy_c,
            max_fiber_distance_y: max_fiber_y,
            max_fiber_distance_x: max_fiber_x,
            zxx_plus,
            zxx_minus,
            zyy_plus,
            zyy_minus,
            z11_plus,
            z11_minus,
            z22_plus,
            z22_minus,
            perimeter,
            qx: first_y,
            qy: first_x,
            ixx_g: ix,
            iyy_g: iy,
            ixy_g: ixy,
            i11_c: i11,
            i22_c: i22,
            phi,
            rx_c,
            ry_c,
            r11_c,
            r22_c,
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
        let rho = ((self.ix + self.iy) / self.area).sqrt();
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
        self.max_fiber_distance_y
    }

    /// Maximum fiber distance from centroid in X direction (for section modulus).
    pub fn max_fiber_distance_x(&self) -> f64 {
        self.max_fiber_distance_x
    }

    /// Elastic section modulus about the x-axis (minimum of plus/minus).
    pub fn section_modulus_x(&self) -> f64 {
        self.zxx_plus.min(self.zxx_minus)
    }

    /// Elastic section modulus about the y-axis (minimum of plus/minus).
    pub fn section_modulus_y(&self) -> f64 {
        self.zyy_plus.min(self.zyy_minus)
    }

    /// Section modulus about the 11-axis (minimum of plus/minus).
    pub fn section_modulus_11(&self) -> f64 {
        self.z11_plus.min(self.z11_minus)
    }

    /// Section modulus about the 22-axis (minimum of plus/minus).
    pub fn section_modulus_22(&self) -> f64 {
        self.z22_plus.min(self.z22_minus)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon};

    fn rectangle(width: f64, height: f64) -> Section {
        let hw = width * 0.5;
        let hh = height * 0.5;
        let poly = Polygon::new(vec![
            Point::new(-hw, -hh),
            Point::new(hw, -hh),
            Point::new(hw, hh),
            Point::new(-hw, hh),
        ]);
        Section::new(poly, vec![])
    }

    fn tee_section() -> Section {
        // T-section: flange 0.2 x 0.02 at top, web 0.02 x 0.08 below.
        let poly = Polygon::new(vec![
            Point::new(-0.1, 0.10),
            Point::new(0.1, 0.10),
            Point::new(0.1, 0.08),
            Point::new(0.01, 0.08),
            Point::new(0.01, 0.0),
            Point::new(-0.01, 0.0),
            Point::new(-0.01, 0.08),
            Point::new(-0.1, 0.08),
        ]);
        Section::new(poly, vec![])
    }

    #[test]
    fn symmetric_section_equal_plus_minus_moduli() {
        let props = SectionProperties::from_section(&rectangle(0.1, 0.2));
        assert!(
            (props.zxx_plus - props.zxx_minus).abs() < 1e-12,
            "symmetric section should have zxx_plus == zxx_minus"
        );
        assert!(
            (props.zyy_plus - props.zyy_minus).abs() < 1e-12,
            "symmetric section should have zyy_plus == zyy_minus"
        );
    }

    #[test]
    fn rectangular_section_modulus_values() {
        let b = 0.1_f64;
        let h = 0.2_f64;
        let props = SectionProperties::from_section(&rectangle(b, h));
        let ix = b * h.powi(3) / 12.0;
        let z_expected = ix / (h * 0.5);
        assert!((props.zxx_plus - z_expected).abs() / z_expected < 1e-10);
        assert!((props.zxx_minus - z_expected).abs() / z_expected < 1e-10);
        assert!((props.section_modulus_x() - z_expected).abs() / z_expected < 1e-10);
    }

    #[test]
    fn asymmetric_section_unequal_plus_minus_moduli() {
        let props = SectionProperties::from_section(&tee_section());
        assert!(
            (props.zxx_plus - props.zxx_minus).abs() > 1e-6,
            "T-section should have zxx_plus != zxx_minus"
        );
        // Flange is at top (y > centroid), web extends downward.
        // y_max (positive) is smaller than |y_min| (negative), so zxx_plus > zxx_minus.
        assert!(
            props.zxx_plus > props.zxx_minus,
            "T-section: zxx_plus should exceed zxx_minus (top fibre closer to centroid)"
        );
    }

    #[test]
    fn section_modulus_x_is_min_of_plus_minus() {
        let props = SectionProperties::from_section(&tee_section());
        let z_min = props.zxx_plus.min(props.zxx_minus);
        assert!((props.section_modulus_x() - z_min).abs() < 1e-12);
    }

    #[test]
    fn principal_moduli_match_centroidal_for_symmetric_section() {
        let props = SectionProperties::from_section(&rectangle(0.1, 0.2));
        assert!((props.z11_plus - props.zxx_plus).abs() / props.zxx_plus < 1e-10);
        assert!((props.z11_minus - props.zxx_minus).abs() / props.zxx_minus < 1e-10);
        assert!((props.z22_plus - props.zyy_plus).abs() / props.zyy_plus < 1e-10);
        assert!((props.z22_minus - props.zyy_minus).abs() / props.zyy_minus < 1e-10);
    }

    #[test]
    fn tee_section_modulus_analytical_check() {
        let props = SectionProperties::from_section(&tee_section());
        // Area = 0.2*0.02 + 0.02*0.08 = 0.0056
        let area = 0.0056_f64;
        // Centroid y (from bottom): (0.004*0.09 + 0.0016*0.04) / 0.0056
        let yc = (0.004 * 0.09 + 0.0016 * 0.04) / area;
        // Ix about centroid (parallel-axis theorem)
        let ix_flange = 0.2 * 0.02_f64.powi(3) / 12.0 + 0.004 * (0.09 - yc).powi(2);
        let ix_web = 0.02 * 0.08_f64.powi(3) / 12.0 + 0.0016 * (0.04 - yc).powi(2);
        let ix = ix_flange + ix_web;
        let y_top = 0.10 - yc;
        let y_bot = yc;
        let z_plus = ix / y_top;
        let z_minus = ix / y_bot;
        assert!((props.zxx_plus - z_plus).abs() / z_plus < 1e-6);
        assert!((props.zxx_minus - z_minus).abs() / z_minus < 1e-6);
        assert!((props.area - area).abs() / area < 1e-10);
    }

    #[test]
    fn perimeter_rectangular() {
        let props = SectionProperties::from_section(&rectangle(0.1, 0.2));
        let expected = 2.0 * (0.1 + 0.2);
        assert!((props.perimeter - expected).abs() / expected < 1e-10);
    }

    #[test]
    fn global_moments_and_first_moments() {
        // Rectangle offset from origin
        let poly = Polygon::new(vec![
            Point::new(1.0, 2.0),
            Point::new(1.1, 2.0),
            Point::new(1.1, 2.2),
            Point::new(1.0, 2.2),
        ]);
        let section = Section::new(poly, vec![]);
        let props = SectionProperties::from_section(&section);

        let b = 0.1_f64;
        let h = 0.2_f64;
        let cx = 1.05_f64;
        let cy = 2.1_f64;
        let area = b * h;

        // First moments about global axes
        assert!((props.qx - area * cy).abs() / (area * cy) < 1e-10);
        assert!((props.qy - area * cx).abs() / (area * cx) < 1e-10);

        // Global second moments
        let ixx_g_expected = b * h.powi(3) / 12.0 + area * cy.powi(2);
        let iyy_g_expected = h * b.powi(3) / 12.0 + area * cx.powi(2);
        let ixy_g_expected = area * cx * cy; // Ixy_c=0 + A*cx*cy
        assert!((props.ixx_g - ixx_g_expected).abs() / ixx_g_expected < 1e-10);
        assert!((props.iyy_g - iyy_g_expected).abs() / iyy_g_expected < 1e-10);
        assert!((props.ixy_g - ixy_g_expected).abs() / ixy_g_expected < 1e-10);
    }

    #[test]
    fn principal_and_gyration_fields() {
        let props = SectionProperties::from_section(&rectangle(0.1, 0.2));
        let area = 0.1 * 0.2;

        // For rectangle: I11 = Ix (if h > b), I22 = Iy
        let i11_expected = 0.1 * 0.2_f64.powi(3) / 12.0;
        let i22_expected = 0.2 * 0.1_f64.powi(3) / 12.0;
        assert!((props.i11_c - i11_expected).abs() / i11_expected < 1e-10);
        assert!((props.i22_c - i22_expected).abs() / i22_expected < 1e-10);

        // Principal angle ~ 0 for doubly-symmetric section
        assert!(props.phi.abs() < 1e-10);

        // Radii of gyration
        assert!((props.rx_c - (props.ix / area).sqrt()).abs() < 1e-12);
        assert!((props.ry_c - (props.iy / area).sqrt()).abs() < 1e-12);
        assert!((props.r11_c - (props.i11_c / area).sqrt()).abs() < 1e-12);
        assert!((props.r22_c - (props.i22_c / area).sqrt()).abs() < 1e-12);
    }
}