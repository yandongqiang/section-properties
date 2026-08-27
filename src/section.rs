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
    /// Returns tuple: `(area, ixx, iyy, ixy, j, phi)`
    ///
    /// - `area`: cross-sectional area [m²]
    /// - `ixx`: second moment of area about x-axis [m⁴]
    /// - `iyy`: second moment of area about y-axis [m⁴]
    /// - `ixy`: product of inertia [m⁴]
    /// - `j`: St. Venant torsion constant [m⁴]
    /// - `phi`: angle from centroidal x-axis to major principal axis (11),
    ///   CCW positive, radians.  `phi = ½ atan2(2·Ixy, Ixx − Iyy)`.
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

    /// Frame properties with transformed-section analysis for a homogeneous
    /// material.
    ///
    /// Returns effective stiffnesses E·A, E·I, G·J and the mass per unit
    /// length. The geometric properties are identical to
    /// [`frame_properties_full`](Self::frame_properties_full) because a
    /// single-material section has modular ratio n = 1 everywhere.
    ///
    /// # Errors
    /// [`MaterialError::InvalidMaterial`] when the material fails validation.
    ///
    /// For multi-material (genuinely composite) sections use
    /// [`CompositeSection::transformed_properties`](crate::section_library::CompositeSection::transformed_properties),
    /// which applies per-group modular ratios.
    pub fn frame_properties_with_material(
        &self,
        material: &crate::material::Material,
    ) -> Result<TransformedFrameProperties, MaterialError> {
        if !material.is_valid() {
            return Err(MaterialError::InvalidMaterial(format!(
                "E={}, G={}, nu={}, rho={}",
                material.youngs_modulus,
                material.shear_modulus,
                material.poissons_ratio,
                material.density
            )));
        }

        let fp = self.frame_properties_full();
        let e = material.youngs_modulus;
        let g = e / (2.0 * (1.0 + material.poissons_ratio));

        Ok(TransformedFrameProperties {
            e_ref: e,
            area: fp.area,
            ea: e * fp.area,
            ei_xx: e * fp.ixx,
            ei_yy: e * fp.iyy,
            ei_w: e * fp.iw,
            gj: g * fp.j,
            mass_per_length: material.density * fp.area,
            geometric: fp,
        })
    }
}

/// Full frame (beam) analysis results, mirroring the 11-value tuple returned
/// by Python `Section.calculate_frame_properties()`.
#[derive(Debug, Clone, Copy)]
pub struct FrameProperties {
    /// Cross-sectional area.
    pub area: f64,
    /// Centroidal second moment about x.
    pub ixx: f64,
    /// Centroidal second moment about y.
    pub iyy: f64,
    /// Centroidal product of inertia.
    pub ixy: f64,
    /// St. Venant torsion constant.
    pub j: f64,
    /// Principal axis angle (radians, CCW from x).
    pub phi: f64,
    /// Major principal second moment.
    pub i11: f64,
    /// Minor principal second moment.
    pub i22: f64,
    /// Shear centre x-coordinate (global axes).
    pub delta_x: f64,
    /// Shear centre y-coordinate (global axes).
    pub delta_y: f64,
    /// Warping constant.
    pub iw: f64,
}

/// Error returned when a material cannot be used for transformed-section
/// analysis.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MaterialError {
    /// Material properties failed validation (non-positive E or G, invalid
    /// Poisson's ratio, negative density).
    InvalidMaterial(String),
}

impl std::fmt::Display for MaterialError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MaterialError::InvalidMaterial(reason) => {
                write!(f, "invalid material: {reason}")
            }
        }
    }
}

impl std::error::Error for MaterialError {}

/// Effective (E-weighted) frame properties from transformed-section analysis
/// with a single homogeneous material.
///
/// The geometric quantities (`area`, `ixx`, ...) are unchanged; the stiffness
/// terms carry the material modulus:
/// `ea = E·A`, `ei_x = E·Ixx`, ..., `ej = G·J` where `G = E / 2(1+nu)`.
///
/// For multi-material sections use
/// [`CompositeSection::transformed_properties`](crate::section_library::CompositeSection::transformed_properties)
/// which applies per-group modular ratios.
#[derive(Debug, Clone, Copy)]
pub struct TransformedFrameProperties {
    /// Reference modulus used for the transformation (= material's E).
    pub e_ref: f64,
    /// Geometric area (unchanged by the transformation for n=1).
    pub area: f64,
    /// Effective axial stiffness E·A.
    pub ea: f64,
    /// Effective bending stiffness about x, E·Ixx.
    pub ei_xx: f64,
    /// Effective bending stiffness about y, E·Iyy.
    pub ei_yy: f64,
    /// Effective warping stiffness, E·Iw.
    pub ei_w: f64,
    /// Effective torsional stiffness G·J with G = E/2(1+nu).
    pub gj: f64,
    /// Mass per unit length rho·A.
    pub mass_per_length: f64,
    /// Underlying geometric frame properties.
    pub geometric: FrameProperties,
}

impl Section {
    /// Full frame properties including shear centre and warping constant.
    ///
    /// Mirrors Python `Section.calculate_frame_properties()` (without the FEA
    /// shear areas, see `mesh` module for those).
    pub fn frame_properties_full(&self) -> FrameProperties {
        use crate::plastic::warping::WarpingProperties;
        use crate::section_properties::SectionProperties;

        let props = SectionProperties::from_section(self);
        let principal = props.principal_properties();
        let warping = WarpingProperties::from_section(self);

        FrameProperties {
            area: props.area,
            ixx: props.ix,
            iyy: props.iy,
            ixy: props.ixy,
            j: warping.j,
            phi: principal.phi,
            i11: principal.i11,
            i22: principal.i22,
            delta_x: warping.shear_center.x,
            delta_y: warping.shear_center.y,
            iw: warping.iw,
        }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::section_library::ParametricSection;

    #[test]
    fn frame_properties_full_i_section() {
        let i = crate::section_library::steel::ISection::from_designation("IPE300").unwrap();
        let sec = i.build();
        let fp = sec.frame_properties_full();
        assert!(fp.area > 0.0);
        assert!(fp.iw.abs() > 0.0, "I-section must have warping constant");
        // Doubly symmetric: shear centre at centroid (mesh-level tolerance)
        let c = sec.centroid();
        assert!((fp.delta_x - c.x).abs() < 5e-3);
        assert!((fp.delta_y - c.y).abs() < 5e-3);
        assert!((fp.j / (fp.area * 1e6)).abs() < 1e3); // sanity
    }

    #[test]
    fn geometry_from_points_rectangle() {
        let pts = vec![
            Point::new(0.0, 0.0),
            Point::new(0.2, 0.0),
            Point::new(0.2, 0.1),
            Point::new(0.0, 0.1),
        ];
        let g = crate::geometry::Geometry::from_points(pts, vec![]);
        assert!((g.area() - 0.02).abs() < 1e-12);

        let outer = vec![
            Point::new(0.0, 0.0),
            Point::new(4.0, 0.0),
            Point::new(4.0, 4.0),
            Point::new(0.0, 4.0),
        ];
        let hole = vec![
            Point::new(1.0, 1.0),
            Point::new(3.0, 1.0),
            Point::new(3.0, 3.0),
            Point::new(1.0, 3.0),
        ];
        let g2 = crate::geometry::Geometry::from_points(outer, vec![hole]);
        assert!((g2.area() - 12.0).abs() < 1e-12);
        #[test]
    fn frame_properties_with_material_rejects_invalid() {
        let sec = crate::section_library::primitive::RectangularSection::new(0.1, 0.2).build();
        let mut bad = crate::material::Material::default();
        bad.youngs_modulus = -1.0;
        assert!(sec.frame_properties_with_material(&bad).is_err());
    }

    #[test]
    fn transformed_frame_properties_scale_with_e() {
        let sec = crate::section_library::primitive::RectangularSection::new(0.1, 0.2).build();
        let geo = sec.frame_properties_full();

        let steel = crate::material::presets::STEEL_S355;
        let t_steel = sec.frame_properties_with_material(&steel).unwrap();
        assert!((t_steel.ea - steel.youngs_modulus * t_steel.area).abs() < 1e-6);
        assert!((t_steel.ei_xx - steel.youngs_modulus * geo.ixx).abs() < 1e-9);
        // G = E / 2(1+nu)
        let g_expected = steel.youngs_modulus / (2.0 * (1.0 + steel.poissons_ratio));
        assert!((t_steel.gj - g_expected * geo.j).abs() < 1e-3);

        // Aluminium: E roughly 1/3 of steel -> EI scales proportionally.
        let alu = crate::material::Material::new(69e9, 0.33, 2700.0, "alu");
        let t_alu = sec.frame_properties_with_material(&alu).unwrap();
        let ratio = t_alu.ei_xx / t_steel.ei_xx;
        assert!((ratio - alu.youngs_modulus / steel.youngs_modulus).abs() < 1e-9);
        // Mass uses density
        assert!((t_alu.mass_per_length - 2700.0 * t_alu.area).abs() < 1e-9);
    }}

}
