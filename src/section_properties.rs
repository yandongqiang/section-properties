use crate::geometry::{BoundaryExtrema, CompoundGeometry, Geometry, Point};
use crate::section::Section;
use std::ops::Deref;

/// Basic geometric properties about the centroidal and global axes.
#[derive(Debug, Clone, Copy)]
pub struct GeometricProperties {
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
}

impl Default for GeometricProperties {
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
            perimeter: 0.0,
            qx: 0.0,
            qy: 0.0,
            ixx_g: 0.0,
            iyy_g: 0.0,
            ixy_g: 0.0,
        }
    }
}

/// Principal-axis properties: moments of inertia, angle, and section moduli
/// about the principal 11-22 axes.
///
/// # Convention (matches standard Mohr's circle, diverges from Python)
///
/// This library uses the **standard structural-engineering convention** for
/// principal axes, consistent with Timoshenko, Pilkey, and most FEA codes:
///
/// ```text
///                    y (centroidal)
///                    |
///                    |     ╱ axis 11 (i11, major)
///                    |    ╱
///                    | φ ╱    φ = angle from x to axis 11 (CCW positive)
///                    |  ╱
///                    | ╱
///                    |╱─────────── x (centroidal)
///                   ╱|
///                  ╱ |
///     axis 22 ───╱  |    axis 22 is perpendicular to axis 11
///    (i22, minor)    |
/// ```
///
/// | Symbol | Meaning | Formula |
/// |--------|---------|---------|
/// | `phi` | Angle from centroidal x-axis to axis 11, CCW positive | `½ atan2(2·I_xy, I_xx − I_yy)` |
/// | `i11` | Major principal moment (largest) | `(I_xx+I_yy)/2 + Δ` |
/// | `i22` | Minor principal moment (smallest) | `(I_xx+I_yy)/2 − Δ` |
/// | `z11` | Section modulus about axis 11 | `i11 / max(|y₂|)` |
/// | `z22` | Section modulus about axis 22 | `i22 / max(|x₁|)` |
///
/// **Coordinate transformation** (global → principal):
/// ```text
/// x₁ = (x − x_c) cos φ + (y − y_c) sin φ    // along axis 11
/// y₂ = −(x − x_c) sin φ + (y − y_c) cos φ   // along axis 22
/// ```
///
/// **Section moduli** use the *perpendicular* distance from each axis:
/// - `z11 = i11 / |y₂|_max` (y₂ is perpendicular to axis 11)
/// - `z22 = i22 / |x₁|_max` (x₁ is perpendicular to axis 22)
///
/// ## Divergence from Python `sectionproperties`
///
/// Python computes `phi_py = atan2(I_xx − i11, I_xy)`, which equals
/// `−φ` (our phi).  Python's coordinate transform then projects onto
/// `(cos(−φ), sin(−φ))`, placing its "x11" axis along the *minor*
/// principal direction when `I_xx > I_yy` and `I_xy > 0`.  Despite this,
/// Python's `z11 = i11 / |y22_max|` still produces correct section
/// moduli because the perpendicular distance from the opposite axis is
/// the same after taking absolute values — but the **sign of phi** and
/// the **direction of x11/y22** differ.
///
/// To convert between conventions: `phi_python = −phi_rust`.
#[derive(Debug, Clone, Copy)]
pub struct PrincipalProperties {
    /// Major principal second moment of area (centroidal). Always `≥ i22`.
    pub i11: f64,

    /// Minor principal second moment of area (centroidal). Always `≤ i11`.
    pub i22: f64,

    /// Angle from the centroidal x-axis to axis 11 (major), measured
    /// **counter-clockwise positive** in radians.  Range: `[−π, π]`.
    ///
    /// Computed as `½ atan2(2·I_xy, I_xx − I_yy)` (standard Mohr's circle).
    pub phi: f64,

    /// Section modulus about axis 11 for positive extreme fibre.
    /// `z11_plus = i11 / |y₂_max|` where `y₂_max > 0`.
    pub z11_plus: f64,
    /// Section modulus about axis 11 for negative extreme fibre.
    /// `z11_minus = i11 / |y₂_min|` where `y₂_min < 0`.
    pub z11_minus: f64,
    /// Section modulus about axis 22 for positive extreme fibre.
    /// `z22_plus = i22 / |x₁_max|` where `x₁_max > 0`.
    pub z22_plus: f64,
    /// Section modulus about axis 22 for negative extreme fibre.
    /// `z22_minus = i22 / |x₁_min|` where `x₁_min < 0`.
    pub z22_minus: f64,
}

impl Default for PrincipalProperties {
    fn default() -> Self {
        Self {
            i11: 0.0,
            i22: 0.0,
            phi: 0.0,
            z11_plus: 0.0,
            z11_minus: 0.0,
            z22_plus: 0.0,
            z22_minus: 0.0,
        }
    }
}

/// Radii of gyration about the centroidal and principal axes.
#[derive(Debug, Clone, Copy)]
pub struct GyrationProperties {
    /// Radius of gyration about the centroidal x-axis.
    pub rx: f64,

    /// Radius of gyration about the centroidal y-axis.
    pub ry: f64,

    /// Radius of gyration about the principal 11-axis.
    pub r11: f64,

    /// Radius of gyration about the principal 22-axis.
    pub r22: f64,

    /// Polar radius of gyration.
    pub polar: f64,
}

impl Default for GyrationProperties {
    fn default() -> Self {
        Self {
            rx: 0.0,
            ry: 0.0,
            r11: 0.0,
            r22: 0.0,
            polar: 0.0,
        }
    }
}

/// Composite section properties: geometric + principal + gyration.
///
/// Field access is split across three sub-structs:
/// - [`GeometricProperties`] (via `Deref`: `props.area`, `props.ix`, …)
/// - [`PrincipalProperties`] (via `props.principal.i11`, `props.principal.phi`, …)
/// - [`GyrationProperties`] (via `props.gyration.rx`, `props.gyration.r11`, …)
#[derive(Debug, Clone, Copy, Default)]
pub struct SectionProperties {
    pub geometric: GeometricProperties,
    pub principal: PrincipalProperties,
    pub gyration: GyrationProperties,
}

impl Deref for SectionProperties {
    type Target = GeometricProperties;
    fn deref(&self) -> &GeometricProperties {
        &self.geometric
    }
}

impl SectionProperties {
    /// Compute section properties from a `CompoundGeometry` (one or more
    /// independent regions, each with an outer boundary and optional holes).
    ///
    /// Regions are aggregated using the composite-area (parallel-axis) method:
    /// each polygon contributes its signed area, first moments, and second
    /// moments about the global axes; the global centroid is then found and
    /// centroidal properties are obtained via the parallel-axis theorem.
    pub fn from_compound(compound: &CompoundGeometry) -> Self {
        let mut area = 0.0;
        let mut first_x = 0.0;
        let mut first_y = 0.0;
        let mut ix = 0.0;
        let mut iy = 0.0;
        let mut ixy = 0.0;

        // Collect every polygon (outer + holes) with transforms applied, so we
        // can iterate them once for moments and again for fibre distances.
        let polygons: Vec<crate::geometry::Polygon> = compound.polygons();

        // Polygon orientation convention (enforced by Section / Geometry):
        // outer -> CCW (positive signed area)
        // holes -> CW (negative signed area)
        // Therefore we use signed geometric quantities directly.
        for poly in &polygons {
            let signed_area = poly.signed_area();
            let centroid = poly.centroid();

            area += signed_area;
            first_x += signed_area * centroid.x;
            first_y += signed_area * centroid.y;

            ix += poly.moment_of_inertia_x();
            iy += poly.moment_of_inertia_y();
            ixy += poly.product_of_inertia_xy();
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
        let all_boundary: Vec<&dyn BoundaryExtrema> =
            polygons.iter().map(|p| p as &dyn BoundaryExtrema).collect();
        let (y_min, y_max) = all_boundary
            .iter()
            .map(|b| b.extreme_y(centroid))
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(mn, mx), (lo, hi)| {
                (mn.min(lo), mx.max(hi))
            });
        let (x_min, x_max) = all_boundary
            .iter()
            .map(|b| b.extreme_x(centroid))
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(mn, mx), (lo, hi)| {
                (mn.min(lo), mx.max(hi))
            });

        let max_fiber_y = y_max.abs().max(y_min.abs());
        let max_fiber_x = x_max.abs().max(x_min.abs());

        // Section moduli about centroidal axes (positive/negative fibres)
        let zxx_plus = if y_max.abs() > 1e-15 {
            ix_c / y_max.abs()
        } else {
            0.0
        };
        let zxx_minus = if y_min.abs() > 1e-15 {
            ix_c / y_min.abs()
        } else {
            0.0
        };
        let zyy_plus = if x_max.abs() > 1e-15 {
            iy_c / x_max.abs()
        } else {
            0.0
        };
        let zyy_minus = if x_min.abs() > 1e-15 {
            iy_c / x_min.abs()
        } else {
            0.0
        };

        // Principal axis properties (standard Mohr's circle convention).
        //
        // φ = ½ atan2(2·I_xy, I_xx − I_yy)
        //
        // This gives the angle from the centroidal x-axis to axis 11
        // (major principal axis), measured CCW positive.
        //
        // Coordinate transformation (global → principal):
        //   x₁ = dx·cos(φ) + dy·sin(φ)   // along axis 11
        //   y₂ = −dx·sin(φ) + dy·cos(φ)  // along axis 22 (⊥ to axis 11)
        //
        // Section moduli use perpendicular distances:
        //   z11 = i11 / |y₂|_max   (y₂ is distance from axis 11)
        //   z22 = i22 / |x₁|_max   (x₁ is distance from axis 22)
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

        // Project extreme boundary points onto principal axes.
        //
        // dir_11 = (cos φ, sin φ)  — unit vector along axis 11 (major)
        // dir_22 = (−sin φ, cos φ) — unit vector along axis 22 (minor, ⊥ to 11)
        //
        // x₁ = dot(P − centroid, dir_11)  — signed distance along axis 11
        // y₂ = dot(P − centroid, dir_22)  — signed distance along axis 22
        let dir_11 = Point::new(cos_phi, sin_phi);
        let dir_22 = Point::new(-sin_phi, cos_phi);
        let (x1_min, x1_max) = all_boundary
            .iter()
            .map(|b| b.extreme_distances(centroid, dir_11))
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(mn, mx), (lo, hi)| {
                (mn.min(lo), mx.max(hi))
            });
        let (y2_min, y2_max) = all_boundary
            .iter()
            .map(|b| b.extreme_distances(centroid, dir_22))
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(mn, mx), (lo, hi)| {
                (mn.min(lo), mx.max(hi))
            });

        // Section moduli: z = I / c
        //
        // For axis 11: c₁ = max|y₂| (perpendicular distance from axis 11)
        // For axis 22: c₂ = max|x₁| (perpendicular distance from axis 22)
        //
        // z11 = i11 / c₁, z22 = i22 / c₂
        let z11_plus = if y2_max.abs() > 1e-15 {
            i11 / y2_max.abs()
        } else {
            0.0
        };
        let z11_minus = if y2_min.abs() > 1e-15 {
            i11 / y2_min.abs()
        } else {
            0.0
        };
        let z22_plus = if x1_max.abs() > 1e-15 {
            i22 / x1_max.abs()
        } else {
            0.0
        };
        let z22_minus = if x1_min.abs() > 1e-15 {
            i22 / x1_min.abs()
        } else {
            0.0
        };

        // Perimeter (sum of outer + holes across all regions)
        let perimeter = polygons.iter().map(|p| p.perimeter()).sum::<f64>();

        // Radii of gyration
        let rx = (ix_c / area).sqrt();
        let ry = (iy_c / area).sqrt();
        let r11 = (i11 / area).sqrt();
        let r22 = (i22 / area).sqrt();

        Self {
            geometric: GeometricProperties {
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
                perimeter,
                qx: first_y,
                qy: first_x,
                ixx_g: ix,
                iyy_g: iy,
                ixy_g: ixy,
            },
            principal: PrincipalProperties {
                i11,
                i22,
                phi,
                z11_plus,
                z11_minus,
                z22_plus,
                z22_minus,
            },
            gyration: GyrationProperties {
                rx,
                ry,
                r11,
                r22,
                polar: ((ix_c + iy_c) / area).sqrt(),
            },
        }
    }

    /// Compute section properties from a single `Geometry` (one region with
    /// optional holes and transforms).
    pub fn from_geometry(geometry: &Geometry) -> Self {
        Self::from_compound(&CompoundGeometry::new(vec![geometry.clone()]))
    }

    /// Compute section properties from a `Section` (outer boundary + holes).
    ///
    /// This is a convenience wrapper that delegates to [`from_compound`]; the
    /// section is treated as a single-region compound geometry.
    pub fn from_section(section: &Section) -> Self {
        Self::from_compound(&CompoundGeometry::from(section.clone()))
    }

    /// Returns `(i11, i22, phi)`.
    ///
    /// - `i11`: major (largest) principal second moment of area.
    /// - `i22`: minor (smallest) principal second moment of area.
    /// - `phi`: angle from centroidal x-axis to axis 11, CCW positive, radians.
    pub fn principal_moments(&self) -> (f64, f64, f64) {
        (self.principal.i11, self.principal.i22, self.principal.phi)
    }

    pub fn principal_properties(&self) -> PrincipalProperties {
        self.principal
    }

    /// Radius of gyration about centroidal axes: (rx, ry, polar).
    pub fn radius_of_gyration(&self) -> (f64, f64, f64) {
        (self.gyration.rx, self.gyration.ry, self.gyration.polar)
    }

    pub fn gyration_properties(&self) -> GyrationProperties {
        self.gyration
    }

    /// Maximum fiber distance from centroid in Y direction (for section modulus).
    pub fn max_fiber_distance_y(&self) -> f64 {
        self.geometric.max_fiber_distance_y
    }

    /// Maximum fiber distance from centroid in X direction (for section modulus).
    pub fn max_fiber_distance_x(&self) -> f64 {
        self.geometric.max_fiber_distance_x
    }

    /// Elastic section modulus about the x-axis (minimum of plus/minus).
    pub fn section_modulus_x(&self) -> f64 {
        self.geometric.zxx_plus.min(self.geometric.zxx_minus)
    }

    /// Elastic section modulus about the y-axis (minimum of plus/minus).
    pub fn section_modulus_y(&self) -> f64 {
        self.geometric.zyy_plus.min(self.geometric.zyy_minus)
    }

    /// Section modulus about the 11-axis (minimum of plus/minus).
    pub fn section_modulus_11(&self) -> f64 {
        self.principal.z11_plus.min(self.principal.z11_minus)
    }

    /// Section modulus about the 22-axis (minimum of plus/minus).
    pub fn section_modulus_22(&self) -> f64 {
        self.principal.z22_plus.min(self.principal.z22_minus)
    }

    /// Format all computed properties as an aligned text table.
    ///
    /// Mirrors Python `Section.print_results()` (geometric + frame sections).
    pub fn format_results(&self) -> String {
        let mut s = String::new();
        let row = |s: &mut String, name: &str, value: f64| {
            s.push_str(&format!("  {:<24}{:>14.6e}\n", name, value));
        };

        s.push_str("Section Properties:\n");
        s.push_str("===================\n");
        row(&mut s, "Area", self.area);
        row(&mut s, "Perimeter", self.perimeter);
        s.push('\n');

        s.push_str("Centroid:\n");
        row(&mut s, "cx", self.centroid.x);
        row(&mut s, "cy", self.centroid.y);
        s.push('\n');

        s.push_str("Second Moments of Area:\n");
        row(&mut s, "Ixx (about cx)", self.ix);
        row(&mut s, "Iyy (about cy)", self.iy);
        row(&mut s, "Ixy", self.ixy);
        s.push('\n');
        s.push_str("Principal Axes:\n");
        row(&mut s, "I11 (major)", self.principal.i11);
        row(&mut s, "I22 (minor)", self.principal.i22);
        let phi_deg = self.principal.phi.to_degrees();
        row(&mut s, "Phi (x→11, CCW°)", phi_deg);
        s.push('\n');

        s.push_str("Section Moduli:\n");
        row(&mut s, "Zxx+", self.zxx_plus);
        row(&mut s, "Zxx-", self.zxx_minus);
        row(&mut s, "Zyy+", self.zyy_plus);
        row(&mut s, "Zyy-", self.zyy_minus);
        row(&mut s, "Z11+", self.principal.z11_plus);
        row(&mut s, "Z11-", self.principal.z11_minus);
        row(&mut s, "Z22+", self.principal.z22_plus);
        row(&mut s, "Z22-", self.principal.z22_minus);
        s.push('\n');

        s.push_str("Radii of Gyration:\n");
        row(&mut s, "rx", self.gyration.rx);
        row(&mut s, "ry", self.gyration.ry);
        row(&mut s, "r11", self.gyration.r11);
        row(&mut s, "r22", self.gyration.r22);
        row(&mut s, "rp (polar)", self.gyration.polar);

        s
    }

    /// Mass per unit length: area * density [kg/m].
    ///
    /// Mirrors Python `Section.get_mass()`.
    pub fn mass(&self, density: f64) -> f64 {
        self.area * density
    }
    /// Print the properties table to stdout.
    ///
    /// Mirrors Python `Section.print_results()`.
    pub fn print_results(&self) {
        print!("{}", self.format_results());
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
    fn format_results_contains_sections() {
        let props = SectionProperties::from_section(&rectangle(0.1, 0.2));
        let table = props.format_results();
        assert!(table.contains("Section Properties:"));
        assert!(table.contains("Second Moments of Area:"));
        assert!(table.contains("Radii of Gyration:"));
        // area row present with scientific notation
        assert!(table.contains("Area"));
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
        assert!((props.principal.z11_plus - props.zxx_plus).abs() / props.zxx_plus < 1e-10);
        assert!((props.principal.z11_minus - props.zxx_minus).abs() / props.zxx_minus < 1e-10);
        assert!((props.principal.z22_plus - props.zyy_plus).abs() / props.zyy_plus < 1e-10);
        assert!((props.principal.z22_minus - props.zyy_minus).abs() / props.zyy_minus < 1e-10);
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
        assert!((props.principal.i11 - i11_expected).abs() / i11_expected < 1e-10);
        assert!((props.principal.i22 - i22_expected).abs() / i22_expected < 1e-10);

        // Principal angle ~ 0 for doubly-symmetric section
        assert!(props.principal.phi.abs() < 1e-10);

        // Radii of gyration
        assert!((props.gyration.rx - (props.ix / area).sqrt()).abs() < 1e-12);
        assert!((props.gyration.ry - (props.iy / area).sqrt()).abs() < 1e-12);
        assert!((props.gyration.r11 - (props.principal.i11 / area).sqrt()).abs() < 1e-12);
        assert!((props.gyration.r22 - (props.principal.i22 / area).sqrt()).abs() < 1e-12);
    }

    // ---- CompoundGeometry / multi-region tests ----

    use crate::geometry::{CompoundGeometry, Geometry};

    fn rect_polygon(width: f64, height: f64, cx: f64, cy: f64) -> Polygon {
        let hw = width * 0.5;
        let hh = height * 0.5;
        Polygon::new(vec![
            Point::new(cx - hw, cy - hh),
            Point::new(cx + hw, cy - hh),
            Point::new(cx + hw, cy + hh),
            Point::new(cx - hw, cy + hh),
        ])
    }

    #[test]
    fn from_section_and_from_compound_agree_for_single_region() {
        let section = rectangle(0.1, 0.2);
        let props_section = SectionProperties::from_section(&section);
        let compound = CompoundGeometry::from(section.clone());
        let props_compound = SectionProperties::from_compound(&compound);
        assert!((props_section.area - props_compound.area).abs() < 1e-15);
        assert!((props_section.ix - props_compound.ix).abs() < 1e-15);
        assert!((props_section.iy - props_compound.iy).abs() < 1e-15);
        assert!((props_section.ixy - props_compound.ixy).abs() < 1e-15);
        assert!((props_section.centroid.x - props_compound.centroid.x).abs() < 1e-15);
        assert!((props_section.centroid.y - props_compound.centroid.y).abs() < 1e-15);
    }

    #[test]
    fn from_geometry_matches_from_section() {
        let section = rectangle(0.1, 0.2);
        let props_section = SectionProperties::from_section(&section);
        let geometry = Geometry::from_section(&section);
        let props_geometry = SectionProperties::from_geometry(&geometry);
        assert!((props_section.area - props_geometry.area).abs() < 1e-15);
        assert!((props_section.ix - props_geometry.ix).abs() < 1e-15);
        assert!((props_section.iy - props_geometry.iy).abs() < 1e-15);
    }

    #[test]
    fn two_disjoint_rectangles_area_and_centroid() {
        // Two 0.1 x 0.2 rectangles side by side, gap 0.05.
        // Left centroid at x=-0.075, right at x=+0.075.
        let g1 = Geometry::new(rect_polygon(0.1, 0.2, -0.075, 0.0), vec![]);
        let g2 = Geometry::new(rect_polygon(0.1, 0.2, 0.075, 0.0), vec![]);
        let compound = CompoundGeometry::new(vec![g1, g2]);
        let props = SectionProperties::from_compound(&compound);

        let area_each = 0.1 * 0.2;
        assert!((props.area - 2.0 * area_each).abs() / (2.0 * area_each) < 1e-12);
        // Symmetric about y-axis => centroid at origin
        assert!(props.centroid.x.abs() < 1e-12);
        assert!(props.centroid.y.abs() < 1e-12);
    }

    #[test]
    fn two_disjoint_rectangles_moment_of_inertia() {
        // Two 0.1 x 0.2 rectangles, centroids at x = ±0.075.
        // Iy_total = 2 * (Iy_local + A * d²) where d = 0.075.
        let g1 = Geometry::new(rect_polygon(0.1, 0.2, -0.075, 0.0), vec![]);
        let g2 = Geometry::new(rect_polygon(0.1, 0.2, 0.075, 0.0), vec![]);
        let compound = CompoundGeometry::new(vec![g1, g2]);
        let props = SectionProperties::from_compound(&compound);

        let b = 0.1_f64;
        let h = 0.2_f64;
        let a = b * h;
        let d = 0.075_f64;
        let iy_local = h * b.powi(3) / 12.0;
        let iy_expected = 2.0 * (iy_local + a * d * d);
        assert!((props.iy - iy_expected).abs() / iy_expected < 1e-10);

        // Ix: both share same y-centroid (0), so Ix = 2 * Ix_local
        let ix_local = b * h.powi(3) / 12.0;
        let ix_expected = 2.0 * ix_local;
        assert!((props.ix - ix_expected).abs() / ix_expected < 1e-10);
    }

    #[test]
    fn compound_with_hole_in_one_region() {
        // Region 1: solid 0.2 x 0.2 square centred at origin.
        // Region 2: 0.2 x 0.2 square with a 0.1 x 0.1 hole, centred at (0.3, 0).
        let outer1 = rect_polygon(0.2, 0.2, 0.0, 0.0);
        let g1 = Geometry::new(outer1, vec![]);

        let outer2 = rect_polygon(0.2, 0.2, 0.3, 0.0);
        let hole2 = rect_polygon(0.1, 0.1, 0.3, 0.0);
        let g2 = Geometry::new(outer2, vec![hole2]);

        let compound = CompoundGeometry::new(vec![g1, g2]);
        let props = SectionProperties::from_compound(&compound);

        let area_expected = 0.2 * 0.2 + (0.2 * 0.2 - 0.1 * 0.1);
        assert!((props.area - area_expected).abs() / area_expected < 1e-10);

        // Centroid: region 1 area 0.04 at x=0, region 2 net area 0.03 at x=0.3
        let cx_expected = (0.04 * 0.0 + 0.03 * 0.3) / 0.07;
        assert!((props.centroid.x - cx_expected).abs() < 1e-12);
        assert!(props.centroid.y.abs() < 1e-12);
    }

    #[test]
    fn compound_with_transform_translate() {
        // A 0.1 x 0.2 rectangle built at origin, then translated to (1.0, 2.0).
        let mut g = Geometry::new(rect_polygon(0.1, 0.2, 0.0, 0.0), vec![]);
        g.transforms
            .push(crate::geometry::Transform::Translate { dx: 1.0, dy: 2.0 });
        let compound = CompoundGeometry::new(vec![g]);
        let props = SectionProperties::from_compound(&compound);

        assert!((props.centroid.x - 1.0).abs() < 1e-12);
        assert!((props.centroid.y - 2.0).abs() < 1e-12);
        assert!((props.area - 0.1 * 0.2).abs() < 1e-15);
    }

    #[test]
    fn compound_with_transform_rotate_preserves_area_and_ix() {
        // Rotating a square about its centroid preserves area and Ix == Iy.
        let mut g = Geometry::new(rect_polygon(0.2, 0.2, 0.0, 0.0), vec![]);
        g.transforms
            .push(crate::geometry::Transform::Rotate { angle: 0.3 });
        let compound = CompoundGeometry::new(vec![g]);
        let props = SectionProperties::from_compound(&compound);

        assert!((props.area - 0.04).abs() < 1e-15);
        // For a square, Ix == Iy regardless of rotation.
        assert!((props.ix - props.iy).abs() / props.ix < 1e-10);
    }

    #[test]
    fn compound_built_up_section_parallel_axis() {
        // Built-up section: two flanges 0.2 x 0.02 at y = ±0.09,
        // web 0.02 x 0.16 at x = 0.  This forms an I-section.
        let flange_top = Geometry::new(rect_polygon(0.2, 0.02, 0.0, 0.09), vec![]);
        let flange_bot = Geometry::new(rect_polygon(0.2, 0.02, 0.0, -0.09), vec![]);
        let web = Geometry::new(rect_polygon(0.02, 0.16, 0.0, 0.0), vec![]);
        let compound = CompoundGeometry::new(vec![flange_top, flange_bot, web]);
        let props = SectionProperties::from_compound(&compound);

        // Total area
        let area_expected = 2.0 * (0.2 * 0.02) + 0.02 * 0.16;
        assert!((props.area - area_expected).abs() / area_expected < 1e-12);

        // Doubly symmetric => centroid at origin
        assert!(props.centroid.x.abs() < 1e-12);
        assert!(props.centroid.y.abs() < 1e-12);

        // Ix via parallel-axis theorem
        let ix_flange = 0.2 * 0.02_f64.powi(3) / 12.0 + 0.2 * 0.02 * 0.09_f64.powi(2);
        let ix_web = 0.02 * 0.16_f64.powi(3) / 12.0;
        let ix_expected = 2.0 * ix_flange + ix_web;
        assert!((props.ix - ix_expected).abs() / ix_expected < 1e-10);
    }

    // ---- L / T / I / rotation / multi-hole test suite ----

    fn l_section() -> Section {
        // L-section: 0.1 x 0.1 square with top-right 0.08 x 0.08 removed.
        // Horizontal leg: 0.1 x 0.02 at bottom.
        // Vertical leg: 0.02 x 0.1 at left.
        let poly = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(0.1, 0.0),
            Point::new(0.1, 0.02),
            Point::new(0.02, 0.02),
            Point::new(0.02, 0.1),
            Point::new(0.0, 0.1),
        ]);
        Section::new(poly, vec![])
    }

    fn i_section() -> Section {
        // I-section: flanges 0.2 x 0.02 at y=±0.09, web 0.02 x 0.16.
        let poly = Polygon::new(vec![
            Point::new(-0.1, 0.10),
            Point::new(0.1, 0.10),
            Point::new(0.1, 0.08),
            Point::new(0.01, 0.08),
            Point::new(0.01, -0.08),
            Point::new(0.1, -0.08),
            Point::new(0.1, -0.10),
            Point::new(-0.1, -0.10),
            Point::new(-0.1, -0.08),
            Point::new(-0.01, -0.08),
            Point::new(-0.01, 0.08),
            Point::new(-0.1, 0.08),
        ]);
        Section::new(poly, vec![])
    }

    #[test]
    fn l_section_area_and_centroid() {
        let props = SectionProperties::from_section(&l_section());
        // Area = 0.1*0.02 + 0.02*0.08 = 0.0036
        let area_expected = 0.1 * 0.02 + 0.02 * 0.08;
        assert!((props.area - area_expected).abs() / area_expected < 1e-10);

        // Centroid by composite method:
        // horizontal leg (0.1 x 0.02): A=0.002, cx=0.05, cy=0.01
        // vertical leg  (0.02 x 0.08): A=0.0016, cx=0.01, cy=0.06
        let cx = (0.002 * 0.05 + 0.0016 * 0.01) / area_expected;
        let cy = (0.002 * 0.01 + 0.0016 * 0.06) / area_expected;
        assert!((props.centroid.x - cx).abs() < 1e-12);
        assert!((props.centroid.y - cy).abs() < 1e-12);
    }

    #[test]
    fn l_section_has_nonzero_ixy_and_rotated_principal_axes() {
        let props = SectionProperties::from_section(&l_section());
        // L-section is asymmetric => Ixy != 0
        assert!(
            props.ixy.abs() > 1e-8,
            "L-section should have non-zero Ixy, got {}",
            props.ixy
        );
        // Principal angle should be non-zero and not 90°
        let phi = props.principal.phi;
        assert!(
            phi.abs() > 1e-8,
            "L-section principal angle should be non-zero"
        );
        assert!(
            (phi.abs() - std::f64::consts::FRAC_PI_2).abs() > 1e-8,
            "L-section principal angle should not be 90°"
        );
    }

    #[test]
    fn l_section_principal_invariants() {
        let props = SectionProperties::from_section(&l_section());
        // Invariant: I11 + I22 = Ix + Iy
        let sum_centroidal = props.ix + props.iy;
        let sum_principal = props.principal.i11 + props.principal.i22;
        assert!((sum_principal - sum_centroidal).abs() / sum_centroidal < 1e-10);

        // Invariant: I11 * I22 = Ix * Iy - Ixy²
        let det_centroidal = props.ix * props.iy - props.ixy.powi(2);
        let det_principal = props.principal.i11 * props.principal.i22;
        assert!((det_principal - det_centroidal).abs() / det_centroidal.abs() < 1e-10);

        // I11 >= I22
        assert!(props.principal.i11 >= props.principal.i22);
    }

    #[test]
    fn t_section_single_symmetric_ixy_zero() {
        let props = SectionProperties::from_section(&tee_section());
        // T-section is symmetric about y-axis => Ixy = 0
        assert!(props.ixy.abs() < 1e-12);

        // zxx_plus != zxx_minus (not symmetric about x)
        assert!((props.zxx_plus - props.zxx_minus).abs() > 1e-6);

        // zyy_plus == zyy_minus (symmetric about y)
        assert!((props.zyy_plus - props.zyy_minus).abs() < 1e-12);

        // Ixy = 0 => principal angle is 0 (if Ix > Iy) or π/2 (if Iy > Ix).
        // For this T-section, Iy > Ix (flange is wide in x), so phi ≈ π/2.
        let phi = props.principal.phi;
        assert!(
            phi.abs() < 1e-10 || (phi.abs() - std::f64::consts::FRAC_PI_2).abs() < 1e-10,
            "phi should be 0 or π/2 for Ixy=0, got {}",
            phi
        );
    }

    #[test]
    fn i_section_doubly_symmetric() {
        let props = SectionProperties::from_section(&i_section());

        // Centroid at origin
        assert!(props.centroid.x.abs() < 1e-12);
        assert!(props.centroid.y.abs() < 1e-12);

        // Ixy = 0
        assert!(props.ixy.abs() < 1e-12);

        // Symmetric => zxx_plus == zxx_minus, zyy_plus == zyy_minus
        assert!((props.zxx_plus - props.zxx_minus).abs() / props.zxx_plus < 1e-10);
        assert!((props.zyy_plus - props.zyy_minus).abs() / props.zyy_plus < 1e-10);

        // Principal angle ~ 0
        assert!(props.principal.phi.abs() < 1e-10);

        // Principal moments == centroidal moments (Ixy = 0)
        assert!((props.principal.i11 - props.ix).abs() / props.ix < 1e-10);
        assert!((props.principal.i22 - props.iy).abs() / props.iy < 1e-10);
    }

    #[test]
    fn i_section_area_and_ix_analytical() {
        let props = SectionProperties::from_section(&i_section());
        let area_expected = 2.0 * (0.2 * 0.02) + 0.02 * 0.16;
        assert!((props.area - area_expected).abs() / area_expected < 1e-10);

        // Ix = 2 * (bf*tf³/12 + bf*tf*d²) + tw*hw³/12
        let ix_flange = 0.2 * 0.02_f64.powi(3) / 12.0 + 0.2 * 0.02 * 0.09_f64.powi(2);
        let ix_web = 0.02 * 0.16_f64.powi(3) / 12.0;
        let ix_expected = 2.0 * ix_flange + ix_web;
        assert!((props.ix - ix_expected).abs() / ix_expected < 1e-10);
    }

    #[test]
    fn rotated_rectangle_principal_angle() {
        // Rectangle 0.2 x 0.1 rotated by 30°.
        // After rotation, Ixy != 0 and principal angle ≈ 30°.
        let angle = std::f64::consts::PI / 6.0; // 30°
        let (s, c) = angle.sin_cos();
        let hw = 0.1_f64;
        let hh = 0.05_f64;
        let rotate = |x: f64, y: f64| Point::new(x * c - y * s, x * s + y * c);
        let poly = Polygon::new(vec![
            rotate(-hw, -hh),
            rotate(hw, -hh),
            rotate(hw, hh),
            rotate(-hw, hh),
        ]);
        let section = Section::new(poly, vec![]);
        let props = SectionProperties::from_section(&section);

        // Ixy should be non-zero after rotation
        assert!(
            props.ixy.abs() > 1e-6,
            "rotated rectangle should have Ixy != 0"
        );

        // Principal angle should be ≈ ±30° (or ±30° + 90°)
        let phi = props.principal.phi;
        let deg = phi.to_degrees();
        let deg_abs = deg.abs();
        // phi could be 30° or -60° (atan2 ambiguity), both valid
        assert!(
            (deg_abs - 30.0).abs() < 1.0 || (deg_abs - 60.0).abs() < 1.0,
            "principal angle should be near 30° or 60°, got {}°",
            deg
        );
    }

    #[test]
    fn rotated_square_ix_equals_iy() {
        // A square rotated by any angle has Ix == Iy.
        let angle: f64 = 0.4; // arbitrary
        let (s, c) = angle.sin_cos();
        let hw = 0.1_f64;
        let rotate = |x: f64, y: f64| Point::new(x * c - y * s, x * s + y * c);
        let poly = Polygon::new(vec![
            rotate(-hw, -hw),
            rotate(hw, -hw),
            rotate(hw, hw),
            rotate(-hw, hw),
        ]);
        let section = Section::new(poly, vec![]);
        let props = SectionProperties::from_section(&section);

        assert!((props.ix - props.iy).abs() / props.ix < 1e-10);
        // Ixy should be ~0 for a square (I11 == I22 => Ixy doesn't matter)
        // Actually for a square Ixy may not be zero after rotation, but I11==I22
        assert!((props.principal.i11 - props.principal.i22).abs() / props.principal.i11 < 1e-10);
    }

    #[test]
    fn multi_hole_section_area() {
        // 0.4 x 0.4 square with two 0.05 x 0.05 square holes
        // at (±0.1, 0).
        let outer = rect_polygon(0.4, 0.4, 0.0, 0.0);
        let hole1 = rect_polygon(0.05, 0.05, 0.1, 0.0);
        let hole2 = rect_polygon(0.05, 0.05, -0.1, 0.0);
        let section = Section::new(outer, vec![hole1, hole2]);
        let props = SectionProperties::from_section(&section);

        let area_expected = 0.4 * 0.4 - 2.0 * 0.05 * 0.05;
        assert!((props.area - area_expected).abs() / area_expected < 1e-10);
    }

    #[test]
    fn multi_hole_section_centroid_at_origin() {
        // Symmetric placement of holes => centroid at origin.
        let outer = rect_polygon(0.4, 0.4, 0.0, 0.0);
        let hole1 = rect_polygon(0.05, 0.05, 0.1, 0.0);
        let hole2 = rect_polygon(0.05, 0.05, -0.1, 0.0);
        let section = Section::new(outer, vec![hole1, hole2]);
        let props = SectionProperties::from_section(&section);

        assert!(props.centroid.x.abs() < 1e-12);
        assert!(props.centroid.y.abs() < 1e-12);
    }

    #[test]
    fn multi_hole_section_ix_parallel_axis() {
        // Ix = Ix_outer - 2 * (Ix_hole_local + A_hole * d²)
        // where d = 0 (holes on x-axis), so Ix = Ix_outer - 2 * Ix_hole_local
        let outer = rect_polygon(0.4, 0.4, 0.0, 0.0);
        let hole1 = rect_polygon(0.05, 0.05, 0.1, 0.0);
        let hole2 = rect_polygon(0.05, 0.05, -0.1, 0.0);
        let section = Section::new(outer, vec![hole1, hole2]);
        let props = SectionProperties::from_section(&section);

        let ix_outer = 0.4 * 0.4_f64.powi(3) / 12.0;
        let ix_hole = 0.05 * 0.05_f64.powi(3) / 12.0;
        let ix_expected = ix_outer - 2.0 * ix_hole;
        assert!((props.ix - ix_expected).abs() / ix_expected < 1e-10);
    }

    #[test]
    fn multi_hole_section_iy_parallel_axis() {
        // Iy = Iy_outer - 2 * (Iy_hole_local + A_hole * d²)
        // where d = 0.1 (holes at x = ±0.1)
        let outer = rect_polygon(0.4, 0.4, 0.0, 0.0);
        let hole1 = rect_polygon(0.05, 0.05, 0.1, 0.0);
        let hole2 = rect_polygon(0.05, 0.05, -0.1, 0.0);
        let section = Section::new(outer, vec![hole1, hole2]);
        let props = SectionProperties::from_section(&section);

        let iy_outer = 0.4 * 0.4_f64.powi(3) / 12.0;
        let iy_hole_local = 0.05 * 0.05_f64.powi(3) / 12.0;
        let a_hole = 0.05 * 0.05;
        let iy_expected = iy_outer - 2.0 * (iy_hole_local + a_hole * 0.1_f64.powi(2));
        assert!((props.iy - iy_expected).abs() / iy_expected < 1e-10);
    }

    #[test]
    fn multi_hole_section_perimeter() {
        // Perimeter = outer + 2 * hole
        let outer = rect_polygon(0.4, 0.4, 0.0, 0.0);
        let hole1 = rect_polygon(0.05, 0.05, 0.1, 0.0);
        let hole2 = rect_polygon(0.05, 0.05, -0.1, 0.0);
        let section = Section::new(outer, vec![hole1, hole2]);
        let props = SectionProperties::from_section(&section);

        let p_expected = 4.0 * 0.4 + 2.0 * 4.0 * 0.05;
        assert!((props.perimeter - p_expected).abs() / p_expected < 1e-10);
    }

    #[test]
    fn compound_l_plus_rectangle_two_regions() {
        // An L-section built as two disjoint rectangles (no shared edge).
        // Horizontal leg: 0.1 x 0.02 at (0.05, 0.01)
        // Vertical leg: 0.02 x 0.08 at (0.01, 0.06)
        let g1 = Geometry::new(rect_polygon(0.1, 0.02, 0.05, 0.01), vec![]);
        let g2 = Geometry::new(rect_polygon(0.02, 0.08, 0.01, 0.06), vec![]);
        let compound = CompoundGeometry::new(vec![g1, g2]);
        let props = SectionProperties::from_compound(&compound);

        // Compare with the single-polygon L-section
        let l_props = SectionProperties::from_section(&l_section());

        assert!((props.area - l_props.area).abs() / l_props.area < 1e-10);
        assert!((props.centroid.x - l_props.centroid.x).abs() < 1e-12);
        assert!((props.centroid.y - l_props.centroid.y).abs() < 1e-12);
        assert!((props.ix - l_props.ix).abs() / l_props.ix < 1e-10);
        assert!((props.iy - l_props.iy).abs() / l_props.iy < 1e-10);
        assert!((props.ixy - l_props.ixy).abs() / props.ixy.abs() < 1e-10);
    }

    /// Non-symmetric hole: rectangle 200×100mm with 40×30mm hole at (120, 65).
    ///
    /// ```text
    /// ┌───────────────────┐
    /// │                   │
    /// │         ┌──┐      │
    /// │         │  │      │
    /// │         └──┘      │
    /// └───────────────────┘
    /// ```
    ///
    /// Hand-computed in SI (metres, m⁴) — verified against Python
    /// `sectionproperties` output.  This exercises centroid shift, Ixy ≠ 0,
    /// and a principal angle near −90°.
    #[test]
    fn non_symmetric_hole_all_properties() {
        let outer = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(0.2, 0.0),
            Point::new(0.2, 0.1),
            Point::new(0.0, 0.1),
        ]);
        // Hole: 40 mm × 30 mm, centered at (120, 65)
        let hole = Polygon::new(vec![
            Point::new(0.10, 0.05),
            Point::new(0.14, 0.05),
            Point::new(0.14, 0.08),
            Point::new(0.10, 0.08),
        ]);
        let section = Section::new(outer, vec![hole]);
        let props = SectionProperties::from_section(&section);

        // --- Area ---
        // A = 0.2×0.1 − 0.04×0.03 = 0.0188
        let a_expected = 0.0188_f64;
        assert!(
            (props.area - a_expected).abs() / a_expected < 1e-12,
            "area: got {}, expected {}",
            props.area,
            a_expected
        );

        // --- Centroid ---
        // Qx = 0.02·0.05 − 0.0012·0.065 = 0.001 − 7.8e-5 = 9.22e-4
        // Qy = 0.02·0.1  − 0.0012·0.12  = 0.002 − 1.44e-4 = 1.856e-3
        // cx = Qy / A = 1.856e-3 / 0.0188 ≈ 0.098723
        // cy = Qx / A = 9.22e-4  / 0.0188 ≈ 0.049043
        let cx_exp = 0.001_856 / a_expected;
        let cy_exp = 0.000_922 / a_expected;
        assert!(
            (props.centroid.x - cx_exp).abs() < 1e-12,
            "cx: got {}, expected {}",
            props.centroid.x,
            cx_exp
        );
        assert!(
            (props.centroid.y - cy_exp).abs() < 1e-12,
            "cy: got {}, expected {}",
            props.centroid.y,
            cy_exp
        );

        // --- Centroidal second moments (parallel-axis theorem) ---
        //
        // Outer (centroid at (0.1, 0.05)):
        //   Ixx_g = 0.2·0.1³/12 + 0.02·0.05² = 1.6667e-5 + 5e-5
        //   Iyy_g = 0.1·0.2³/12 + 0.02·0.1²  = 6.6667e-5 + 2e-4
        //   Ixy_g = 0 + 0.02·0.1·0.05          = 1e-4
        //
        // Hole (centroid at (0.12, 0.065)):
        //   Ixx_g = 0.04·0.03³/12 + 0.0012·0.065² = 9e-8 + 5.07e-6
        //   Iyy_g = 0.03·0.04³/12 + 0.0012·0.12²  = 1.6e-7 + 1.728e-5
        //   Ixy_g = 0 + 0.0012·0.12·0.065           = 9.36e-6
        //
        // Net global:
        let ixx_g = 6.666666666666667e-5_f64 - 5.160000000000000e-6;
        let iyy_g = 2.666666666666667e-4_f64 - 1.744000000000000e-5;
        let ixy_g = 1.000000000000000e-4_f64 - 9.360000000000000e-6;
        // Shift to centroid:
        let cx = props.centroid.x;
        let cy = props.centroid.y;
        let ixx_c = ixx_g - a_expected * cy * cy;
        let iyy_c = iyy_g - a_expected * cx * cx;
        let ixy_c = ixy_g - a_expected * cx * cy;

        assert!(
            (props.ix - ixx_c).abs() / ixx_c.abs() < 1e-10,
            "Ixx: got {}, expected {}",
            props.ix,
            ixx_c
        );
        assert!(
            (props.iy - iyy_c).abs() / iyy_c.abs() < 1e-10,
            "Iyy: got {}, expected {}",
            props.iy,
            iyy_c
        );
        assert!(
            (props.ixy - ixy_c).abs() / ixy_c.abs().max(1e-15) < 1e-6,
            "Ixy: got {}, expected {}",
            props.ixy,
            ixy_c
        );

        // --- Principal moments ---
        let avg = (ixx_c + iyy_c) * 0.5;
        let diff = (ixx_c - iyy_c) * 0.5;
        let delta = (diff * diff + ixy_c * ixy_c).sqrt();
        let i11_exp = avg + delta;
        let i22_exp = avg - delta;

        assert!(
            (props.principal.i11 - i11_exp).abs() / i11_exp < 1e-10,
            "I11: got {}, expected {}",
            props.principal.i11,
            i11_exp
        );
        assert!(
            (props.principal.i22 - i22_exp).abs() / i22_exp < 1e-10,
            "I22: got {}, expected {}",
            props.principal.i22,
            i22_exp
        );

        // --- Invariants ---
        // I11 + I22 == Ixx + Iyy
        let sum_c = ixx_c + iyy_c;
        let sum_p = props.principal.i11 + props.principal.i22;
        assert!(
            (sum_p - sum_c).abs() / sum_c < 1e-10,
            "invariant sum: got {}, expected {}",
            sum_p,
            sum_c
        );
        // I11 · I22 == Ixx·Iyy − Ixy²
        let det_c = ixx_c * iyy_c - ixy_c * ixy_c;
        let det_p = props.principal.i11 * props.principal.i22;
        assert!(
            (det_p - det_c).abs() / det_c.abs() < 1e-10,
            "invariant product: got {}, expected {}",
            det_p,
            det_c
        );

        // --- Principal angle ---
        // phi = 0.5 · atan2(2·Ixy, Ixx − Iyy)
        //     ≈ −89.56° (nearly vertical principal axis due to wide flange)
        let phi_exp = 0.5 * (2.0 * ixy_c).atan2(ixx_c - iyy_c);
        assert!(
            (props.principal.phi - phi_exp).abs() < 1e-12,
            "phi: got {} rad ({}°), expected {} rad ({}°)",
            props.principal.phi,
            props.principal.phi.to_degrees(),
            phi_exp,
            phi_exp.to_degrees()
        );
        // Cross-check: phi ≈ −π/2 (within 1°)
        assert!(
            (props.principal.phi + std::f64::consts::FRAC_PI_2).abs() < (1.0_f64).to_radians(),
            "phi should be near −π/2, got {}°",
            props.principal.phi.to_degrees()
        );
    }
}
