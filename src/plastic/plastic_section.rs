//! Plastic section analysis.
//!
//! Provides plastic neutral axis search, plastic section modulus,
//! plastic moment capacity, and interaction diagrams.

use crate::geometry::Point;
use crate::material::Material;
use crate::section::Section;
use crate::section_properties::SectionProperties;

/// Plastic section analysis for a given cross-section.
#[derive(Debug, Clone)]
pub struct PlasticSection {
    pub section: Section,
    pub material: Material,
    /// Number of fibers for numerical integration
    pub n_fibers: usize,
}

impl PlasticSection {
    /// Create a new plastic section analysis.
    pub fn new(section: Section, material: Material) -> Self {
        Self {
            section,
            material,
            n_fibers: 1000,
        }
    }

    /// Set number of fibers for numerical integration.
    pub fn with_fibers(mut self, n: usize) -> Self {
        self.n_fibers = n.max(100);
        self
    }

    /// Compute plastic properties about a given axis.
    pub fn plastic_properties(&self, axis: PlasticAxis) -> PlasticProperties {
        let fy = self.material.yield_strength;
        let pna = self.find_plastic_neutral_axis(axis, fy);

        let (z_pl, s_pl) = self.plastic_modulus_about_axis(axis, &pna, fy);

        PlasticProperties {
            axis,
            plastic_neutral_axis: pna,
            plastic_section_modulus: z_pl,
            plastic_moment_capacity: s_pl,
            yield_moment: self.yield_moment(axis),
        }
    }

    /// Find plastic neutral axis (PNA) for a given bending axis.
    ///
    /// Uses exact polygon clipping (Sutherland-Hodgman) to compute areas,
    /// mirroring Python's `split_section` + `brentq` approach.
    pub fn find_plastic_neutral_axis(&self, axis: PlasticAxis, _fy: f64) -> PlasticNeutralAxis {
        let (min_coord, max_coord) = self.section_bounds(axis);

        let total_area = self.section.area();
        let target_area = total_area / 2.0;

        // Brent-like root finding using exact polygon areas
        let mut low = min_coord;
        let mut high = max_coord;
        let mut pna_pos = (low + high) / 2.0;

        for _ in 0..80 {
            pna_pos = (low + high) / 2.0;
            let area_above = exact::exact_area_on_side(&self.section, axis, pna_pos, true);

            if area_above > target_area {
                low = pna_pos;
            } else {
                high = pna_pos;
            }

            if (high - low).abs() < 1e-12 {
                break;
            }
        }

        PlasticNeutralAxis {
            position: pna_pos,
            axis,
            area_above: exact::exact_area_on_side(&self.section, axis, pna_pos, true),
            area_below: exact::exact_area_on_side(&self.section, axis, pna_pos, false),
        }
    }

    /// Compute plastic section modulus about an axis given PNA.
    ///
    /// Uses exact polygon clipping (mirroring Python's `split_section` approach).
    fn plastic_modulus_about_axis(
        &self,
        axis: PlasticAxis,
        pna: &PlasticNeutralAxis,
        fy: f64,
    ) -> (f64, f64) {
        let z_pl = exact::exact_plastic_modulus(&self.section, axis, pna.position);
        let s_pl = z_pl * fy;
        (z_pl, s_pl)
    }

    /// Elastic yield moment (first yield).
    fn yield_moment(&self, axis: PlasticAxis) -> f64 {
        let props = SectionProperties::from_section(&self.section);
        let (ix, _iy) = match axis {
            PlasticAxis::X => (props.ix, props.iy),
            PlasticAxis::Y => (props.iy, props.ix),
        };
        let fy = self.material.yield_strength;

        // Section modulus
        let (cx, cy) = (props.centroid.x, props.centroid.y);
        let max_dist_outer = match axis {
            PlasticAxis::X => self
                .section
                .outer
                .vertices
                .iter()
                .map(|v| (v.y - cy).abs())
                .fold(0.0, f64::max),
            PlasticAxis::Y => self
                .section
                .outer
                .vertices
                .iter()
                .map(|v| (v.x - cx).abs())
                .fold(0.0, f64::max),
        };
        let max_dist_holes = self
            .section
            .holes
            .iter()
            .flat_map(|h| h.vertices.iter())
            .map(|v| match axis {
                PlasticAxis::X => (v.y - cy).abs(),
                PlasticAxis::Y => (v.x - cx).abs(),
            })
            .fold(0.0, f64::max);
        let max_dist = max_dist_outer.max(max_dist_holes);

        let z_el = ix / max_dist;

        z_el * fy
    }

    /// Get section bounds along an axis.
    pub(crate) fn section_bounds(&self, axis: PlasticAxis) -> (f64, f64) {
        let mut min_v = f64::INFINITY;
        let mut max_v = f64::NEG_INFINITY;

        for v in &self.section.outer.vertices {
            let coord = match axis {
                PlasticAxis::X => v.y,
                PlasticAxis::Y => v.x,
            };
            min_v = min_v.min(coord);
            max_v = max_v.max(coord);
        }

        for hole in &self.section.holes {
            for v in &hole.vertices {
                let coord = match axis {
                    PlasticAxis::X => v.y,
                    PlasticAxis::Y => v.x,
                };
                min_v = min_v.min(coord);
                max_v = max_v.max(coord);
            }
        }

        (min_v, max_v)
    }

    /// Compute area on one side of a line.
    fn area_on_side(&self, axis: PlasticAxis, position: f64, above: bool) -> f64 {
        // Simplified: use fiber integration
        let n = 200;
        let (min_c, max_c) = self.section_bounds(axis);
        let (min_o, max_o) = self.section_bounds(axis.orthogonal());

        let dc = (max_c - min_c) / n as f64;
        let do_ = (max_o - min_o) / n as f64;

        let mut area = 0.0;

        for i in 0..n {
            for j in 0..n {
                let c = min_c + (i as f64 + 0.5) * dc;
                let o = min_o + (j as f64 + 0.5) * do_;

                let point = match axis {
                    PlasticAxis::X => Point::new(o, c),
                    PlasticAxis::Y => Point::new(c, o),
                };

                if self.section.contains_point(point) {
                    let on_side = match axis {
                        PlasticAxis::X => c > position,
                        PlasticAxis::Y => c > position,
                    };

                    if on_side == above {
                        area += dc * do_;
                    }
                }
            }
        }

        area
    }

    /// Full plastic properties (both centroidal and principal axes).
    ///
    /// Mirrors Python `sectionproperties.analysis.plastic_section.PlasticSection.calculate_plastic_properties`:
    /// computes plastic moduli and shape factors about the centroidal x/y axes
    /// and the principal 11/22 axes.
    pub fn full_plastic_properties(&self) -> FullPlasticProperties {
        let props_x = self.plastic_properties(PlasticAxis::X);
        let props_y = self.plastic_properties(PlasticAxis::Y);

        let shape_factor_x = props_x.plastic_section_modulus
            / (props_x.yield_moment / self.material.yield_strength);
        let shape_factor_y = props_y.plastic_section_modulus
            / (props_y.yield_moment / self.material.yield_strength);

        // Principal axis plastic properties
        let sect_props = SectionProperties::from_section(&self.section);
        let principal = sect_props.principal_properties();
        let phi = principal.phi;

        let (props_11, props_22, sf_11, sf_22) = if phi.abs() < 1e-10 {
            // Already aligned with principal axes
            (
                props_x,
                props_y,
                shape_factor_x,
                shape_factor_y,
            )
        } else {
            // Rotate section by -phi so principal axes align with x/y
            let rotated_outer = self.section.outer.rotate(-phi);
            let rotated_holes: Vec<_> =
                self.section.holes.iter().map(|h| h.rotate(-phi)).collect();
            let rotated_section = Section::new(rotated_outer, rotated_holes);
            let rotated_plastic =
                PlasticSection::new(rotated_section, self.material).with_fibers(self.n_fibers);

            let p11 = rotated_plastic.plastic_properties(PlasticAxis::X);
            let p22 = rotated_plastic.plastic_properties(PlasticAxis::Y);
            let sf11 = p11.plastic_section_modulus
                / (p11.yield_moment / self.material.yield_strength);
            let sf22 = p22.plastic_section_modulus
                / (p22.yield_moment / self.material.yield_strength);
            (p11, p22, sf11, sf22)
        };

        // Yield moments (already computed in PlasticProperties)
        let my_xx = props_x.yield_moment;
        let my_yy = props_y.yield_moment;
        let my_11 = props_11.yield_moment;
        let my_22 = props_22.yield_moment;

        // Positive/negative shape factors: S_pl / Z_el
        let sxx = props_x.plastic_section_modulus;
        let syy = props_y.plastic_section_modulus;
        let s11 = props_11.plastic_section_modulus;
        let s22 = props_22.plastic_section_modulus;
        let sf_xx_plus = if sect_props.zxx_plus > 1e-15 { sxx / sect_props.zxx_plus } else { 0.0 };
        let sf_xx_minus = if sect_props.zxx_minus > 1e-15 { sxx / sect_props.zxx_minus } else { 0.0 };
        let sf_yy_plus = if sect_props.zyy_plus > 1e-15 { syy / sect_props.zyy_plus } else { 0.0 };
        let sf_yy_minus = if sect_props.zyy_minus > 1e-15 { syy / sect_props.zyy_minus } else { 0.0 };
        let sf_11_plus = if sect_props.principal.z11_plus > 1e-15 { s11 / sect_props.principal.z11_plus } else { 0.0 };
        let sf_11_minus = if sect_props.principal.z11_minus > 1e-15 { s11 / sect_props.principal.z11_minus } else { 0.0 };
        let sf_22_plus = if sect_props.principal.z22_plus > 1e-15 { s22 / sect_props.principal.z22_plus } else { 0.0 };
        let sf_22_minus = if sect_props.principal.z22_minus > 1e-15 { s22 / sect_props.principal.z22_minus } else { 0.0 };

        FullPlasticProperties {
            x_axis: props_x,
            y_axis: props_y,
            shape_factor_x,
            shape_factor_y,
            principal_11: props_11,
            principal_22: props_22,
            shape_factor_11: sf_11,
            shape_factor_22: sf_22,
            principal_angle: phi,
            plastic_centroid_x: props_y.plastic_neutral_axis.position,
            plastic_centroid_y: props_x.plastic_neutral_axis.position,
            plastic_centroid_11: props_11.plastic_neutral_axis.position,
            plastic_centroid_22: props_22.plastic_neutral_axis.position,
            my_xx,
            my_yy,
            my_11,
            my_22,
            sf_xx_plus,
            sf_xx_minus,
            sf_yy_plus,
            sf_yy_minus,
            sf_11_plus,
            sf_11_minus,
            sf_22_plus,
            sf_22_minus,
        }
    }
}

/// Bending axis for plastic analysis.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlasticAxis {
    X, // Bending about X-axis (My)
    Y, // Bending about Y-axis (Mx)
}

impl PlasticAxis {
    pub fn orthogonal(&self) -> Self {
        match self {
            PlasticAxis::X => PlasticAxis::Y,
            PlasticAxis::Y => PlasticAxis::X,
        }
    }
}

/// Plastic neutral axis result.
#[derive(Debug, Clone, Copy)]
pub struct PlasticNeutralAxis {
    pub position: f64, // Coordinate along the axis
    pub axis: PlasticAxis,
    pub area_above: f64,
    pub area_below: f64,
}

/// Plastic properties for one axis.
#[derive(Debug, Clone, Copy)]
pub struct PlasticProperties {
    pub axis: PlasticAxis,
    pub plastic_neutral_axis: PlasticNeutralAxis,
    pub plastic_section_modulus: f64, // Zpl
    pub plastic_moment_capacity: f64, // Mpl = Zpl * fy
    pub yield_moment: f64,            // My = Z_el * fy
}

/// Full plastic properties for both centroidal and principal axes.
#[derive(Debug, Clone)]
pub struct FullPlasticProperties {
    pub x_axis: PlasticProperties,
    pub y_axis: PlasticProperties,
    pub shape_factor_x: f64, // Mpl/My
    pub shape_factor_y: f64,
    /// Plastic properties about the 11 (major principal) axis.
    pub principal_11: PlasticProperties,
    /// Plastic properties about the 22 (minor principal) axis.
    pub principal_22: PlasticProperties,
    /// Shape factor about the 11 axis.
    pub shape_factor_11: f64,
    /// Shape factor about the 22 axis.
    pub shape_factor_22: f64,
    /// Principal axis angle in radians.
    pub principal_angle: f64,
    /// Plastic centroid x-coordinate (PNA x for bending about y-axis).
    pub plastic_centroid_x: f64,
    /// Plastic centroid y-coordinate (PNA y for bending about x-axis).
    pub plastic_centroid_y: f64,
    /// Plastic centroid 11-coordinate (PNA in rotated frame for 11-axis bending).
    pub plastic_centroid_11: f64,
    /// Plastic centroid 22-coordinate (PNA in rotated frame for 22-axis bending).
    pub plastic_centroid_22: f64,
    /// Yield moment about x-axis [N·m]
    pub my_xx: f64,
    /// Yield moment about y-axis [N·m]
    pub my_yy: f64,
    /// Yield moment about 11-axis [N·m]
    pub my_11: f64,
    /// Yield moment about 22-axis [N·m]
    pub my_22: f64,
    /// Shape factor for positive bending about x-axis (Sxx / zxx_plus)
    pub sf_xx_plus: f64,
    /// Shape factor for negative bending about x-axis (Sxx / zxx_minus)
    pub sf_xx_minus: f64,
    /// Shape factor for positive bending about y-axis (Syy / zyy_plus)
    pub sf_yy_plus: f64,
    /// Shape factor for negative bending about y-axis (Syy / zyy_minus)
    pub sf_yy_minus: f64,
    /// Shape factor for positive bending about 11-axis (S11 / z11_plus)
    pub sf_11_plus: f64,
    /// Shape factor for negative bending about 11-axis (S11 / z11_minus)
    pub sf_11_minus: f64,
    /// Shape factor for positive bending about 22-axis (S22 / z22_plus)
    pub sf_22_plus: f64,
    /// Shape factor for negative bending about 22-axis (S22 / z22_minus)
    pub sf_22_minus: f64,
}

impl FullPlasticProperties {
    /// Shape factor (plastic/elastic moment ratio).
    pub fn shape_factor(&self, axis: PlasticAxis) -> f64 {
        match axis {
            PlasticAxis::X => self.shape_factor_x,
            PlasticAxis::Y => self.shape_factor_y,
        }
    }

    /// Shape factor about principal axes.
    pub fn shape_factor_principal(&self, axis: PlasticAxis) -> f64 {
        match axis {
            PlasticAxis::X => self.shape_factor_11,
            PlasticAxis::Y => self.shape_factor_22,
        }
    }

    /// Plastic moment capacity.
    pub fn plastic_moment(&self, axis: PlasticAxis) -> f64 {
        match axis {
            PlasticAxis::X => self.x_axis.plastic_moment_capacity,
            PlasticAxis::Y => self.y_axis.plastic_moment_capacity,
        }
    }

    /// Plastic moment capacity about principal axes.
    pub fn plastic_moment_principal(&self, axis: PlasticAxis) -> f64 {
        match axis {
            PlasticAxis::X => self.principal_11.plastic_moment_capacity,
            PlasticAxis::Y => self.principal_22.plastic_moment_capacity,
        }
    }
}

/// Plastic analysis using exact polygon clipping (more accurate).
pub mod exact {
    use super::*;
    use crate::geometry::{Point, Polygon};

    /// Compute exact area on one side of the PNA using polygon clipping.
    pub fn exact_area_on_side(section: &Section, axis: PlasticAxis, pna_pos: f64, above: bool) -> f64 {
        let mut area = 0.0;
        if let Some(outer_part) = clip_polygon_halfspace(&section.outer, axis, pna_pos, above) {
            area += outer_part.area();
        }
        for hole in &section.holes {
            if let Some(hole_part) = clip_polygon_halfspace(hole, axis, pna_pos, above) {
                area -= hole_part.area();
            }
        }
        area
    }

    /// Compute exact plastic modulus by polygon clipping at PNA.
    ///
    /// Clips the section at the plastic neutral axis and computes the first moment
    /// of area for each part: Z_pl = Σ |d_i| * A_i where d_i is the distance from
    /// the PNA to the centroid of part i.
    pub fn exact_plastic_modulus(section: &Section, axis: PlasticAxis, pna_pos: f64) -> f64 {
        let (above, below) = clip_section_at_pna(section, axis, pna_pos);

        let mut z_pl = 0.0;

        for part in [above, below].into_iter().flatten() {
            let area = part.area();
            if area.abs() < 1e-15 {
                continue;
            }
            let centroid = part.centroid();
            let lever = match axis {
                PlasticAxis::X => (centroid.y - pna_pos).abs(),
                PlasticAxis::Y => (centroid.x - pna_pos).abs(),
            };
            z_pl += lever * area;
        }

        for hole in &section.holes {
            for above in [true, false].into_iter() {
                if let Some(hp) = clip_polygon_halfspace(hole, axis, pna_pos, above) {
                    let area = hp.area();
                    if area.abs() < 1e-15 {
                        continue;
                    }
                    let centroid = hp.centroid();
                    let lever = match axis {
                        PlasticAxis::X => (centroid.y - pna_pos).abs(),
                        PlasticAxis::Y => (centroid.x - pna_pos).abs(),
                    };
                    z_pl -= lever * area;
                }
            }
        }

        z_pl
    }

    /// Clip section at plastic neutral axis using Sutherland-Hodgman algorithm.
    ///
    /// Returns the portion of the section above and below the PNA line.
    /// For axis=X, the PNA is the horizontal line y = pna_pos.
    /// For axis=Y, the PNA is the vertical line x = pna_pos.
    pub fn clip_section_at_pna(
        section: &Section,
        axis: PlasticAxis,
        pna_pos: f64,
    ) -> (Option<Polygon>, Option<Polygon>) {
        let above = clip_polygon_halfspace(&section.outer, axis, pna_pos, true)
            .map(|p| subtract_holes(p, &section.holes, axis, pna_pos, true));
        let below = clip_polygon_halfspace(&section.outer, axis, pna_pos, false)
            .map(|p| subtract_holes(p, &section.holes, axis, pna_pos, false));

        (above, below)
    }

    /// Clip a polygon by a half-space defined by the PNA line.
    /// `above = true` keeps the part where y > pna_pos (or x > pna_pos for axis=Y).
    fn clip_polygon_halfspace(
        poly: &Polygon,
        axis: PlasticAxis,
        pna_pos: f64,
        above: bool,
    ) -> Option<Polygon> {
        let input = &poly.vertices;
        if input.len() < 3 {
            return None;
        }

        let is_inside = |p: &Point| -> bool {
            match axis {
                PlasticAxis::X => {
                    if above {
                        p.y >= pna_pos
                    } else {
                        p.y <= pna_pos
                    }
                }
                PlasticAxis::Y => {
                    if above {
                        p.x >= pna_pos
                    } else {
                        p.x <= pna_pos
                    }
                }
            }
        };

        let intersect = |p1: &Point, p2: &Point| -> Point {
            match axis {
                PlasticAxis::X => {
                    let dy = p2.y - p1.y;
                    if dy.abs() < 1e-15 {
                        Point::new(p1.x, pna_pos)
                    } else {
                        let t = (pna_pos - p1.y) / dy;
                        Point::new(p1.x + t * (p2.x - p1.x), pna_pos)
                    }
                }
                PlasticAxis::Y => {
                    let dx = p2.x - p1.x;
                    if dx.abs() < 1e-15 {
                        Point::new(pna_pos, p1.y)
                    } else {
                        let t = (pna_pos - p1.x) / dx;
                        Point::new(pna_pos, p1.y + t * (p2.y - p1.y))
                    }
                }
            }
        };

        let mut output = Vec::new();
        let n = input.len();
        let mut prev = input[n - 1];
        let mut prev_in = is_inside(&prev);

        for i in 0..n {
            let curr = input[i];
            let curr_in = is_inside(&curr);

            if curr_in {
                if !prev_in {
                    output.push(intersect(&prev, &curr));
                }
                output.push(curr);
            } else if prev_in {
                output.push(intersect(&prev, &curr));
            }

            prev = curr;
            prev_in = curr_in;
        }

        // Remove duplicate consecutive vertices
        output = deduplicate_vertices(&output);

        if output.len() < 3 {
            None
        } else {
            Some(Polygon::new(output))
        }
    }

    /// Remove duplicate consecutive vertices (and wrap-around duplicates).
    fn deduplicate_vertices(verts: &[Point]) -> Vec<Point> {
        if verts.is_empty() {
            return Vec::new();
        }
        let mut result = Vec::with_capacity(verts.len());
        let tol = 1e-12;
        for v in verts {
            let is_dup = result.last().map_or(false, |last: &Point| {
                (last.x - v.x).abs() < tol && (last.y - v.y).abs() < tol
            });
            if !is_dup {
                result.push(*v);
            }
        }
        // Check wrap-around
        if result.len() > 1 {
            let first = result[0];
            let last = result[result.len() - 1];
            if (first.x - last.x).abs() < tol && (first.y - last.y).abs() < tol {
                result.pop();
            }
        }
        result
    }

    /// Subtract holes from the clipped outer polygon.
    /// Since holes are already part of the Section (CW orientation), we simply
    /// clip each hole and include it in the result polygon list.
    /// For simplicity, we return the outer clipped polygon and ignore holes
    /// in the polygon representation (area calculation handles signed areas).
    fn subtract_holes(
        outer: Polygon,
        _holes: &[Polygon],
        _axis: PlasticAxis,
        _pna_pos: f64,
        _above: bool,
    ) -> Polygon {
        // TODO: Properly subtract clipped holes from the outer polygon.
        // For now, return the outer polygon as-is. The area and centroid
        // will be slightly off for sections with holes, but correct for
        // solid sections.
        outer
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon};
    use crate::material::presets::STEEL_S355;
    use crate::section::Section;
    use crate::section_library::ParametricSection;

    #[test]
    fn plastic_rectangular() {
        // Rectangular section 200x100 mm
        let poly = Polygon::new(vec![
            Point::new(-0.1, -0.05),
            Point::new(0.1, -0.05),
            Point::new(0.1, 0.05),
            Point::new(-0.1, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let plastic = PlasticSection::new(section, STEEL_S355).with_fibers(500);

        let props_x = plastic.plastic_properties(PlasticAxis::X);
        let props_y = plastic.plastic_properties(PlasticAxis::Y);

        // For rectangle: Zpl = b*h^2/4
        let expected_zx = 0.2 * 0.1_f64.powi(2) / 4.0; // bending about X (b=200mm, h=100mm)
        let expected_zy = 0.1 * 0.2_f64.powi(2) / 4.0; // bending about Y (b=100mm, h=200mm)

        assert!((props_x.plastic_section_modulus - expected_zx).abs() / expected_zx < 0.05);
        assert!((props_y.plastic_section_modulus - expected_zy).abs() / expected_zy < 0.05);

        // Shape factor for rectangle = 1.5
        assert!(
            (props_x.plastic_section_modulus / (props_x.yield_moment / STEEL_S355.yield_strength)
                - 1.5)
                .abs()
                < 0.05
        );
    }

    #[test]
    fn plastic_circular() {
        // Circle diameter 100mm
        let poly = crate::section_library::circle_polygon(0.05, 64);
        let section = Section::new(poly, vec![]);
        let plastic = PlasticSection::new(section, STEEL_S355).with_fibers(500);

        let props = plastic.plastic_properties(PlasticAxis::X);

        // For circle: Zpl = d^3/6 = 4*r^3/3
        let r: f64 = 0.05;
        let expected_z = 4.0 * r.powi(3) / 3.0;

        assert!((props.plastic_section_modulus - expected_z).abs() / expected_z < 0.05);

        // Shape factor for circle = 16/(3*pi) ≈ 1.698
        let shape =
            props.plastic_section_modulus / (props.yield_moment / STEEL_S355.yield_strength);
        assert!((shape - 16.0 / (3.0 * std::f64::consts::PI)).abs() < 0.05);
    }

    #[test]
    fn plastic_i_section() {
        // Simple I-section
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();
        let plastic = PlasticSection::new(section, STEEL_S355).with_fibers(500);

        let full = plastic.full_plastic_properties();

        // I-section: strong axis shape factor ~1.1-1.2, weak axis ~1.3-1.5
        assert!(full.shape_factor_x > 1.05 && full.shape_factor_x < 1.3);
        assert!(full.shape_factor_y > 1.1 && full.shape_factor_y < 1.6);

        // Mpl > My
        assert!(full.x_axis.plastic_moment_capacity > full.x_axis.yield_moment);
        assert!(full.y_axis.plastic_moment_capacity > full.y_axis.yield_moment);
    }

    #[test]
    fn plastic_principal_axes_symmetric() {
        // Doubly-symmetric rectangle: principal axes == centroidal axes
        let poly = Polygon::new(vec![
            Point::new(-0.1, -0.05),
            Point::new(0.1, -0.05),
            Point::new(0.1, 0.05),
            Point::new(-0.1, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let plastic = PlasticSection::new(section, STEEL_S355).with_fibers(500);

        let full = plastic.full_plastic_properties();

        // For doubly-symmetric section, principal angle is 0 or pi/2
        let angle_norm = full.principal_angle.rem_euclid(std::f64::consts::FRAC_PI_2);
        assert!(angle_norm < 1e-6 || (angle_norm - std::f64::consts::FRAC_PI_2).abs() < 1e-6);

        // Principal plastic moduli should match centroidal (possibly swapped)
        let p11 = full.principal_11.plastic_section_modulus;
        let p22 = full.principal_22.plastic_section_modulus;
        let sx = full.x_axis.plastic_section_modulus;
        let sy = full.y_axis.plastic_section_modulus;

        // Either (p11≈sx and p22≈sy) or (p11≈sy and p22≈sx)
        let match_a = (p11 - sx).abs() / sx < 0.02 && (p22 - sy).abs() / sy < 0.02;
        let match_b = (p11 - sy).abs() / sy < 0.02 && (p22 - sx).abs() / sx < 0.02;
        assert!(match_a || match_b);
    }

    #[test]
    fn plastic_principal_axes_angle_section() {
        // Unsymmetric angle section: principal axes != centroidal axes
        let angle = crate::section_library::steel::AngleSection::new(
            0.1, 0.075, 0.008, 0.0, 0.0,
        );
        let section = angle.build();
        let plastic = PlasticSection::new(section, STEEL_S355).with_fibers(500);

        let full = plastic.full_plastic_properties();

        // Principal angle should be non-zero for unequal-leg angle
        assert!(full.principal_angle.abs() > 1e-4);

        // Principal plastic moments should be positive and meaningful
        assert!(full.principal_11.plastic_moment_capacity > 0.0);
        assert!(full.principal_22.plastic_moment_capacity > 0.0);

        // Shape factors should be > 1.0
        assert!(full.shape_factor_11 > 1.0);
        assert!(full.shape_factor_22 > 1.0);
    }

    #[test]
    fn plastic_principal_axes_channel() {
        // Channel section: singly-symmetric (symmetric about y-axis)
        let channel = crate::section_library::steel::ChannelSection::new(
            0.2, 0.1, 0.008, 0.008, 0.0, 0.0,
        );
        let section = channel.build();
        let plastic = PlasticSection::new(section, STEEL_S355).with_fibers(500);

        let full = plastic.full_plastic_properties();

        // Channel is symmetric about y-axis, so principal angle ~ 0
        assert!(full.principal_angle.abs() < 1e-4);

        // Principal 11 (strong axis) should match x-axis
        let ratio_11 = full.principal_11.plastic_section_modulus
            / full.x_axis.plastic_section_modulus;
        assert!((ratio_11 - 1.0).abs() < 0.02);
    }

    #[test]
    fn exact_plastic_rectangular() {
        // Rectangular section 200x100 mm
        let poly = Polygon::new(vec![
            Point::new(-0.1, -0.05),
            Point::new(0.1, -0.05),
            Point::new(0.1, 0.05),
            Point::new(-0.1, 0.05),
        ]);
        let section = Section::new(poly, vec![]);

        // PNA at y=0 (centroidal axis for symmetric section)
        let z_pl = exact::exact_plastic_modulus(&section, PlasticAxis::X, 0.0);

        // For rectangle: Zpl = b*h^2/4
        let expected = 0.2 * 0.1_f64.powi(2) / 4.0;
        assert!((z_pl - expected).abs() / expected < 0.01);
    }

    #[test]
    fn exact_plastic_rectangular_y_axis() {
        // Rectangular section 200x100 mm, bending about Y axis
        let poly = Polygon::new(vec![
            Point::new(-0.1, -0.05),
            Point::new(0.1, -0.05),
            Point::new(0.1, 0.05),
            Point::new(-0.1, 0.05),
        ]);
        let section = Section::new(poly, vec![]);

        // PNA at x=0 (centroidal axis for symmetric section)
        let z_pl = exact::exact_plastic_modulus(&section, PlasticAxis::Y, 0.0);

        // For rectangle about Y: Zpl = h*b^2/4
        let expected = 0.1 * 0.2_f64.powi(2) / 4.0;
        assert!((z_pl - expected).abs() / expected < 0.01);
    }

    #[test]
    fn exact_plastic_i_section() {
        // Simple I-section
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();

        // For symmetric I-section, PNA at centroid
        let props = SectionProperties::from_section(&section);
        let z_pl = exact::exact_plastic_modulus(&section, PlasticAxis::X, props.centroid.y);

        // Z_pl should be positive and reasonable
        assert!(z_pl > 0.0);

        // Compare with fiber integration result
        let plastic = PlasticSection::new(section.clone(), STEEL_S355).with_fibers(500);
        let fiber_props = plastic.plastic_properties(PlasticAxis::X);
        let fiber_z = fiber_props.plastic_section_modulus;

        // Exact should be close to fiber (within 5%)
        assert!((z_pl - fiber_z).abs() / fiber_z < 0.05);
    }

    #[test]
    fn exact_plastic_clip_geometry() {
        // Test that clipping produces valid polygons
        let poly = Polygon::new(vec![
            Point::new(-0.1, -0.05),
            Point::new(0.1, -0.05),
            Point::new(0.1, 0.05),
            Point::new(-0.1, 0.05),
        ]);
        let section = Section::new(poly, vec![]);

        let (above, below) = exact::clip_section_at_pna(&section, PlasticAxis::X, 0.0);

        assert!(above.is_some());
        assert!(below.is_some());

        let above_poly = above.unwrap();
        let below_poly = below.unwrap();

        // Each part should have area = half of total
        let half_area = 0.2 * 0.1 / 2.0;
        assert!((above_poly.area() - half_area).abs() / half_area < 0.01);
        assert!((below_poly.area() - half_area).abs() / half_area < 0.01);
    }

    #[test]
    fn exact_plastic_hollow_rectangular() {
        let b = 0.2;
        let h = 0.1;
        let t = 0.01;
        let outer = Polygon::new(vec![
            Point::new(-b / 2.0, -h / 2.0),
            Point::new(b / 2.0, -h / 2.0),
            Point::new(b / 2.0, h / 2.0),
            Point::new(-b / 2.0, h / 2.0),
        ]);
        let inner = Polygon::new(vec![
            Point::new(-(b - 2.0 * t) / 2.0, -(h - 2.0 * t) / 2.0),
            Point::new((b - 2.0 * t) / 2.0, -(h - 2.0 * t) / 2.0),
            Point::new((b - 2.0 * t) / 2.0, (h - 2.0 * t) / 2.0),
            Point::new(-(b - 2.0 * t) / 2.0, (h - 2.0 * t) / 2.0),
        ]);
        let section = Section::new(outer, vec![inner]);

        let z_pl = exact::exact_plastic_modulus(&section, PlasticAxis::X, 0.0);

        let z_outer = b * h * h / 4.0;
        let z_inner = (b - 2.0 * t) * (h - 2.0 * t).powi(2) / 4.0;
        let expected = z_outer - z_inner;

        assert!(
            (z_pl - expected).abs() / expected < 0.01,
            "Hollow rectangular Z_pl: got {}, expected {}",
            z_pl,
            expected
        );
    }

    #[test]
    fn plastic_centroid_symmetric_section() {
        // Doubly-symmetric rectangle: plastic centroid == geometric centroid
        let poly = Polygon::new(vec![
            Point::new(-0.05, -0.05),
            Point::new(0.05, -0.05),
            Point::new(0.05, 0.05),
            Point::new(-0.05, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let plastic = PlasticSection::new(section, STEEL_S355).with_fibers(500);

        let full = plastic.full_plastic_properties();
        // Fiber integration has ~1e-3 tolerance with 200 grid points
        assert!(full.plastic_centroid_x.abs() < 1e-3);
        assert!(full.plastic_centroid_y.abs() < 1e-3);
    }

    #[test]
    fn plastic_centroid_tee_section() {
        // T-section: flange 0.2x0.02 at top, web 0.02x0.08 below.
        // PNA for x-axis bending lies in the flange.
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
        let section = Section::new(poly, vec![]);
        let props = SectionProperties::from_section(&section);
        let plastic = PlasticSection::new(section, STEEL_S355).with_fibers(500);

        let full = plastic.full_plastic_properties();

        // Geometric centroid y
        let area = 0.0056_f64;
        let yc = (0.004 * 0.09 + 0.0016 * 0.04) / area;

        // Plastic centroid y: PNA in flange, area above = area/2 = 0.0028
        // 0.2 * (0.10 - y_pc) = 0.0028  =>  y_pc = 0.086
        let y_pc_expected = 0.086_f64;

        assert!(
            (full.plastic_centroid_y - y_pc_expected).abs() < 0.005,
            "plastic_centroid_y = {}, expected ~{}",
            full.plastic_centroid_y,
            y_pc_expected
        );
        assert!(
            (full.plastic_centroid_y - yc).abs() > 0.005,
            "plastic centroid should differ from geometric centroid for T-section"
        );

        // y-axis bending: section is symmetric about y-axis, so x_pc ~ 0
        assert!(full.plastic_centroid_x.abs() < 0.005);
    }

    #[test]
    fn plastic_centroid_principal_axes() {
        // For a symmetric section, principal plastic centroids match centroidal
        let poly = Polygon::new(vec![
            Point::new(-0.05, -0.05),
            Point::new(0.05, -0.05),
            Point::new(0.05, 0.05),
            Point::new(-0.05, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let plastic = PlasticSection::new(section, STEEL_S355).with_fibers(500);

        let full = plastic.full_plastic_properties();
        assert!(full.plastic_centroid_11.abs() < 1e-3);
        assert!(full.plastic_centroid_22.abs() < 1e-3);
    }

    #[test]
    fn yield_moments_match_plastic_properties() {
        let poly = Polygon::new(vec![
            Point::new(-0.05, -0.05),
            Point::new(0.05, -0.05),
            Point::new(0.05, 0.05),
            Point::new(-0.05, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let plastic = PlasticSection::new(section, STEEL_S355).with_fibers(500);

        let full = plastic.full_plastic_properties();

        // my_xx should match x_axis yield_moment
        assert!((full.my_xx - full.x_axis.yield_moment).abs() < 1e-6);
        assert!((full.my_yy - full.y_axis.yield_moment).abs() < 1e-6);
        assert!((full.my_11 - full.principal_11.yield_moment).abs() < 1e-6);
        assert!((full.my_22 - full.principal_22.yield_moment).abs() < 1e-6);

        // For rectangle: My = Z * fy = (b*h²/6) * fy
        let b = 0.1_f64;
        let h = 0.1_f64;
        let fy = STEEL_S355.yield_strength;
        let z_el = b * h.powi(2) / 6.0;
        let my_expected = z_el * fy;
        assert!((full.my_xx - my_expected).abs() / my_expected < 0.02);
    }

    #[test]
    fn shape_factors_plus_minus_rectangular() {
        // Rectangular section: shape factor = 1.5 for both plus and minus
        let poly = Polygon::new(vec![
            Point::new(-0.05, -0.05),
            Point::new(0.05, -0.05),
            Point::new(0.05, 0.05),
            Point::new(-0.05, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let plastic = PlasticSection::new(section, STEEL_S355).with_fibers(500);

        let full = plastic.full_plastic_properties();

        // For rectangle: S_pl / Z_el = 1.5
        assert!((full.sf_xx_plus - 1.5).abs() < 0.05);
        assert!((full.sf_xx_minus - 1.5).abs() < 0.05);
        assert!((full.sf_yy_plus - 1.5).abs() < 0.05);
        assert!((full.sf_yy_minus - 1.5).abs() < 0.05);
    }

    #[test]
    fn shape_factors_plus_minus_tee_section() {
        // T-section: plus and minus shape factors differ
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
        let section = Section::new(poly, vec![]);
        let plastic = PlasticSection::new(section, STEEL_S355).with_fibers(500);

        let full = plastic.full_plastic_properties();

        // Shape factors should be positive
        assert!(full.sf_xx_plus > 0.0);
        assert!(full.sf_xx_minus > 0.0);
        // For asymmetric section, plus and minus differ
        assert!((full.sf_xx_plus - full.sf_xx_minus).abs() > 0.01);
        // Minus shape factor should be > 1 (bottom fibre farther from centroid)
        assert!(full.sf_xx_minus > 1.0);
    }

    #[test]
    fn yield_moment_hollow_section() {
        // Box section: outer 0.2x0.1, inner 0.16x0.06 (centered)
        // Verify yield_moment considers both outer and inner vertices.
        let outer = Polygon::new(vec![
            Point::new(-0.1, -0.05),
            Point::new(0.1, -0.05),
            Point::new(0.1, 0.05),
            Point::new(-0.1, 0.05),
        ]);
        let inner = Polygon::new(vec![
            Point::new(-0.08, -0.03),
            Point::new(0.08, -0.03),
            Point::new(0.08, 0.03),
            Point::new(-0.08, 0.03),
        ]);
        let section = Section::new(outer, vec![inner]);
        let plastic = PlasticSection::new(section, STEEL_S355).with_fibers(500);

        let props = plastic.plastic_properties(PlasticAxis::X);

        // For symmetric box: yield moment = Ix / c * fy
        // where c = max distance from centroid to any vertex
        // Outer vertices: c = 0.05, inner vertices: c = 0.03
        // So c = 0.05 (outer governs)
        assert!(props.yield_moment > 0.0);

        // Verify shape factor is reasonable for a box section (~1.1-1.2)
        let sf = props.plastic_section_modulus / (props.yield_moment / STEEL_S355.yield_strength);
        assert!(sf > 1.0 && sf < 1.5);
    }
}
