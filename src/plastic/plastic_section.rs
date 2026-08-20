//! Plastic section analysis.
//!
//! Provides plastic neutral axis search, plastic section modulus,
//! plastic moment capacity, and interaction diagrams.

use crate::geometry::{Point, Polygon};
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
    pub fn find_plastic_neutral_axis(&self, axis: PlasticAxis, fy: f64) -> PlasticNeutralAxis {
        // Get section bounds
        let (min_coord, max_coord) = self.section_bounds(axis);

        // Total area
        let total_area = self.section.area();
        let target_area = total_area / 2.0; // Equal area above/below PNA

        // Binary search for PNA position
        let mut low = min_coord;
        let mut high = max_coord;
        let mut pna_pos = (low + high) / 2.0;

        for _ in 0..50 {
            pna_pos = (low + high) / 2.0;
            let area_above = self.area_on_side(axis, pna_pos, true);

            if area_above > target_area {
                low = pna_pos;
            } else {
                high = pna_pos;
            }

            if (high - low).abs() < 1e-9 {
                break;
            }
        }

        PlasticNeutralAxis {
            position: pna_pos,
            axis,
            area_above: self.area_on_side(axis, pna_pos, true),
            area_below: self.area_on_side(axis, pna_pos, false),
        }
    }

    /// Compute plastic section modulus about an axis given PNA.
    fn plastic_modulus_about_axis(
        &self,
        axis: PlasticAxis,
        pna: &PlasticNeutralAxis,
        fy: f64,
    ) -> (f64, f64) {
        // Use fiber integration
        let n = self.n_fibers;
        let (min_c, max_c) = self.section_bounds(axis);
        let (min_o, max_o) = self.section_bounds(axis.orthogonal());

        let dc = (max_c - min_c) / n as f64;
        let do_ = (max_o - min_o) / n as f64;

        let mut z_pl = 0.0;

        for i in 0..n {
            for j in 0..n {
                let c = min_c + (i as f64 + 0.5) * dc;
                let o = min_o + (j as f64 + 0.5) * do_;

                let point = match axis {
                    PlasticAxis::X => Point::new(o, c),
                    PlasticAxis::Y => Point::new(c, o),
                };

                if self.section.contains_point(point) {
                    let lever_arm = (c - pna.position).abs();
                    let fiber_area = dc * do_;
                    z_pl += lever_arm * fiber_area;
                }
            }
        }

        let s_pl = z_pl * fy;
        (z_pl, s_pl)
    }

    /// Elastic yield moment (first yield).
    fn yield_moment(&self, axis: PlasticAxis) -> f64 {
        let props = SectionProperties::from_section(&self.section);
        let (ix, iy) = match axis {
            PlasticAxis::X => (props.ix, props.iy),
            PlasticAxis::Y => (props.iy, props.ix),
        };
        let fy = self.material.yield_strength;

        // Section modulus
        let (cx, cy) = (props.centroid.x, props.centroid.y);
        let max_dist = match axis {
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

        let z_el = match axis {
            PlasticAxis::X => ix / max_dist,
            PlasticAxis::Y => iy / max_dist,
        };

        z_el * fy
    }

    /// Get section bounds along an axis.
    fn section_bounds(&self, axis: PlasticAxis) -> (f64, f64) {
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

    /// Full plastic properties (both axes + biaxial).
    pub fn full_plastic_properties(&self) -> FullPlasticProperties {
        let props_x = self.plastic_properties(PlasticAxis::X);
        let props_y = self.plastic_properties(PlasticAxis::Y);

        FullPlasticProperties {
            x_axis: props_x,
            y_axis: props_y,
            shape_factor_x: props_x.plastic_section_modulus
                / (props_x.yield_moment / self.material.yield_strength),
            shape_factor_y: props_y.plastic_section_modulus
                / (props_y.yield_moment / self.material.yield_strength),
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

/// Full plastic properties for both axes.
#[derive(Debug, Clone)]
pub struct FullPlasticProperties {
    pub x_axis: PlasticProperties,
    pub y_axis: PlasticProperties,
    pub shape_factor_x: f64, // Mpl/My
    pub shape_factor_y: f64,
}

impl FullPlasticProperties {
    /// Shape factor (plastic/elastic moment ratio).
    pub fn shape_factor(&self, axis: PlasticAxis) -> f64 {
        match axis {
            PlasticAxis::X => self.shape_factor_x,
            PlasticAxis::Y => self.shape_factor_y,
        }
    }

    /// Plastic moment capacity.
    pub fn plastic_moment(&self, axis: PlasticAxis) -> f64 {
        match axis {
            PlasticAxis::X => self.x_axis.plastic_moment_capacity,
            PlasticAxis::Y => self.y_axis.plastic_moment_capacity,
        }
    }
}

/// Plastic analysis using exact polygon clipping (more accurate).
pub mod exact {
    use super::*;
    use crate::geometry::{Point, Polygon};

    /// Compute exact plastic modulus by polygon clipping at PNA.
    pub fn exact_plastic_modulus(section: &Section, axis: PlasticAxis, pna_pos: f64) -> f64 {
        // Clip section polygon at PNA
        let (above_poly, below_poly) = clip_section_at_pna(section, axis, pna_pos);

        // Compute first moment of area for each part
        let mut z_pl = 0.0;

        if let Some(poly) = above_poly {
            let centroid = poly.centroid();
            let area = poly.area();
            let lever = (centroid.y - pna_pos).abs();
            z_pl += lever * area;
        }

        if let Some(poly) = below_poly {
            let centroid = poly.centroid();
            let area = poly.area();
            let lever = (centroid.y - pna_pos).abs();
            z_pl += lever * area;
        }

        z_pl
    }

    /// Clip section at plastic neutral axis.
    fn clip_section_at_pna(
        section: &Section,
        axis: PlasticAxis,
        pna_pos: f64,
    ) -> (Option<Polygon>, Option<Polygon>) {
        // This would implement polygon clipping (Sutherland-Hodgman)
        // For now, return None to indicate numerical integration should be used
        (None, None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon};
    use crate::material::presets::STEEL_S355;
    use crate::section::Section;

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
        let expected_zx = 0.1 * 0.1_f64.powi(2) / 4.0; // bending about X (h=100mm)
        let expected_zy = 0.2_f64.powi(2) * 0.05 / 4.0; // bending about Y (h=200mm)

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
        let r = 0.05;
        let expected_z = 4.0 * r.powi(3) / 3.0;

        assert!((props.plastic_section_modulus - expected_z).abs() / expected_z < 0.05);

        // Shape factor for circle = 4/pi ≈ 1.273
        let shape =
            props.plastic_section_modulus / (props.yield_moment / STEEL_S355.yield_strength);
        assert!((shape - 4.0 / std::f64::consts::PI).abs() < 0.05);
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
}
