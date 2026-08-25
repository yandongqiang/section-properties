//! Interaction diagrams for combined axial + biaxial bending.
//!
//! Provides PMM (axial-moment-moment) interaction surfaces,
//! section capacity checks per EN 1993, AISC 360, etc.

use crate::geometry::{Point, Polygon};
use crate::material::Material;
use crate::plastic::plastic_section::PlasticSection;
use crate::section::Section;

/// 3D load case (axial + biaxial moments).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoadCase3D {
    pub n: f64,  // Axial force (positive = compression)
    pub mx: f64, // Moment about X-axis
    pub my: f64, // Moment about Y-axis
}

impl LoadCase3D {
    pub fn new(n: f64, mx: f64, my: f64) -> Self {
        Self { n, mx, my }
    }

    pub fn pure_compression(n: f64) -> Self {
        Self {
            n,
            mx: 0.0,
            my: 0.0,
        }
    }

    pub fn pure_bending_x(mx: f64) -> Self {
        Self {
            n: 0.0,
            mx,
            my: 0.0,
        }
    }

    pub fn pure_bending_y(my: f64) -> Self {
        Self {
            n: 0.0,
            mx: 0.0,
            my,
        }
    }
}

/// Point on interaction diagram.
#[derive(Debug, Clone, Copy)]
pub struct InteractionPoint {
    pub n: f64,
    pub mx: f64,
    pub my: f64,
    /// 0 = inside (safe), 1 = on surface, -1 = outside (failed)
    pub status: i32,
}

/// Interaction diagram/surface for a section.
#[derive(Debug, Clone)]
pub struct InteractionDiagram {
    pub section: Section,
    pub material: Material,
    /// Points on the interaction surface
    pub surface_points: Vec<InteractionPoint>,
    /// Pure compression capacity
    pub n_rd: f64,
    /// Pure bending capacities
    pub m_x_rd: f64,
    pub m_y_rd: f64,
}

impl InteractionDiagram {
    /// Create interaction diagram for a section.
    pub fn new(section: Section, material: Material) -> Self {
        let plastic = PlasticSection::new(section.clone(), material);
        let full_props = plastic.full_plastic_properties();

        let n_rd = section.area() * material.yield_strength;
        let m_x_rd = full_props.x_axis.plastic_moment_capacity;
        let m_y_rd = full_props.y_axis.plastic_moment_capacity;

        let mut diagram = Self {
            section,
            material,
            surface_points: Vec::new(),
            n_rd,
            m_x_rd,
            m_y_rd,
        };

        diagram.generate_surface();
        diagram
    }

    /// Generate interaction surface points.
    fn generate_surface(&mut self) {
        let plastic = PlasticSection::new(self.section.clone(), self.material);
        let fy = self.material.yield_strength;

        // Generate points for different axial force ratios
        let n_ratios = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];

        for &n_ratio in &n_ratios {
            let n = n_ratio * self.n_rd;

            // For each n, generate moment interaction curve (Mx-My)
            let m_points = self.moment_interaction_at_n(&plastic, n, fy);
            self.surface_points.extend(m_points);
        }

        // Generate pure bending points (n=0) with finer resolution
        let m_points = self.moment_interaction_at_n(&plastic, 0.0, fy);
        self.surface_points.extend(m_points);
    }

    /// Moment interaction curve at a given axial force.
    fn moment_interaction_at_n(
        &self,
        plastic: &PlasticSection,
        n: f64,
        fy: f64,
    ) -> Vec<InteractionPoint> {
        let mut points = Vec::new();
        let _area = self.section.area();

        // Area in compression
        let a_c = n / fy;

        // Find PNA for this axial force
        // For biaxial, we need to find the PNA orientation and position
        // that gives the correct compression area and moment capacity

        // Simplified: generate points by varying PNA angle
        for i in 0..36 {
            let theta = i as f64 * std::f64::consts::PI / 18.0; // 0 to 350 degrees

            // PNA line: n_x * x + n_y * y = c
            let n_x = theta.cos();
            let n_y = theta.sin();

            // Find PNA position c such that compression area = a_c
            let c = self.find_pna_position(plastic, n_x, n_y, a_c, fy);

            if let Some(c_val) = c {
                // Compute moments about centroid
                let (mx, my) = self.moments_from_pna(plastic, n_x, n_y, c_val, fy);

                points.push(InteractionPoint {
                    n,
                    mx,
                    my,
                    status: 1,
                });
            }
        }

        points
    }

    /// Find PNA position for given normal and compression area.
    fn find_pna_position(
        &self,
        plastic: &PlasticSection,
        n_x: f64,
        n_y: f64,
        a_c: f64,
        _fy: f64,
    ) -> Option<f64> {
        // Get bounds in PNA normal direction
        let mut min_proj = f64::INFINITY;
        let mut max_proj = f64::NEG_INFINITY;

        for v in &self.section.outer.vertices {
            let proj = n_x * v.x + n_y * v.y;
            min_proj = min_proj.min(proj);
            max_proj = max_proj.max(proj);
        }

        // Binary search for PNA position
        let mut low = min_proj;
        let mut high = max_proj;

        for _ in 0..40 {
            let mid = (low + high) / 2.0;
            let area_compression = self.area_in_halfspace(plastic, n_x, n_y, mid, true);

            if area_compression > a_c {
                low = mid;
            } else {
                high = mid;
            }

            if (high - low).abs() < 1e-9 {
                break;
            }
        }

        Some((low + high) / 2.0)
    }

    /// Points inside test for half-space clipping.
    fn inside_halfspace(p: Point, n_x: f64, n_y: f64, c: f64, below: bool) -> bool {
        let proj = n_x * p.x + n_y * p.y;
        if below { proj <= c } else { proj >= c }
    }

    /// Intersect segment (a, b) with line n·x = c.
    fn line_intersection(a: Point, b: Point, n_x: f64, n_y: f64, c: f64) -> Point {
        let proj_a = n_x * a.x + n_y * a.y;
        let proj_b = n_x * b.x + n_y * b.y;
        let t = (c - proj_a) / (proj_b - proj_a);

        Point::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y))
    }

    /// Clip a polygon by the half-space n·x &lt;= c (or >= c when below=false).
    /// Sutherland–Hodgman.
    fn clip_polygon_halfspace(
        poly: &Polygon,
        n_x: f64,
        n_y: f64,
        c: f64,
        below: bool,
    ) -> Option<Polygon> {
        let mut out: Vec<Point> = Vec::new();
        let verts = &poly.vertices;

        if verts.is_empty() {
            return None;
        }

        let s = verts.len();
        let mut prev = verts[s - 1];
        let mut prev_inside = Self::inside_halfspace(prev, n_x, n_y, c, below);

        for i in 0..s {
            let curr = verts[i];
            let curr_inside = Self::inside_halfspace(curr, n_x, n_y, c, below);

            if prev_inside != curr_inside {
                out.push(Self::line_intersection(prev, curr, n_x, n_y, c));
            }

            if curr_inside {
                out.push(curr);
            }

            prev = curr;
            prev_inside = curr_inside;
        }

        out.pop(); // remove duplicate closing vertex

        if out.len() < 3 {
            return None;
        }
        Some(Polygon::new(out))
    }

    /// Compute area in half
    /// Uses exact polygon clipping for speed.
    fn area_in_halfspace(
        &self,
        _plastic: &PlasticSection,
        n_x: f64,
        n_y: f64,
        c: f64,
        below: bool,
    ) -> f64 {
        // Clip each polygon (outer + holes) by the half-plane
        let mut clipped_outer = Vec::new();
        for poly in std::iter::once(&self.section.outer).chain(self.section.holes.iter()) {
            if let Some(clipped) = Self::clip_polygon_halfspace(poly, n_x, n_y, c, below) {
                clipped_outer.push(clipped);
            }
        }

        // Sum signed areas (outer CCW positive, holes CW negative)
        clipped_outer
            .iter()
            .map(|p| p.signed_area())
            .sum::<f64>()
            .abs()
    }

    /// Compute moments from PNA definition.
    fn moments_from_pna(
        &self,
        _plastic: &PlasticSection,
        n_x: f64,
        n_y: f64,
        c: f64,
        fy: f64,
    ) -> (f64, f64) {
        let n = 200;
        let (min_x, max_x, min_y, max_y) = self.section.bounds();

        let dx = (max_x - min_x) / n as f64;
        let dy = (max_y - min_y) / n as f64;

        let props = crate::section_properties::SectionProperties::from_section(&self.section);
        let cx = props.centroid.x;
        let cy = props.centroid.y;

        let mut mx = 0.0;
        let mut my = 0.0;

        for i in 0..n {
            for j in 0..n {
                let x = min_x + (i as f64 + 0.5) * dx;
                let y = min_y + (j as f64 + 0.5) * dy;
                let point = Point::new(x, y);

                if self.section.contains_point(point) {
                    let proj = n_x * x + n_y * y;

                    // Stress: +fy in compression, -fy in tension
                    let stress = if proj <= c { fy } else { -fy };

                    // Moments about centroid
                    mx += stress * (y - cy) * dx * dy;
                    my += stress * (x - cx) * dx * dy;
                }
            }
        }

        (mx, my)
    }

    /// Check if a load case is within the interaction surface.
    pub fn check_capacity(&self, load: LoadCase3D, gamma_m0: f64) -> CapacityCheck {
        let n_rd = self.n_rd / gamma_m0;
        let m_x_rd = self.m_x_rd / gamma_m0;
        let m_y_rd = self.m_y_rd / gamma_m0;

        // Simple interaction check (conservative)
        // For I-sections: (N/N_rd) + (Mx/M_x_rd) + (My/M_y_rd) <= 1.0
        // More accurate: use actual interaction surface

        let n_ratio = load.n.abs() / n_rd;
        let mx_ratio = load.mx.abs() / m_x_rd;
        let my_ratio = load.my.abs() / m_y_rd;

        // Conservative linear interaction
        let _interaction = n_ratio + mx_ratio + my_ratio;

        // For biaxial bending, use ellipse approximation
        let bending_interaction = if mx_ratio > 0.0 || my_ratio > 0.0 {
            (mx_ratio * mx_ratio + my_ratio * my_ratio).sqrt()
        } else {
            0.0
        };

        let combined = n_ratio + bending_interaction;

        CapacityCheck {
            utilization: combined,
            n_ratio,
            mx_ratio,
            my_ratio,
            passed: combined <= 1.0,
            method: "Linear + Elliptical".to_string(),
        }
    }

    /// Check capacity using exact plastic analysis (more accurate).
    pub fn check_capacity_exact(&self, load: LoadCase3D, gamma_m0: f64) -> CapacityCheck {
        // Find the closest point on the interaction surface
        let mut min_dist = f64::INFINITY;
        let mut closest = self.surface_points[0];

        for pt in &self.surface_points {
            let dn = pt.n - load.n.abs();
            let dmx = pt.mx - load.mx.abs();
            let dmy = pt.my - load.my.abs();
            let dist = (dn * dn + dmx * dmx + dmy * dmy).sqrt();

            if dist < min_dist {
                min_dist = dist;
                closest = *pt;
            }
        }

        // Scale to design values
        let scale = 1.0 / gamma_m0;
        let utilization = (load.n.abs() / (closest.n * scale))
            .max(load.mx.abs() / (closest.mx * scale))
            .max(load.my.abs() / (closest.my * scale));

        CapacityCheck {
            utilization,
            n_ratio: load.n.abs() / (closest.n * scale),
            mx_ratio: load.mx.abs() / (closest.mx * scale),
            my_ratio: load.my.abs() / (closest.my * scale),
            passed: utilization <= 1.0,
            method: "Exact Plastic Surface".to_string(),
        }
    }
}

/// Capacity check result.
#[derive(Debug, Clone)]
pub struct CapacityCheck {
    pub utilization: f64,
    pub n_ratio: f64,
    pub mx_ratio: f64,
    pub my_ratio: f64,
    pub passed: bool,
    pub method: String,
}

/// EN 1993-1-1 interaction formulas.
pub mod en1993 {
    use super::*;

    /// Clause 6.2.9.1 - Uniform members in bending and axial compression.
    pub fn check_combined_bending_axial(
        section: &Section,
        material: &Material,
        n_ed: f64,
        m_y_ed: f64,
        m_z_ed: f64,
        gamma_m0: f64,
        _gamma_m1: f64,
    ) -> CapacityCheck {
        let props = crate::section_properties::SectionProperties::from_section(section);
        let fy = material.yield_strength;
        let area = section.area();

        // Design resistances
        let n_rd = area * fy / gamma_m0;
        let m_y_rd = props.ix / props.max_fiber_distance_y() * fy / gamma_m0;
        let m_z_rd = props.iy / props.max_fiber_distance_x() * fy / gamma_m0;

        // Plastic moduli for Class 1/2
        // (Would need section classification first)

        let n_ratio = n_ed / n_rd;
        let my_ratio = m_y_ed / m_y_rd;
        let mz_ratio = m_z_ed / m_z_rd;

        // Eq 6.61 & 6.62 (simplified for Class 1/2)
        let interaction = n_ratio + my_ratio + mz_ratio;

        CapacityCheck {
            utilization: interaction,
            n_ratio,
            mx_ratio: my_ratio,
            my_ratio: mz_ratio,
            passed: interaction <= 1.0,
            method: "EN 1993-1-1 6.2.9".to_string(),
        }
    }

    /// Clause 6.3.3 - Biaxial bending.
    pub fn check_biaxial_bending(
        section: &Section,
        material: &Material,
        m_y_ed: f64,
        m_z_ed: f64,
        gamma_m0: f64,
    ) -> CapacityCheck {
        let props = crate::section_properties::SectionProperties::from_section(section);
        let fy = material.yield_strength;

        let m_y_rd = props.ix / props.max_fiber_distance_y() * fy / gamma_m0;
        let m_z_rd = props.iy / props.max_fiber_distance_x() * fy / gamma_m0;

        let my_ratio = m_y_ed / m_y_rd;
        let mz_ratio = m_z_ed / m_z_rd;

        // Eq 6.55 (for I-sections)
        let alpha = 2.0; // conservative
        let interaction = (my_ratio / alpha).powi(2) + (mz_ratio / alpha).powi(2);

        CapacityCheck {
            utilization: interaction.sqrt(),
            n_ratio: 0.0,
            mx_ratio: my_ratio,
            my_ratio: mz_ratio,
            passed: interaction <= 1.0,
            method: "EN 1993-1-1 6.3.3".to_string(),
        }
    }
}

/// AISC 360 interaction equations.
pub mod aisc360 {
    use super::*;

    /// Chapter H - Members under combined forces.
    pub fn check_h1(
        section: &Section,
        material: &Material,
        p_u: f64,   // Required axial strength
        m_ux: f64,  // Required flexural strength about X
        m_uy: f64,  // Required flexural strength about Y
        phi_c: f64, // Resistance factor for compression
        phi_b: f64, // Resistance factor for flexure
    ) -> CapacityCheck {
        let props = crate::section_properties::SectionProperties::from_section(section);
        let fy = material.yield_strength;
        let area = section.area();

        // Nominal strengths
        let p_n = area * fy;
        let m_nx = props.ix / props.max_fiber_distance_y() * fy;
        let m_ny = props.iy / props.max_fiber_distance_x() * fy;

        let p_rd = phi_c * p_n;
        let m_rdx = phi_b * m_nx;
        let m_rdy = phi_b * m_ny;

        let p_ratio = p_u / p_rd;
        let mx_ratio = m_ux / m_rdx;
        let my_ratio = m_uy / m_rdy;

        // AISC 360 Eq H1-1a and H1-1b
        let interaction = if p_ratio >= 0.2 {
            p_ratio + 8.0 / 9.0 * (mx_ratio + my_ratio)
        } else {
            p_ratio / 2.0 + mx_ratio + my_ratio
        };

        CapacityCheck {
            utilization: interaction,
            n_ratio: p_ratio,
            mx_ratio,
            my_ratio,
            passed: interaction <= 1.0,
            method: "AISC 360 H1".to_string(),
        }
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
    fn interaction_rectangular() {
        let poly = Polygon::new(vec![
            Point::new(-0.1, -0.05),
            Point::new(0.1, -0.05),
            Point::new(0.1, 0.05),
            Point::new(-0.1, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let diagram = InteractionDiagram::new(section, STEEL_S355);

        assert!(diagram.n_rd > 0.0);
        assert!(diagram.m_x_rd > 0.0);
        assert!(diagram.m_y_rd > 0.0);
        assert!(!diagram.surface_points.is_empty());
    }

    #[test]
    fn capacity_check_pure_compression() {
        let poly = Polygon::new(vec![
            Point::new(-0.1, -0.05),
            Point::new(0.1, -0.05),
            Point::new(0.1, 0.05),
            Point::new(-0.1, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let diagram = InteractionDiagram::new(section, STEEL_S355);

        let load = LoadCase3D::pure_compression(diagram.n_rd * 0.5);
        let check = diagram.check_capacity(load, 1.0);

        assert!(check.passed);
        assert!((check.utilization - 0.5).abs() < 0.1);
    }

    #[test]
    fn capacity_check_pure_bending() {
        let poly = Polygon::new(vec![
            Point::new(-0.1, -0.05),
            Point::new(0.1, -0.05),
            Point::new(0.1, 0.05),
            Point::new(-0.1, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let diagram = InteractionDiagram::new(section, STEEL_S355);

        let load = LoadCase3D::pure_bending_x(diagram.m_x_rd * 0.8);
        let check = diagram.check_capacity(load, 1.0);

        assert!(check.passed);
        assert!((check.utilization - 0.8).abs() < 0.1);
    }

    #[test]
    fn capacity_check_combined() {
        let poly = Polygon::new(vec![
            Point::new(-0.1, -0.05),
            Point::new(0.1, -0.05),
            Point::new(0.1, 0.05),
            Point::new(-0.1, 0.05),
        ]);
        let section = Section::new(poly, vec![]);
        let diagram = InteractionDiagram::new(section, STEEL_S355);

        // 50% compression + 60% Mx -> should fail (1.1 > 1.0)
        let load = LoadCase3D::new(diagram.n_rd * 0.5, diagram.m_x_rd * 0.6, 0.0);
        let check = diagram.check_capacity(load, 1.0);

        assert!(!check.passed);
        assert!(check.utilization > 1.0);
    }

    #[test]
    fn en1993_check() {
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();

        let check = en1993::check_combined_bending_axial(
            &section,
            &STEEL_S355,
            500e3, // N = 500 kN
            50e3,  // My = 50 kNm
            10e3,  // Mz = 10 kNm
            1.0,
            1.0,
        );

        assert!(check.utilization > 0.0);
    }

    #[test]
    fn aisc360_check() {
        let i = crate::section_library::steel::ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
        let section = i.build();

        let check = aisc360::check_h1(
            &section,
            &STEEL_S355,
            500e3, // Pu = 500 kN
            50e3,  // Mux = 50 kNm
            10e3,  // Muy = 10 kNm
            0.9,
            0.9,
        );

        assert!(check.utilization > 0.0);
    }
}
