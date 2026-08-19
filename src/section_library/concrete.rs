//! Concrete sections (EN 1992, ACI 318, etc.)

use crate::geometry::{Point, Polygon};
use crate::material::Material;
use crate::section::Section;
use crate::section_library::{ParametricSection, circle_polygon, rectangle_polygon, rounded_rectangle_polygon};
use std::f64::consts::PI;

/// Rectangular concrete section.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RectangularConcreteSection {
    pub width: f64,   // b
    pub height: f64,  // h
}

impl RectangularConcreteSection {
    pub fn new(width: f64, height: f64) -> Self {
        assert!(width > 0.0 && height > 0.0, "Dimensions must be positive");
        Self { width, height }
    }
}

impl ParametricSection for RectangularConcreteSection {
    fn build(&self) -> Section {
        Section::new(rectangle_polygon(self.width, self.height), Vec::new())
    }

    fn designation(&self) -> String {
        format!("CONC RECT {:.0}x{:.0}", self.width * 1000.0, self.height * 1000.0)
    }
}

/// Circular concrete section.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CircularConcreteSection {
    pub diameter: f64,
    pub n_vertices: usize,
}

impl CircularConcreteSection {
    pub fn new(diameter: f64) -> Self {
        Self::with_vertices(diameter, 64)
    }

    pub fn with_vertices(diameter: f64, n_vertices: usize) -> Self {
        assert!(diameter > 0.0, "Diameter must be positive");
        assert!(n_vertices >= 8, "At least 8 vertices");
        Self { diameter, n_vertices }
    }
}

impl ParametricSection for CircularConcreteSection {
    fn build(&self) -> Section {
        Section::new(circle_polygon(self.diameter / 2.0, self.n_vertices), Vec::new())
    }

    fn designation(&self) -> String {
        format!("CONC CIRC Ø{:.0}", self.diameter * 1000.0)
    }
}

/// T-beam concrete section (flanged beam).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TBeamConcreteSection {
    pub flange_width: f64,      // b_f (effective flange width)
    pub web_width: f64,         // b_w
    pub height: f64,            // h
    pub flange_thickness: f64,  // h_f
}

impl TBeamConcreteSection {
    pub fn new(flange_width: f64, web_width: f64, height: f64, flange_thickness: f64) -> Self {
        assert!(flange_width > 0.0 && web_width > 0.0 && height > 0.0 && flange_thickness > 0.0);
        assert!(web_width <= flange_width, "Web width cannot exceed flange width");
        assert!(flange_thickness < height, "Flange thickness must be less than height");
        Self { flange_width, web_width, height, flange_thickness }
    }
}

impl ParametricSection for TBeamConcreteSection {
    fn build(&self) -> Section {
        let bf = self.flange_width;
        let bw = self.web_width;
        let h = self.height;
        let hf = self.flange_thickness;

        let bf2 = bf / 2.0;
        let bw2 = bw / 2.0;
        let hh = h / 2.0;

        let mut vertices = Vec::new();

        // Top flange left to right
        vertices.push(Point::new(-bf2, hh - hf));
        vertices.push(Point::new(bf2, hh - hf));
        vertices.push(Point::new(bf2, hh));
        vertices.push(Point::new(-bf2, hh));

        // Web right side
        vertices.push(Point::new(bw2, hh - hf));
        vertices.push(Point::new(bw2, -hh));

        // Bottom
        vertices.push(Point::new(-bw2, -hh));
        vertices.push(Point::new(-bw2, hh - hf));

        let mut clean = Vec::new();
        for v in vertices {
            if clean.last() != Some(&v) {
                clean.push(v);
            }
        }
        if clean.len() > 1 && clean[0] == clean[clean.len() - 1] {
            clean.pop();
        }

        Section::new(Polygon::new(clean), Vec::new())
    }

    fn designation(&self) -> String {
        format!(
            "CONC T {:.0}x{:.0}x{:.0}x{:.0}",
            self.flange_width * 1000.0,
            self.web_width * 1000.0,
            self.height * 1000.0,
            self.flange_thickness * 1000.0
        )
    }
}

/// L-beam concrete section (edge beam).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LBeamConcreteSection {
    pub flange_width: f64,
    pub web_width: f64,
    pub height: f64,
    pub flange_thickness: f64,
}

impl LBeamConcreteSection {
    pub fn new(flange_width: f64, web_width: f64, height: f64, flange_thickness: f64) -> Self {
        assert!(flange_width > 0.0 && web_width > 0.0 && height > 0.0 && flange_thickness > 0.0);
        assert!(web_width <= flange_width, "Web width cannot exceed flange width");
        assert!(flange_thickness < height, "Flange thickness must be less than height");
        Self { flange_width, web_width, height, flange_thickness }
    }
}

impl ParametricSection for LBeamConcreteSection {
    fn build(&self) -> Section {
        let bf = self.flange_width;
        let bw = self.web_width;
        let h = self.height;
        let hf = self.flange_thickness;

        let hh = h / 2.0;

        let mut vertices = Vec::new();

        // Top flange (only one side)
        vertices.push(Point::new(0.0, hh - hf));
        vertices.push(Point::new(bf, hh - hf));
        vertices.push(Point::new(bf, hh));
        vertices.push(Point::new(0.0, hh));

        // Web
        vertices.push(Point::new(bw, hh - hf));
        vertices.push(Point::new(bw, -hh));
        vertices.push(Point::new(0.0, -hh));
        vertices.push(Point::new(0.0, hh - hf));

        let mut clean = Vec::new();
        for v in vertices {
            if clean.last() != Some(&v) {
                clean.push(v);
            }
        }
        if clean.len() > 1 && clean[0] == clean[clean.len() - 1] {
            clean.pop();
        }

        Section::new(Polygon::new(clean), Vec::new())
    }

    fn designation(&self) -> String {
        format!(
            "CONC L {:.0}x{:.0}x{:.0}x{:.0}",
            self.flange_width * 1000.0,
            self.web_width * 1000.0,
            self.height * 1000.0,
            self.flange_thickness * 1000.0
        )
    }
}

/// Hollow rectangular concrete section (box girder).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoxConcreteSection {
    pub outer_width: f64,
    pub outer_height: f64,
    pub wall_thickness: f64,
    pub bottom_thickness: f64, // can be different
    pub top_thickness: f64,
    pub outer_radius: f64,
    pub inner_radius: f64,
}

impl BoxConcreteSection {
    pub fn new(
        outer_width: f64,
        outer_height: f64,
        wall_thickness: f64,
        bottom_thickness: f64,
        top_thickness: f64,
        outer_radius: f64,
        inner_radius: f64,
    ) -> Self {
        assert!(outer_width > 0.0 && outer_height > 0.0);
        assert!(wall_thickness > 0.0 && bottom_thickness > 0.0 && top_thickness > 0.0);
        assert!(2.0 * wall_thickness < outer_width);
        assert!(top_thickness + bottom_thickness < outer_height);
        Self { outer_width, outer_height, wall_thickness, bottom_thickness, top_thickness, outer_radius, inner_radius }
    }

    pub fn uniform(outer_width: f64, outer_height: f64, thickness: f64) -> Self {
        Self::new(outer_width, outer_height, thickness, thickness, thickness, 0.0, 0.0)
    }
}

impl ParametricSection for BoxConcreteSection {
    fn build(&self) -> Section {
        let bw = self.outer_width;
        let bh = self.outer_height;
        let tw = self.wall_thickness;
        let tb = self.bottom_thickness;
        let tt = self.top_thickness;
        let r_out = self.outer_radius;
        let r_in = self.inner_radius;

        // Outer polygon
        let outer = if r_out > 0.0 {
            rounded_rectangle_polygon(bw, bh, r_out, 8)
        } else {
            rectangle_polygon(bw, bh)
        };

        // Inner polygon (hole)
        let inner_w = bw - 2.0 * tw;
        let inner_h = bh - tt - tb;
        let inner = if r_in > 0.0 {
            rounded_rectangle_polygon(inner_w, inner_h, r_in, 8)
        } else {
            rectangle_polygon(inner_w, inner_h)
        };

        // Need to center the inner polygon
        let inner_centered = Polygon::new(
            inner.vertices.iter().map(|v| Point::new(v.x, v.y + (tb - tt) / 2.0)).collect()
        );

        Section::new(outer, vec![inner_centered])
    }

    fn designation(&self) -> String {
        format!(
            "CONC BOX {:.0}x{:.0} t{:.0}/{:.0}/{:.0}",
            self.outer_width * 1000.0,
            self.outer_height * 1000.0,
            self.wall_thickness * 1000.0,
            self.bottom_thickness * 1000.0,
            self.top_thickness * 1000.0
        )
    }
}

/// Concrete section with reinforcement (rebar).
#[derive(Debug, Clone)]
pub struct ReinforcedConcreteSection {
    pub concrete_section: Box<dyn ParametricSection>,
    pub reinforcement: Vec<RebarLayer>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RebarLayer {
    pub y: f64,           // Distance from centroid (positive up)
    pub z: f64,           // Distance from centroid (positive right)
    pub diameter: f64,    // Bar diameter
    pub count: usize,     // Number of bars
    pub material: Material,
}

impl ReinforcedConcreteSection {
    pub fn new(concrete_section: impl ParametricSection + 'static) -> Self {
        Self {
            concrete_section: Box::new(concrete_section),
            reinforcement: Vec::new(),
        }
    }

    pub fn add_rebar(mut self, y: f64, z: f64, diameter: f64, count: usize, material: Material) -> Self {
        self.reinforcement.push(RebarLayer { y, z, diameter, count, material });
        self
    }

    pub fn add_rebar_layer(mut self, layer: RebarLayer) -> Self {
        self.reinforcement.push(layer);
        self
    }

    /// Build as composite section with transformed steel areas
    pub fn build_composite(&self) -> crate::section_library::CompositeSection {
        let concrete_sec = self.concrete_section.build();
        let concrete_mat = crate::material::presets::CONCRETE_C30_37;

        let mut material_groups = vec![crate::material::MaterialGroup::new(
            concrete_mat,
            vec![0], // concrete is group 0
        )];

        // Add each rebar layer as a material group
        // For simplicity, we represent rebar as equivalent transformed area
        // In full implementation, this would create small circular polygons for each bar
        for (i, layer) in self.reinforcement.iter().enumerate() {
            let n = layer.material.modular_ratio(&concrete_mat);
            // Create a transformed area polygon (equivalent rectangle)
            let bar_area = PI * (layer.diameter / 2.0).powi(2) * layer.count as f64;
            let transformed_area = bar_area * n;
            // For now just add to material groups - actual geometry would need mesh
            material_groups.push(crate::material::MaterialGroup::new(
                layer.material,
                vec![i + 1],
            ));
        }

        crate::section_library::CompositeSection::new(
            concrete_sec.outer,
            concrete_sec.holes,
            material_groups,
        )
    }
}

/// Precast hollow core slab.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HollowCoreSlab {
    pub width: f64,
    pub height: f64,
    pub core_diameter: f64,
    pub core_spacing: f64,
    pub n_cores: usize,
    pub bottom_flange: f64,
    pub top_flange: f64,
}

impl HollowCoreSlab {
    pub fn new(
        width: f64,
        height: f64,
        core_diameter: f64,
        core_spacing: f64,
        n_cores: usize,
        bottom_flange: f64,
        top_flange: f64,
    ) -> Self {
        assert!(width > 0.0 && height > 0.0);
        assert!(core_diameter > 0.0 && core_spacing > core_diameter);
        assert!(n_cores > 0);
        assert!(bottom_flange > 0.0 && top_flange > 0.0);
        assert!(core_diameter + top_flange + bottom_flange < height);
        Self { width, height, core_diameter, core_spacing, n_cores, bottom_flange, top_flange }
    }
}

impl ParametricSection for HollowCoreSlab {
    fn build(&self) -> Section {
        let w = self.width;
        let h = self.height;
        let d = self.core_diameter;
        let spacing = self.core_spacing;
        let n = self.n_cores;
        let bf = self.bottom_flange;
        let tf = self.top_flange;

        let hw = w / 2.0;
        let hh = h / 2.0;

        // Outer polygon
        let mut outer_vertices = Vec::new();
        outer_vertices.push(Point::new(-hw, -hh + bf));
        outer_vertices.push(Point::new(hw, -hh + bf));
        outer_vertices.push(Point::new(hw, hh - tf));
        outer_vertices.push(Point::new(-hw, hh - tf));
        let outer = Polygon::new(outer_vertices);

        // Core holes
        let mut holes = Vec::new();
        let start_x = -hw + spacing / 2.0 + (w - spacing * (n - 1) as f64) / 2.0;
        for i in 0..n {
            let cx = start_x + i as f64 * spacing;
            let cy = 0.0; // centered vertically
            let mut core_vertices = Vec::new();
            let nv = 32;
            for j in (0..nv).rev() { // CW for hole
                let theta = 2.0 * PI * j as f64 / nv as f64;
                core_vertices.push(Point::new(cx + d / 2.0 * theta.cos(), cy + d / 2.0 * theta.sin()));
            }
            holes.push(Polygon::new(core_vertices));
        }

        Section::new(outer, holes)
    }

    fn designation(&self) -> String {
        format!(
            "HOLLOWCORE {:.0}x{:.0} Ø{:.0}x{}",
            self.width * 1000.0,
            self.height * 1000.0,
            self.core_diameter * 1000.0,
            self.n_cores
        )
    }
}

/// Concrete pile section.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConcretePile {
    pub diameter: f64,
    pub wall_thickness: f64, // 0 for solid
    pub n_vertices: usize,
}

impl ConcretePile {
    pub fn solid(diameter: f64) -> Self {
        Self::with_vertices(diameter, 0.0, 64)
    }

    pub fn hollow(diameter: f64, wall_thickness: f64) -> Self {
        Self::with_vertices(diameter, wall_thickness, 64)
    }

    pub fn with_vertices(diameter: f64, wall_thickness: f64, n_vertices: usize) -> Self {
        assert!(diameter > 0.0);
        assert!(wall_thickness >= 0.0);
        assert!(2.0 * wall_thickness <= diameter);
        assert!(n_vertices >= 8);
        Self { diameter, wall_thickness, n_vertices }
    }
}

impl ParametricSection for ConcretePile {
    fn build(&self) -> Section {
        use crate::section_library::{circle_polygon, hollow_circle_polygon};
        if self.wall_thickness > 0.0 {
            let ro = self.diameter / 2.0;
            let ri = ro - self.wall_thickness;
            let (outer, inner) = hollow_circle_polygon(ro, ri, self.n_vertices);
            Section::new(outer, vec![inner])
        } else {
            Section::new(circle_polygon(self.diameter / 2.0, self.n_vertices), Vec::new())
        }
    }

    fn designation(&self) -> String {
        if self.wall_thickness > 0.0 {
            format!("PILE Ø{:.0}x{:.0}", self.diameter * 1000.0, self.wall_thickness * 1000.0)
        } else {
            format!("PILE Ø{:.0}", self.diameter * 1000.0)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::presets::*;

    #[test]
    fn rectangular_concrete() {
        let rc = RectangularConcreteSection::new(0.3, 0.5);
        let sec = rc.build();
        assert!((sec.area() - 0.15).abs() < 1e-10);
    }

    #[test]
    fn circular_concrete() {
        let cc = CircularConcreteSection::new(0.4);
        let sec = cc.build();
        assert!((sec.area() - PI * 0.04).abs() < 1e-4);
    }

    #[test]
    fn t_beam_concrete() {
        let tb = TBeamConcreteSection::new(1.0, 0.3, 0.6, 0.15);
        let sec = tb.build();
        let expected = 1.0 * 0.15 + 0.3 * 0.45;
        assert!((sec.area() - expected).abs() < 1e-10);
    }

    #[test]
    fn l_beam_concrete() {
        let lb = LBeamConcreteSection::new(0.6, 0.3, 0.5, 0.12);
        let sec = lb.build();
        let expected = 0.6 * 0.12 + 0.3 * 0.38;
        assert!((sec.area() - expected).abs() < 1e-10);
    }

    #[test]
    fn box_concrete() {
        let box_sec = BoxConcreteSection::uniform(1.0, 1.5, 0.1);
        let sec = box_sec.build();
        let expected = 1.0 * 1.5 - 0.8 * 1.3;
        assert!((sec.area() - expected).abs() < 1e-6);
    }

    #[test]
    fn hollow_core_slab() {
        let hc = HollowCoreSlab::new(1.2, 0.2, 0.1, 0.15, 6, 0.03, 0.03);
        let sec = hc.build();
        let outer = 1.2 * 0.2 - 1.2 * 0.06; // minus flanges
        let holes = 6.0 * PI * 0.05_f64.powi(2);
        let expected = outer - holes;
        assert!((sec.area() - expected).abs() < 1e-4);
    }

    #[test]
    fn concrete_pile_solid() {
        let pile = ConcretePile::solid(0.4);
        let sec = pile.build();
        assert!((sec.area() - PI * 0.04).abs() < 1e-4);
    }

    #[test]
    fn concrete_pile_hollow() {
        let pile = ConcretePile::hollow(0.5, 0.05);
        let sec = pile.build();
        let expected = PI * (0.25_f64.powi(2) - 0.20_f64.powi(2));
        assert!((sec.area() - expected).abs() < 1e-4);
    }

    #[test]
    fn reinforced_concrete_section() {
        let rc = RectangularConcreteSection::new(0.3, 0.5);
        let mut rcs = ReinforcedConcreteSection::new(rc)
            .add_rebar(-0.2, -0.1, 0.02, 4, STEEL_S355)
            .add_rebar(0.2, -0.1, 0.02, 4, STEEL_S355)
            .add_rebar(-0.2, 0.1, 0.016, 2, STEEL_S355)
            .add_rebar(0.2, 0.1, 0.016, 2, STEEL_S355);

        let comp = rcs.build_composite();
        assert_eq!(comp.material_groups.len(), 5); // concrete + 4 rebar layers
    }
}
