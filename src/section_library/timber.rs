//! Timber sections (EN 1995, NDS, etc.)
//!
//! Includes solid, glued-laminated (glulam), CLT, and built-up sections.

use crate::geometry::{Point, Polygon};
use crate::material::Material;
use crate::section::Section;
use crate::section_library::{ParametricSection, rectangle_polygon, rounded_rectangle_polygon};
use std::f64::consts::PI;

/// Solid rectangular timber section.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SolidRectangularTimber {
    pub width: f64,  // b
    pub height: f64, // h
}

impl SolidRectangularTimber {
    pub fn new(width: f64, height: f64) -> Self {
        assert!(width > 0.0 && height > 0.0, "Dimensions must be positive");
        Self { width, height }
    }

    /// Standard sawn timber sizes (mm)
    pub fn from_designation(designation: &str) -> Option<Self> {
        // Common European sawn timber (b x h in mm)
        let dims = match designation.to_uppercase().as_str() {
            "38X75" => (38.0, 75.0),
            "38X100" => (38.0, 100.0),
            "38X125" => (38.0, 125.0),
            "38X150" => (38.0, 150.0),
            "38X175" => (38.0, 175.0),
            "38X200" => (38.0, 200.0),
            "38X225" => (38.0, 225.0),
            "38X250" => (38.0, 250.0),
            "47X75" => (47.0, 75.0),
            "47X100" => (47.0, 100.0),
            "47X125" => (47.0, 125.0),
            "47X150" => (47.0, 150.0),
            "47X175" => (47.0, 175.0),
            "47X200" => (47.0, 200.0),
            "47X225" => (47.0, 225.0),
            "47X250" => (47.0, 250.0),
            "50X100" => (50.0, 100.0),
            "50X150" => (50.0, 150.0),
            "50X200" => (50.0, 200.0),
            "50X250" => (50.0, 250.0),
            "50X300" => (50.0, 300.0),
            "63X100" => (63.0, 100.0),
            "63X125" => (63.0, 125.0),
            "63X150" => (63.0, 150.0),
            "63X175" => (63.0, 175.0),
            "63X200" => (63.0, 200.0),
            "63X225" => (63.0, 225.0),
            "63X250" => (63.0, 250.0),
            "63X300" => (63.0, 300.0),
            "75X100" => (75.0, 100.0),
            "75X150" => (75.0, 150.0),
            "75X200" => (75.0, 200.0),
            "75X250" => (75.0, 250.0),
            "75X300" => (75.0, 300.0),
            "100X100" => (100.0, 100.0),
            "100X150" => (100.0, 150.0),
            "100X200" => (100.0, 200.0),
            "100X250" => (100.0, 250.0),
            "100X300" => (100.0, 300.0),
            "150X150" => (150.0, 150.0),
            "150X200" => (150.0, 200.0),
            "150X250" => (150.0, 250.0),
            "150X300" => (150.0, 300.0),
            "200X200" => (200.0, 200.0),
            "200X250" => (200.0, 250.0),
            "200X300" => (200.0, 300.0),
            _ => return None,
        };

        let (b, h) = dims;
        Some(Self::new(b / 1000.0, h / 1000.0))
    }
}

impl ParametricSection for SolidRectangularTimber {
    fn build(&self) -> Section {
        Section::new(rectangle_polygon(self.width, self.height), Vec::new())
    }

    fn designation(&self) -> String {
        format!(
            "TIMBER RECT {:.0}x{:.0}",
            self.width * 1000.0,
            self.height * 1000.0
        )
    }
}

/// Solid circular timber section (log/pole).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SolidCircularTimber {
    pub diameter: f64,
    pub n_vertices: usize,
}

impl SolidCircularTimber {
    pub fn new(diameter: f64) -> Self {
        Self::with_vertices(diameter, 64)
    }

    pub fn with_vertices(diameter: f64, n_vertices: usize) -> Self {
        assert!(diameter > 0.0, "Diameter must be positive");
        assert!(n_vertices >= 8, "At least 8 vertices");
        Self {
            diameter,
            n_vertices,
        }
    }
}

impl ParametricSection for SolidCircularTimber {
    fn build(&self) -> Section {
        use crate::section_library::circle_polygon;
        Section::new(
            circle_polygon(self.diameter / 2.0, self.n_vertices),
            Vec::new(),
        )
    }

    fn designation(&self) -> String {
        format!("TIMBER LOG Ø{:.0}", self.diameter * 1000.0)
    }
}

/// Glulam (glued-laminated) rectangular section.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GlulamRectangular {
    pub width: f64,
    pub height: f64,
    pub lamination_thickness: f64, // typical 33-45mm
    pub n_laminations: usize,      // computed from height / lamination_thickness
}

impl GlulamRectangular {
    pub fn new(width: f64, height: f64, lamination_thickness: f64) -> Self {
        assert!(width > 0.0 && height > 0.0 && lamination_thickness > 0.0);
        let n_lam = (height / lamination_thickness).round() as usize;
        assert!(n_lam >= 2, "At least 2 laminations");
        Self {
            width,
            height,
            lamination_thickness,
            n_laminations: n_lam,
        }
    }

    /// Standard glulam sizes (mm) - typical widths x heights
    pub fn from_designation(designation: &str) -> Option<Self> {
        // Format: GL{width}x{height} e.g. GL115x300
        let upper = designation.to_uppercase();
        if upper.starts_with("GL") {
            let rest = &upper[2..];
            let parts: Vec<&str> = rest.split('X').collect();
            if parts.len() == 2 {
                let w = parts[0].parse::<f64>().ok()?;
                let h = parts[1].parse::<f64>().ok()?;
                // Standard lamination thickness ~33-45mm
                let lam_t = if h <= 300.0 { 33.0 } else { 45.0 };
                return Some(Self::new(w / 1000.0, h / 1000.0, lam_t / 1000.0));
            }
        }
        None
    }
}

impl ParametricSection for GlulamRectangular {
    fn build(&self) -> Section {
        // Glulam is solid for geometric properties
        // Lamination info is for stress analysis
        Section::new(rectangle_polygon(self.width, self.height), Vec::new())
    }

    fn designation(&self) -> String {
        format!(
            "GLULAM {:.0}x{:.0} (lam={:.0}mm x{})",
            self.width * 1000.0,
            self.height * 1000.0,
            self.lamination_thickness * 1000.0,
            self.n_laminations
        )
    }
}

/// Glulam curved/arched section (simplified as straight for properties).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GlulamCurved {
    pub width: f64,
    pub height: f64,
    pub radius: f64, // Radius of curvature
    pub angle: f64,  // Central angle in radians
    pub lamination_thickness: f64,
}

impl GlulamCurved {
    pub fn new(
        width: f64,
        height: f64,
        radius: f64,
        angle: f64,
        lamination_thickness: f64,
    ) -> Self {
        assert!(width > 0.0 && height > 0.0 && radius > 0.0 && angle > 0.0);
        assert!(lamination_thickness > 0.0);
        Self {
            width,
            height,
            radius,
            angle,
            lamination_thickness,
        }
    }
}

impl ParametricSection for GlulamCurved {
    fn build(&self) -> Section {
        // For section properties, treat as straight (conservative)
        Section::new(rectangle_polygon(self.width, self.height), Vec::new())
    }

    fn designation(&self) -> String {
        format!(
            "GLULAM CURVED {:.0}x{:.0} R{:.0} ∠{:.1}°",
            self.width * 1000.0,
            self.height * 1000.0,
            self.radius * 1000.0,
            self.angle.to_degrees()
        )
    }
}

/// Cross-Laminated Timber (CLT) panel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CLTPanel {
    pub thickness: f64,
    pub layers: Vec<CLTLayer>,
    pub width: f64, // Panel width (for section properties per meter width)
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CLTLayer {
    pub thickness: f64,
    pub orientation: CLTOrientation,
    pub grade: CLTGrade,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CLTOrientation {
    Longitudinal, // || to panel span
    Transverse,   // ⟂ to panel span
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CLTGrade {
    C16,
    C24,
    C30, // EN 338
    GL24h,
    GL28h,
    GL32h, // Glulam grades
}

impl CLTPanel {
    pub fn new(thickness: f64, layers: Vec<CLTLayer>, width: f64) -> Self {
        assert!(thickness > 0.0 && width > 0.0);
        assert!(!layers.is_empty(), "At least one layer");
        let sum_t: f64 = layers.iter().map(|l| l.thickness).sum();
        assert!(
            (sum_t - thickness).abs() < 1e-6,
            "Layer thicknesses must sum to total thickness"
        );
        Self {
            thickness,
            layers,
            width,
        }
    }

    /// Standard 3-layer CLT
    pub fn three_layer(thickness: f64, lam_thickness: f64, width: f64) -> Self {
        let t = thickness / 3.0;
        Self::new(
            thickness,
            vec![
                CLTLayer {
                    thickness: t,
                    orientation: CLTOrientation::Longitudinal,
                    grade: CLTGrade::C24,
                },
                CLTLayer {
                    thickness: t,
                    orientation: CLTOrientation::Transverse,
                    grade: CLTGrade::C24,
                },
                CLTLayer {
                    thickness: t,
                    orientation: CLTOrientation::Longitudinal,
                    grade: CLTGrade::C24,
                },
            ],
            width,
        )
    }

    /// Standard 5-layer CLT
    pub fn five_layer(thickness: f64, lam_thickness: f64, width: f64) -> Self {
        let t = thickness / 5.0;
        Self::new(
            thickness,
            vec![
                CLTLayer {
                    thickness: t,
                    orientation: CLTOrientation::Longitudinal,
                    grade: CLTGrade::C24,
                },
                CLTLayer {
                    thickness: t,
                    orientation: CLTOrientation::Transverse,
                    grade: CLTGrade::C24,
                },
                CLTLayer {
                    thickness: t,
                    orientation: CLTOrientation::Longitudinal,
                    grade: CLTGrade::C24,
                },
                CLTLayer {
                    thickness: t,
                    orientation: CLTOrientation::Transverse,
                    grade: CLTGrade::C24,
                },
                CLTLayer {
                    thickness: t,
                    orientation: CLTOrientation::Longitudinal,
                    grade: CLTGrade::C24,
                },
            ],
            width,
        )
    }

    /// Standard 7-layer CLT
    pub fn seven_layer(thickness: f64, lam_thickness: f64, width: f64) -> Self {
        let t = thickness / 7.0;
        Self::new(
            thickness,
            vec![
                CLTLayer {
                    thickness: t,
                    orientation: CLTOrientation::Longitudinal,
                    grade: CLTGrade::C24,
                },
                CLTLayer {
                    thickness: t,
                    orientation: CLTOrientation::Transverse,
                    grade: CLTGrade::C24,
                },
                CLTLayer {
                    thickness: t,
                    orientation: CLTOrientation::Longitudinal,
                    grade: CLTGrade::C24,
                },
                CLTLayer {
                    thickness: t,
                    orientation: CLTOrientation::Transverse,
                    grade: CLTGrade::C24,
                },
                CLTLayer {
                    thickness: t,
                    orientation: CLTOrientation::Longitudinal,
                    grade: CLTGrade::C24,
                },
                CLTLayer {
                    thickness: t,
                    orientation: CLTOrientation::Transverse,
                    grade: CLTGrade::C24,
                },
                CLTLayer {
                    thickness: t,
                    orientation: CLTOrientation::Longitudinal,
                    grade: CLTGrade::C24,
                },
            ],
            width,
        )
    }

    /// Get effective stiffness per meter width (EI)_eff
    pub fn effective_bending_stiffness(&self, ref_material: &Material) -> f64 {
        let mut ei = 0.0;
        let mut z = -self.thickness / 2.0;
        for layer in &self.layers {
            let mat = match layer.grade {
                CLTGrade::C16 => crate::material::presets::TIMBER_GL24H, // approximate
                CLTGrade::C24 => crate::material::presets::TIMBER_GL24H,
                CLTGrade::C30 => crate::material::presets::TIMBER_GL24H,
                CLTGrade::GL24h => crate::material::presets::TIMBER_GL24H,
                _ => crate::material::presets::TIMBER_GL24H,
            };
            let n = mat.modular_ratio(ref_material);
            let layer_z = z + layer.thickness / 2.0;
            let i_self = self.width * layer.thickness.powi(3) / 12.0;
            ei += n
                * (mat.youngs_modulus * i_self
                    + mat.youngs_modulus * self.width * layer.thickness * layer_z.powi(2));
            z += layer.thickness;
        }
        ei
    }
}

impl ParametricSection for CLTPanel {
    fn build(&self) -> Section {
        // CLT panel per meter width
        Section::new(rectangle_polygon(self.width, self.thickness), Vec::new())
    }

    fn designation(&self) -> String {
        format!(
            "CLT {}L {:.0}mm x {:.0}mm",
            self.layers.len(),
            self.thickness * 1000.0,
            self.width * 1000.0
        )
    }
}

/// Built-up timber section (nail-laminated, dowel-laminated, etc.)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BuiltUpTimber {
    pub width: f64,
    pub height: f64,
    pub element_width: f64,  // Individual board width
    pub element_height: f64, // Individual board height
    pub n_elements_wide: usize,
    pub n_elements_high: usize,
    pub gap: f64, // Gap between elements
}

impl BuiltUpTimber {
    pub fn new(width: f64, height: f64, element_width: f64, element_height: f64, gap: f64) -> Self {
        assert!(width > 0.0 && height > 0.0);
        assert!(element_width > 0.0 && element_height > 0.0);
        assert!(gap >= 0.0);
        let n_w = (width / (element_width + gap)).floor() as usize;
        let n_h = (height / (element_height + gap)).floor() as usize;
        assert!(n_w >= 1 && n_h >= 1);
        Self {
            width,
            height,
            element_width,
            element_height,
            n_elements_wide: n_w,
            n_elements_high: n_h,
            gap,
        }
    }

    pub fn nail_laminated(width: f64, height: f64, board_size: (f64, f64), gap: f64) -> Self {
        Self::new(width, height, board_size.0, board_size.1, gap)
    }

    pub fn dowel_laminated(width: f64, height: f64, board_size: (f64, f64)) -> Self {
        Self::new(width, height, board_size.0, board_size.1, 0.0)
    }
}

impl ParametricSection for BuiltUpTimber {
    fn build(&self) -> Section {
        // For section properties, treat as solid with reduced area
        // Actual implementation would model individual boards
        let solid_area = self.width * self.height;
        let board_area = self.element_width
            * self.element_height
            * self.n_elements_wide as f64
            * self.n_elements_high as f64;
        let fill_factor = board_area / solid_area;

        // Effective solid section
        Section::new(rectangle_polygon(self.width, self.height), Vec::new())
    }

    fn designation(&self) -> String {
        format!(
            "BUILT-UP TIMBER {:.0}x{:.0} ({:.0}x{:.0} x{}x{})",
            self.width * 1000.0,
            self.height * 1000.0,
            self.element_width * 1000.0,
            self.element_height * 1000.0,
            self.n_elements_wide,
            self.n_elements_high
        )
    }

    fn mass_per_length(&self, material: &Material) -> f64 {
        let solid_area = self.width * self.height;
        let board_area = self.element_width
            * self.element_height
            * self.n_elements_wide as f64
            * self.n_elements_high as f64;
        let fill_factor = board_area / solid_area;
        solid_area * fill_factor * material.density
    }
}

/// Timber I-joist (engineered wood).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TimberIJoist {
    pub height: f64,
    pub flange_width: f64,
    pub flange_thickness: f64,
    pub web_thickness: f64,
    pub flange_material: TimberFlangeMaterial,
    pub web_material: TimberWebMaterial,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimberFlangeMaterial {
    SolidSawn, // LVL, LSL
    Glulam,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimberWebMaterial {
    OSB,
    Plywood,
    LVL,
}

impl TimberIJoist {
    pub fn new(
        height: f64,
        flange_width: f64,
        flange_thickness: f64,
        web_thickness: f64,
        flange_material: TimberFlangeMaterial,
        web_material: TimberWebMaterial,
    ) -> Self {
        assert!(height > 0.0 && flange_width > 0.0);
        assert!(flange_thickness > 0.0 && web_thickness > 0.0);
        assert!(2.0 * flange_thickness < height);
        Self {
            height,
            flange_width,
            flange_thickness,
            web_thickness,
            flange_material,
            web_material,
        }
    }

    /// Common I-joist series (approximate)
    pub fn from_series(series: &str, depth_mm: f64) -> Option<Self> {
        match series.to_uppercase().as_str() {
            "TJI" => {
                // Trus Joist TJI series
                let (fw, ft, wt) = match (depth_mm as i32) {
                    235..=241 => (38.0, 38.0, 9.5),
                    300..=302 => (38.0, 38.0, 9.5),
                    356..=360 => (38.0, 38.0, 11.0),
                    400..=406 => (38.0, 38.0, 11.0),
                    450..=457 => (38.0, 38.0, 12.5),
                    500..=510 => (38.0, 38.0, 12.5),
                    550..=560 => (38.0, 38.0, 14.0),
                    600..=610 => (38.0, 38.0, 14.0),
                    _ => return None,
                };
                Some(Self::new(
                    depth_mm / 1000.0,
                    fw / 1000.0,
                    ft / 1000.0,
                    wt / 1000.0,
                    TimberFlangeMaterial::SolidSawn,
                    TimberWebMaterial::OSB,
                ))
            }
            "IJ" => {
                // Generic I-joist
                let (fw, ft, wt) = match (depth_mm as i32) {
                    200..=250 => (45.0, 35.0, 8.0),
                    251..=300 => (45.0, 35.0, 9.0),
                    301..=350 => (45.0, 35.0, 10.0),
                    351..=400 => (45.0, 35.0, 11.0),
                    401..=450 => (45.0, 40.0, 12.0),
                    451..=500 => (45.0, 40.0, 12.0),
                    _ => return None,
                };
                Some(Self::new(
                    depth_mm / 1000.0,
                    fw / 1000.0,
                    ft / 1000.0,
                    wt / 1000.0,
                    TimberFlangeMaterial::SolidSawn,
                    TimberWebMaterial::OSB,
                ))
            }
            _ => None,
        }
    }
}

impl ParametricSection for TimberIJoist {
    fn build(&self) -> Section {
        // Build as composite section with different materials
        // For now, simple geometric section
        let h = self.height;
        let bf = self.flange_width;
        let tf = self.flange_thickness;
        let tw = self.web_thickness;

        let bf2 = bf / 2.0;
        let tw2 = tw / 2.0;
        let hh = h / 2.0;

        let mut vertices = Vec::new();

        // Top flange
        vertices.push(Point::new(-bf2, hh - tf));
        vertices.push(Point::new(bf2, hh - tf));
        vertices.push(Point::new(bf2, hh));
        vertices.push(Point::new(-bf2, hh));

        // Web right
        vertices.push(Point::new(tw2, hh - tf));
        vertices.push(Point::new(tw2, -hh + tf));

        // Bottom flange
        vertices.push(Point::new(bf2, -hh + tf));
        vertices.push(Point::new(bf2, -hh));
        vertices.push(Point::new(-bf2, -hh));
        vertices.push(Point::new(-bf2, -hh + tf));

        // Web left
        vertices.push(Point::new(-tw2, -hh + tf));
        vertices.push(Point::new(-tw2, hh - tf));

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
            "TIMBER I-JOIST {:.0}x{:.0}x{:.0}x{:.0}",
            self.height * 1000.0,
            self.flange_width * 1000.0,
            self.flange_thickness * 1000.0,
            self.web_thickness * 1000.0
        )
    }
}

/// Timber box beam (built-up).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TimberBoxBeam {
    pub width: f64,
    pub height: f64,
    pub wall_thickness: f64,
    pub web_spacing: f64, // Distance between webs
    pub n_webs: usize,
}

impl TimberBoxBeam {
    pub fn new(
        width: f64,
        height: f64,
        wall_thickness: f64,
        web_spacing: f64,
        n_webs: usize,
    ) -> Self {
        assert!(width > 0.0 && height > 0.0);
        assert!(wall_thickness > 0.0 && 2.0 * wall_thickness < width.min(height));
        assert!(web_spacing > 0.0);
        assert!(n_webs >= 1);
        Self {
            width,
            height,
            wall_thickness,
            web_spacing,
            n_webs,
        }
    }
}

impl ParametricSection for TimberBoxBeam {
    fn build(&self) -> Section {
        let w = self.width;
        let h = self.height;
        let t = self.wall_thickness;

        // Outer
        let outer = rectangle_polygon(w, h);
        // Inner
        let inner = rectangle_polygon(w - 2.0 * t, h - 2.0 * t);
        Section::new(outer, vec![inner])
    }

    fn designation(&self) -> String {
        format!(
            "TIMBER BOX {:.0}x{:.0} t{:.0} w{}",
            self.width * 1000.0,
            self.height * 1000.0,
            self.wall_thickness * 1000.0,
            self.n_webs
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::presets::TIMBER_GL24H;

    #[test]
    fn solid_rectangular_timber() {
        let t = SolidRectangularTimber::new(0.1, 0.2);
        let sec = t.build();
        assert!((sec.area() - 0.02).abs() < 1e-10);
    }

    #[test]
    fn solid_rectangular_from_designation() {
        let t = SolidRectangularTimber::from_designation("100X200").expect("Should exist");
        assert!((t.width - 0.1).abs() < 1e-6);
        assert!((t.height - 0.2).abs() < 1e-6);
    }

    #[test]
    fn solid_circular_timber() {
        let t = SolidCircularTimber::new(0.2);
        let sec = t.build();
        assert!((sec.area() - PI * 0.01).abs() < 1e-4);
    }

    #[test]
    fn glulam_rectangular() {
        let g = GlulamRectangular::new(0.115, 0.3, 0.033);
        let sec = g.build();
        assert!((sec.area() - 0.115 * 0.3).abs() < 1e-10);
        assert_eq!(g.n_laminations, 9); // 300/33 ≈ 9
    }

    #[test]
    fn glulam_from_designation() {
        let g = GlulamRectangular::from_designation("GL115x300").expect("Should exist");
        assert!((g.width - 0.115).abs() < 1e-6);
        assert!((g.height - 0.3).abs() < 1e-6);
    }

    #[test]
    fn clt_three_layer() {
        let clt = CLTPanel::three_layer(0.1, 0.033, 1.0);
        assert_eq!(clt.layers.len(), 3);
        assert!((clt.thickness - 0.1).abs() < 1e-6);
        let sec = clt.build();
        assert!((sec.area() - 1.0 * 0.1).abs() < 1e-10);
    }

    #[test]
    fn clt_five_layer() {
        let clt = CLTPanel::five_layer(0.17, 0.034, 1.0);
        assert_eq!(clt.layers.len(), 5);
        let ei = clt.effective_bending_stiffness(&TIMBER_GL24H);
        assert!(ei > 0.0);
    }

    #[test]
    fn clt_seven_layer() {
        let clt = CLTPanel::seven_layer(0.24, 0.034, 1.0);
        assert_eq!(clt.layers.len(), 7);
    }

    #[test]
    fn built_up_timber() {
        let bt = BuiltUpTimber::nail_laminated(0.2, 0.3, (0.038, 0.14), 0.002);
        let sec = bt.build();
        assert!(sec.area() > 0.0);
    }

    #[test]
    fn timber_i_joist() {
        let ij = TimberIJoist::new(
            0.3,
            0.038,
            0.038,
            0.0095,
            TimberFlangeMaterial::SolidSawn,
            TimberWebMaterial::OSB,
        );
        let sec = ij.build();
        assert!(sec.area() > 0.0);
    }

    #[test]
    fn timber_i_joist_from_series() {
        let ij = TimberIJoist::from_series("TJI", 300.0).expect("TJI 300 should exist");
        assert!((ij.height - 0.3).abs() < 0.01);
    }

    #[test]
    fn timber_box_beam() {
        let bb = TimberBoxBeam::new(0.3, 0.5, 0.038, 0.6, 1);
        let sec = bb.build();
        let expected = 0.3 * 0.5 - 0.224 * 0.424;
        assert!((sec.area() - expected).abs() < 1e-6);
    }
}
