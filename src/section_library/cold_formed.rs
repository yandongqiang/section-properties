//! Cold-formed steel sections (EN 1993-1-3, AISI S100, GB/T 6723, etc.).
//!
//! Includes lipped channels, Z-sections, hat sections, and custom profiles.

use crate::geometry::{Point, Polygon};
use crate::material::Material;
use crate::section::Section;
use crate::section_library::{ParametricSection, rectangle_polygon, rounded_rectangle_polygon};
use std::f64::consts::PI;

/// Cold-formed lipped channel (C-section with lips).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LippedChannel {
    pub depth: f64,        // h - overall depth
    pub flange_width: f64, // b - flange width
    pub lip_length: f64,   // c - lip length
    pub thickness: f64,    // t - thickness
    pub inner_radius: f64, // r - inner bend radius
    pub outer_radius: f64, // r + t
}

impl LippedChannel {
    pub fn new(
        depth: f64,
        flange_width: f64,
        lip_length: f64,
        thickness: f64,
        inner_radius: f64,
    ) -> Self {
        assert!(depth > 0.0 && flange_width > 0.0 && lip_length >= 0.0 && thickness > 0.0);
        assert!(
            2.0 * thickness < depth && thickness < flange_width && thickness < lip_length + 1e-9
        );
        Self {
            depth,
            flange_width,
            lip_length,
            thickness,
            inner_radius,
            outer_radius: inner_radius + thickness,
        }
    }

    /// Standard sizes per EN 1993-1-3 / GB/T 6723
    pub fn from_designation(designation: &str) -> Option<Self> {
        // Format: "LC{d}h{b}l{c}t{t}r{r}" or standard codes
        let dims = match designation.to_uppercase().as_str() {
            // GB/T 6723 Cold-formed equal-leg lipped channels
            "LC80X40X15X2.0" => (80.0, 40.0, 15.0, 2.0, 3.0),
            "LC100X50X15X2.0" => (100.0, 50.0, 15.0, 2.0, 3.0),
            "LC100X50X20X2.5" => (100.0, 50.0, 20.0, 2.5, 4.0),
            "LC120X50X20X2.5" => (120.0, 50.0, 20.0, 2.5, 4.0),
            "LC140X60X20X3.0" => (140.0, 60.0, 20.0, 3.0, 4.5),
            "LC150X50X20X2.5" => (150.0, 50.0, 20.0, 2.5, 4.0),
            "LC160X60X20X3.0" => (160.0, 60.0, 20.0, 3.0, 4.5),
            "LC180X60X25X3.0" => (180.0, 60.0, 25.0, 3.0, 4.5),
            "LC200X70X25X3.0" => (200.0, 70.0, 25.0, 3.0, 5.0),
            "LC200X75X25X3.0" => (200.0, 75.0, 25.0, 3.0, 5.0),
            "LC220X70X25X3.0" => (220.0, 70.0, 25.0, 3.0, 5.0),
            "LC250X75X30X3.0" => (250.0, 75.0, 30.0, 3.0, 5.0),
            "LC300X90X30X4.0" => (300.0, 90.0, 30.0, 4.0, 6.0),

            // AISI standard lipped channels
            "LC350S162-33" => (88.9, 41.1, 12.7, 0.84, 2.5), // 3.5" x 1.625" x 0.125" x 33mil
            "LC350S162-43" => (88.9, 41.1, 12.7, 1.09, 3.3),
            "LC350S162-54" => (88.9, 41.1, 12.7, 1.37, 4.1),
            "LC350S162-68" => (88.9, 41.1, 12.7, 1.73, 5.2),
            "LC350S162-97" => (88.9, 41.1, 12.7, 2.46, 7.4),
            "LC550S162-43" => (139.7, 41.1, 12.7, 1.09, 3.3),
            "LC550S162-54" => (139.7, 41.1, 12.7, 1.37, 4.1),
            "LC550S162-68" => (139.7, 41.1, 12.7, 1.73, 5.2),
            "LC800S162-54" => (203.2, 41.1, 12.7, 1.37, 4.1),
            "LC800S162-68" => (203.2, 41.1, 12.7, 1.73, 5.2),
            "LC1000S162-68" => (254.0, 41.1, 12.7, 1.73, 5.2),
            "LC1200S162-68" => (304.8, 41.1, 12.7, 1.73, 5.2),

            // Wide flange lipped channels
            "LC200S200-54" => (50.8, 50.8, 15.0, 1.37, 4.1),
            "LC250S200-68" => (63.5, 50.8, 15.0, 1.73, 5.2),

            _ => return None,
        };

        let (h, b, c, t, r) = dims;
        Some(Self::new(
            h / 1000.0,
            b / 1000.0,
            c / 1000.0,
            t / 1000.0,
            r / 1000.0,
        ))
    }
}

impl ParametricSection for LippedChannel {
    fn build(&self) -> Section {
        let h = self.depth;
        let b = self.flange_width;
        let c = self.lip_length;
        let t = self.thickness;
        let r = self.inner_radius;

        let mut vertices = Vec::new();

        // Build from bottom-left, going CCW
        // Start at web bottom
        vertices.push(Point::new(t, -h / 2.0));
        vertices.push(Point::new(0.0, -h / 2.0));

        // Bottom flange with lip
        vertices.push(Point::new(0.0, -h / 2.0 + t));
        if c > 0.0 {
            vertices.push(Point::new(-c, -h / 2.0 + t));
            vertices.push(Point::new(-c, -h / 2.0));
            vertices.push(Point::new(-c + t, -h / 2.0));
            vertices.push(Point::new(-c + t, -h / 2.0 + t));
        }
        vertices.push(Point::new(b - t, -h / 2.0 + t));

        // Web bottom to top
        vertices.push(Point::new(b - t, h / 2.0 - t));

        // Top flange with lip
        if c > 0.0 {
            vertices.push(Point::new(b - t, h / 2.0 - t));
            vertices.push(Point::new(-c + t, h / 2.0 - t));
            vertices.push(Point::new(-c + t, h / 2.0));
            vertices.push(Point::new(-c, h / 2.0));
            vertices.push(Point::new(-c, h / 2.0 - t));
        }
        vertices.push(Point::new(0.0, h / 2.0 - t));
        vertices.push(Point::new(0.0, h / 2.0));
        vertices.push(Point::new(t, h / 2.0));
        vertices.push(Point::new(t, -h / 2.0));

        // Clean duplicates
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
            "LC{:.0}x{:.0}x{:.0}x{:.1}",
            self.depth * 1000.0,
            self.flange_width * 1000.0,
            self.lip_length * 1000.0,
            self.thickness * 1000.0
        )
    }
}

/// Cold-formed Z-section (Z-purlin).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ZSection {
    pub depth: f64,
    pub flange_width: f64,
    pub lip_length: f64,
    pub thickness: f64,
    pub inner_radius: f64,
}

impl ZSection {
    pub fn new(
        depth: f64,
        flange_width: f64,
        lip_length: f64,
        thickness: f64,
        inner_radius: f64,
    ) -> Self {
        assert!(depth > 0.0 && flange_width > 0.0 && lip_length >= 0.0 && thickness > 0.0);
        Self {
            depth,
            flange_width,
            lip_length,
            thickness,
            inner_radius,
        }
    }

    pub fn from_designation(designation: &str) -> Option<Self> {
        let dims = match designation.to_uppercase().as_str() {
            // Common Z-purlin sizes (mm)
            "Z100X50X15X2.0" => (100.0, 50.0, 15.0, 2.0, 3.0),
            "Z120X50X15X2.0" => (120.0, 50.0, 15.0, 2.0, 3.0),
            "Z140X60X20X2.5" => (140.0, 60.0, 20.0, 2.5, 4.0),
            "Z150X50X20X2.5" => (150.0, 50.0, 20.0, 2.5, 4.0),
            "Z160X60X20X3.0" => (160.0, 60.0, 20.0, 3.0, 4.5),
            "Z180X60X25X3.0" => (180.0, 60.0, 25.0, 3.0, 4.5),
            "Z200X70X25X3.0" => (200.0, 70.0, 25.0, 3.0, 5.0),
            "Z200X75X25X3.0" => (200.0, 75.0, 25.0, 3.0, 5.0),
            "Z250X75X30X3.0" => (250.0, 75.0, 30.0, 3.0, 5.0),
            "Z300X90X30X4.0" => (300.0, 90.0, 30.0, 4.0, 6.0),

            // AISI Z-sections
            "Z350S162-33" => (88.9, 41.1, 12.7, 0.84, 2.5),
            "Z350S162-43" => (88.9, 41.1, 12.7, 1.09, 3.3),
            "Z350S162-54" => (88.9, 41.1, 12.7, 1.37, 4.1),
            "Z550S162-54" => (139.7, 41.1, 12.7, 1.37, 4.1),
            "Z800S162-68" => (203.2, 41.1, 12.7, 1.73, 5.2),

            _ => return None,
        };

        let (h, b, c, t, r) = dims;
        Some(Self::new(
            h / 1000.0,
            b / 1000.0,
            c / 1000.0,
            t / 1000.0,
            r / 1000.0,
        ))
    }
}

impl ParametricSection for ZSection {
    fn build(&self) -> Section {
        let h = self.depth;
        let b = self.flange_width;
        let c = self.lip_length;
        let t = self.thickness;
        let r = self.inner_radius;

        let mut vertices = Vec::new();

        // Z-section: flanges on opposite sides of web
        // Start at bottom of left flange
        vertices.push(Point::new(0.0, -h / 2.0));
        vertices.push(Point::new(b, -h / 2.0));
        vertices.push(Point::new(b, -h / 2.0 + t));
        if c > 0.0 {
            vertices.push(Point::new(c, -h / 2.0 + t));
            vertices.push(Point::new(c, -h / 2.0));
            vertices.push(Point::new(c - t, -h / 2.0));
            vertices.push(Point::new(c - t, -h / 2.0 + t));
        }
        vertices.push(Point::new(t, -h / 2.0 + t));

        // Web
        vertices.push(Point::new(t, h / 2.0 - t));
        vertices.push(Point::new(0.0, h / 2.0 - t));
        vertices.push(Point::new(0.0, h / 2.0));
        vertices.push(Point::new(t, h / 2.0));
        vertices.push(Point::new(t, h / 2.0 - t));

        // Top flange (opposite side)
        if c > 0.0 {
            vertices.push(Point::new(b - t, h / 2.0 - t));
            vertices.push(Point::new(b - c + t, h / 2.0 - t));
            vertices.push(Point::new(b - c + t, h / 2.0));
            vertices.push(Point::new(b - c, h / 2.0));
            vertices.push(Point::new(b - c, h / 2.0 - t));
        }
        vertices.push(Point::new(b, h / 2.0 - t));
        vertices.push(Point::new(b, h / 2.0));
        vertices.push(Point::new(0.0, h / 2.0));
        vertices.push(Point::new(0.0, -h / 2.0));

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
            "Z{:.0}x{:.0}x{:.0}x{:.1}",
            self.depth * 1000.0,
            self.flange_width * 1000.0,
            self.lip_length * 1000.0,
            self.thickness * 1000.0
        )
    }
}

/// Cold-formed hat section (omega section).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HatSection {
    pub depth: f64,
    pub top_flange: f64,
    pub bottom_flange: f64,
    pub lip_length: f64,
    pub thickness: f64,
    pub inner_radius: f64,
}

impl HatSection {
    pub fn new(
        depth: f64,
        top_flange: f64,
        bottom_flange: f64,
        lip_length: f64,
        thickness: f64,
        inner_radius: f64,
    ) -> Self {
        assert!(depth > 0.0 && top_flange > 0.0 && bottom_flange > 0.0 && thickness > 0.0);
        Self {
            depth,
            top_flange,
            bottom_flange,
            lip_length,
            thickness,
            inner_radius,
        }
    }
}

impl ParametricSection for HatSection {
    fn build(&self) -> Section {
        let h = self.depth;
        let bt = self.top_flange;
        let bb = self.bottom_flange;
        let c = self.lip_length;
        let t = self.thickness;

        let mut vertices = Vec::new();

        // Bottom flange (wider)
        vertices.push(Point::new(-bb / 2.0, -h / 2.0));
        vertices.push(Point::new(bb / 2.0, -h / 2.0));
        vertices.push(Point::new(bb / 2.0, -h / 2.0 + t));
        if c > 0.0 {
            vertices.push(Point::new(c, -h / 2.0 + t));
            vertices.push(Point::new(c, -h / 2.0));
            vertices.push(Point::new(c - t, -h / 2.0));
            vertices.push(Point::new(c - t, -h / 2.0 + t));
        }
        vertices.push(Point::new(t, -h / 2.0 + t));

        // Web
        vertices.push(Point::new(t, h / 2.0 - t));
        vertices.push(Point::new(0.0, h / 2.0 - t));
        vertices.push(Point::new(0.0, h / 2.0));
        vertices.push(Point::new(t, h / 2.0));

        // Top flange (narrower)
        vertices.push(Point::new(t, h / 2.0 - t));
        if c > 0.0 {
            vertices.push(Point::new(bt / 2.0 - c + t, h / 2.0 - t));
            vertices.push(Point::new(bt / 2.0 - c + t, h / 2.0));
            vertices.push(Point::new(bt / 2.0 - c, h / 2.0));
            vertices.push(Point::new(bt / 2.0 - c, h / 2.0 - t));
        }
        vertices.push(Point::new(bt / 2.0 - t, h / 2.0 - t));
        vertices.push(Point::new(bt / 2.0 - t, h / 2.0));
        vertices.push(Point::new(-bt / 2.0 + t, h / 2.0));
        vertices.push(Point::new(-bt / 2.0 + t, h / 2.0 - t));
        if c > 0.0 {
            vertices.push(Point::new(-bt / 2.0 + c - t, h / 2.0 - t));
            vertices.push(Point::new(-bt / 2.0 + c - t, h / 2.0));
            vertices.push(Point::new(-bt / 2.0 + c, h / 2.0));
            vertices.push(Point::new(-bt / 2.0 + c, h / 2.0 - t));
        }
        vertices.push(Point::new(0.0, h / 2.0 - t));
        vertices.push(Point::new(0.0, -h / 2.0 + t));
        vertices.push(Point::new(-t, -h / 2.0 + t));
        vertices.push(Point::new(-t, -h / 2.0));
        vertices.push(Point::new(-bb / 2.0, -h / 2.0));

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
            "HAT{:.0}x{:.0}x{:.0}x{:.0}x{:.1}",
            self.depth * 1000.0,
            self.top_flange * 1000.0,
            self.bottom_flange * 1000.0,
            self.lip_length * 1000.0,
            self.thickness * 1000.0
        )
    }
}

/// Cold-formed sigma section (Σ-section).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SigmaSection {
    pub depth: f64,
    pub top_flange: f64,
    pub bottom_flange: f64,
    pub top_lip: f64,
    pub bottom_lip: f64,
    pub thickness: f64,
    pub inner_radius: f64,
}

impl SigmaSection {
    pub fn new(
        depth: f64,
        top_flange: f64,
        bottom_flange: f64,
        top_lip: f64,
        bottom_lip: f64,
        thickness: f64,
        inner_radius: f64,
    ) -> Self {
        assert!(depth > 0.0 && top_flange > 0.0 && bottom_flange > 0.0 && thickness > 0.0);
        Self {
            depth,
            top_flange,
            bottom_flange,
            top_lip,
            bottom_lip,
            thickness,
            inner_radius,
        }
    }
}

impl ParametricSection for SigmaSection {
    fn build(&self) -> Section {
        // Similar to hat but with lips on both flanges
        let h = self.depth;
        let bt = self.top_flange;
        let bb = self.bottom_flange;
        let ct = self.top_lip;
        let cb = self.bottom_lip;
        let t = self.thickness;

        let mut vertices = Vec::new();

        // Bottom flange
        vertices.push(Point::new(-bb / 2.0, -h / 2.0));
        vertices.push(Point::new(bb / 2.0, -h / 2.0));
        vertices.push(Point::new(bb / 2.0, -h / 2.0 + t));
        if cb > 0.0 {
            vertices.push(Point::new(cb, -h / 2.0 + t));
            vertices.push(Point::new(cb, -h / 2.0));
            vertices.push(Point::new(cb - t, -h / 2.0));
            vertices.push(Point::new(cb - t, -h / 2.0 + t));
        }
        vertices.push(Point::new(t, -h / 2.0 + t));

        // Web
        vertices.push(Point::new(t, h / 2.0 - t));
        vertices.push(Point::new(0.0, h / 2.0 - t));
        vertices.push(Point::new(0.0, h / 2.0));
        vertices.push(Point::new(t, h / 2.0));

        // Top flange
        vertices.push(Point::new(t, h / 2.0 - t));
        if ct > 0.0 {
            vertices.push(Point::new(bt / 2.0 - ct + t, h / 2.0 - t));
            vertices.push(Point::new(bt / 2.0 - ct + t, h / 2.0));
            vertices.push(Point::new(bt / 2.0 - ct, h / 2.0));
            vertices.push(Point::new(bt / 2.0 - ct, h / 2.0 - t));
        }
        vertices.push(Point::new(bt / 2.0 - t, h / 2.0 - t));
        vertices.push(Point::new(bt / 2.0 - t, h / 2.0));
        vertices.push(Point::new(-bt / 2.0 + t, h / 2.0));
        vertices.push(Point::new(-bt / 2.0 + t, h / 2.0 - t));
        if ct > 0.0 {
            vertices.push(Point::new(-bt / 2.0 + ct - t, h / 2.0 - t));
            vertices.push(Point::new(-bt / 2.0 + ct - t, h / 2.0));
            vertices.push(Point::new(-bt / 2.0 + ct, h / 2.0));
            vertices.push(Point::new(-bt / 2.0 + ct, h / 2.0 - t));
        }
        vertices.push(Point::new(0.0, h / 2.0 - t));
        vertices.push(Point::new(0.0, -h / 2.0 + t));
        vertices.push(Point::new(-t, -h / 2.0 + t));
        vertices.push(Point::new(-t, -h / 2.0));
        vertices.push(Point::new(-bb / 2.0, -h / 2.0));

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
            "SIGMA{:.0}x{:.0}x{:.0}x{:.0}x{:.0}x{:.1}",
            self.depth * 1000.0,
            self.top_flange * 1000.0,
            self.bottom_flange * 1000.0,
            self.top_lip * 1000.0,
            self.bottom_lip * 1000.0,
            self.thickness * 1000.0
        )
    }
}

/// Cold-formed deck profile (trapezoidal sheet).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DeckProfile {
    pub depth: f64,        // h - profile height
    pub pitch: f64,        // w - pitch (width of one wave)
    pub top_width: f64,    // top flat width
    pub bottom_width: f64, // bottom flat width
    pub thickness: f64,
    pub web_angle: f64, // web angle from horizontal (degrees)
}

impl DeckProfile {
    pub fn new(
        depth: f64,
        pitch: f64,
        top_width: f64,
        bottom_width: f64,
        thickness: f64,
        web_angle: f64,
    ) -> Self {
        assert!(depth > 0.0 && pitch > 0.0 && thickness > 0.0);
        assert!(top_width >= 0.0 && bottom_width >= 0.0);
        assert!(web_angle > 0.0 && web_angle < 90.0);
        Self {
            depth,
            pitch,
            top_width,
            bottom_width,
            thickness,
            web_angle,
        }
    }

    pub fn reentrant(
        depth: f64,
        pitch: f64,
        top_width: f64,
        bottom_width: f64,
        thickness: f64,
        reentrant_depth: f64,
    ) -> Self {
        // Reentrant profile has stiffener in web
        Self::new(depth, pitch, top_width, bottom_width, thickness, 45.0)
    }
}

impl ParametricSection for DeckProfile {
    fn build(&self) -> Section {
        let h = self.depth;
        let w = self.pitch;
        let wt = self.top_width;
        let wb = self.bottom_width;
        let t = self.thickness;
        let angle = self.web_angle.to_radians();

        // Web horizontal projection
        let web_proj = h / angle.tan();

        let mut vertices = Vec::new();

        // One wave from left to right, centered
        let start_x = -w / 2.0;

        // Bottom left
        vertices.push(Point::new(start_x, -h / 2.0));
        vertices.push(Point::new(start_x + wb, -h / 2.0));
        vertices.push(Point::new(start_x + wb, -h / 2.0 + t));
        vertices.push(Point::new(start_x + wb + t, -h / 2.0 + t));

        // Web up
        vertices.push(Point::new(start_x + wb + t, h / 2.0 - t));

        // Top
        vertices.push(Point::new(start_x + wb + t - wt, h / 2.0 - t));
        vertices.push(Point::new(start_x + wb + t - wt, h / 2.0));
        vertices.push(Point::new(start_x + wb + t, h / 2.0));
        vertices.push(Point::new(start_x + wb + t, h / 2.0 - t));

        // Web down (next wave)
        vertices.push(Point::new(start_x + w + t, h / 2.0 - t));
        vertices.push(Point::new(start_x + w + t, -h / 2.0 + t));
        vertices.push(Point::new(start_x + w, -h / 2.0 + t));
        vertices.push(Point::new(start_x + w, -h / 2.0));
        vertices.push(Point::new(start_x, -h / 2.0));

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
            "DECK{:.0}x{:.0}x{:.0}x{:.0}x{:.1}",
            self.depth * 1000.0,
            self.pitch * 1000.0,
            self.top_width * 1000.0,
            self.bottom_width * 1000.0,
            self.thickness * 1000.0
        )
    }

    /// Effective section properties per unit width (for slab design).
    pub fn effective_properties_per_width(
        &self,
        material: &Material,
    ) -> crate::section_properties::SectionProperties {
        // For deck profiles, properties are typically given per meter width
        let sec = self.build();
        let props = crate::section_properties::SectionProperties::from_section(&sec);

        // Scale to per meter width
        let scale = 1.0 / self.pitch;
        crate::section_properties::SectionProperties {
            area: props.area * scale,
            centroid: props.centroid,
            ix: props.ix * scale,
            iy: props.iy * scale,
            ixy: props.ixy * scale,
        }
    }
}

/// Custom cold-formed profile from arbitrary points.
#[derive(Debug, Clone)]
pub struct CustomColdFormed {
    pub vertices: Vec<Point>,
    pub holes: Vec<Vec<Point>>,
    pub thickness: f64,
    pub material: Option<Material>,
}

impl CustomColdFormed {
    pub fn new(vertices: Vec<Point>, thickness: f64) -> Self {
        Self {
            vertices,
            holes: Vec::new(),
            thickness,
            material: None,
        }
    }

    pub fn with_holes(mut self, holes: Vec<Vec<Point>>) -> Self {
        self.holes = holes;
        self
    }

    pub fn with_material(mut self, material: Material) -> Self {
        self.material = Some(material);
        self
    }
}

impl ParametricSection for CustomColdFormed {
    fn build(&self) -> Section {
        let outer = Polygon::new(self.vertices.clone());
        let holes: Vec<Polygon> = self.holes.iter().map(|h| Polygon::new(h.clone())).collect();
        Section::new(outer, holes)
    }

    fn designation(&self) -> String {
        "CUSTOM".to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::presets::STEEL_S355;

    #[test]
    fn lipped_channel_basic() {
        let lc = LippedChannel::new(0.2, 0.07, 0.015, 0.002, 0.003);
        let sec = lc.build();
        assert!(sec.area() > 0.0);

        let props = crate::section_properties::SectionProperties::from_section(&sec);
        assert!(props.ix > 0.0);
        assert!(props.iy > 0.0);
    }

    #[test]
    fn lipped_channel_gb() {
        let lc = LippedChannel::from_designation("LC200X70X25X3.0").unwrap();
        assert!((lc.depth - 0.2).abs() < 1e-6);
        assert!((lc.flange_width - 0.07).abs() < 1e-6);
        assert!((lc.lip_length - 0.025).abs() < 1e-6);
        assert!((lc.thickness - 0.003).abs() < 1e-6);
    }

    #[test]
    fn lipped_channel_aisi() {
        let lc = LippedChannel::from_designation("LC550S162-54").unwrap();
        assert!((lc.depth - 0.1397).abs() < 1e-4);
        assert!((lc.thickness - 0.00137).abs() < 1e-5);
    }

    #[test]
    fn z_section_basic() {
        let z = ZSection::new(0.2, 0.07, 0.015, 0.002, 0.003);
        let sec = z.build();
        assert!(sec.area() > 0.0);

        // Z-section should have Ix > Iy typically
        let props = crate::section_properties::SectionProperties::from_section(&sec);
        assert!(props.ix > 0.0);
    }

    #[test]
    fn z_section_gb() {
        let z = ZSection::from_designation("Z200X75X25X3.0").unwrap();
        assert!((z.depth - 0.2).abs() < 1e-6);
    }

    #[test]
    fn hat_section() {
        let hat = HatSection::new(0.15, 0.05, 0.08, 0.015, 0.002, 0.003);
        let sec = hat.build();
        assert!(sec.area() > 0.0);
    }

    #[test]
    fn sigma_section() {
        let sigma = SigmaSection::new(0.2, 0.06, 0.08, 0.02, 0.02, 0.002, 0.003);
        let sec = sigma.build();
        assert!(sec.area() > 0.0);
    }

    #[test]
    fn deck_profile() {
        let deck = DeckProfile::new(0.05, 0.3, 0.1, 0.15, 0.001, 45.0);
        let sec = deck.build();
        assert!(sec.area() > 0.0);

        let props = deck.effective_properties_per_width(&STEEL_S355);
        assert!(props.area > 0.0);
        assert!(props.ix > 0.0);
    }

    #[test]
    fn custom_cold_formed() {
        let vertices = vec![
            Point::new(0.0, 0.0),
            Point::new(0.1, 0.0),
            Point::new(0.1, 0.1),
            Point::new(0.0, 0.1),
        ];
        let custom = CustomColdFormed::new(vertices, 0.002);
        let sec = custom.build();
        assert!((sec.area() - 0.01).abs() < 1e-10);
    }
}

