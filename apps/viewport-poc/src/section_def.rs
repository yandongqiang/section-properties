//! Cross-section definition and property computation (no GUI dependencies).
//!
//! This module is the testable core of the **Section** work module.  It maps a
//! small family of standard shapes onto the real `section-properties` crate,
//! computes their properties (area, second moments, centroid, and — on
//! demand — the St. Venant torsion constant `J`), and exposes the values
//! needed to bind a section to a frame member.
//!
//! # Units
//!
//! * Shape dimensions are stored in **millimetres** (the usual drawing unit in
//!   this domain) and converted to **metres** before they reach the crate.
//! * All computed properties are reported in **SI base units** (`m²`, `m⁴`,
//!   `Pa`), matching the rest of the application.
//!
//! # Axis mapping (section frame → member local frame)
//!
//! `section-properties` works in a 2-D `(x, y)` plane.  We map it onto a
//! member's local `(y, z)` section plane as
//!
//! ```text
//! section x  →  member local y
//! section y  →  member local z
//! ```
//!
//! With the crate computing `ixx = ∫ y² dA` and `iyy = ∫ x² dA`, this gives
//!
//! ```text
//! Iy (member, about local y) = ixx (section)
//! Iz (member, about local z) = iyy (section)
//! ```
//!
//! For the default member reference vector `[0, 0, 1]` this makes the section
//! *height* (its `y` extent) vertical and the strong axis of a beam with a
//! vertical web the in-plane axis — matching the app's built-in
//! `steel_section()` defaults (`Iy > Iz` for an I-shaped beam).
//!
//! # Cost of `J`
//!
//! The crate computes the St. Venant torsion constant `J` with a warping FEM
//! whose cost grows very steeply with the boundary vertex count (measured in
//! this workspace: a smooth 64-gon circle takes minutes).  Geometry-only
//! properties are plain polygon integrals and are effectively free, so the
//! module exposes them separately ([`SectionDef::compute_geometry`]) from the
//! torsion solve ([`SectionDef::compute_torsion`]).
//!
//! Circular sections (solid circle, tube) have an **exact closed form** for
//! `J` ([`analytical_torsion_j`]) and therefore never touch the FEM: the crate
//! would otherwise approximate a circle as a 64-vertex polygon and spend
//! minutes producing a value we can write down directly.  Only the
//! polygonal / thin-walled shapes fall back to the crate's warping FEM.

use serde::{Deserialize, Serialize};

use section_properties::material::{Material, presets};
use section_properties::section_library::ParametricSection;
use section_properties::section_library::primitive::{
    CircularHollowSection, CircularSection, RectangularSection, TriangularSection,
};
use section_properties::section_library::steel::{ISection, RectangularHollowSection};
use section_properties::{Section, SectionProperties, WarpingProperties};

/// Millimetre → metre conversion factor.
const MM: f64 = 1.0e-3;

// ── Shape catalogue ─────────────────────────────────────────────────────

/// The standard shapes the Section module can build.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SectionKind {
    /// Solid rectangle; `width` × `height`.
    Rectangle,
    /// Solid circle; `diameter`.
    Circle,
    /// Circular hollow section (tube); `diameter` and wall `thickness`.
    Tube,
    /// Doubly-symmetric I-beam; `height`, `width`, `web_thickness`,
    /// flange `thickness`, root `radius`.
    IBeam,
    /// Rectangular hollow section (box); `width`, `height`, wall `thickness`.
    Box,
    /// Solid triangle; `width` base and `height`.
    Triangle,
}

impl SectionKind {
    /// All shapes, in picker order.
    pub const ALL: [SectionKind; 6] = [
        SectionKind::Rectangle,
        SectionKind::Circle,
        SectionKind::Tube,
        SectionKind::IBeam,
        SectionKind::Box,
        SectionKind::Triangle,
    ];

    /// Short human label for the picker.
    pub const fn label(self) -> &'static str {
        match self {
            SectionKind::Rectangle => "Rectangle",
            SectionKind::Circle => "Circle",
            SectionKind::Tube => "Hollow circle (tube)",
            SectionKind::IBeam => "I-beam",
            SectionKind::Box => "Box (RHS)",
            SectionKind::Triangle => "Triangle",
        }
    }

    /// Sensible default dimensions (mm) for this shape.
    pub const fn default_params(self) -> SectionParams {
        match self {
            SectionKind::Rectangle => SectionParams {
                width: 100.0,
                height: 200.0,
                diameter: 100.0,
                thickness: 5.0,
                web_thickness: 5.0,
                radius: 0.0,
            },
            SectionKind::Circle => SectionParams {
                width: 100.0,
                height: 100.0,
                diameter: 100.0,
                thickness: 5.0,
                web_thickness: 5.0,
                radius: 0.0,
            },
            SectionKind::Tube => SectionParams {
                width: 100.0,
                height: 100.0,
                diameter: 100.0,
                thickness: 5.0,
                web_thickness: 5.0,
                radius: 0.0,
            },
            SectionKind::IBeam => SectionParams {
                width: 150.0,
                height: 300.0,
                diameter: 100.0,
                thickness: 10.0,
                web_thickness: 7.0,
                radius: 0.0,
            },
            SectionKind::Box => SectionParams {
                width: 100.0,
                height: 100.0,
                diameter: 100.0,
                thickness: 5.0,
                web_thickness: 5.0,
                radius: 0.0,
            },
            SectionKind::Triangle => SectionParams {
                width: 100.0,
                height: 100.0,
                diameter: 100.0,
                thickness: 5.0,
                web_thickness: 5.0,
                radius: 0.0,
            },
        }
    }

    /// Whether the torsion FEM for this shape is expected to be cheap.
    ///
    /// Polygonal outlines (few vertices) solve quickly; curved outlines are
    /// approximated by many vertices and are expensive, so the UI asks the
    /// user to trigger those explicitly.
    pub const fn torsion_is_cheap(self) -> bool {
        !matches!(self, SectionKind::Circle | SectionKind::Tube)
    }

    /// Whether `J` for this shape has an exact closed form (no FEM needed).
    ///
    /// Circles and tubes are true circular sections whose St. Venant torsion
    /// constant is known analytically; see [`analytical_torsion_j`].
    pub const fn has_analytical_torsion(self) -> bool {
        matches!(self, SectionKind::Circle | SectionKind::Tube)
    }
}

/// Shape dimensions, all in **millimetres**.  Which fields a shape uses is
/// documented on [`SectionKind`]; unused fields are ignored.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SectionParams {
    /// Overall width / flange width / base (mm).
    pub width: f64,
    /// Overall height / depth (mm).
    pub height: f64,
    /// Outer diameter for circular shapes (mm).
    pub diameter: f64,
    /// Wall thickness (tube/box) or flange thickness (I-beam) (mm).
    pub thickness: f64,
    /// Web thickness for I-beams (mm).
    pub web_thickness: f64,
    /// Root fillet radius for I-beams (mm).
    pub radius: f64,
}

impl Default for SectionParams {
    fn default() -> Self {
        SectionKind::Rectangle.default_params()
    }
}

// ── Material ────────────────────────────────────────────────────────────

/// Preset materials plus a custom entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MaterialPreset {
    /// Structural steel S355.
    Steel,
    /// Concrete C30/37.
    Concrete,
    /// Glulam timber GL24h.
    Timber,
    /// Aluminium 6061-T6.
    Aluminium,
    /// User-supplied `E` and `ν`.
    Custom,
}

impl MaterialPreset {
    /// All presets, in picker order.
    pub const ALL: [MaterialPreset; 5] = [
        MaterialPreset::Steel,
        MaterialPreset::Concrete,
        MaterialPreset::Timber,
        MaterialPreset::Aluminium,
        MaterialPreset::Custom,
    ];

    /// Picker label.
    pub const fn label(self) -> &'static str {
        match self {
            MaterialPreset::Steel => "Steel (S355)",
            MaterialPreset::Concrete => "Concrete (C30/37)",
            MaterialPreset::Timber => "Timber (GL24h)",
            MaterialPreset::Aluminium => "Aluminium (6061-T6)",
            MaterialPreset::Custom => "Custom",
        }
    }

    fn preset_material(self) -> Option<Material> {
        match self {
            MaterialPreset::Steel => Some(presets::STEEL_S355),
            MaterialPreset::Concrete => Some(presets::CONCRETE_C30_37),
            MaterialPreset::Timber => Some(presets::TIMBER_GL24H),
            MaterialPreset::Aluminium => Some(presets::ALUMINUM_6061_T6),
            MaterialPreset::Custom => None,
        }
    }
}

/// Material selection for a section: a preset, or a custom `(E, ν)` pair.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SectionMaterial {
    /// Active preset.
    pub preset: MaterialPreset,
    /// Custom Young's modulus (Pa); used when `preset == Custom`.
    pub custom_e: f64,
    /// Custom Poisson's ratio; used when `preset == Custom`.
    pub custom_nu: f64,
}

impl Default for SectionMaterial {
    fn default() -> Self {
        Self {
            preset: MaterialPreset::Steel,
            custom_e: 210.0e9,
            custom_nu: 0.3,
        }
    }
}

impl SectionMaterial {
    /// Resolve the selection into a crate [`Material`].
    ///
    /// Presets keep their own stored shear modulus (important for orthotropic
    /// entries such as glulam, whose `G` is **not** `E / 2(1+ν)`); the custom
    /// entry derives `G` from `E` and `ν`.
    pub fn material(&self) -> Material {
        match self.preset.preset_material() {
            Some(m) => m,
            None => Material::new(self.custom_e, self.custom_nu, 0.0, "custom"),
        }
    }

    /// Young's modulus `E` (Pa) for the current selection.
    pub fn youngs_modulus(&self) -> f64 {
        self.material().youngs_modulus
    }

    /// Shear modulus `G` (Pa) for the current selection.
    pub fn shear_modulus(&self) -> f64 {
        self.material().shear_modulus
    }

    /// Poisson's ratio `ν`.
    pub fn poissons_ratio(&self) -> f64 {
        self.material().poissons_ratio
    }

    /// Display name for the current selection.
    pub fn name(&self) -> String {
        match self.preset.preset_material() {
            Some(m) => m.name.to_string(),
            None => "Custom".to_string(),
        }
    }
}

// ── Section definition ──────────────────────────────────────────────────

/// A complete, editable section definition.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SectionDef {
    /// Shape family.
    pub kind: SectionKind,
    /// Shape dimensions (mm).
    pub params: SectionParams,
    /// Material selection.
    pub material: SectionMaterial,
}

impl Default for SectionDef {
    fn default() -> Self {
        Self {
            kind: SectionKind::Rectangle,
            params: SectionKind::Rectangle.default_params(),
            material: SectionMaterial::default(),
        }
    }
}

impl SectionDef {
    /// Build the crate [`Section`] geometry.
    ///
    /// Returns `Err` with a descriptive message for any degenerate or
    /// physically invalid dimension set.  Validation happens *before* the
    /// crate constructors (which panic on invalid input) are called, so this
    /// method never panics on bad user input.
    pub fn build_section(&self) -> Result<Section, String> {
        self.validate()?;
        let p = &self.params;
        let section = match self.kind {
            SectionKind::Rectangle => RectangularSection::new(p.width * MM, p.height * MM).build(),
            SectionKind::Circle => CircularSection::new(p.diameter * MM / 2.0).build(),
            SectionKind::Tube => CircularHollowSection::new(
                p.diameter * MM / 2.0,
                (p.diameter / 2.0 - p.thickness) * MM,
            )
            .build(),
            SectionKind::IBeam => ISection::new(
                p.height * MM,
                p.width * MM,
                p.web_thickness * MM,
                p.thickness * MM,
                p.radius * MM,
            )
            .build(),
            SectionKind::Box => RectangularHollowSection::new(
                p.width * MM,
                p.height * MM,
                p.thickness * MM,
                0.0,
                0.0,
            )
            .build(),
            SectionKind::Triangle => TriangularSection::new(p.width * MM, p.height * MM).build(),
        };

        // Guard against a section whose net area collapses (e.g. a box whose
        // wall equals half the width): property/FEM code would divide by zero.
        let area = section.area();
        if !area.is_finite() || area <= 0.0 {
            return Err(format!(
                "resulting section has non-positive area (A = {area:.3e} m²)"
            ));
        }
        Ok(section)
    }

    /// Validate dimensions for the current shape.
    ///
    /// All dimensions are in mm and must be strictly positive; shape-specific
    /// constraints (wall thickness < half the smallest dimension, tube inner
    /// radius > 0, …) are enforced here so the crate's panicking constructors
    /// are never called with invalid input.
    pub fn validate(&self) -> Result<(), String> {
        let p = &self.params;
        for (label, v) in [
            ("width", p.width),
            ("height", p.height),
            ("diameter", p.diameter),
            ("thickness", p.thickness),
            ("web thickness", p.web_thickness),
        ] {
            if !v.is_finite() {
                return Err(format!("{label} must be finite"));
            }
        }
        if !p.radius.is_finite() || p.radius < 0.0 {
            return Err("radius must be finite and ≥ 0".into());
        }

        match self.kind {
            SectionKind::Rectangle => {
                require_positive("width", p.width)?;
                require_positive("height", p.height)?;
            }
            SectionKind::Circle => {
                require_positive("diameter", p.diameter)?;
            }
            SectionKind::Tube => {
                require_positive("diameter", p.diameter)?;
                require_positive("thickness", p.thickness)?;
                if p.thickness >= p.diameter / 2.0 {
                    return Err(format!(
                        "wall thickness ({:.3} mm) must be less than the radius ({:.3} mm)",
                        p.thickness,
                        p.diameter / 2.0
                    ));
                }
            }
            SectionKind::IBeam => {
                require_positive("height", p.height)?;
                require_positive("width", p.width)?;
                require_positive("flange thickness", p.thickness)?;
                require_positive("web thickness", p.web_thickness)?;
                if 2.0 * p.thickness >= p.height {
                    return Err(format!(
                        "flange thickness ({:.3} mm) too large: 2·tf must be < height ({:.3} mm)",
                        p.thickness, p.height
                    ));
                }
                if p.web_thickness >= p.width {
                    return Err(format!(
                        "web thickness ({:.3} mm) must be less than flange width ({:.3} mm)",
                        p.web_thickness, p.width
                    ));
                }
                let r_max_width = (p.width - p.web_thickness) / 2.0;
                let r_max_height = (p.height - 2.0 * p.thickness) / 2.0;
                if p.radius > r_max_width.min(r_max_height) {
                    return Err(format!(
                        "root radius ({:.3} mm) too large (max {:.3} mm)",
                        p.radius,
                        r_max_width.min(r_max_height)
                    ));
                }
            }
            SectionKind::Box => {
                require_positive("width", p.width)?;
                require_positive("height", p.height)?;
                require_positive("thickness", p.thickness)?;
                if 2.0 * p.thickness >= p.width.min(p.height) {
                    return Err(format!(
                        "wall thickness ({:.3} mm) too large: 2·t must be < min(width, height) ({:.3} mm)",
                        p.thickness,
                        p.width.min(p.height)
                    ));
                }
            }
            SectionKind::Triangle => {
                require_positive("width", p.width)?;
                require_positive("height", p.height)?;
            }
        }
        Ok(())
    }

    /// Validate the material selection, returning the resolved crate material.
    fn checked_material(&self) -> Result<Material, String> {
        let material = self.material.material();
        if material.is_valid() {
            Ok(material)
        } else {
            Err(format!(
                "invalid material: E = {:.4e} Pa, ν = {:.4}, G = {:.4e} Pa",
                material.youngs_modulus, material.poissons_ratio, material.shear_modulus
            ))
        }
    }

    /// Fast path: geometry, centroid and second moments (no FEM).
    ///
    /// This is safe to run on every edit; it is plain polygon integration.
    pub fn compute_geometry(&self) -> Result<(Section, GeometryProperties), String> {
        let section = self.build_section()?;
        let material = self.checked_material()?;

        let geo = SectionProperties::from_section(&section);
        let centroid = section
            .try_centroid()
            .ok_or_else(|| "section area is too small to locate the centroid".to_string())?;
        let principal = geo.principal_properties();

        // Axis mapping (see module docs): Iy = ixx, Iz = iyy.
        let iy = geo.ix;
        let iz = geo.iy;
        let e = material.youngs_modulus;

        let props = GeometryProperties {
            area: geo.area,
            iy,
            iz,
            ixy: geo.ixy,
            i11: principal.i11,
            i22: principal.i22,
            phi: principal.phi,
            cx: centroid.x,
            cy: centroid.y,
            youngs_modulus: e,
            shear_modulus: material.shear_modulus,
            poissons_ratio: material.poissons_ratio,
            ea: e * geo.area,
            eiy: e * iy,
            eiz: e * iz,
        };

        Ok((section, props))
    }

    /// Expensive path: the St. Venant torsion constant `J` and warping
    /// constant `Iw`.
    ///
    /// For circular shapes (circle/tube) this is an **exact closed form** and
    /// returns immediately; every other shape goes through the crate's
    /// fallible warping FEM.  Errors (mesh/solve failures, invalid material)
    /// are propagated; no default is substituted.
    ///
    /// The crate models a circle as a 64-vertex polygon, and the warping FEM
    /// for that polygon takes minutes (see module docs).  For a true circular
    /// section the torsion constant is known in closed form, so we bypass the
    /// crate entirely instead of paying minutes for an approximation.
    pub fn compute_torsion(&self) -> Result<TorsionProperties, String> {
        if let Some(props) = self.analytical_torsion()? {
            return Ok(props);
        }
        let section = self.build_section()?;
        let material = self.checked_material()?;
        let warping = WarpingProperties::try_from_section(&section, material.poissons_ratio)
            .map_err(|e| format!("warping/torsion analysis failed: {e}"))?;
        Ok(TorsionProperties {
            j: warping.j,
            iw: warping.iw,
            gj: material.shear_modulus * warping.j,
        })
    }

    /// Exact closed-form torsion properties, or `None` when the shape needs
    /// the warping FEM.
    ///
    /// Validates the dimensions and material so degenerate input is rejected
    /// exactly as the FEM path would reject it.
    pub fn analytical_torsion(&self) -> Result<Option<TorsionProperties>, String> {
        let Some(j) = analytical_torsion_j(self.kind, &self.params) else {
            return Ok(None);
        };
        self.validate()?;
        let material = self.checked_material()?;
        Ok(Some(TorsionProperties {
            j,
            iw: 0.0, // a circular section does not warp
            gj: material.shear_modulus * j,
        }))
    }

    /// Full computation: geometry *and* torsion.
    ///
    /// Convenience for callers that want everything in one shot (e.g. tests);
    /// the GUI keeps the two halves separate so the fast path never blocks.
    pub fn compute(&self) -> Result<SectionResult, String> {
        let (section, geo) = self.compute_geometry()?;
        let torsion = self.compute_torsion()?;
        Ok(SectionResult {
            section,
            geo,
            torsion,
        })
    }

    /// Human-readable designation, e.g. `"I 300×150"` or `"RECT 100×200"`.
    pub fn designation(&self) -> String {
        let p = &self.params;
        match self.kind {
            SectionKind::Rectangle => format!("RECT {:.0}×{:.0}", p.width, p.height),
            SectionKind::Circle => format!("CIRC Ø{:.0}", p.diameter),
            SectionKind::Tube => format!("CHS Ø{:.0}×{:.1}", p.diameter, p.thickness),
            SectionKind::IBeam => format!(
                "I {:.0}×{:.0}×{:.1}×{:.1}",
                p.height, p.width, p.web_thickness, p.thickness
            ),
            SectionKind::Box => format!("SHS {:.0}×{:.0}×{:.1}", p.width, p.height, p.thickness),
            SectionKind::Triangle => format!("TRI {:.0}×{:.0}", p.width, p.height),
        }
    }
}

fn require_positive(label: &str, v: f64) -> Result<(), String> {
    if v > 0.0 {
        Ok(())
    } else {
        Err(format!("{label} must be positive, got {v:.3} mm"))
    }
}

/// Closed-form St. Venant torsion constant `J` (m⁴) for shapes with an exact
/// analytical value, or `None` for shapes that require the warping FEM.
///
/// * Solid circle, radius `r`: `J = π r⁴ / 2`
/// * Tube, outer radius `rₒ`, inner radius `rᵢ`: `J = π (rₒ⁴ − rᵢ⁴) / 2`
///
/// Dimensions are read from `params` in millimetres.  For a circular section
/// the torsion constant equals the polar second moment of area and there is no
/// warping (`Iw = 0`).
pub fn analytical_torsion_j(kind: SectionKind, params: &SectionParams) -> Option<f64> {
    let pi = std::f64::consts::PI;
    match kind {
        SectionKind::Circle => {
            let r = params.diameter * MM / 2.0;
            Some(pi * r.powi(4) / 2.0)
        }
        SectionKind::Tube => {
            let ro = params.diameter * MM / 2.0;
            let ri = (params.diameter / 2.0 - params.thickness) * MM;
            Some(pi * (ro.powi(4) - ri.powi(4)) / 2.0)
        }
        _ => None,
    }
}

/// A built section together with its computed properties.
pub struct SectionResult {
    /// The crate geometry (outer boundary + holes), for previewing.
    pub section: Section,
    /// Fast geometric properties.
    pub geo: GeometryProperties,
    /// Torsion properties.
    pub torsion: TorsionProperties,
}

impl SectionResult {
    /// The `(E, G, A, Iy, Iz, J)` tuple to bind onto a frame member.
    pub fn member_section_values(&self) -> (f64, f64, f64, f64, f64, f64) {
        (
            self.geo.youngs_modulus,
            self.geo.shear_modulus,
            self.geo.area,
            self.geo.iy,
            self.geo.iz,
            self.torsion.j,
        )
    }
}

/// Fast geometric properties in SI base units (no FEM).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeometryProperties {
    /// Cross-sectional area `m²`.
    pub area: f64,
    /// Second moment about the member's local y axis `m⁴` (= section `ixx`).
    pub iy: f64,
    /// Second moment about the member's local z axis `m⁴` (= section `iyy`).
    pub iz: f64,
    /// Product of inertia `m⁴`.
    pub ixy: f64,
    /// Major principal second moment `m⁴`.
    pub i11: f64,
    /// Minor principal second moment `m⁴`.
    pub i22: f64,
    /// Principal-axis angle (radians, CCW from x).
    pub phi: f64,
    /// Centroid x coordinate in the section frame `m`.
    pub cx: f64,
    /// Centroid y coordinate in the section frame `m`.
    pub cy: f64,
    /// Young's modulus `Pa`.
    pub youngs_modulus: f64,
    /// Shear modulus `Pa`.
    pub shear_modulus: f64,
    /// Poisson's ratio.
    pub poissons_ratio: f64,
    /// Effective axial stiffness `E·A` (N).
    pub ea: f64,
    /// Effective bending stiffness `E·Iy` (N·m²).
    pub eiy: f64,
    /// Effective bending stiffness `E·Iz` (N·m²).
    pub eiz: f64,
}

/// Torsion properties from the warping FEM.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TorsionProperties {
    /// St. Venant torsion constant `m⁴`.
    pub j: f64,
    /// Warping constant `m⁶`.
    pub iw: f64,
    /// Effective torsional stiffness `G·J` (N·m²).
    pub gj: f64,
}

/// Extract the outline of a section as loops of 2-D points (metres).
///
/// The first loop is the outer boundary; any following loops are holes.
pub fn outline(section: &Section) -> Vec<Vec<[f64; 2]>> {
    let mut loops = Vec::with_capacity(1 + section.holes.len());
    loops.push(section.outer.vertices.iter().map(|p| [p.x, p.y]).collect());
    for hole in &section.holes {
        loops.push(hole.vertices.iter().map(|p| [p.x, p.y]).collect());
    }
    loops
}

// ── Tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(width_mm: f64, height_mm: f64) -> SectionDef {
        SectionDef {
            kind: SectionKind::Rectangle,
            params: SectionParams {
                width: width_mm,
                height: height_mm,
                ..SectionKind::Rectangle.default_params()
            },
            material: SectionMaterial::default(),
        }
    }

    fn def_of(kind: SectionKind) -> SectionDef {
        SectionDef {
            kind,
            params: kind.default_params(),
            material: SectionMaterial::default(),
        }
    }

    #[test]
    fn rectangle_100x200_geometry_matches_closed_form() {
        let (_, p) = rect(100.0, 200.0)
            .compute_geometry()
            .expect("rectangle should compute");

        // A = b·h = 0.1 · 0.2 = 0.02 m²
        assert!((p.area - 0.02).abs() < 1e-12, "A = {}", p.area);

        // ixx = b·h³/12 = 0.1·0.2³/12 = 6.6667e-5 m⁴  → member Iy
        let ixx = 0.1 * 0.2_f64.powi(3) / 12.0;
        // iyy = h·b³/12 = 0.2·0.1³/12 = 1.6667e-5 m⁴  → member Iz
        let iyy = 0.2 * 0.1_f64.powi(3) / 12.0;
        assert!((p.iy - ixx).abs() < 1e-12, "Iy = {}", p.iy);
        assert!((p.iz - iyy).abs() < 1e-12, "Iz = {}", p.iz);

        // Product of inertia is zero for an axis-aligned rectangle.
        assert!(p.ixy.abs() < 1e-12);

        // Centroid at (b/2, h/2) = (0.05, 0.1) m.
        assert!((p.cx - 0.05).abs() < 1e-12);
        assert!((p.cy - 0.10).abs() < 1e-12);

        // Effective stiffnesses scale with E.
        let e = SectionMaterial::default().youngs_modulus();
        assert!((p.ea - e * 0.02).abs() < 1e-6);
        assert!((p.eiy - e * ixx).abs() < 1e-3);
    }

    #[test]
    fn rectangle_iy_is_strong_axis_when_tall() {
        // height > width  ⇒  Iy (about local y, = ixx) is the larger moment,
        // matching the app's default member sections (Iy > Iz).
        let (_, p) = rect(100.0, 200.0).compute_geometry().unwrap();
        assert!(p.iy > p.iz);
    }

    #[test]
    fn rectangle_torsion_matches_roark_within_a_few_percent() {
        // Rectangle 100 (short, b) × 200 (long, h): J ≈ β·h·b³ with
        // h/b = 2 ⇒ β ≈ 0.229 (Roark), i.e. J ≈ 0.229·0.2·0.1³ = 4.58e-5 m⁴.
        let torsion = rect(100.0, 200.0).compute_torsion().unwrap();
        let j_roark = 0.229 * 0.2 * 0.1_f64.powi(3);
        assert!(
            (torsion.j - j_roark).abs() / j_roark < 0.05,
            "J = {} vs Roark {}",
            torsion.j,
            j_roark
        );
        // J for a solid rectangle is much smaller than the polar moment.
        assert!(torsion.j > 0.0);
    }

    #[test]
    fn circle_geometry_approximates_closed_form() {
        // Geometry only — the curved-section torsion FEM is intentionally not
        // exercised here (it is minutes-slow; see module docs).
        let (_, p) = def_of(SectionKind::Circle).compute_geometry().unwrap();
        let r = 0.05_f64;
        let pi = std::f64::consts::PI;
        let a_exact = pi * r * r;
        assert!(
            (p.area - a_exact).abs() / a_exact < 0.01,
            "A = {} vs {}",
            p.area,
            a_exact
        );
        let i_exact = pi * r.powi(4) / 4.0; // π d⁴/64
        assert!((p.iy - i_exact).abs() / i_exact < 0.02);
        assert!((p.iz - i_exact).abs() / i_exact < 0.02);
        // A circle is symmetric: no product of inertia.
        assert!(p.ixy.abs() / i_exact < 1e-9);
    }

    #[test]
    fn tube_area_matches_closed_form() {
        let def = SectionDef {
            kind: SectionKind::Tube,
            params: SectionParams {
                diameter: 100.0,
                thickness: 10.0,
                ..SectionKind::Tube.default_params()
            },
            material: SectionMaterial::default(),
        };
        let (_, p) = def.compute_geometry().unwrap();
        let ro = 0.05_f64;
        let ri = 0.04_f64;
        let a_exact = std::f64::consts::PI * (ro * ro - ri * ri);
        assert!(
            (p.area - a_exact).abs() / a_exact < 0.01,
            "A = {} vs {}",
            p.area,
            a_exact
        );
    }

    #[test]
    fn degenerate_shapes_return_err_not_panic() {
        // Zero-width rectangle.
        assert!(rect(0.0, 200.0).compute_geometry().is_err());
        // Tube with zero wall.
        let tube = SectionDef {
            kind: SectionKind::Tube,
            params: SectionParams {
                diameter: 100.0,
                thickness: 0.0,
                ..SectionKind::Tube.default_params()
            },
            material: SectionMaterial::default(),
        };
        assert!(tube.compute_geometry().is_err());
        // Tube whose wall closes the hole.
        let solid_tube = SectionDef {
            kind: SectionKind::Tube,
            params: SectionParams {
                diameter: 100.0,
                thickness: 50.0,
                ..SectionKind::Tube.default_params()
            },
            material: SectionMaterial::default(),
        };
        assert!(solid_tube.compute_geometry().is_err());
        // Box with wall equal to half the width.
        let box_def = SectionDef {
            kind: SectionKind::Box,
            params: SectionParams {
                width: 100.0,
                height: 100.0,
                thickness: 50.0,
                ..SectionKind::Box.default_params()
            },
            material: SectionMaterial::default(),
        };
        assert!(box_def.compute_geometry().is_err());
        // I-beam with an oversized flange.
        let ib = SectionDef {
            kind: SectionKind::IBeam,
            params: SectionParams {
                height: 100.0,
                width: 100.0,
                thickness: 60.0,
                web_thickness: 7.0,
                ..SectionKind::IBeam.default_params()
            },
            material: SectionMaterial::default(),
        };
        assert!(ib.compute_geometry().is_err());
    }

    #[test]
    fn invalid_custom_material_returns_err() {
        let def = SectionDef {
            kind: SectionKind::Rectangle,
            params: SectionKind::Rectangle.default_params(),
            material: SectionMaterial {
                preset: MaterialPreset::Custom,
                custom_e: -1.0, // non-physical
                custom_nu: 0.3,
            },
        };
        assert!(def.compute_geometry().is_err());
        assert!(def.compute_torsion().is_err());
    }

    #[test]
    fn preset_material_uses_stored_shear_modulus() {
        let def = SectionDef {
            kind: SectionKind::Rectangle,
            params: SectionParams {
                width: 100.0,
                height: 200.0,
                ..SectionKind::Rectangle.default_params()
            },
            material: SectionMaterial {
                preset: MaterialPreset::Steel,
                ..SectionMaterial::default()
            },
        };
        let (_, p) = def.compute_geometry().unwrap();
        let m = presets::STEEL_S355;
        assert!((p.youngs_modulus - m.youngs_modulus).abs() < 1e-6);
        assert!((p.shear_modulus - m.shear_modulus).abs() < 1e-6);

        // G·J must use the stored G, not E/(2(1+ν)).
        let torsion = def.compute_torsion().unwrap();
        assert!((torsion.gj - m.shear_modulus * torsion.j).abs() < 1e-6);
    }

    #[test]
    fn timber_preset_keeps_orthotropic_shear_modulus() {
        // Regression guard mirroring the crate's Phase 18.5 fix: glulam's G
        // must come from the preset, not E/(2(1+ν)).
        let def = SectionDef {
            kind: SectionKind::Rectangle,
            params: SectionKind::Rectangle.default_params(),
            material: SectionMaterial {
                preset: MaterialPreset::Timber,
                ..SectionMaterial::default()
            },
        };
        let (_, p) = def.compute_geometry().unwrap();
        let m = presets::TIMBER_GL24H;
        assert!((p.shear_modulus - m.shear_modulus).abs() < 1e-6);
        let g_iso = m.youngs_modulus / (2.0 * (1.0 + m.poissons_ratio));
        assert!((p.shear_modulus - g_iso).abs() > 1e9);
    }

    #[test]
    fn polygonal_shapes_build_and_compute_geometry() {
        for kind in [SectionKind::IBeam, SectionKind::Box, SectionKind::Triangle] {
            let def = def_of(kind);
            let (_, p) = def
                .compute_geometry()
                .unwrap_or_else(|e| panic!("{kind:?}: {e}"));
            assert!(p.area > 0.0, "{kind:?} area");
            assert!(p.iy > 0.0 && p.iz > 0.0, "{kind:?} I");
        }
    }

    #[test]
    fn full_compute_yields_member_values() {
        let res = rect(100.0, 200.0).compute().unwrap();
        let (e, g, a, iy, iz, j) = res.member_section_values();
        assert_eq!(e, res.geo.youngs_modulus);
        assert_eq!(g, res.geo.shear_modulus);
        assert_eq!(a, res.geo.area);
        assert_eq!(iy, res.geo.iy);
        assert_eq!(iz, res.geo.iz);
        assert_eq!(j, res.torsion.j);
        // Every member-section value must be strictly positive (the
        // `set_member_section` contract).
        for v in [e, g, a, iy, iz, j] {
            assert!(v > 0.0 && v.is_finite(), "value {v}");
        }
    }

    #[test]
    fn outline_reports_outer_loop_and_holes() {
        // Solid rectangle → one loop of four points.
        let solid = rect(100.0, 200.0).build_section().unwrap();
        let loops = outline(&solid);
        assert_eq!(loops.len(), 1);
        assert_eq!(loops[0].len(), 4);

        // Box → outer loop plus one hole loop.
        let boxed = def_of(SectionKind::Box).build_section().unwrap();
        let loops = outline(&boxed);
        assert_eq!(loops.len(), 2);
        assert_eq!(loops[1].len(), 4);
    }

    #[test]
    fn designation_is_unit_aware() {
        assert_eq!(rect(100.0, 200.0).designation(), "RECT 100×200");
        assert_eq!(
            def_of(SectionKind::IBeam).designation(),
            "I 300×150×7.0×10.0"
        );
        assert_eq!(def_of(SectionKind::Tube).designation(), "CHS Ø100×5.0");
    }

    #[test]
    fn torsion_cheap_flag_marks_curved_shapes() {
        assert!(!SectionKind::Circle.torsion_is_cheap());
        assert!(!SectionKind::Tube.torsion_is_cheap());
        assert!(SectionKind::Rectangle.torsion_is_cheap());
        assert!(SectionKind::IBeam.torsion_is_cheap());
    }

    fn circle(diameter_mm: f64) -> SectionDef {
        SectionDef {
            kind: SectionKind::Circle,
            params: SectionParams {
                diameter: diameter_mm,
                ..SectionKind::Circle.default_params()
            },
            material: SectionMaterial::default(),
        }
    }

    fn tube(diameter_mm: f64, thickness_mm: f64) -> SectionDef {
        SectionDef {
            kind: SectionKind::Tube,
            params: SectionParams {
                diameter: diameter_mm,
                thickness: thickness_mm,
                ..SectionKind::Tube.default_params()
            },
            material: SectionMaterial::default(),
        }
    }

    #[test]
    fn solid_circle_analytical_j_matches_closed_form() {
        // d = 100 mm ⇒ r = 0.05 m ⇒ J = π·0.05⁴/2 = 9.817477e-6 m⁴
        let j = analytical_torsion_j(SectionKind::Circle, &circle(100.0).params)
            .expect("circle has a closed form");
        let exact = std::f64::consts::PI * 0.05_f64.powi(4) / 2.0;
        assert!((j - exact).abs() < 1e-18, "J = {j} vs {exact}");
        assert!(
            (j - 9.817477042468103e-6).abs() / 9.817477042468103e-6 < 1e-12,
            "J = {j}"
        );

        // The whole SectionDef path yields the same value, instantly.
        let t = circle(100.0).compute_torsion().unwrap();
        assert!((t.j - exact).abs() < 1e-18, "J = {}", t.j);
        // A circular section does not warp.
        assert_eq!(t.iw, 0.0);
        // G·J uses the stored shear modulus.
        let g = SectionMaterial::default().shear_modulus();
        assert!((t.gj - g * exact).abs() < 1e-6);
    }

    #[test]
    fn tube_analytical_j_matches_closed_form() {
        // Ø100/80 mm ⇒ rₒ = 0.05 m, rᵢ = 0.04 m
        // J = π(0.05⁴ − 0.04⁴)/2 = 5.7962384e-6 m⁴
        let j = analytical_torsion_j(SectionKind::Tube, &tube(100.0, 10.0).params)
            .expect("tube has a closed form");
        let exact = std::f64::consts::PI * (0.05_f64.powi(4) - 0.04_f64.powi(4)) / 2.0;
        assert!((j - exact).abs() < 1e-18, "J = {j} vs {exact}");
        assert!(
            (j - 5.796238445873168e-6).abs() / 5.796238445873168e-6 < 1e-12,
            "J = {j}"
        );

        let t = tube(100.0, 10.0).compute_torsion().unwrap();
        assert!((t.j - exact).abs() < 1e-18);
        assert_eq!(t.iw, 0.0);
    }

    #[test]
    fn analytical_torsion_none_for_polygonal_shapes() {
        for kind in [
            SectionKind::Rectangle,
            SectionKind::IBeam,
            SectionKind::Box,
            SectionKind::Triangle,
        ] {
            assert!(
                analytical_torsion_j(kind, &kind.default_params()).is_none(),
                "{kind:?} must fall back to the FEM"
            );
            assert!(!kind.has_analytical_torsion());
        }
        assert!(SectionKind::Circle.has_analytical_torsion());
        assert!(SectionKind::Tube.has_analytical_torsion());
    }

    #[test]
    fn degenerate_circle_torsion_returns_err_not_panic() {
        // Closed-form path must still reject non-physical dimensions.
        assert!(circle(0.0).compute_torsion().is_err());
        // Tube whose wall closes the hole (thickness ≥ radius).
        assert!(tube(100.0, 50.0).compute_torsion().is_err());
        // Invalid custom material is rejected before any value is returned.
        let bad = SectionDef {
            material: SectionMaterial {
                preset: MaterialPreset::Custom,
                custom_e: -1.0,
                custom_nu: 0.3,
            },
            ..circle(100.0)
        };
        assert!(bad.compute_torsion().is_err());
    }

    #[test]
    fn circle_analytical_j_matches_polygon_polar_moment() {
        // Cheap cross-check (no FEM): for a solid circular section the torsion
        // constant equals the polar second moment of area, J = Ixx + Iyy.  The
        // crate's 64-gon geometry approximation should agree to well under 1%.
        let def = circle(100.0);
        let (_, p) = def.compute_geometry().unwrap();
        let polar = p.iy + p.iz;
        let j = def.compute_torsion().unwrap().j;
        assert!(
            (j - polar).abs() / polar < 5e-3,
            "J = {j} vs polar moment {polar}"
        );
    }
}
