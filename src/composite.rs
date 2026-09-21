//! Elastic composite section analysis using the modular-ratio (transformed-
//! section) method.
//!
//! # Overview
//!
//! For a multi-material cross-section, each component's geometric properties
//! are transformed to an equivalent single-material section by multiplying
//! area and moments of inertia by the **modular ratio**:
//!
//! ```text
//! n_i = E_i / E_ref
//! ```
//!
//! where `E_i` is the Young's modulus of component *i* and `E_ref` is the
//! Young's modulus of a chosen reference material.  The transformed section
//! is then analysed as a homogeneous section to obtain composite area,
//! centroid, and second moments of area.
//!
//! # Coordinate convention
//!
//! This module follows the same convention as the rest of the crate:
//!
//! * `x` — horizontal axis (positive right)
//! * `y` — vertical axis (positive up)
//! * `Ix` — second moment of area about the centroidal x-axis (`∫y² dA`)
//! * `Iy` — second moment of area about the centroidal y-axis (`∫x² dA`)
//! * `Ixy` — product of inertia (`∫xy dA`, signed)
//! * Principal angle `phi` — CCW positive, `½ atan2(2·Ixy, Ix − Iy)`
//!
//! # Limitations
//!
//! This module implements **elastic transformed-section analysis** only.
//! It does **not** implement:
//!
//! * EN 1994 composite design (plastic resistance, shear connection)
//! * Creep or shrinkage of concrete
//! * Cracked-section analysis
//! * Construction-stage analysis
//! * Fire design
//! * Plastic composite analysis
//! * Stress recovery across material interfaces
//!
//! # Quick start
//!
//! ```rust
//! use section_properties::composite::*;
//! use section_properties::material::presets::*;
//! use section_properties::{Point, Polygon, Section};
//!
//! // Concrete slab 0.3 m × 0.15 m
//! let slab = Section::new(
//!     Polygon::new(vec![
//!         Point::new(0.0, 0.0),
//!         Point::new(0.3, 0.0),
//!         Point::new(0.3, 0.15),
//!         Point::new(0.0, 0.15),
//!     ]),
//!     Vec::new(),
//! );
//!
//! // Steel plate 0.2 m × 0.01 m on top
//! let plate = Section::new(
//!     Polygon::new(vec![
//!         Point::new(0.05, 0.15),
//!         Point::new(0.25, 0.15),
//!         Point::new(0.25, 0.16),
//!         Point::new(0.05, 0.16),
//!     ]),
//!     Vec::new(),
//! );
//!
//! let composite = ElasticComposite::new(vec![
//!     CompositeComponent::new(slab, CONCRETE_C30_37).unwrap(),
//!     CompositeComponent::new(plate, STEEL_S355).unwrap(),
//! ]).unwrap();
//!
//! // Analyse with concrete as the reference material
//! let result = composite.analyze(&CONCRETE_C30_37).unwrap();
//! assert!(result.area > 0.0);
//! assert!(result.ix > 0.0);
//! ```

use crate::geometry::Point;
use crate::material::Material;
use crate::section::Section;
use crate::section_properties::SectionProperties;
use thiserror::Error;

/// Errors that can occur during composite section construction or analysis.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum CompositeError {
    /// Young's modulus is zero, negative, NaN, or infinite.
    #[error("invalid Young's modulus for material '{name}': E={value}")]
    InvalidModulus { name: String, value: f64 },

    /// Section geometry is degenerate (zero or near-zero net area).
    #[error("degenerate section geometry in component {index}: {detail}")]
    InvalidGeometry { index: usize, detail: String },

    /// No components were provided.
    #[error("composite section must contain at least one component")]
    NoComponents,

    /// The transformed section has zero or near-zero net area.
    #[error("transformed section net area is zero or near-zero")]
    ZeroTransformedArea,

    /// The reference material modulus is invalid.
    #[error("invalid reference material modulus: E={value}")]
    InvalidReferenceModulus { value: f64 },

    /// The modular ratio `E_i / E_ref` is not finite (overflow).
    #[error(
        "non-finite modular ratio for component {index}: E={e_component:.6e} / E_ref={e_ref:.6e}"
    )]
    NonFiniteModularRatio {
        index: usize,
        e_component: f64,
        e_ref: f64,
    },

    /// A derived section property (centroid, centroidal inertia, or principal-axis
    /// quantity) is not finite even though all input components were finite.
    ///
    /// This indicates that the combination of coordinates, areas, and modular
    /// ratios causes an intermediate arithmetic overflow (e.g. `diff * diff`
    /// in the Mohr's-circle principal-axis calculation) or a catastrophic
    /// cancellation that produces `Inf` or `NaN`.
    #[error("non-finite derived section property at stage '{stage}': {detail}")]
    NonFiniteDerivedProperty { stage: &'static str, detail: String },
}

/// A geometric section component paired with a material.
///
/// The section geometry is owned (cloned) so that the original user data
/// is never mutated.  The transformed section is an analytical
/// representation only — the original geometry remains recoverable.
#[derive(Debug, Clone)]
pub struct CompositeComponent {
    /// The geometric section (outer boundary + optional holes).
    pub section: Section,
    /// The isotropic linear-elastic material assigned to this component.
    pub material: Material,
}

impl CompositeComponent {
    /// Create a new composite component from a section and a material.
    ///
    /// Returns `Err` if the Young's modulus is not a positive, finite value
    /// or if the section geometry is degenerate.
    pub fn new(section: Section, material: Material) -> Result<Self, CompositeError> {
        validate_modulus(material.youngs_modulus, material.name)?;
        let area = section.area();
        if area.abs() <= f64::EPSILON {
            return Err(CompositeError::InvalidGeometry {
                index: 0,
                detail: format!("net area = {area:.6e} (too small)"),
            });
        }
        if !area.is_finite() {
            return Err(CompositeError::InvalidGeometry {
                index: 0,
                detail: format!("net area = {area} (not finite)"),
            });
        }
        Ok(Self { section, material })
    }

    /// The Young's modulus of this component's material.
    pub fn youngs_modulus(&self) -> f64 {
        self.material.youngs_modulus
    }

    /// The net geometric area of this component's section.
    pub fn area(&self) -> f64 {
        self.section.area()
    }
}

fn validate_modulus(e: f64, name: &str) -> Result<(), CompositeError> {
    if !e.is_finite() || e <= 0.0 {
        return Err(CompositeError::InvalidModulus {
            name: name.to_string(),
            value: e,
        });
    }
    Ok(())
}

/// Per-component result of a composite section analysis.
///
/// Retains enough information to recover the original material properties,
/// the modular ratio used, and the original vs transformed area.
#[derive(Debug, Clone)]
pub struct ComponentAnalysis {
    /// The original material assigned to this component.
    pub material: Material,
    /// Modular ratio `n = E_i / E_ref` used for this component.
    pub modular_ratio: f64,
    /// Original (geometric) net area of the component \[m²\].
    pub original_area: f64,
    /// Transformed area `n × A_original` \[m²\].
    pub transformed_area: f64,
    /// Centroid of the component in global coordinates.
    pub centroid: Point,
}

/// Result of an elastic composite section analysis.
///
/// Contains the transformed section properties (equivalent homogeneous
/// section in the reference material) plus per-component analysis data.
#[derive(Debug, Clone)]
pub struct CompositeAnalysisResult {
    /// Transformed area `Σ(n_i · A_i)` \[m²\].
    pub area: f64,
    /// Centroid of the transformed section in global coordinates.
    pub centroid: Point,
    /// Transformed second moment of area about the centroidal x-axis \[m⁴\].
    ///
    /// This is the **reference-material transformed** value `I'_x`, not the
    /// physical inertia.  The physical bending stiffness is `E_ref · I'_x`.
    pub ix: f64,
    /// Transformed second moment of area about the centroidal y-axis \[m⁴\].
    ///
    /// This is the **reference-material transformed** value `I'_y`, not the
    /// physical inertia.  The physical bending stiffness is `E_ref · I'_y`.
    pub iy: f64,
    /// Transformed product of inertia about the centroidal axes \[m⁴\].
    ///
    /// This is the **reference-material transformed** value `I'_xy`.
    /// The physical value is `E_ref · I'_xy`.
    pub ixy: f64,
    /// Major principal second moment of area \[m⁴\].
    pub principal_i11: f64,
    /// Minor principal second moment of area \[m⁴\].
    pub principal_i22: f64,
    /// Principal angle from centroidal x-axis to axis 11, CCW positive \[rad\].
    pub principal_phi: f64,
    /// The reference material used for the transformation.
    pub reference_material: Material,
    /// Per-component analysis data.
    pub components: Vec<ComponentAnalysis>,
}

/// A multi-material composite section for elastic transformed-section analysis.
///
/// Components are stored in insertion order.  The original geometry is
/// never mutated — [`analyze`](Self::analyze) produces an analytical
/// transformed representation only.
///
/// # Component semantics
///
/// Each component represents a **distinct physical material region**.
/// Components are summed algebraically — if two components overlap
/// geometrically, the overlapping region is **double-counted** in the
/// transformed section.  No Boolean union is performed.  Callers are
/// responsible for ensuring components represent disjoint regions.
#[derive(Debug, Clone)]
pub struct ElasticComposite {
    components: Vec<CompositeComponent>,
}

impl ElasticComposite {
    /// Create a new composite section from a list of components.
    ///
    /// Returns `Err` if the list is empty or if any component has invalid
    /// material or geometry.
    pub fn new(components: Vec<CompositeComponent>) -> Result<Self, CompositeError> {
        if components.is_empty() {
            return Err(CompositeError::NoComponents);
        }
        Ok(Self { components })
    }

    /// Number of components.
    pub fn n_components(&self) -> usize {
        self.components.len()
    }

    /// Borrow the component at `index`.
    pub fn component(&self, index: usize) -> Option<&CompositeComponent> {
        self.components.get(index)
    }

    /// Analyse the composite section using the transformed-section method.
    ///
    /// # Arguments
    ///
    /// * `reference_material` — the reference material whose Young's modulus
    ///   `E_ref` defines the modular ratio `n_i = E_i / E_ref`.  Must have
    ///   a positive, finite Young's modulus.
    ///
    /// The reference material **need not** be one of the component materials.
    /// Any positive, finite modulus produces a valid transformed section.
    /// The physical stiffnesses `E_ref · A'`, `E_ref · I'_x`, etc. are
    /// invariant to the choice of reference material.
    ///
    /// # Algorithm
    ///
    /// 1. For each component, compute geometric properties (area, centroid,
    ///    centroidal moments) via [`SectionProperties`].
    /// 2. Transfer centroidal moments to global axes using the parallel-axis
    ///    theorem.
    /// 3. Multiply each component's contribution by its modular ratio `n_i`.
    /// 4. Sum to obtain the transformed section's global moments.
    /// 5. Compute the transformed centroid and transfer back to centroidal
    ///    axes.
    /// 6. Compute principal axes via Mohr's circle.
    ///
    /// # Errors
    ///
    /// Returns `Err` if:
    /// * the reference material modulus is invalid,
    /// * any component's geometry is degenerate,
    /// * the transformed net area is zero or near-zero.
    pub fn analyze(
        &self,
        reference_material: &Material,
    ) -> Result<CompositeAnalysisResult, CompositeError> {
        validate_modulus(reference_material.youngs_modulus, reference_material.name).map_err(
            |_| CompositeError::InvalidReferenceModulus {
                value: reference_material.youngs_modulus,
            },
        )?;

        let e_ref = reference_material.youngs_modulus;

        // --- Pass 1: composite centroid ---
        // Accumulate transformed area and first moments only.
        // Per-component inertia data is stored for Pass 2.
        let mut total_area = 0.0_f64;
        let mut first_x = 0.0_f64;
        let mut first_y = 0.0_f64;

        let mut component_results: Vec<ComponentAnalysis> =
            Vec::with_capacity(self.components.len());

        // (n, area, cx, cy, ix, iy, ixy) for each component
        let mut comp_data: Vec<(f64, f64, f64, f64, f64, f64, f64)> =
            Vec::with_capacity(self.components.len());

        for (i, comp) in self.components.iter().enumerate() {
            let props = SectionProperties::try_from_section(&comp.section)
                .map_err(|detail| CompositeError::InvalidGeometry { index: i, detail })?;

            let n = comp.material.youngs_modulus / e_ref;
            if !n.is_finite() {
                return Err(CompositeError::NonFiniteModularRatio {
                    index: i,
                    e_component: comp.material.youngs_modulus,
                    e_ref,
                });
            }
            let area = props.area;
            let cx = props.centroid.x;
            let cy = props.centroid.y;

            let n_area = n * area;
            if !n_area.is_finite() {
                return Err(CompositeError::NonFiniteModularRatio {
                    index: i,
                    e_component: comp.material.youngs_modulus,
                    e_ref,
                });
            }
            total_area += n_area;
            first_x += n_area * cx;
            first_y += n_area * cy;

            comp_data.push((n, area, cx, cy, props.ix, props.iy, props.ixy));
            component_results.push(ComponentAnalysis {
                material: comp.material,
                modular_ratio: n,
                original_area: area,
                transformed_area: n_area,
                centroid: props.centroid,
            });
        }

        if total_area.abs() <= f64::EPSILON {
            return Err(CompositeError::ZeroTransformedArea);
        }
        if !total_area.is_finite() || !first_x.is_finite() || !first_y.is_finite() {
            return Err(CompositeError::NonFiniteDerivedProperty {
                stage: "weighted_accumulation",
                detail: format!(
                    "total_area={}, first_x={}, first_y={}",
                    total_area, first_x, first_y
                ),
            });
        }

        // Transformed centroid.
        let centroid = Point::new(first_x / total_area, first_y / total_area);
        if !centroid.x.is_finite() || !centroid.y.is_finite() {
            return Err(CompositeError::NonFiniteDerivedProperty {
                stage: "centroid",
                detail: format!(
                    "centroid = ({}, {}) from first_x={}, first_y={}, total_area={}",
                    centroid.x, centroid.y, first_x, first_y, total_area
                ),
            });
        }

        // --- Pass 2: direct centroidal accumulation ---
        // Accumulate inertia directly about the composite centroid using
        // the parallel-axis theorem with (y_i - cy) offsets.  This avoids
        // the catastrophic cancellation that occurs when subtracting
        // large `ix_global - total_area * cy^2` values.
        let mut ix_c = 0.0_f64;
        let mut iy_c = 0.0_f64;
        let mut ixy_c = 0.0_f64;
        for &(n, area, cx, cy, ix_i, iy_i, ixy_i) in &comp_data {
            let dx = cx - centroid.x;
            let dy = cy - centroid.y;
            ix_c += n * (ix_i + area * dy * dy);
            iy_c += n * (iy_i + area * dx * dx);
            ixy_c += n * (ixy_i + area * dx * dy);
        }
        if !ix_c.is_finite() || !iy_c.is_finite() || !ixy_c.is_finite() {
            return Err(CompositeError::NonFiniteDerivedProperty {
                stage: "centroidal_inertia",
                detail: format!(
                    "ix_c={}, iy_c={}, ixy_c={} (centroid=({}, {}), total_area={})",
                    ix_c, iy_c, ixy_c, centroid.x, centroid.y, total_area
                ),
            });
        }

        // Principal axes (Mohr's circle, same convention as SectionProperties).
        let avg = (ix_c + iy_c) * 0.5;
        let diff = (ix_c - iy_c) * 0.5;
        let radius = (diff * diff + ixy_c * ixy_c).sqrt();
        let i11 = avg + radius;
        let i22 = avg - radius;
        let phi = 0.5 * (2.0 * ixy_c).atan2(ix_c - iy_c);
        if !i11.is_finite() || !i22.is_finite() || !phi.is_finite() {
            return Err(CompositeError::NonFiniteDerivedProperty {
                stage: "principal_axes",
                detail: format!(
                    "i11={}, i22={}, phi={} (ix_c={}, iy_c={}, ixy_c={})",
                    i11, i22, phi, ix_c, iy_c, ixy_c
                ),
            });
        }

        Ok(CompositeAnalysisResult {
            area: total_area,
            centroid,
            ix: ix_c,
            iy: iy_c,
            ixy: ixy_c,
            principal_i11: i11,
            principal_i22: i22,
            principal_phi: phi,
            reference_material: *reference_material,
            components: component_results,
        })
    }

    /// Convenience: analyse using the first component's material as reference.
    ///
    /// # Errors
    ///
    /// Same as [`analyze`](Self::analyze).
    pub fn analyze_with_first_reference(&self) -> Result<CompositeAnalysisResult, CompositeError> {
        let ref_mat = self
            .components
            .first()
            .map(|c| c.material)
            .ok_or(CompositeError::NoComponents)?;
        self.analyze(&ref_mat)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::presets::*;
    use crate::{Point, Polygon};

    fn rect_section(x0: f64, y0: f64, w: f64, h: f64) -> Section {
        Section::new(
            Polygon::new(vec![
                Point::new(x0, y0),
                Point::new(x0 + w, y0),
                Point::new(x0 + w, y0 + h),
                Point::new(x0, y0 + h),
            ]),
            Vec::new(),
        )
    }

    #[test]
    fn single_material_degeneration() {
        let sec = rect_section(0.0, 0.0, 0.3, 0.5);
        let comp = ElasticComposite::new(vec![
            CompositeComponent::new(sec.clone(), CONCRETE_C30_37).unwrap(),
        ])
        .unwrap();

        let result = comp.analyze(&CONCRETE_C30_37).unwrap();
        let homogeneous = SectionProperties::from_section(&sec);

        let tol = 1e-10;
        assert!((result.area - homogeneous.area).abs() < tol);
        assert!((result.ix - homogeneous.ix).abs() < tol);
        assert!((result.iy - homogeneous.iy).abs() < tol);
        assert!((result.ixy - homogeneous.ixy).abs() < tol);
    }

    #[test]
    fn two_material_rectangular_analytical() {
        // Two stacked rectangles:
        //   bottom: 0.2 × 0.1, E = 30 GPa, centroid (0.1, 0.05)
        //   top:    0.2 × 0.1, E = 60 GPa, centroid (0.1, 0.15)
        // Reference: E_ref = 30 GPa → n_bottom = 1, n_top = 2

        let e_ref = 30e9;
        let e_bottom = 30e9;
        let e_top = 60e9;

        let mat_ref = Material::new(e_ref, 0.2, 2400.0, "ref");
        let mat_bottom = Material::new(e_bottom, 0.2, 2400.0, "bottom");
        let mat_top = Material::new(e_top, 0.2, 2400.0, "top");

        let bottom = rect_section(0.0, 0.0, 0.2, 0.1);
        let top = rect_section(0.0, 0.1, 0.2, 0.1);

        let comp = ElasticComposite::new(vec![
            CompositeComponent::new(bottom, mat_bottom).unwrap(),
            CompositeComponent::new(top, mat_top).unwrap(),
        ])
        .unwrap();

        let result = comp.analyze(&mat_ref).unwrap();

        // Analytical:
        // A' = 1·(0.02) + 2·(0.02) = 0.06
        let a_bottom = 0.2 * 0.1_f64;
        let a_top = 0.2 * 0.1_f64;
        let n_bottom = e_bottom / e_ref;
        let n_top = e_top / e_ref;
        let a_transformed = n_bottom * a_bottom + n_top * a_top;
        assert!((result.area - a_transformed).abs() < 1e-12);

        // y_bar = (1·0.02·0.05 + 2·0.02·0.15) / 0.06
        let y_bar = (n_bottom * a_bottom * 0.05 + n_top * a_top * 0.15) / a_transformed;
        assert!((result.centroid.y - y_bar).abs() < 1e-12);
        assert!((result.centroid.x - 0.1).abs() < 1e-12);

        // Ix about centroidal:
        // Ix_bottom,c = b·h³/12 + A·(cy - y_bar)²
        let ix_bottom_local = 0.2 * 0.1_f64.powi(3) / 12.0;
        let ix_top_local = 0.2 * 0.1_f64.powi(3) / 12.0;
        let ix_analytical = n_bottom * (ix_bottom_local + a_bottom * (0.05 - y_bar).powi(2))
            + n_top * (ix_top_local + a_top * (0.15 - y_bar).powi(2));
        assert!((result.ix - ix_analytical).abs() < 1e-12);

        // Iy about centroidal (both rectangles have same x-centroid = 0.1):
        let iy_bottom_local = 0.1 * 0.2_f64.powi(3) / 12.0;
        let iy_top_local = 0.1 * 0.2_f64.powi(3) / 12.0;
        let iy_analytical = n_bottom * iy_bottom_local + n_top * iy_top_local;
        assert!((result.iy - iy_analytical).abs() < 1e-12);

        // Ixy = 0 (symmetric about x = 0.1)
        assert!(result.ixy.abs() < 1e-12);
    }

    #[test]
    fn steel_concrete_composite() {
        let slab = rect_section(0.0, 0.0, 0.3, 0.15);
        let steel = rect_section(0.05, 0.15, 0.2, 0.01);

        let comp = ElasticComposite::new(vec![
            CompositeComponent::new(slab, CONCRETE_C30_37).unwrap(),
            CompositeComponent::new(steel, STEEL_S355).unwrap(),
        ])
        .unwrap();

        // Reference: concrete
        let result = comp.analyze(&CONCRETE_C30_37).unwrap();

        let n_steel = STEEL_S355.youngs_modulus / CONCRETE_C30_37.youngs_modulus;
        let n_conc = 1.0_f64;

        // Check modular ratios in component results
        assert!((result.components[0].modular_ratio - n_conc).abs() < 1e-10);
        assert!((result.components[1].modular_ratio - n_steel).abs() < 1e-10);

        // Transformed area = A_conc + n_steel * A_steel
        let a_conc = 0.3 * 0.15;
        let a_steel = 0.2 * 0.01;
        let expected_area = n_conc * a_conc + n_steel * a_steel;
        assert!((result.area - expected_area).abs() / expected_area < 1e-10);

        // Original areas preserved
        assert!((result.components[0].original_area - a_conc).abs() < 1e-12);
        assert!((result.components[1].original_area - a_steel).abs() < 1e-12);

        // Transformed areas
        assert!((result.components[0].transformed_area - n_conc * a_conc).abs() < 1e-12);
        assert!((result.components[1].transformed_area - n_steel * a_steel).abs() < 1e-12);
    }

    #[test]
    fn reference_material_reversal() {
        // Use an asymmetric section so Iy and Ixy are non-trivial.
        // Slab offset left, steel plate offset right → non-zero Ixy.
        let slab = rect_section(0.0, 0.0, 0.3, 0.15);
        let steel = rect_section(0.15, 0.15, 0.2, 0.01);

        let comp = ElasticComposite::new(vec![
            CompositeComponent::new(slab, CONCRETE_C30_37).unwrap(),
            CompositeComponent::new(steel, STEEL_S355).unwrap(),
        ])
        .unwrap();

        let r_conc = comp.analyze(&CONCRETE_C30_37).unwrap();
        let r_steel = comp.analyze(&STEEL_S355).unwrap();

        // Physical stiffnesses are reference-independent.
        // EA = E_ref * A'
        let ea_conc = CONCRETE_C30_37.youngs_modulus * r_conc.area;
        let ea_steel = STEEL_S355.youngs_modulus * r_steel.area;
        assert!((ea_conc - ea_steel).abs() / ea_conc < 1e-10);

        // EIx = E_ref * Ix'
        let ei_conc = CONCRETE_C30_37.youngs_modulus * r_conc.ix;
        let ei_steel = STEEL_S355.youngs_modulus * r_steel.ix;
        assert!((ei_conc - ei_steel).abs() / ei_conc < 1e-10);

        // EIy = E_ref * Iy'
        let eiy_conc = CONCRETE_C30_37.youngs_modulus * r_conc.iy;
        let eiy_steel = STEEL_S355.youngs_modulus * r_steel.iy;
        assert!((eiy_conc - eiy_steel).abs() / eiy_conc < 1e-10);

        // EIxy = E_ref * Ixy' (use absolute tolerance — Ixy can be near zero)
        let eixy_conc = CONCRETE_C30_37.youngs_modulus * r_conc.ixy;
        let eixy_steel = STEEL_S355.youngs_modulus * r_steel.ixy;
        assert!((eixy_conc - eixy_steel).abs() < 1e-3);

        // Centroid must be the same (physical location)
        assert!((r_conc.centroid.x - r_steel.centroid.x).abs() < 1e-10);
        assert!((r_conc.centroid.y - r_steel.centroid.y).abs() < 1e-10);
    }

    #[test]
    fn symmetric_composite() {
        // Two identical rectangles symmetric about y-axis:
        //   left:  0.1 × 0.2 at x = [-0.15, -0.05], E = 30 GPa
        //   right: 0.1 × 0.2 at x = [0.05, 0.15],  E = 200 GPa
        // Despite different materials, geometry is symmetric → x_bar = 0
        // but the centroid shifts towards the stiffer side.
        //
        // Actually with different E, the *transformed* centroid shifts.
        // For a true symmetry test we need same material on both sides.

        let left = rect_section(-0.15, -0.1, 0.1, 0.2);
        let right = rect_section(0.05, -0.1, 0.1, 0.2);

        let mat = Material::new(30e9, 0.2, 2400.0, "same");

        let comp = ElasticComposite::new(vec![
            CompositeComponent::new(left, mat).unwrap(),
            CompositeComponent::new(right, mat).unwrap(),
        ])
        .unwrap();

        let result = comp.analyze(&mat).unwrap();

        // Symmetric → centroid at origin
        assert!(result.centroid.x.abs() < 1e-12);
        assert!(result.centroid.y.abs() < 1e-12);

        // Ixy = 0 for doubly-symmetric section
        assert!(result.ixy.abs() < 1e-12);
    }

    #[test]
    fn material_position_sensitivity() {
        // Two scenarios:
        //   A: stiff material on top, soft on bottom
        //   B: stiff material on bottom, soft on top
        // The centroid must move towards the stiff material.

        let e_soft = 30e9;
        let e_stiff = 200e9;
        let mat_soft = Material::new(e_soft, 0.2, 2400.0, "soft");
        let mat_stiff = Material::new(e_stiff, 0.2, 2400.0, "stiff");

        let bottom = rect_section(0.0, 0.0, 0.2, 0.1);
        let top = rect_section(0.0, 0.1, 0.2, 0.1);

        // Scenario A: stiff on top
        let comp_a = ElasticComposite::new(vec![
            CompositeComponent::new(bottom.clone(), mat_soft).unwrap(),
            CompositeComponent::new(top.clone(), mat_stiff).unwrap(),
        ])
        .unwrap();
        let result_a = comp_a.analyze(&mat_soft).unwrap();

        // Scenario B: stiff on bottom
        let comp_b = ElasticComposite::new(vec![
            CompositeComponent::new(bottom, mat_stiff).unwrap(),
            CompositeComponent::new(top, mat_soft).unwrap(),
        ])
        .unwrap();
        let result_b = comp_b.analyze(&mat_soft).unwrap();

        // Geometric centroid of the two stacked rectangles is at y = 0.1.
        // In A, stiff on top → transformed centroid shifts above 0.1.
        assert!(
            result_a.centroid.y > 0.1,
            "centroid should shift towards stiff top"
        );
        // In B, stiff on bottom → transformed centroid shifts below 0.1.
        assert!(
            result_b.centroid.y < 0.1,
            "centroid should shift towards stiff bottom"
        );

        // The two centroids must be different
        assert!((result_a.centroid.y - result_b.centroid.y).abs() > 0.01);

        // Ix is identical by the y ↔ (0.2 − y) symmetry of the geometry.
        assert!((result_a.ix - result_b.ix).abs() < 1e-10);

        // The result is not merely based on total transformed area:
        // a single-material section with the same total transformed area
        // and same overall bounding box would have a different Ix.
        let total_transformed_area = result_a.area;
        let single_mat_ix = 0.2 * 0.2_f64.powi(3) / 12.0 * (total_transformed_area / (0.2 * 0.2));
        assert!((result_a.ix - single_mat_ix).abs() > 1e-6);
    }

    #[test]
    fn geometric_scaling() {
        let e1 = 30e9;
        let e2 = 200e9;
        let mat1 = Material::new(e1, 0.2, 2400.0, "m1");
        let mat2 = Material::new(e2, 0.3, 7850.0, "m2");

        let alpha = 2.5_f64;

        // Original
        let s1 = rect_section(0.0, 0.0, 0.2, 0.1);
        let s2 = rect_section(0.1, 0.1, 0.1, 0.05);

        let comp_orig = ElasticComposite::new(vec![
            CompositeComponent::new(s1.clone(), mat1).unwrap(),
            CompositeComponent::new(s2.clone(), mat2).unwrap(),
        ])
        .unwrap();
        let r_orig = comp_orig.analyze(&mat1).unwrap();

        // Scaled by alpha
        let s1_scaled = rect_section(0.0, 0.0, 0.2 * alpha, 0.1 * alpha);
        let s2_scaled = rect_section(0.1 * alpha, 0.1 * alpha, 0.1 * alpha, 0.05 * alpha);

        let comp_scaled = ElasticComposite::new(vec![
            CompositeComponent::new(s1_scaled, mat1).unwrap(),
            CompositeComponent::new(s2_scaled, mat2).unwrap(),
        ])
        .unwrap();
        let r_scaled = comp_scaled.analyze(&mat1).unwrap();

        // A ~ α²
        assert!((r_scaled.area / r_orig.area - alpha * alpha).abs() < 1e-10);
        // centroid ~ α
        assert!((r_scaled.centroid.x / r_orig.centroid.x - alpha).abs() < 1e-10);
        assert!((r_scaled.centroid.y / r_orig.centroid.y - alpha).abs() < 1e-10);
        // Ix, Iy, Ixy ~ α⁴
        assert!((r_scaled.ix / r_orig.ix - alpha.powi(4)).abs() < 1e-10);
        assert!((r_scaled.iy / r_orig.iy - alpha.powi(4)).abs() < 1e-10);
        assert!((r_scaled.ixy / r_orig.ixy - alpha.powi(4)).abs() < 1e-10);
    }

    #[test]
    fn invalid_modulus_zero() {
        let sec = rect_section(0.0, 0.0, 0.1, 0.1);
        let mat = Material::new(0.0, 0.2, 2400.0, "zero E");
        assert!(CompositeComponent::new(sec, mat).is_err());
    }

    #[test]
    fn invalid_modulus_negative() {
        let sec = rect_section(0.0, 0.0, 0.1, 0.1);
        let mat = Material::new(-1e9, 0.2, 2400.0, "negative E");
        assert!(CompositeComponent::new(sec, mat).is_err());
    }

    #[test]
    fn invalid_modulus_nan() {
        let sec = rect_section(0.0, 0.0, 0.1, 0.1);
        let mat = Material::new(f64::NAN, 0.2, 2400.0, "NaN E");
        assert!(CompositeComponent::new(sec, mat).is_err());
    }

    #[test]
    fn invalid_modulus_infinity() {
        let sec = rect_section(0.0, 0.0, 0.1, 0.1);
        let mat = Material::new(f64::INFINITY, 0.2, 2400.0, "inf E");
        assert!(CompositeComponent::new(sec, mat).is_err());
    }

    #[test]
    fn invalid_reference_material() {
        let sec = rect_section(0.0, 0.0, 0.1, 0.1);
        let mat = Material::new(200e9, 0.3, 7850.0, "steel");
        let comp = ElasticComposite::new(vec![CompositeComponent::new(sec, mat).unwrap()]).unwrap();

        let bad_ref = Material::new(0.0, 0.2, 2400.0, "zero");
        assert!(comp.analyze(&bad_ref).is_err());

        let nan_ref = Material::new(f64::NAN, 0.2, 2400.0, "nan");
        assert!(comp.analyze(&nan_ref).is_err());
    }

    #[test]
    fn empty_components_rejected() {
        let result = ElasticComposite::new(Vec::new());
        assert!(result.is_err());
    }

    #[test]
    fn original_geometry_preserved() {
        let sec = rect_section(0.0, 0.0, 0.3, 0.5);
        let sec_clone = sec.clone();
        let mat = Material::new(30e9, 0.2, 2400.0, "concrete");

        let comp = ElasticComposite::new(vec![CompositeComponent::new(sec, mat).unwrap()]).unwrap();

        let _ = comp.analyze(&mat).unwrap();

        // Original geometry in the component must be unchanged
        let stored = comp.component(0).unwrap();
        assert_eq!(stored.section.area(), sec_clone.area());
        assert_eq!(stored.section.centroid().x, sec_clone.centroid().x);
        assert_eq!(stored.section.centroid().y, sec_clone.centroid().y);
    }

    // ---- Phase 33 additional tests ----

    #[test]
    fn composite_with_holes() {
        // Hollow rectangular section as a single component with a hole.
        let outer = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(0.4, 0.0),
            Point::new(0.4, 0.3),
            Point::new(0.0, 0.3),
        ]);
        let hole = Polygon::new(vec![
            Point::new(0.1, 0.05),
            Point::new(0.3, 0.05),
            Point::new(0.3, 0.25),
            Point::new(0.1, 0.25),
        ]);
        let section = Section::new(outer, vec![hole]);
        let mat = Material::new(200e9, 0.3, 7850.0, "steel");

        let comp =
            ElasticComposite::new(vec![CompositeComponent::new(section.clone(), mat).unwrap()])
                .unwrap();
        let result = comp.analyze(&mat).unwrap();
        let homogeneous = SectionProperties::from_section(&section);

        // Single material → transformed == homogeneous
        let tol = 1e-10;
        assert!((result.area - homogeneous.area).abs() < tol);
        assert!((result.ix - homogeneous.ix).abs() < tol);
        assert!((result.iy - homogeneous.iy).abs() < tol);
        assert!((result.ixy - homogeneous.ixy).abs() < tol);

        // Net area = 0.12 - 0.04 = 0.08
        assert!((result.area - 0.08).abs() < 1e-10);
    }

    #[test]
    fn three_material_components() {
        // Three stacked rectangles with different materials.
        let mat1 = Material::new(30e9, 0.2, 2400.0, "concrete");
        let mat2 = Material::new(200e9, 0.3, 7850.0, "steel");
        let mat3 = Material::new(70e9, 0.33, 2700.0, "aluminum");

        let s1 = rect_section(0.0, 0.0, 0.3, 0.1);
        let s2 = rect_section(0.0, 0.1, 0.2, 0.05);
        let s3 = rect_section(0.0, 0.15, 0.25, 0.08);

        let comp = ElasticComposite::new(vec![
            CompositeComponent::new(s1, mat1).unwrap(),
            CompositeComponent::new(s2, mat2).unwrap(),
            CompositeComponent::new(s3, mat3).unwrap(),
        ])
        .unwrap();

        assert_eq!(comp.n_components(), 3);

        let result = comp.analyze(&mat1).unwrap();
        assert!(result.area > 0.0);
        assert!(result.ix > 0.0);
        assert_eq!(result.components.len(), 3);

        // Modular ratios
        assert!((result.components[0].modular_ratio - 1.0).abs() < 1e-10);
        assert!((result.components[1].modular_ratio - 200.0 / 30.0).abs() < 1e-10);
        assert!((result.components[2].modular_ratio - 70.0 / 30.0).abs() < 1e-10);

        // Transformed area = Σ n_i * A_i
        let expected_area =
            1.0 * (0.3 * 0.1) + (200.0 / 30.0) * (0.2 * 0.05) + (70.0 / 30.0) * (0.25 * 0.08);
        assert!((result.area - expected_area).abs() / expected_area < 1e-10);

        // Physical stiffness is reference-invariant
        let r2 = comp.analyze(&mat2).unwrap();
        let ea1 = mat1.youngs_modulus * result.area;
        let ea2 = mat2.youngs_modulus * r2.area;
        assert!((ea1 - ea2).abs() / ea1 < 1e-10);
    }

    #[test]
    fn reference_not_present_in_components() {
        // Reference material not among component materials — mathematically valid.
        let mat1 = Material::new(30e9, 0.2, 2400.0, "concrete");
        let mat2 = Material::new(200e9, 0.3, 7850.0, "steel");
        let mat_ref = Material::new(100e9, 0.25, 5000.0, "reference");

        let s1 = rect_section(0.0, 0.0, 0.2, 0.1);
        let s2 = rect_section(0.0, 0.1, 0.2, 0.1);

        let comp = ElasticComposite::new(vec![
            CompositeComponent::new(s1, mat1).unwrap(),
            CompositeComponent::new(s2, mat2).unwrap(),
        ])
        .unwrap();

        let result = comp.analyze(&mat_ref).unwrap();

        // n1 = 30/100 = 0.3, n2 = 200/100 = 2.0
        assert!((result.components[0].modular_ratio - 0.3).abs() < 1e-10);
        assert!((result.components[1].modular_ratio - 2.0).abs() < 1e-10);

        // Physical EA must still be reference-invariant
        let r_conc = comp.analyze(&mat1).unwrap();
        let ea_ref = mat_ref.youngs_modulus * result.area;
        let ea_conc = mat1.youngs_modulus * r_conc.area;
        assert!((ea_ref - ea_conc).abs() / ea_conc < 1e-10);
    }

    #[test]
    fn asymmetric_principal_axes() {
        // L-shaped asymmetric section with two materials.
        // Vertical leg: 0.02 × 0.2, E = 200 GPa
        // Horizontal leg: 0.1 × 0.02, E = 30 GPa
        let vert = rect_section(0.0, 0.0, 0.02, 0.2);
        let horiz = rect_section(0.0, 0.0, 0.1, 0.02);

        let mat_steel = Material::new(200e9, 0.3, 7850.0, "steel");
        let mat_conc = Material::new(30e9, 0.2, 2400.0, "concrete");

        let comp = ElasticComposite::new(vec![
            CompositeComponent::new(vert, mat_steel).unwrap(),
            CompositeComponent::new(horiz, mat_conc).unwrap(),
        ])
        .unwrap();

        let result = comp.analyze(&mat_steel).unwrap();

        // For an asymmetric section, Ixy should be non-zero.
        assert!(
            result.ixy.abs() > 1e-8,
            "Ixy should be non-zero for asymmetric section"
        );

        // Principal moments must satisfy I11 >= I22
        assert!(result.principal_i11 >= result.principal_i22);

        // Principal moments must satisfy the invariant:
        // I11 + I22 = Ix + Iy
        let sum_principal = result.principal_i11 + result.principal_i22;
        let sum_centroidal = result.ix + result.iy;
        assert!((sum_principal - sum_centroidal).abs() < 1e-10);

        // Principal angle must be non-zero for asymmetric section
        assert!(
            result.principal_phi.abs() > 1e-6,
            "principal angle should be non-zero"
        );

        // Verify principal angle convention matches SectionProperties:
        // phi = 0.5 * atan2(2*Ixy, Ix - Iy)
        let expected_phi = 0.5 * (2.0 * result.ixy).atan2(result.ix - result.iy);
        assert!((result.principal_phi - expected_phi).abs() < 1e-10);
    }

    // ---- Phase 35 boundary tests ----

    #[test]
    fn translation_invariance() {
        // Centroidal properties (Ix, Iy, Ixy, I11, I22) must be invariant
        // under translation.  Centroid must shift by the translation vector.
        let mat1 = Material::new(30e9, 0.2, 2400.0, "concrete");
        let mat2 = Material::new(200e9, 0.3, 7850.0, "steel");

        let s1 = rect_section(0.0, 0.0, 0.3, 0.15);
        let s2 = rect_section(0.05, 0.15, 0.2, 0.01);

        let comp_orig = ElasticComposite::new(vec![
            CompositeComponent::new(s1, mat1).unwrap(),
            CompositeComponent::new(s2, mat2).unwrap(),
        ])
        .unwrap();
        let r_orig = comp_orig.analyze(&mat1).unwrap();

        // Translate all geometry by (dx, dy).
        let dx = 1.7;
        let dy = -0.4;

        let s1_t = rect_section(dx, dy, 0.3, 0.15);
        let s2_t = rect_section(0.05 + dx, 0.15 + dy, 0.2, 0.01);

        let comp_t = ElasticComposite::new(vec![
            CompositeComponent::new(s1_t, mat1).unwrap(),
            CompositeComponent::new(s2_t, mat2).unwrap(),
        ])
        .unwrap();
        let r_t = comp_t.analyze(&mat1).unwrap();

        // Centroid shifts by (dx, dy).
        assert!((r_t.centroid.x - (r_orig.centroid.x + dx)).abs() < 1e-10);
        assert!((r_t.centroid.y - (r_orig.centroid.y + dy)).abs() < 1e-10);

        // Centroidal properties are translation-invariant.
        assert!((r_t.ix - r_orig.ix).abs() < 1e-10);
        assert!((r_t.iy - r_orig.iy).abs() < 1e-10);
        assert!((r_t.ixy - r_orig.ixy).abs() < 1e-10);
        assert!((r_t.principal_i11 - r_orig.principal_i11).abs() < 1e-10);
        assert!((r_t.principal_i22 - r_orig.principal_i22).abs() < 1e-10);
    }

    #[test]
    fn multi_component_identical_material_degeneration() {
        // Multiple components with the same material must produce the same
        // result as a single SectionProperties of the combined geometry.
        let mat = Material::new(200e9, 0.3, 7850.0, "steel");

        // Two disjoint rectangles.
        let s1 = rect_section(0.0, 0.0, 0.2, 0.1);
        let s2 = rect_section(0.3, 0.0, 0.15, 0.1);

        let comp = ElasticComposite::new(vec![
            CompositeComponent::new(s1.clone(), mat).unwrap(),
            CompositeComponent::new(s2.clone(), mat).unwrap(),
        ])
        .unwrap();
        let r_comp = comp.analyze(&mat).unwrap();

        // Build a CompoundGeometry with both rectangles for SectionProperties.
        use crate::geometry::{CompoundGeometry, Geometry};
        let g1 = Geometry::new(s1.outer, s1.holes);
        let g2 = Geometry::new(s2.outer, s2.holes);
        let compound = CompoundGeometry::new(vec![g1, g2]);
        let r_homogeneous = SectionProperties::from_compound(&compound);

        let tol = 1e-10;
        assert!((r_comp.area - r_homogeneous.area).abs() < tol);
        assert!((r_comp.centroid.x - r_homogeneous.centroid.x).abs() < tol);
        assert!((r_comp.centroid.y - r_homogeneous.centroid.y).abs() < tol);
        assert!((r_comp.ix - r_homogeneous.ix).abs() < tol);
        assert!((r_comp.iy - r_homogeneous.iy).abs() < tol);
        assert!((r_comp.ixy - r_homogeneous.ixy).abs() < tol);
        assert!((r_comp.principal_i11 - r_homogeneous.principal.i11).abs() < tol);
        assert!((r_comp.principal_i22 - r_homogeneous.principal.i22).abs() < tol);
    }

    #[test]
    fn reference_modulus_scaling() {
        // E_ref → k·E_ref  ⟹  A' → A'/k, I' → I'/k, EA/EI invariant.
        let mat1 = Material::new(30e9, 0.2, 2400.0, "concrete");
        let mat2 = Material::new(200e9, 0.3, 7850.0, "steel");

        let s1 = rect_section(0.0, 0.0, 0.3, 0.15);
        let s2 = rect_section(0.05, 0.15, 0.2, 0.01);

        let comp = ElasticComposite::new(vec![
            CompositeComponent::new(s1, mat1).unwrap(),
            CompositeComponent::new(s2, mat2).unwrap(),
        ])
        .unwrap();

        let r1 = comp.analyze(&mat1).unwrap();

        // Scale reference by k.
        let k = 1e6;
        let mat_scaled = Material::new(mat1.youngs_modulus * k, 0.2, 2400.0, "scaled");
        let r2 = comp.analyze(&mat_scaled).unwrap();

        // A' → A'/k
        assert!((r2.area * k - r1.area).abs() / r1.area < 1e-10);
        // I' → I'/k
        assert!((r2.ix * k - r1.ix).abs() / r1.ix < 1e-10);
        assert!((r2.iy * k - r1.iy).abs() / r1.iy < 1e-10);

        // EA and EI are invariant.
        let ea1 = mat1.youngs_modulus * r1.area;
        let ea2 = mat_scaled.youngs_modulus * r2.area;
        assert!((ea1 - ea2).abs() / ea1 < 1e-10);

        let ei1 = mat1.youngs_modulus * r1.ix;
        let ei2 = mat_scaled.youngs_modulus * r2.ix;
        assert!((ei1 - ei2).abs() / ei1 < 1e-10);

        // Centroid is reference-invariant.
        assert!((r1.centroid.x - r2.centroid.x).abs() < 1e-10);
        assert!((r1.centroid.y - r2.centroid.y).abs() < 1e-10);
    }

    #[test]
    fn near_identical_materials_continuity() {
        // Two materials with E2 = E1*(1+ε) should produce results close to
        // the single-material case.  No catastrophic cancellation.
        let e = 200e9;
        let mat_base = Material::new(e, 0.3, 7850.0, "base");

        let s1 = rect_section(0.0, 0.0, 0.2, 0.1);
        let s2 = rect_section(0.0, 0.1, 0.2, 0.1);

        // Single material baseline.
        let comp_single = ElasticComposite::new(vec![
            CompositeComponent::new(s1.clone(), mat_base).unwrap(),
            CompositeComponent::new(s2.clone(), mat_base).unwrap(),
        ])
        .unwrap();
        let r_single = comp_single.analyze(&mat_base).unwrap();

        // Near-identical: E2 = E1 * (1 + 1e-12).
        let eps = 1e-12;
        let mat_near = Material::new(e * (1.0 + eps), 0.3, 7850.0, "near");
        let comp_near = ElasticComposite::new(vec![
            CompositeComponent::new(s1, mat_base).unwrap(),
            CompositeComponent::new(s2, mat_near).unwrap(),
        ])
        .unwrap();
        let r_near = comp_near.analyze(&mat_base).unwrap();

        // Results should be very close (continuity).
        assert!((r_near.area - r_single.area).abs() / r_single.area < 1e-10);
        assert!((r_near.ix - r_single.ix).abs() / r_single.ix < 1e-10);
        assert!((r_near.centroid.y - r_single.centroid.y).abs() < 1e-10);
    }

    #[test]
    fn p1_01_reproduce_modular_ratio_overflow() {
        let sec = rect_section(0.0, 0.0, 0.3, 0.5);
        let mat_huge = Material::new(1e308, 0.3, 7850.0, "huge");
        let mat_tiny = Material::new(1e-308, 0.3, 7850.0, "tiny");
        let comp =
            ElasticComposite::new(vec![CompositeComponent::new(sec, mat_huge).unwrap()]).unwrap();
        let result = comp.analyze(&mat_tiny);
        match result {
            Ok(r) => panic!(
                "P1-01 NOT fixed: expected Err but got Ok with area={}, ix={}",
                r.area, r.ix
            ),
            Err(e) => println!("P1-01 confirmed: got Err = {:?}", e),
        }
    }

    #[test]
    fn p1_01_large_but_finite_modular_ratio() {
        let sec = rect_section(0.0, 0.0, 0.3, 0.5);
        let mat_huge = Material::new(1e20, 0.3, 7850.0, "huge");
        let mat_tiny = Material::new(1e-10, 0.3, 7850.0, "tiny");
        let comp =
            ElasticComposite::new(vec![CompositeComponent::new(sec, mat_huge).unwrap()]).unwrap();
        let result = comp.analyze(&mat_tiny).unwrap();
        assert!(result.area.is_finite(), "area should be finite");
        assert!(result.ix.is_finite(), "ix should be finite");
        assert!(result.principal_i11.is_finite(), "i11 should be finite");
        assert!(result.principal_i22.is_finite(), "i22 should be finite");
    }

    #[test]
    fn p0_01_reproduce_principal_axis_overflow() {
        let mat = Material::new(1.0, 0.0, 1.0, "unit");
        let s1 = rect_section(0.0, 1e100, 0.1, 1e90);
        let s2 = rect_section(0.0, -1e100, 0.1, 1e90);
        let comp = ElasticComposite::new(vec![
            CompositeComponent::new(s1, mat).unwrap(),
            CompositeComponent::new(s2, mat).unwrap(),
        ])
        .unwrap();
        let result = comp.analyze(&mat);
        match result {
            Ok(r) => panic!(
                "P0-01 NOT fixed: expected Err but got Ok with i11={}, i22={}",
                r.principal_i11, r.principal_i22
            ),
            Err(CompositeError::NonFiniteDerivedProperty { stage, .. }) => {
                println!("P0-01 confirmed: stage = {stage}");
                assert!(stage == "principal_axes" || stage == "centroidal_inertia");
            }
            Err(e) => panic!("P0-01 unexpected error: {e:?}"),
        }
    }

    #[test]
    fn p0_01_normal_composite_not_rejected() {
        let mat = Material::new(1.0, 0.0, 1.0, "unit");
        let s1 = rect_section(0.0, 1e10, 0.1, 0.1);
        let s2 = rect_section(0.0, -1e10, 0.1, 0.1);
        let comp = ElasticComposite::new(vec![
            CompositeComponent::new(s1, mat).unwrap(),
            CompositeComponent::new(s2, mat).unwrap(),
        ])
        .unwrap();
        let result = comp.analyze(&mat).unwrap();
        assert!(result.area.is_finite());
        assert!(result.ix.is_finite());
        assert!(result.iy.is_finite());
        assert!(result.ixy.is_finite());
        assert!(result.principal_i11.is_finite());
        assert!(result.principal_i22.is_finite());
        assert!(result.principal_phi.is_finite());
        assert!(result.centroid.x.is_finite());
        assert!(result.centroid.y.is_finite());
    }

    #[test]
    fn p0_02_all_finite_results_invariant() {
        let mat1 = Material::new(30e9, 0.2, 2400.0, "concrete");
        let mat2 = Material::new(200e9, 0.3, 7850.0, "steel");
        let s1 = rect_section(0.0, 0.0, 0.3, 0.5);
        let s2 = rect_section(0.1, 0.1, 0.05, 0.3);
        let comp = ElasticComposite::new(vec![
            CompositeComponent::new(s1, mat1).unwrap(),
            CompositeComponent::new(s2, mat2).unwrap(),
        ])
        .unwrap();
        let r = comp.analyze(&mat1).unwrap();
        assert!(r.area.is_finite());
        assert!(r.centroid.x.is_finite());
        assert!(r.centroid.y.is_finite());
        assert!(r.ix.is_finite());
        assert!(r.iy.is_finite());
        assert!(r.ixy.is_finite());
        assert!(r.principal_i11.is_finite());
        assert!(r.principal_i22.is_finite());
        assert!(r.principal_phi.is_finite());
    }

    #[test]
    fn p2_01_case_a_modular_ratio_overflow_variant() {
        let sec = rect_section(0.0, 0.0, 0.3, 0.5);
        let mat_huge = Material::new(1e308, 0.3, 7850.0, "huge");
        let mat_tiny = Material::new(1e-308, 0.3, 7850.0, "tiny");
        let comp =
            ElasticComposite::new(vec![CompositeComponent::new(sec, mat_huge).unwrap()]).unwrap();
        let result = comp.analyze(&mat_tiny);
        assert!(
            matches!(result, Err(CompositeError::NonFiniteModularRatio { .. })),
            "Case A: expected NonFiniteModularRatio, got {:?}",
            result
        );
    }

    #[test]
    fn p2_01_case_b_accumulation_overflow_variant() {
        let mat = Material::new(1.0, 0.0, 1.0, "unit");
        let s = rect_section(0.0, 1e200, 1e-10, 1e190);
        let comp = ElasticComposite::new(vec![CompositeComponent::new(s, mat).unwrap()]).unwrap();
        let result = comp.analyze(&mat);
        assert!(
            matches!(
                result,
                Err(CompositeError::NonFiniteDerivedProperty {
                    stage: "weighted_accumulation",
                    ..
                })
            ),
            "Case B: expected NonFiniteDerivedProperty {{ stage: \"weighted_accumulation\" }}, got {:?}",
            result
        );
    }

    #[test]
    fn p2_01_case_c_finite_input_ok() {
        let mat1 = Material::new(30e9, 0.2, 2400.0, "concrete");
        let mat2 = Material::new(200e9, 0.3, 7850.0, "steel");
        let s1 = rect_section(0.0, 0.0, 0.3, 0.5);
        let s2 = rect_section(0.1, 0.1, 0.05, 0.3);
        let comp = ElasticComposite::new(vec![
            CompositeComponent::new(s1, mat1).unwrap(),
            CompositeComponent::new(s2, mat2).unwrap(),
        ])
        .unwrap();
        let result = comp.analyze(&mat1);
        assert!(
            matches!(result, Ok(_)),
            "Case C: expected Ok for finite inputs, got {:?}",
            result
        );
    }

    /// Analytical centroidal Ix for a rectangle: b*h^3/12
    fn rect_ix_local(b: f64, h: f64) -> f64 {
        b * h * h * h / 12.0
    }

    /// Analytical centroidal Iy for a rectangle: h*b^3/12
    fn rect_iy_local(b: f64, h: f64) -> f64 {
        h * b * b * b / 12.0
    }

    #[test]
    fn composite_centroidal_inertia_stable_at_large_translation() {
        let b = 0.1_f64;
        let h = 0.1_f64;
        let ix_true = rect_ix_local(b, h);
        let iy_true = rect_iy_local(b, h);
        let mat = Material::new(1.0, 0.0, 1.0, "unit");

        let scales: [f64; 7] = [1e0, 1e2, 1e4, 1e6, 1e8, 1e10, 1e12];

        for &y in &scales {
            let sec = rect_section(0.0, y, b, h);
            let comp =
                ElasticComposite::new(vec![CompositeComponent::new(sec, mat).unwrap()]).unwrap();
            let r = comp.analyze(&mat).unwrap();

            let ix_relerr = (r.ix - ix_true).abs() / ix_true;
            let iy_relerr = (r.iy - iy_true).abs() / iy_true;

            assert!(
                r.ix.is_finite() && r.ix > 0.0,
                "Y={:.0e}: Ix={} should be finite and positive",
                y,
                r.ix
            );
            assert!(
                r.iy.is_finite() && r.iy > 0.0,
                "Y={:.0e}: Iy={} should be finite and positive",
                y,
                r.iy
            );

            let tol = if y <= 1e8 { 1e-3 } else { 1e-1 };
            assert!(
                ix_relerr < tol,
                "Y={:.0e}: Ix relative error {:.2e} should be < {:.0e}",
                y,
                ix_relerr,
                tol
            );
            assert!(
                iy_relerr < tol,
                "Y={:.0e}: Iy relative error {:.2e} should be < {:.0e}",
                y,
                iy_relerr,
                tol
            );
        }
    }

    #[test]
    fn composite_translation_invariance_at_large_scales() {
        let b = 0.1_f64;
        let h = 0.1_f64;
        let mat = Material::new(1.0, 0.0, 1.0, "unit");

        let s1_base = rect_section(0.0, 0.0, b, h);
        let s2_base = rect_section(0.05, 0.05, b, h);
        let comp_base = ElasticComposite::new(vec![
            CompositeComponent::new(s1_base, mat).unwrap(),
            CompositeComponent::new(s2_base, mat).unwrap(),
        ])
        .unwrap();
        let r_base = comp_base.analyze(&mat).unwrap();

        for &dy in &[1e4, 1e6, 1e8] {
            let s1 = rect_section(0.0, dy, b, h);
            let s2 = rect_section(0.05, dy + 0.05, b, h);
            let comp = ElasticComposite::new(vec![
                CompositeComponent::new(s1, mat).unwrap(),
                CompositeComponent::new(s2, mat).unwrap(),
            ])
            .unwrap();
            let r = comp.analyze(&mat).unwrap();

            let ix_relerr = (r.ix - r_base.ix).abs() / r_base.ix;
            let iy_relerr = (r.iy - r_base.iy).abs() / r_base.iy;

            let tol = if dy <= 1e6 { 1e-6 } else { 1e-3 };
            assert!(
                ix_relerr < tol,
                "dy={:.0e}: Ix relative error {:.2e} should be < {:.0e}",
                dy,
                ix_relerr,
                tol
            );
            assert!(
                iy_relerr < tol,
                "dy={:.0e}: Iy relative error {:.2e} should be < {:.0e}",
                dy,
                iy_relerr,
                tol
            );
        }
    }

    #[test]
    fn composite_symmetric_pair_at_large_scales() {
        let b = 0.1_f64;
        let h = 0.1_f64;
        let area = b * h;
        let ix_local = rect_ix_local(b, h);
        let mat = Material::new(1.0, 0.0, 1.0, "unit");

        for &y in &[1e4, 1e8, 1e12] {
            let s1 = rect_section(0.0, y, b, h);
            let s2 = rect_section(0.0, -y - h, b, h);
            let comp = ElasticComposite::new(vec![
                CompositeComponent::new(s1, mat).unwrap(),
                CompositeComponent::new(s2, mat).unwrap(),
            ])
            .unwrap();
            let r = comp.analyze(&mat).unwrap();

            assert!(
                r.centroid.y.abs() < 1e-6,
                "Y={:.0e}: centroid.y={} should be ~0",
                y,
                r.centroid.y
            );
            assert!(
                r.ix > 0.0 && r.iy > 0.0,
                "Y={:.0e}: Ix={} and Iy={} should be positive",
                y,
                r.ix,
                r.iy
            );

            let d = y + h / 2.0;
            let ix_ref = 2.0 * (ix_local + area * d * d);
            let ix_relerr = (r.ix - ix_ref).abs() / ix_ref;
            let tol = if y <= 1e8 { 1e-6 } else { 1e-3 };
            assert!(
                ix_relerr < tol,
                "Y={:.0e}: Ix relative error {:.2e} should be < {:.0e}",
                y,
                ix_relerr,
                tol
            );
        }
    }

    #[test]
    fn composite_independent_reference_comparison() {
        let b = 0.1_f64;
        let h = 0.1_f64;
        let area = b * h;
        let ix_local = rect_ix_local(b, h);
        let iy_local = rect_iy_local(b, h);
        let mat = Material::new(1.0, 0.0, 1.0, "unit");

        for &y in &[1e0, 1e4, 1e8, 1e12] {
            let sec = rect_section(0.0, y, b, h);
            let comp =
                ElasticComposite::new(vec![CompositeComponent::new(sec, mat).unwrap()]).unwrap();
            let r = comp.analyze(&mat).unwrap();

            let cy_component = y + h / 2.0;
            let cx_component = 0.0 + b / 2.0;
            let total_area_ref = area;
            let cy_ref = cy_component;
            let cx_ref = cx_component;
            let dy = cy_component - cy_ref;
            let dx = cx_component - cx_ref;
            let ix_ref = ix_local + area * dy * dy;
            let iy_ref = iy_local + area * dx * dx;

            let area_tol = if y <= 1e6 {
                1e-10
            } else if y <= 1e10 {
                1e-6
            } else {
                1e-3
            };
            assert!(
                (total_area_ref - r.area).abs() / total_area_ref < area_tol,
                "Y={:.0e}: area mismatch: r.area={:.6e} vs ref={:.6e}",
                y,
                r.area,
                total_area_ref
            );

            let ix_relerr = (r.ix - ix_local).abs() / ix_local;
            let iy_relerr = (r.iy - iy_local).abs() / iy_local;
            let tol = if y <= 1e8 { 1e-3 } else { 1e-1 };
            assert!(
                ix_relerr < tol,
                "Y={:.0e}: Ix relerr {:.2e} vs analytical {:.6e}",
                y,
                ix_relerr,
                ix_local
            );
            assert!(
                iy_relerr < tol,
                "Y={:.0e}: Iy relerr {:.2e} vs analytical {:.6e}",
                y,
                iy_relerr,
                iy_local
            );
        }
    }
}
