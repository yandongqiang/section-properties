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

        // Accumulators (global axes, transformed).
        let mut total_area = 0.0_f64;
        let mut first_x = 0.0_f64;
        let mut first_y = 0.0_f64;
        let mut ix_global = 0.0_f64;
        let mut iy_global = 0.0_f64;
        let mut ixy_global = 0.0_f64;

        let mut component_results: Vec<ComponentAnalysis> =
            Vec::with_capacity(self.components.len());

        for (i, comp) in self.components.iter().enumerate() {
            let props = SectionProperties::try_from_section(&comp.section)
                .map_err(|detail| CompositeError::InvalidGeometry { index: i, detail })?;

            let n = comp.material.youngs_modulus / e_ref;
            let area = props.area;
            let cx = props.centroid.x;
            let cy = props.centroid.y;

            // Centroidal → global via parallel-axis theorem:
            //   I_x,global = I_x,c + A · cy²
            //   I_y,global = I_y,c + A · cx²
            //   I_xy,global = I_xy,c + A · cx · cy
            let ix_g = props.ix + area * cy * cy;
            let iy_g = props.iy + area * cx * cx;
            let ixy_g = props.ixy + area * cx * cy;

            // Apply modular ratio.
            let n_area = n * area;
            total_area += n_area;
            first_x += n_area * cx;
            first_y += n_area * cy;
            ix_global += n * ix_g;
            iy_global += n * iy_g;
            ixy_global += n * ixy_g;

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

        // Transformed centroid.
        let centroid = Point::new(first_x / total_area, first_y / total_area);

        // Global → centroidal via parallel-axis theorem:
        //   I_x,c = I_x,global − A · ȳ²
        //   I_y,c = I_y,global − A · x̄²
        //   I_xy,c = I_xy,global − A · x̄ · ȳ
        let ix_c = ix_global - total_area * centroid.y * centroid.y;
        let iy_c = iy_global - total_area * centroid.x * centroid.x;
        let ixy_c = ixy_global - total_area * centroid.x * centroid.y;

        // Principal axes (Mohr's circle, same convention as SectionProperties).
        let avg = (ix_c + iy_c) * 0.5;
        let diff = (ix_c - iy_c) * 0.5;
        let radius = (diff * diff + ixy_c * ixy_c).sqrt();
        let i11 = avg + radius;
        let i22 = avg - radius;
        let phi = 0.5 * (2.0 * ixy_c).atan2(ix_c - iy_c);

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
}
