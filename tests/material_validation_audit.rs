//! Phase 62: Material & Composite validation hardening regression tests.
//!
//! Verifies fixes for findings P1-1 (ν = -1 accepted), P1-2 (E = Inf
//! accepted), P1-3 (new() unchecked), P1-6 (CompositeComponent pub fields
//! bypass validation), and P2-10 (CompositeComponent::new doesn't validate
//! ν/density).

use section_properties::composite::{CompositeComponent, CompositeError, ElasticComposite};
use section_properties::geometry::{Point, Polygon};
use section_properties::material::Material;
use section_properties::material::presets::*;
use section_properties::section::Section;

// ─────────────────────────────────────────────────────────────────────────
// Helper
// ─────────────────────────────────────────────────────────────────────────

fn rect_section(w: f64, h: f64) -> Section {
    Section::new(
        Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(w, 0.0),
            Point::new(w, h),
            Point::new(0.0, h),
        ]),
        Vec::new(),
    )
}

fn valid_material() -> Material {
    Material::new(200e9, 0.3, 7850.0, "steel")
}

// ─────────────────────────────────────────────────────────────────────────
// P1-1: ν = -1.0 must be rejected (gives G = E/0 = Inf)
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn poissons_ratio_minus_one_rejected() {
    let mat = Material::with_all(200e9, 76.9e9, -1.0, 7850.0, 0.0, 0.0, 0.0, "nu=-1");
    assert!(!mat.is_valid(), "ν = -1.0 must be invalid (G = E/0 = Inf)");
}

#[test]
fn poissons_ratio_just_above_minus_one_accepted() {
    let mat = Material::with_all(200e9, 76.9e9, -0.999, 7850.0, 0.0, 0.0, 0.0, "nu=-0.999");
    assert!(mat.is_valid(), "ν = -0.999 should be valid");
}

// ─────────────────────────────────────────────────────────────────────────
// P1-2: E = Inf / NaN must be rejected
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn youngs_modulus_infinity_rejected() {
    let mat = Material::with_all(f64::INFINITY, 76.9e9, 0.3, 7850.0, 0.0, 0.0, 0.0, "E=inf");
    assert!(!mat.is_valid(), "E = Inf must be invalid");
}

#[test]
fn youngs_modulus_nan_rejected() {
    let mat = Material::with_all(f64::NAN, 76.9e9, 0.3, 7850.0, 0.0, 0.0, 0.0, "E=NaN");
    assert!(!mat.is_valid(), "E = NaN must be invalid");
}

#[test]
fn shear_modulus_infinity_rejected() {
    let mat = Material::with_all(200e9, f64::INFINITY, 0.3, 7850.0, 0.0, 0.0, 0.0, "G=inf");
    assert!(!mat.is_valid(), "G = Inf must be invalid");
}

#[test]
fn shear_modulus_nan_rejected() {
    let mat = Material::with_all(200e9, f64::NAN, 0.3, 7850.0, 0.0, 0.0, 0.0, "G=NaN");
    assert!(!mat.is_valid(), "G = NaN must be invalid");
}

// ─────────────────────────────────────────────────────────────────────────
// is_finite checks for ν and density
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn poissons_ratio_nan_rejected() {
    let mat = Material::with_all(200e9, 76.9e9, f64::NAN, 7850.0, 0.0, 0.0, 0.0, "nu=NaN");
    assert!(!mat.is_valid(), "ν = NaN must be invalid");
}

#[test]
fn poissons_ratio_inf_rejected() {
    let mat = Material::with_all(200e9, 76.9e9, f64::INFINITY, 7850.0, 0.0, 0.0, 0.0, "nu=inf");
    assert!(!mat.is_valid(), "ν = Inf must be invalid");
}

#[test]
fn poissons_ratio_half_rejected() {
    let mat = Material::with_all(200e9, 76.9e9, 0.5, 7850.0, 0.0, 0.0, 0.0, "nu=0.5");
    assert!(!mat.is_valid(), "ν = 0.5 must be invalid (incompressible limit)");
}

#[test]
fn density_nan_rejected() {
    let mat = Material::with_all(200e9, 76.9e9, 0.3, f64::NAN, 0.0, 0.0, 0.0, "rho=NaN");
    assert!(!mat.is_valid(), "density = NaN must be invalid");
}

#[test]
fn density_inf_rejected() {
    let mat = Material::with_all(200e9, 76.9e9, 0.3, f64::INFINITY, 0.0, 0.0, 0.0, "rho=inf");
    assert!(!mat.is_valid(), "density = Inf must be invalid");
}

#[test]
fn density_zero_accepted() {
    let mat = Material::with_all(200e9, 76.9e9, 0.3, 0.0, 0.0, 0.0, 0.0, "rho=0");
    assert!(mat.is_valid(), "density = 0 should be valid (massless placeholder)");
}

// ─────────────────────────────────────────────────────────────────────────
// All presets must pass is_valid()
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn all_presets_valid() {
    assert!(STEEL_S235.is_valid(), "STEEL_S235");
    assert!(STEEL_S275.is_valid(), "STEEL_S275");
    assert!(STEEL_S355.is_valid(), "STEEL_S355");
    assert!(STEEL_S460.is_valid(), "STEEL_S460");
    assert!(STAINLESS_304.is_valid(), "STAINLESS_304");
    assert!(STAINLESS_316.is_valid(), "STAINLESS_316");
    assert!(ALUMINUM_6061_T6.is_valid(), "ALUMINUM_6061_T6");
    assert!(ALUMINUM_6063_T5.is_valid(), "ALUMINUM_6063_T5");
    assert!(ALUMINUM_7075_T6.is_valid(), "ALUMINUM_7075_T6");
    assert!(CONCRETE_C25_30.is_valid(), "CONCRETE_C25_30");
    assert!(CONCRETE_C30_37.is_valid(), "CONCRETE_C30_37");
    assert!(CONCRETE_C40_50.is_valid(), "CONCRETE_C40_50");
    assert!(TIMBER_GL24H.is_valid(), "TIMBER_GL24H");
    assert!(TITANIUM_GR5.is_valid(), "TITANIUM_GR5");
}

// ─────────────────────────────────────────────────────────────────────────
// P2-10: CompositeComponent::new() must reject invalid ν / density / G
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn composite_rejects_nu_minus_one() {
    let sec = rect_section(0.1, 0.1);
    let mat = Material::with_all(200e9, 76.9e9, -1.0, 7850.0, 0.0, 0.0, 0.0, "nu=-1");
    let result = CompositeComponent::new(sec, mat);
    assert!(result.is_err());
    assert!(matches!(result.unwrap_err(), CompositeError::InvalidModulus { .. }));
}

#[test]
fn composite_rejects_nu_nan() {
    let sec = rect_section(0.1, 0.1);
    let mat = Material::with_all(200e9, 76.9e9, f64::NAN, 7850.0, 0.0, 0.0, 0.0, "nu=NaN");
    assert!(CompositeComponent::new(sec, mat).is_err());
}

#[test]
fn composite_rejects_shear_modulus_zero() {
    let sec = rect_section(0.1, 0.1);
    let mat = Material::with_all(200e9, 0.0, 0.3, 7850.0, 0.0, 0.0, 0.0, "G=0");
    assert!(CompositeComponent::new(sec, mat).is_err());
}

#[test]
fn composite_rejects_density_nan() {
    let sec = rect_section(0.1, 0.1);
    let mat = Material::with_all(200e9, 76.9e9, 0.3, f64::NAN, 0.0, 0.0, 0.0, "rho=NaN");
    assert!(CompositeComponent::new(sec, mat).is_err());
}

#[test]
fn composite_rejects_density_inf() {
    let sec = rect_section(0.1, 0.1);
    let mat = Material::with_all(200e9, 76.9e9, 0.3, f64::INFINITY, 0.0, 0.0, 0.0, "rho=inf");
    assert!(CompositeComponent::new(sec, mat).is_err());
}

#[test]
fn composite_accepts_valid_material() {
    let sec = rect_section(0.1, 0.1);
    let mat = valid_material();
    assert!(CompositeComponent::new(sec, mat).is_ok());
}

// ─────────────────────────────────────────────────────────────────────────
// P1-6: ElasticComposite::analyze() must re-validate components
// (CompositeComponent has pub fields — users can bypass new())
// ─────────────────────────────────────────────────────────────────────────

#[test]
fn analyze_rejects_pub_field_bypass_nu_minus_one() {
    let sec = rect_section(0.1, 0.1);
    let mat = Material::with_all(200e9, 76.9e9, -1.0, 7850.0, 0.0, 0.0, 0.0, "bypass");
    let comp = CompositeComponent { section: sec, material: mat };
    let ec = ElasticComposite::new(vec![comp]).unwrap();
    let ref_mat = valid_material();
    let result = ec.analyze(&ref_mat);
    assert!(result.is_err());
    assert!(matches!(result.unwrap_err(), CompositeError::InvalidModulus { .. }));
}

#[test]
fn analyze_rejects_pub_field_bypass_nu_nan() {
    let sec = rect_section(0.1, 0.1);
    let mat = Material::with_all(200e9, 76.9e9, f64::NAN, 7850.0, 0.0, 0.0, 0.0, "bypass");
    let comp = CompositeComponent { section: sec, material: mat };
    let ec = ElasticComposite::new(vec![comp]).unwrap();
    let ref_mat = valid_material();
    assert!(ec.analyze(&ref_mat).is_err());
}

#[test]
fn analyze_rejects_pub_field_bypass_g_inf() {
    let sec = rect_section(0.1, 0.1);
    let mat = Material::with_all(200e9, f64::INFINITY, 0.3, 7850.0, 0.0, 0.0, 0.0, "bypass");
    let comp = CompositeComponent { section: sec, material: mat };
    let ec = ElasticComposite::new(vec![comp]).unwrap();
    let ref_mat = valid_material();
    assert!(ec.analyze(&ref_mat).is_err());
}

#[test]
fn analyze_rejects_pub_field_bypass_density_nan() {
    let sec = rect_section(0.1, 0.1);
    let mat = Material::with_all(200e9, 76.9e9, 0.3, f64::NAN, 0.0, 0.0, 0.0, "bypass");
    let comp = CompositeComponent { section: sec, material: mat };
    let ec = ElasticComposite::new(vec![comp]).unwrap();
    let ref_mat = valid_material();
    assert!(ec.analyze(&ref_mat).is_err());
}

#[test]
fn analyze_accepts_valid_pub_field_construction() {
    let sec = rect_section(0.1, 0.1);
    let mat = valid_material();
    let comp = CompositeComponent { section: sec, material: mat };
    let ec = ElasticComposite::new(vec![comp]).unwrap();
    let ref_mat = valid_material();
    assert!(ec.analyze(&ref_mat).is_ok());
}
