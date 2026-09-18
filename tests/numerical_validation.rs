//! Phase 28 — Numerical Validation & Cross-Language Parity Audit
//!
//! Tests physical invariants, geometry coverage (multiple holes, thin ligaments),
//! scale invariance, and hole boundary filtering for section properties.

use section_properties::{CompoundGeometry, Geometry, Point, Polygon, Section, SectionProperties};

fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() < tol
}

fn rel_err(a: f64, b: f64) -> f64 {
    if b.abs() < 1e-15 {
        a.abs()
    } else {
        (a - b).abs() / b.abs()
    }
}

// ===========================================================================
// 1. Multiple holes — area and centroid
// ===========================================================================

/// Outer 10×10 rectangle with two 2×2 square holes.
/// A = 100 - 2×4 = 92
#[test]
fn multi_hole_two_square_holes_area() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 10.0),
        Point::new(0.0, 10.0),
    ]);
    let h1 = Polygon::new(vec![
        Point::new(2.0, 2.0),
        Point::new(4.0, 2.0),
        Point::new(4.0, 4.0),
        Point::new(2.0, 4.0),
    ]);
    let h2 = Polygon::new(vec![
        Point::new(6.0, 6.0),
        Point::new(8.0, 6.0),
        Point::new(8.0, 8.0),
        Point::new(6.0, 8.0),
    ]);
    let sec = Section::new(outer, vec![h1, h2]);
    let props = SectionProperties::from_section(&sec);
    assert!(approx_eq(props.area, 92.0, 1e-10), "area = {}", props.area);
}

/// Outer 10×10 rectangle with four 1×1 square holes.
/// A = 100 - 4×1 = 96
#[test]
fn multi_hole_four_square_holes_area() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 10.0),
        Point::new(0.0, 10.0),
    ]);
    let holes: Vec<Polygon> = vec![
        Polygon::new(vec![
            Point::new(2.0, 2.0),
            Point::new(3.0, 2.0),
            Point::new(3.0, 3.0),
            Point::new(2.0, 3.0),
        ]),
        Polygon::new(vec![
            Point::new(7.0, 2.0),
            Point::new(8.0, 2.0),
            Point::new(8.0, 3.0),
            Point::new(7.0, 3.0),
        ]),
        Polygon::new(vec![
            Point::new(2.0, 7.0),
            Point::new(3.0, 7.0),
            Point::new(3.0, 8.0),
            Point::new(2.0, 8.0),
        ]),
        Polygon::new(vec![
            Point::new(7.0, 7.0),
            Point::new(8.0, 7.0),
            Point::new(8.0, 8.0),
            Point::new(7.0, 8.0),
        ]),
    ];
    let sec = Section::new(outer, holes);
    let props = SectionProperties::from_section(&sec);
    assert!(approx_eq(props.area, 96.0, 1e-10), "area = {}", props.area);
    // Symmetric layout → centroid at (5, 5)
    assert!(
        approx_eq(props.centroid.x, 5.0, 1e-10),
        "cx = {}",
        props.centroid.x
    );
    assert!(
        approx_eq(props.centroid.y, 5.0, 1e-10),
        "cy = {}",
        props.centroid.y
    );
}

/// Outer 10×10 with two symmetric holes → centroid at center.
#[test]
fn multi_hole_symmetric_centroid() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 10.0),
        Point::new(0.0, 10.0),
    ]);
    let h1 = Polygon::new(vec![
        Point::new(2.0, 4.0),
        Point::new(4.0, 4.0),
        Point::new(4.0, 6.0),
        Point::new(2.0, 6.0),
    ]);
    let h2 = Polygon::new(vec![
        Point::new(6.0, 4.0),
        Point::new(8.0, 4.0),
        Point::new(8.0, 6.0),
        Point::new(6.0, 6.0),
    ]);
    let sec = Section::new(outer, vec![h1, h2]);
    let props = SectionProperties::from_section(&sec);
    assert!(
        approx_eq(props.centroid.x, 5.0, 1e-10),
        "cx = {}",
        props.centroid.x
    );
    assert!(
        approx_eq(props.centroid.y, 5.0, 1e-10),
        "cy = {}",
        props.centroid.y
    );
}

/// Eccentric hole shifts centroid.
/// Outer 10×10 at origin, hole 4×4 centered at (7, 7).
/// A = 100 - 16 = 84
/// cx = (100×5 - 16×7) / 84 = (500 - 112) / 84 = 388/84 ≈ 4.619...
#[test]
fn eccentric_hole_centroid_shift() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 10.0),
        Point::new(0.0, 10.0),
    ]);
    let hole = Polygon::new(vec![
        Point::new(5.0, 5.0),
        Point::new(9.0, 5.0),
        Point::new(9.0, 9.0),
        Point::new(5.0, 9.0),
    ]);
    let sec = Section::new(outer, vec![hole]);
    let props = SectionProperties::from_section(&sec);
    let expected_area = 84.0;
    let expected_cx = (100.0 * 5.0 - 16.0 * 7.0) / expected_area;
    let expected_cy = (100.0 * 5.0 - 16.0 * 7.0) / expected_area;
    assert!(
        approx_eq(props.area, expected_area, 1e-10),
        "area = {}",
        props.area
    );
    assert!(
        approx_eq(props.centroid.x, expected_cx, 1e-10),
        "cx = {}",
        props.centroid.x
    );
    assert!(
        approx_eq(props.centroid.y, expected_cy, 1e-10),
        "cy = {}",
        props.centroid.y
    );
}

// ===========================================================================
// 2. Physical invariants — principal axis properties
// ===========================================================================

/// Invariant: I1 + I2 = Ix + Iy (trace preservation)
#[test]
fn invariant_trace_preservation() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(8.0, 6.0),
        Point::new(2.0, 5.0),
    ]);
    let sec = Section::new(outer, Vec::new());
    let props = SectionProperties::from_section(&sec);
    let principal = props.principal_properties();
    let trace = principal.i11 + principal.i22;
    let expected = props.ix + props.iy;
    assert!(
        approx_eq(trace, expected, 1e-10),
        "trace = {}, expected = {}",
        trace,
        expected
    );
}

/// Invariant: I1 * I2 = Ix * Iy - Ixy² (determinant preservation)
#[test]
fn invariant_determinant_preservation() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(8.0, 6.0),
        Point::new(2.0, 5.0),
    ]);
    let sec = Section::new(outer, Vec::new());
    let props = SectionProperties::from_section(&sec);
    let principal = props.principal_properties();
    let det_principal = principal.i11 * principal.i22;
    let det_centroidal = props.ix * props.iy - props.ixy.powi(2);
    assert!(
        approx_eq(det_principal, det_centroidal, 1e-8),
        "det = {}, expected = {}",
        det_principal,
        det_centroidal
    );
}

/// Invariant: I1 >= I2 (ordering)
#[test]
fn invariant_principal_ordering() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(8.0, 6.0),
        Point::new(2.0, 5.0),
    ]);
    let sec = Section::new(outer, Vec::new());
    let props = SectionProperties::from_section(&sec);
    let principal = props.principal_properties();
    assert!(
        principal.i11 >= principal.i22,
        "i11 = {}, i22 = {}",
        principal.i11,
        principal.i22
    );
}

/// Invariant: Ix, Iy >= 0 for any valid section
#[test]
fn invariant_non_negative_moments() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 5.0),
        Point::new(0.0, 5.0),
    ]);
    let hole = Polygon::new(vec![
        Point::new(3.0, 1.0),
        Point::new(7.0, 1.0),
        Point::new(7.0, 4.0),
        Point::new(3.0, 4.0),
    ]);
    let sec = Section::new(outer, vec![hole]);
    let props = SectionProperties::from_section(&sec);
    assert!(props.ix >= 0.0, "ix = {}", props.ix);
    assert!(props.iy >= 0.0, "iy = {}", props.iy);
    let principal = props.principal_properties();
    assert!(principal.i11 >= 0.0, "i11 = {}", principal.i11);
    assert!(principal.i22 >= 0.0, "i22 = {}", principal.i22);
}

/// Invariant: polar moment J = Ix + Iy (about centroid)
#[test]
fn invariant_polar_moment() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 5.0),
        Point::new(0.0, 5.0),
    ]);
    let sec = Section::new(outer, Vec::new());
    let props = SectionProperties::from_section(&sec);
    let j = props.ix + props.iy;
    // For rectangle b×h: J = bh(b²+h²)/12
    let expected_j = 10.0 * 5.0 * (100.0 + 25.0) / 12.0;
    assert!(
        approx_eq(j, expected_j, 1e-10),
        "J = {}, expected = {}",
        j,
        expected_j
    );
}

/// Invariant: radius of gyration r² = I / A
#[test]
fn invariant_gyration_relation() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 5.0),
        Point::new(0.0, 5.0),
    ]);
    let sec = Section::new(outer, Vec::new());
    let props = SectionProperties::from_section(&sec);
    let gyration = props.gyration_properties();
    assert!(approx_eq(gyration.rx.powi(2), props.ix / props.area, 1e-10));
    assert!(approx_eq(gyration.ry.powi(2), props.iy / props.area, 1e-10));
    // Polar: rp² = rx² + ry²
    assert!(approx_eq(
        gyration.polar.powi(2),
        gyration.rx.powi(2) + gyration.ry.powi(2),
        1e-10
    ));
}

// ===========================================================================
// 3. Scale invariance — A∝s², I∝s⁴, centroid∝s
// ===========================================================================

/// Scale a rectangle by factor s and verify A∝s², I∝s⁴.
#[test]
fn scale_invariance_rectangle() {
    let make_rect = |w: f64, h: f64| {
        let outer = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(w, 0.0),
            Point::new(w, h),
            Point::new(0.0, h),
        ]);
        SectionProperties::from_section(&Section::new(outer, Vec::new()))
    };

    let p1 = make_rect(10.0, 5.0);
    let s = 3.0;
    let p2 = make_rect(10.0 * s, 5.0 * s);

    assert!(rel_err(p2.area, p1.area * s * s) < 1e-10, "area scale");
    assert!(rel_err(p2.ix, p1.ix * s.powi(4)) < 1e-10, "ix scale");
    assert!(rel_err(p2.iy, p1.iy * s.powi(4)) < 1e-10, "iy scale");
    // Centroid scales linearly
    assert!(
        rel_err(p2.centroid.x, p1.centroid.x * s) < 1e-10,
        "cx scale"
    );
    assert!(
        rel_err(p2.centroid.y, p1.centroid.y * s) < 1e-10,
        "cy scale"
    );
}

/// Scale a section with hole and verify scaling laws.
#[test]
fn scale_invariance_with_hole() {
    let make = |s: f64| {
        let outer = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(10.0 * s, 0.0),
            Point::new(10.0 * s, 10.0 * s),
            Point::new(0.0, 10.0 * s),
        ]);
        let hole = Polygon::new(vec![
            Point::new(3.0 * s, 3.0 * s),
            Point::new(7.0 * s, 3.0 * s),
            Point::new(7.0 * s, 7.0 * s),
            Point::new(3.0 * s, 7.0 * s),
        ]);
        SectionProperties::from_section(&Section::new(outer, vec![hole]))
    };

    let p1 = make(1.0);
    let p2 = make(2.0);
    let s = 2.0;

    assert!(
        rel_err(p2.area, p1.area * s * s) < 1e-10,
        "area scale with hole"
    );
    assert!(
        rel_err(p2.ix, p1.ix * s.powi(4)) < 1e-10,
        "ix scale with hole"
    );
    assert!(
        rel_err(p2.iy, p1.iy * s.powi(4)) < 1e-10,
        "iy scale with hole"
    );
}

/// Scale an asymmetric section and verify principal moments scale as s⁴.
#[test]
fn scale_invariance_principal_moments() {
    let make = |s: f64| {
        let outer = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(10.0 * s, 0.0),
            Point::new(8.0 * s, 6.0 * s),
            Point::new(2.0 * s, 5.0 * s),
        ]);
        SectionProperties::from_section(&Section::new(outer, Vec::new()))
    };

    let p1 = make(1.0);
    let p2 = make(3.0);
    let s: f64 = 3.0;
    let pr1 = p1.principal_properties();
    let pr2 = p2.principal_properties();

    assert!(rel_err(pr2.i11, pr1.i11 * s.powi(4)) < 1e-10, "i11 scale");
    assert!(rel_err(pr2.i22, pr1.i22 * s.powi(4)) < 1e-10, "i22 scale");
    // Principal angle is scale-invariant
    assert!(
        approx_eq(pr2.phi, pr1.phi, 1e-10),
        "phi scale: {} vs {}",
        pr2.phi,
        pr1.phi
    );
}

// ===========================================================================
// 4. Thin ligament — geometry validity and finite results
// ===========================================================================

/// Outer 10×10 rectangle with a 9.8×9.8 hole → 0.1 ligament.
/// Remaining area = 100 - 96.04 = 3.96
#[test]
fn thin_ligament_0p1() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 10.0),
        Point::new(0.0, 10.0),
    ]);
    let hole = Polygon::new(vec![
        Point::new(0.1, 0.1),
        Point::new(9.9, 0.1),
        Point::new(9.9, 9.9),
        Point::new(0.1, 9.9),
    ]);
    let sec = Section::new(outer, vec![hole]);
    let result = SectionProperties::try_from_section(&sec);
    assert!(
        result.is_ok(),
        "thin ligament 0.1 should produce valid properties"
    );
    let props = result.unwrap();
    let expected_area = 100.0 - 9.8 * 9.8;
    assert!(
        approx_eq(props.area, expected_area, 1e-8),
        "area = {}",
        props.area
    );
    assert!(props.ix.is_finite(), "ix not finite");
    assert!(props.iy.is_finite(), "iy not finite");
    assert!(props.ix > 0.0, "ix > 0");
    assert!(props.iy > 0.0, "iy > 0");
}

/// Outer 10×10 rectangle with a 9.98×9.98 hole → 0.02 ligament.
#[test]
fn thin_ligament_0p02() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 10.0),
        Point::new(0.0, 10.0),
    ]);
    let hole = Polygon::new(vec![
        Point::new(0.01, 0.01),
        Point::new(9.99, 0.01),
        Point::new(9.99, 9.99),
        Point::new(0.01, 9.99),
    ]);
    let sec = Section::new(outer, vec![hole]);
    let result = SectionProperties::try_from_section(&sec);
    assert!(
        result.is_ok(),
        "thin ligament 0.02 should produce valid properties"
    );
    let props = result.unwrap();
    let expected_area = 100.0 - 9.98 * 9.98;
    assert!(
        approx_eq(props.area, expected_area, 1e-6),
        "area = {}",
        props.area
    );
    assert!(props.ix.is_finite(), "ix not finite");
    assert!(props.iy.is_finite(), "iy not finite");
}

/// Thin ligament on one side only: 10×10 outer, 9×9.9 hole offset.
#[test]
fn thin_ligament_one_sided() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 10.0),
        Point::new(0.0, 10.0),
    ]);
    // Hole leaves 0.5 on left/right, 0.05 on top/bottom
    let hole = Polygon::new(vec![
        Point::new(0.5, 0.05),
        Point::new(9.5, 0.05),
        Point::new(9.5, 9.95),
        Point::new(0.5, 9.95),
    ]);
    let sec = Section::new(outer, vec![hole]);
    let result = SectionProperties::try_from_section(&sec);
    assert!(
        result.is_ok(),
        "one-sided thin ligament should produce valid properties"
    );
    let props = result.unwrap();
    assert!(props.area > 0.0, "area > 0");
    assert!(props.ix.is_finite(), "ix not finite");
    assert!(props.iy.is_finite(), "iy not finite");
}

// ===========================================================================
// 5. Compound geometry — multi-region
// ===========================================================================

/// Two disjoint rectangles as a compound geometry.
/// Each 4×2, separated by gap 2.
/// Total A = 2 × 8 = 16
#[test]
fn compound_two_disjoint_rectangles() {
    let r1 = Geometry::new(
        Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(4.0, 0.0),
            Point::new(4.0, 2.0),
            Point::new(0.0, 2.0),
        ]),
        Vec::new(),
    );
    let r2 = Geometry::new(
        Polygon::new(vec![
            Point::new(6.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 2.0),
            Point::new(6.0, 2.0),
        ]),
        Vec::new(),
    );
    let compound = CompoundGeometry::new(vec![r1, r2]);
    let props = SectionProperties::from_compound(&compound);
    assert!(approx_eq(props.area, 16.0, 1e-10), "area = {}", props.area);
    // Centroid at (5, 1) by symmetry
    assert!(
        approx_eq(props.centroid.x, 5.0, 1e-10),
        "cx = {}",
        props.centroid.x
    );
    assert!(
        approx_eq(props.centroid.y, 1.0, 1e-10),
        "cy = {}",
        props.centroid.y
    );
}

/// Compound geometry with hole in one region.
#[test]
fn compound_region_with_hole() {
    let outer1 = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 10.0),
        Point::new(0.0, 10.0),
    ]);
    let hole1 = Polygon::new(vec![
        Point::new(3.0, 3.0),
        Point::new(7.0, 3.0),
        Point::new(7.0, 7.0),
        Point::new(3.0, 7.0),
    ]);
    let g1 = Geometry::new(outer1, vec![hole1]);

    let outer2 = Polygon::new(vec![
        Point::new(15.0, 0.0),
        Point::new(20.0, 0.0),
        Point::new(20.0, 5.0),
        Point::new(15.0, 5.0),
    ]);
    let g2 = Geometry::new(outer2, Vec::new());

    let compound = CompoundGeometry::new(vec![g1, g2]);
    let props = SectionProperties::from_compound(&compound);
    // A = (100 - 16) + 25 = 109
    assert!(approx_eq(props.area, 109.0, 1e-10), "area = {}", props.area);
}

// ===========================================================================
// 6. Analytical cross-validation — known solutions
// ===========================================================================

/// Rectangle b×h: Ix = bh³/12, Iy = hb³/12 about centroid
#[test]
fn analytical_rectangle_moments() {
    let b = 10.0;
    let h = 5.0;
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(b, 0.0),
        Point::new(b, h),
        Point::new(0.0, h),
    ]);
    let props = SectionProperties::from_section(&Section::new(outer, Vec::new()));
    let expected_ix = b * h.powi(3) / 12.0;
    let expected_iy = h * b.powi(3) / 12.0;
    assert!(
        approx_eq(props.ix, expected_ix, 1e-10),
        "ix = {}, expected = {}",
        props.ix,
        expected_ix
    );
    assert!(
        approx_eq(props.iy, expected_iy, 1e-10),
        "iy = {}, expected = {}",
        props.iy,
        expected_iy
    );
    assert!(approx_eq(props.ixy, 0.0, 1e-10), "ixy = {}", props.ixy);
}

/// Hollow rectangle: Ix = (b*h³ - bi*hi³) / 12
#[test]
fn analytical_hollow_rectangle_moments() {
    let b = 10.0;
    let h = 10.0;
    let bi = 6.0;
    let hi = 6.0;
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(b, 0.0),
        Point::new(b, h),
        Point::new(0.0, h),
    ]);
    let hole = Polygon::new(vec![
        Point::new((b - bi) / 2.0, (h - hi) / 2.0),
        Point::new((b + bi) / 2.0, (h - hi) / 2.0),
        Point::new((b + bi) / 2.0, (h + hi) / 2.0),
        Point::new((b - bi) / 2.0, (h + hi) / 2.0),
    ]);
    let props = SectionProperties::from_section(&Section::new(outer, vec![hole]));
    let expected_ix = (b * h.powi(3) - bi * hi.powi(3)) / 12.0;
    let expected_iy = (h * b.powi(3) - hi * bi.powi(3)) / 12.0;
    assert!(
        approx_eq(props.ix, expected_ix, 1e-10),
        "ix = {}, expected = {}",
        props.ix,
        expected_ix
    );
    assert!(
        approx_eq(props.iy, expected_iy, 1e-10),
        "iy = {}, expected = {}",
        props.iy,
        expected_iy
    );
}

/// Triangle: area = bh/2, centroid at (b/3, h/3) from right angle
#[test]
fn analytical_triangle_properties() {
    let b = 6.0;
    let h = 4.0;
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(b, 0.0),
        Point::new(0.0, h),
    ]);
    let props = SectionProperties::from_section(&Section::new(outer, Vec::new()));
    let expected_area = b * h / 2.0;
    assert!(
        approx_eq(props.area, expected_area, 1e-10),
        "area = {}",
        props.area
    );
    assert!(
        approx_eq(props.centroid.x, b / 3.0, 1e-10),
        "cx = {}",
        props.centroid.x
    );
    assert!(
        approx_eq(props.centroid.y, h / 3.0, 1e-10),
        "cy = {}",
        props.centroid.y
    );
}

/// Section modulus: Z = I / c
#[test]
fn analytical_section_modulus() {
    let b = 10.0;
    let h = 5.0;
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(b, 0.0),
        Point::new(b, h),
        Point::new(0.0, h),
    ]);
    let props = SectionProperties::from_section(&Section::new(outer, Vec::new()));
    // For rectangle: Zx = bh²/6
    let expected_zx = b * h.powi(2) / 6.0;
    let zxx = props.section_modulus_x();
    assert!(
        approx_eq(zxx, expected_zx, 1e-10),
        "Zx = {}, expected = {}",
        zxx,
        expected_zx
    );
}

// ===========================================================================
// 7. Rotation invariance — rotating a symmetric section
// ===========================================================================

/// Rotating a square by 45° should give same Ix, Iy (but Ixy may change).
#[test]
fn rotation_square_45deg() {
    let s = 10.0_f64;
    let sqrt2 = 2.0_f64.sqrt();

    // Axis-aligned square
    let outer1 = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(s, 0.0),
        Point::new(s, s),
        Point::new(0.0, s),
    ]);
    let props1 = SectionProperties::from_section(&Section::new(outer1, Vec::new()));

    // 45°-rotated square (diamond)
    let c = s / 2.0;
    let outer2 = Polygon::new(vec![
        Point::new(c, c - s * sqrt2 / 2.0),
        Point::new(c + s * sqrt2 / 2.0, c),
        Point::new(c, c + s * sqrt2 / 2.0),
        Point::new(c - s * sqrt2 / 2.0, c),
    ]);
    let props2 = SectionProperties::from_section(&Section::new(outer2, Vec::new()));

    // Area is the same
    assert!(
        approx_eq(props2.area, props1.area, 1e-8),
        "area: {} vs {}",
        props2.area,
        props1.area
    );
    // For a square, Ix = Iy regardless of rotation
    assert!(
        approx_eq(props2.ix, props2.iy, 1e-8),
        "ix == iy for rotated square"
    );
    // And Ix of rotated square should equal Ix of original
    assert!(
        approx_eq(props2.ix, props1.ix, 1e-8),
        "ix: {} vs {}",
        props2.ix,
        props1.ix
    );
}

// ===========================================================================
// 8. Perimeter validation
// ===========================================================================

/// Rectangle perimeter = 2(b + h), with hole adds inner perimeter.
#[test]
fn perimeter_rectangle_with_hole() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 5.0),
        Point::new(0.0, 5.0),
    ]);
    let hole = Polygon::new(vec![
        Point::new(3.0, 1.0),
        Point::new(7.0, 1.0),
        Point::new(7.0, 4.0),
        Point::new(3.0, 4.0),
    ]);
    let props = SectionProperties::from_section(&Section::new(outer, vec![hole]));
    // Outer perimeter = 30, hole perimeter = 14
    let expected = 30.0 + 14.0;
    assert!(
        approx_eq(props.perimeter, expected, 1e-10),
        "perimeter = {}",
        props.perimeter
    );
}
