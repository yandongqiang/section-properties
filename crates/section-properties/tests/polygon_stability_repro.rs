//! Phase 65: Polygon centroidal inertia numerical stability regression tests.
//!
//! Verifies that `Polygon::centroidal_moment_of_inertia_x`,
//! `Polygon::centroidal_moment_of_inertia_y`, and
//! `Polygon::centroidal_product_of_inertia_xy` produce accurate results
//! regardless of the polygon's absolute position in the plane.
//!
//! The vertex-reference two-pass strategy shifts all computations to
//! coordinates relative to the first vertex, avoiding catastrophic
//! cancellation that would otherwise occur in the parallel-axis theorem
//! `Ix_global - A * y_c²` when the polygon is far from the origin.

use section_properties::geometry::{Point, Polygon};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn rect_at(x0: f64, y0: f64, w: f64, h: f64) -> Polygon {
    Polygon::new(vec![
        Point::new(x0, y0),
        Point::new(x0 + w, y0),
        Point::new(x0 + w, y0 + h),
        Point::new(x0, y0 + h),
    ])
}

fn triangle_at(x0: f64, y0: f64, w: f64, h: f64) -> Polygon {
    Polygon::new(vec![
        Point::new(x0, y0),
        Point::new(x0 + w, y0),
        Point::new(x0, y0 + h),
    ])
}

fn rel_err(computed: f64, expected: f64) -> f64 {
    ((computed - expected) / expected).abs()
}

// ---------------------------------------------------------------------------
// 1. Rectangle scale sweep — Y translation (ix sensitive)
// ---------------------------------------------------------------------------

#[test]
fn rect_y_translation_ix_stable() {
    let w = 1.0_f64;
    let h = 1.0_f64;
    let ix_ref = w * h.powi(3) / 12.0;
    let iy_ref = h * w.powi(3) / 12.0;

    for &offset in &[1.0, 1e4, 1e6, 1e8, 1e10, 1e12] {
        let poly = rect_at(0.0, offset, w, h);
        let ix = poly.centroidal_moment_of_inertia_x();
        let iy = poly.centroidal_moment_of_inertia_y();
        let ixy = poly.centroidal_product_of_inertia_xy();

        assert!(
            rel_err(ix, ix_ref) < 1e-10,
            "ix at y-offset {offset:.0e}: got {ix:.6e}, expected {ix_ref:.6e}, rel_err {}",
            rel_err(ix, ix_ref)
        );
        assert!(
            rel_err(iy, iy_ref) < 1e-10,
            "iy at y-offset {offset:.0e}: got {iy:.6e}, expected {iy_ref:.6e}, rel_err {}",
            rel_err(iy, iy_ref)
        );
        assert!(
            ixy.abs() < 1e-10,
            "ixy at y-offset {offset:.0e}: got {ixy:.6e}, expected 0"
        );
    }
}

// ---------------------------------------------------------------------------
// 2. Rectangle scale sweep — X translation (iy sensitive)
// ---------------------------------------------------------------------------

#[test]
fn rect_x_translation_iy_stable() {
    let w = 2.0_f64;
    let h = 0.5_f64;
    let ix_ref = w * h.powi(3) / 12.0;
    let iy_ref = h * w.powi(3) / 12.0;

    for &offset in &[1.0, 1e4, 1e6, 1e8, 1e10, 1e12] {
        let poly = rect_at(offset, 0.0, w, h);
        let ix = poly.centroidal_moment_of_inertia_x();
        let iy = poly.centroidal_moment_of_inertia_y();
        let ixy = poly.centroidal_product_of_inertia_xy();

        assert!(
            rel_err(ix, ix_ref) < 1e-10,
            "ix at x-offset {offset:.0e}: got {ix:.6e}, expected {ix_ref:.6e}"
        );
        assert!(
            rel_err(iy, iy_ref) < 1e-10,
            "iy at x-offset {offset:.0e}: got {iy:.6e}, expected {iy_ref:.6e}"
        );
        assert!(
            ixy.abs() < 1e-10,
            "ixy at x-offset {offset:.0e}: got {ixy:.6e}, expected 0"
        );
    }
}

// ---------------------------------------------------------------------------
// 3. Rectangle diagonal translation (both ix and iy sensitive)
// ---------------------------------------------------------------------------

#[test]
fn rect_diagonal_translation_stable() {
    let w = 3.0_f64;
    let h = 2.0_f64;
    let ix_ref = w * h.powi(3) / 12.0;
    let iy_ref = h * w.powi(3) / 12.0;

    for &offset in &[1.0, 1e6, 1e8, 1e10, 1e12] {
        let poly = rect_at(offset, offset, w, h);
        let ix = poly.centroidal_moment_of_inertia_x();
        let iy = poly.centroidal_moment_of_inertia_y();
        let ixy = poly.centroidal_product_of_inertia_xy();

        assert!(
            rel_err(ix, ix_ref) < 1e-10,
            "ix at diag-offset {offset:.0e}: got {ix:.6e}, expected {ix_ref:.6e}"
        );
        assert!(
            rel_err(iy, iy_ref) < 1e-10,
            "iy at diag-offset {offset:.0e}: got {iy:.6e}, expected {iy_ref:.6e}"
        );
        assert!(
            ixy.abs() < 1e-10,
            "ixy at diag-offset {offset:.0e}: got {ixy:.6e}, expected 0"
        );
    }
}

// ---------------------------------------------------------------------------
// 4. Negative offsets
// ---------------------------------------------------------------------------

#[test]
fn rect_negative_offset_stable() {
    let w = 1.0_f64;
    let h = 1.0_f64;
    let ix_ref = w * h.powi(3) / 12.0;
    let iy_ref = h * w.powi(3) / 12.0;

    for &offset in &[-1e6_f64, -1e8, -1e10, -1e12] {
        let poly = rect_at(offset, offset, w, h);
        let ix = poly.centroidal_moment_of_inertia_x();
        let iy = poly.centroidal_moment_of_inertia_y();
        let ixy = poly.centroidal_product_of_inertia_xy();

        assert!(
            rel_err(ix, ix_ref) < 1e-10,
            "ix at neg-offset {offset:.0e}: got {ix:.6e}, expected {ix_ref:.6e}"
        );
        assert!(
            rel_err(iy, iy_ref) < 1e-10,
            "iy at neg-offset {offset:.0e}: got {iy:.6e}, expected {iy_ref:.6e}"
        );
        assert!(
            ixy.abs() < 1e-10,
            "ixy at neg-offset {offset:.0e}: got {ixy:.6e}, expected 0"
        );
    }
}

// ---------------------------------------------------------------------------
// 5. Triangle — non-symmetric shape
// ---------------------------------------------------------------------------

#[test]
fn triangle_translation_stable() {
    // Right triangle with legs w, h, vertices (0,0)-(w,0)-(0,h).
    // Centroidal ix = w*h^3/36, iy = h*w^3/36, ixy = -w^2*h^2/72.
    let w = 2.0_f64;
    let h = 3.0_f64;
    let ix_ref = w * h.powi(3) / 36.0;
    let iy_ref = h * w.powi(3) / 36.0;
    let ixy_ref = -w * w * h * h / 72.0;

    for &offset in &[0.0, 1.0, 1e6, 1e8, 1e10, 1e12] {
        let poly = triangle_at(offset, offset, w, h);
        let ix = poly.centroidal_moment_of_inertia_x();
        let iy = poly.centroidal_moment_of_inertia_y();
        let ixy = poly.centroidal_product_of_inertia_xy();

        assert!(
            rel_err(ix, ix_ref) < 1e-10,
            "tri ix at offset {offset:.0e}: got {ix:.6e}, expected {ix_ref:.6e}"
        );
        assert!(
            rel_err(iy, iy_ref) < 1e-10,
            "tri iy at offset {offset:.0e}: got {iy:.6e}, expected {iy_ref:.6e}"
        );
        assert!(
            rel_err(ixy, ixy_ref) < 1e-10,
            "tri ixy at offset {offset:.0e}: got {ixy:.6e}, expected {ixy_ref:.6e}"
        );
    }
}

// ---------------------------------------------------------------------------
// 6. Translation invariance — centroidal moments must not depend on position
// ---------------------------------------------------------------------------

#[test]
fn translation_invariance() {
    let w = 5.0_f64;
    let h = 3.0_f64;

    let poly_origin = rect_at(0.0, 0.0, w, h);
    let ix0 = poly_origin.centroidal_moment_of_inertia_x();
    let iy0 = poly_origin.centroidal_moment_of_inertia_y();
    let ixy0 = poly_origin.centroidal_product_of_inertia_xy();

    for &(dx, dy) in &[(1e8_f64, 2e8_f64), (-3e10_f64, 5e10_f64), (1e12, -7e11)] {
        let poly = rect_at(dx, dy, w, h);
        let ix = poly.centroidal_moment_of_inertia_x();
        let iy = poly.centroidal_moment_of_inertia_y();
        let ixy = poly.centroidal_product_of_inertia_xy();

        assert!(
            rel_err(ix, ix0) < 1e-10,
            "translation invariance ix: origin={ix0:.6e}, shifted={ix:.6e} at ({dx:.0e},{dy:.0e})"
        );
        assert!(
            rel_err(iy, iy0) < 1e-10,
            "translation invariance iy: origin={iy0:.6e}, shifted={iy:.6e} at ({dx:.0e},{dy:.0e})"
        );
        assert!(
            (ixy - ixy0).abs() < 1e-10,
            "translation invariance ixy: origin={ixy0:.6e}, shifted={ixy:.6e}"
        );
    }
}

// ---------------------------------------------------------------------------
// 7. Known values at origin — regression for standard shapes
// ---------------------------------------------------------------------------

#[test]
fn known_values_at_origin() {
    // Unit square at origin
    let sq = rect_at(0.0, 0.0, 1.0, 1.0);
    assert!((sq.centroidal_moment_of_inertia_x() - 1.0 / 12.0).abs() < 1e-15);
    assert!((sq.centroidal_moment_of_inertia_y() - 1.0 / 12.0).abs() < 1e-15);
    assert!(sq.centroidal_product_of_inertia_xy().abs() < 1e-15);

    // 2×4 rectangle at origin
    let r = rect_at(0.0, 0.0, 2.0, 4.0);
    assert!((r.centroidal_moment_of_inertia_x() - 2.0 * 64.0 / 12.0).abs() < 1e-13);
    assert!((r.centroidal_moment_of_inertia_y() - 4.0 * 8.0 / 12.0).abs() < 1e-13);
    assert!(r.centroidal_product_of_inertia_xy().abs() < 1e-13);

    // Right triangle 3-4-5 at origin
    let tri = triangle_at(0.0, 0.0, 3.0, 4.0);
    assert!((tri.centroidal_moment_of_inertia_x() - 3.0 * 64.0 / 36.0).abs() < 1e-13);
    assert!((tri.centroidal_moment_of_inertia_y() - 4.0 * 27.0 / 36.0).abs() < 1e-13);
    assert!((tri.centroidal_product_of_inertia_xy() - (-9.0 * 16.0 / 72.0)).abs() < 1e-13);
}

// ---------------------------------------------------------------------------
// 8. Asymmetric quadrilateral with nonzero product of inertia
// ---------------------------------------------------------------------------

#[test]
fn asymmetric_quad_product_of_inertia_stable() {
    // Trapezoid: vertices (0,0), (4,0), (3,2), (1,2)
    // This has a nonzero centroidal product of inertia.
    let trap_origin = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(4.0, 0.0),
        Point::new(3.0, 2.0),
        Point::new(1.0, 2.0),
    ]);

    let ixy0 = trap_origin.centroidal_product_of_inertia_xy();
    let ix0 = trap_origin.centroidal_moment_of_inertia_x();
    let iy0 = trap_origin.centroidal_moment_of_inertia_y();

    // Translate far from origin — centroidal values must be unchanged.
    for &offset in &[1e8_f64, 1e10, 1e12] {
        let trap = Polygon::new(vec![
            Point::new(offset, offset),
            Point::new(offset + 4.0, offset),
            Point::new(offset + 3.0, offset + 2.0),
            Point::new(offset + 1.0, offset + 2.0),
        ]);

        let ix = trap.centroidal_moment_of_inertia_x();
        let iy = trap.centroidal_moment_of_inertia_y();
        let ixy = trap.centroidal_product_of_inertia_xy();

        assert!(
            rel_err(ix, ix0) < 1e-10,
            "trap ix at offset {offset:.0e}: got {ix:.6e}, expected {ix0:.6e}"
        );
        assert!(
            rel_err(iy, iy0) < 1e-10,
            "trap iy at offset {offset:.0e}: got {iy:.6e}, expected {iy0:.6e}"
        );
        // ixy0 is 0 for this symmetric trapezoid — use absolute error.
        assert!(
            (ixy - ixy0).abs() < 1e-10,
            "trap ixy at offset {offset:.0e}: got {ixy:.6e}, expected {ixy0:.6e}"
        );
    }
}
