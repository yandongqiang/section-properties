//! Regression tests for the ear-clipping triangulation defect found in Phase 10.
//!
//! # Defect
//!
//! `triangulate_polygon_ear_clipping` accepted an "ear" whose new diagonal
//! passed exactly through another polygon vertex. For I-shaped (double-notch)
//! polygons that produced a self-intersecting remainder; the loop then found no
//! further ear and fell back to a fan triangulation, which covers the **convex
//! hull** instead of the polygon. The mesh area was 7.75x the section area,
//! which corrupted every domain integral - in particular the warping FEM
//! torsion constant, which came out negative (`J_raw < 0`) and triggered the
//! analytical fallback.
//!
//! The fix rejects an ear when any other vertex lies on its open diagonal. The
//! containment test is deliberately left strict (interior only) because bridged
//! hole/keyhole polygons legitimately carry collinear vertices on the two
//! polygon edges of an ear.

use section_properties::geometry::{Point, Polygon};
use section_properties::mesh::MeshControl;
use section_properties::mesh::triangulation::triangulate_polygon_ear_clipping;
use section_properties::plastic::warping_fem::compute_fem_warping_solution;
use section_properties::section::Section;
use section_properties::section_properties::SectionProperties;

fn polygon_area(pts: &[(f64, f64)]) -> f64 {
    let mut a = 0.0;
    for i in 0..pts.len() {
        let (x0, y0) = pts[i];
        let (x1, y1) = pts[(i + 1) % pts.len()];
        a += x0 * y1 - x1 * y0;
    }
    a.abs() * 0.5
}

fn snapped_area(pts: &[(f64, f64)]) -> f64 {
    let poly = Polygon::new(pts.iter().map(|&(x, y)| Point::new(x, y)).collect());
    let tris = triangulate_polygon_ear_clipping(&poly);
    let mut total = 0.0;
    for t in &tris {
        let p0 = poly.vertices[t.v[0]];
        let p1 = poly.vertices[t.v[1]];
        let p2 = poly.vertices[t.v[2]];
        total += 0.5 * ((p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y)).abs();
    }
    total
}

/// I-section 300x150, tf=12, tw=8 (the Phase 9/10 failing case).
const I_SECTION: [(f64, f64); 12] = [
    (0.0, 0.0),
    (150.0, 0.0),
    (150.0, 12.0),
    (79.0, 12.0),
    (79.0, 288.0),
    (150.0, 288.0),
    (150.0, 300.0),
    (0.0, 300.0),
    (0.0, 288.0),
    (71.0, 288.0),
    (71.0, 12.0),
    (0.0, 12.0),
];

#[test]
fn ear_clipping_preserves_polygon_area() {
    let cases: [(&str, &[(f64, f64)]); 5] = [
        ("I-section", &I_SECTION),
        (
            "simple I (symmetric)",
            &[
                (0.0, 0.0),
                (100.0, 0.0),
                (100.0, 10.0),
                (55.0, 10.0),
                (55.0, 90.0),
                (100.0, 90.0),
                (100.0, 100.0),
                (0.0, 100.0),
                (0.0, 90.0),
                (45.0, 90.0),
                (45.0, 10.0),
                (0.0, 10.0),
            ],
        ),
        (
            "T-section",
            &[
                (46.0, 0.0),
                (54.0, 0.0),
                (54.0, 188.0),
                (100.0, 188.0),
                (100.0, 200.0),
                (0.0, 200.0),
                (0.0, 188.0),
                (46.0, 188.0),
            ],
        ),
        (
            "channel",
            &[
                (0.0, 0.0),
                (75.0, 0.0),
                (75.0, 10.0),
                (8.0, 10.0),
                (8.0, 190.0),
                (75.0, 190.0),
                (75.0, 200.0),
                (0.0, 200.0),
            ],
        ),
        (
            "rectangle",
            &[(0.0, 0.0), (100.0, 0.0), (100.0, 50.0), (0.0, 50.0)],
        ),
    ];

    for (name, pts) in cases {
        let exact = polygon_area(pts);
        let tri = snapped_area(pts);
        assert!(
            (tri - exact).abs() <= 1e-9 * exact.max(1.0),
            "{}: triangulation area {} != polygon area {} (ratio {:.6})",
            name,
            tri,
            exact,
            tri / exact
        );
        // A correct triangulation of a simple polygon with n vertices has
        // exactly n-2 triangles.
        assert_eq!(
            triangulate_polygon_ear_clipping(&Polygon::new(
                pts.iter().map(|&(x, y)| Point::new(x, y)).collect()
            ))
            .len(),
            pts.len() - 2,
            "{}: unexpected triangle count",
            name
        );
    }
}

/// End-to-end regression for the original failure: the I-section warping FEM
/// must integrate over the correct domain and produce a physical (positive)
/// raw torsion constant, without invoking the analytical fallback.
#[test]
fn i_section_warping_is_physical() {
    let poly = Polygon::new(I_SECTION.iter().map(|&(x, y)| Point::new(x, y)).collect());
    let section = Section::new(poly, vec![]);
    let props = SectionProperties::from_section(&section);

    let fem = compute_fem_warping_solution(&section, &props, 0.3, MeshControl::Normal)
        .expect("warping FEM must succeed");

    // (1) The mesh must cover the polygon, not its convex hull.
    let mesh_area: f64 = fem
        .elements
        .iter()
        .map(|e| {
            let (x, y) = (&e.coords[0], &e.coords[1]);
            0.5 * ((x[1] - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (y[1] - y[0])).abs()
        })
        .sum();
    assert!(
        (mesh_area - props.area).abs() <= 1e-6 * props.area,
        "mesh area {} != section area {} (convex-hull triangulation?)",
        mesh_area,
        props.area
    );

    // (2) J_raw must be physically valid; the fallback must not be needed.
    assert!(
        fem.j_raw > 0.0,
        "J_raw must be positive for a valid section, got {}",
        fem.j_raw
    );
    assert!(
        !fem.used_analytical_fallback,
        "the analytical fallback must not be triggered"
    );

    // (3) Compare against the thin-walled reference J = sum(b t^3 / 3) for an
    // open I-section: 2 flanges 150x12 plus a 276x8 web.
    let j_thin = 2.0 * (150.0 * 12f64.powi(3) / 3.0) + 276.0 * 8f64.powi(3) / 3.0;
    assert!(
        (fem.j - j_thin).abs() <= 0.05 * j_thin,
        "J = {:.6e} deviates more than 5% from the thin-wall reference {:.6e}",
        fem.j,
        j_thin
    );

    // (4) Python sectionproperties 3.10.2 reference for the same geometry.
    let j_python = 2.16514910e5;
    assert!(
        (fem.j - j_python).abs() <= 0.05 * j_python,
        "J = {:.6e} deviates more than 5% from the Python reference {:.6e}",
        fem.j,
        j_python
    );
}
