//! Phase 11 - ear-clipping degeneracy audit, fan-fallback audit, and
//! warping-domain regression.
//!
//! The ear test must reject a candidate ear when another polygon vertex lies on
//! its open diagonal (case A) while still accepting legitimate ears whose
//! boundary carries collinear polygon vertices (case B) - the distinction that
//! Phase 10 showed is essential for bridged hole/keyhole geometry.

use section_properties::geometry::{Point, Polygon};
use section_properties::mesh::triangulation::triangulate_polygon_ear_clipping;
use section_properties::mesh::{MeshParams, mesh_section};
use section_properties::plastic::warping_fem::compute_fem_warping_solution;
use section_properties::section::Section;
use section_properties::section_properties::SectionProperties;
use serde::Deserialize;

type Ring = Vec<(f64, f64)>;

fn poly(ring: &Ring) -> Polygon {
    Polygon::new(ring.iter().map(|&(x, y)| Point::new(x, y)).collect())
}

fn ring_area(ring: &Ring) -> f64 {
    let mut a = 0.0;
    for i in 0..ring.len() {
        let (x0, y0) = ring[i];
        let (x1, y1) = ring[(i + 1) % ring.len()];
        a += x0 * y1 - x1 * y0;
    }
    a.abs() * 0.5
}

/// Exact triangulation invariants for a simple polygon (no refinement).
fn assert_ear_exact(label: &str, ring: &Ring) -> (usize, f64) {
    let p = poly(ring);
    let tris = triangulate_polygon_ear_clipping(&p);
    let mut total = 0.0;
    for t in &tris {
        let (a, b, c) = (p.vertices[t.v[0]], p.vertices[t.v[1]], p.vertices[t.v[2]]);
        let a2 = 0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)).abs();
        assert!(a2 > 0.0, "{label}: degenerate triangle");
        total += a2;
    }
    let expected = ring_area(ring);
    assert_eq!(
        tris.len(),
        ring.len() - 2,
        "{label}: expected n-2 = {} triangles, got {}",
        ring.len() - 2,
        tris.len()
    );
    assert!(
        (total - expected).abs() <= 1e-9 * expected.max(1.0),
        "{label}: ear-clip area {} != polygon area {} (ratio {:.6})",
        total,
        expected,
        total / expected
    );
    (tris.len(), total)
}

/// Convex-hull area, used to prove the Phase 10 failure mode is absent.
fn convex_hull_area(ring: &Ring) -> f64 {
    let mut pts = ring.clone();
    pts.sort_by(|a, b| {
        a.0.partial_cmp(&b.0)
            .unwrap()
            .then(a.1.partial_cmp(&b.1).unwrap())
    });
    let cross = |o: (f64, f64), a: (f64, f64), b: (f64, f64)| {
        (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0)
    };
    let mut hull: Vec<(f64, f64)> = Vec::new();
    for &p in &pts {
        while hull.len() >= 2 && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0 {
            hull.pop();
        }
        hull.push(p);
    }
    let lower = hull.len() + 1;
    for &p in pts.iter().rev() {
        while hull.len() >= lower && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0 {
            hull.pop();
        }
        hull.push(p);
    }
    hull.pop();
    ring_area(&hull)
}

fn i_section() -> Ring {
    vec![
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
    ]
}

// ---------------------------------------------------------------------------
// Case A - a vertex lies on the candidate diagonal: must reject the ear
// ---------------------------------------------------------------------------

#[test]
fn case_a_vertex_on_diagonal_rejects_ear() {
    let ring = i_section();
    let (n_tris, area) = assert_ear_exact("I-section (Phase 10 trigger)", &ring);
    let hull = convex_hull_area(&ring);
    assert!(
        (area - hull).abs() > 0.5 * hull,
        "triangulation must not cover the convex hull"
    );
    println!(
        "  case A: I-section tris={} area={} (polygon {} / hull {}) - convex hull NOT covered",
        n_tris,
        area,
        ring_area(&ring),
        hull
    );
    // The same defect appeared for every rotation of the geometry.
    let (cx, cy) = (75.0, 150.0);
    for deg in [45.0_f64, 90.0, -45.0, 135.0] {
        let t = deg.to_radians();
        let (c, s) = (t.cos(), t.sin());
        let rot: Ring = ring
            .iter()
            .map(|&(x, y)| {
                let (dx, dy) = (x - cx, y - cy);
                (cx + c * dx - s * dy, cy + s * dx + c * dy)
            })
            .collect();
        let (_, a) = assert_ear_exact(&format!("I-section rot {deg}"), &rot);
        assert!(
            (a - area).abs() <= 1e-9 * area,
            "rotation {deg}: area changed"
        );
    }
}

// ---------------------------------------------------------------------------
// Case B - collinear vertex on an adjacent edge: legitimate ears still accepted
// ---------------------------------------------------------------------------

#[test]
fn case_b_collinear_on_adjacent_edge_still_accepted() {
    // A rectangle whose bottom edge carries two extra collinear vertices, and
    // a notch whose flanks carry collinear vertices. An over-broad boundary
    // rejection rule (Phase 10 intermediate attempt) fails these.
    let cases: [(&str, Ring); 3] = [
        (
            "rectangle with collinear bottom-edge vertices",
            vec![
                (0.0, 0.0),
                (25.0, 0.0),
                (50.0, 0.0),
                (75.0, 0.0),
                (100.0, 0.0),
                (100.0, 100.0),
                (0.0, 100.0),
            ],
        ),
        (
            "notch with collinear flank vertices",
            vec![
                (0.0, 0.0),
                (100.0, 0.0),
                (100.0, 40.0),
                (60.0, 40.0),
                (60.0, 60.0),
                (60.0, 80.0),
                (100.0, 80.0),
                (100.0, 120.0),
                (0.0, 120.0),
            ],
        ),
        (
            "collinear vertices on both flange edges",
            vec![
                (0.0, 0.0),
                (50.0, 0.0),
                (100.0, 0.0),
                (100.0, 10.0),
                (55.0, 10.0),
                (55.0, 90.0),
                (100.0, 90.0),
                (100.0, 100.0),
                (50.0, 100.0),
                (0.0, 100.0),
                (0.0, 90.0),
                (45.0, 90.0),
                (45.0, 10.0),
                (0.0, 10.0),
            ],
        ),
    ];
    for (name, ring) in cases {
        let (n, a) = assert_ear_exact(name, &ring);
        println!("  case B: {name}: tris={n} area={a}");
    }
}

// ---------------------------------------------------------------------------
// Case D - another vertex strictly inside the candidate triangle: reject
// ---------------------------------------------------------------------------

#[test]
fn case_d_vertex_inside_candidate_triangle_rejected() {
    // Dart polygon: the reflex vertex (50,50) lies strictly inside the triangle
    // formed by its neighbours (0,0),(100,50),(0,100), so that ear must be
    // rejected. The only valid triangulation has 2 triangles.
    let ring: Ring = vec![(0.0, 0.0), (100.0, 50.0), (0.0, 100.0), (50.0, 50.0)];
    let (n, a) = assert_ear_exact("dart (reflex vertex inside neighbour triangle)", &ring);
    assert_eq!(n, 2, "dart must triangulate into exactly 2 triangles");
    println!("  case D: dart tris={n} area={a}");

    // A deeper variant with the interior vertex further inside.
    let ring2: Ring = vec![
        (0.0, 0.0),
        (120.0, 40.0),
        (0.0, 120.0),
        (60.0, 40.0),
        (10.0, 60.0),
    ];
    let (n2, a2) = assert_ear_exact("multi-notch interior vertices", &ring2);
    println!("  case D: multi-notch tris={n2} area={a2}");
}

// ---------------------------------------------------------------------------
// Case E - candidate diagonal crosses the polygon boundary: reject
// ---------------------------------------------------------------------------

#[test]
fn case_e_diagonal_crossing_boundary_rejected() {
    // Comb geometry: many diagonals available to the clipper cross the notches.
    let ring: Ring = vec![
        (0.0, 0.0),
        (120.0, 0.0),
        (120.0, 100.0),
        (100.0, 100.0),
        (100.0, 20.0),
        (80.0, 20.0),
        (80.0, 100.0),
        (60.0, 100.0),
        (60.0, 20.0),
        (40.0, 20.0),
        (40.0, 100.0),
        (20.0, 100.0),
        (20.0, 20.0),
        (0.0, 20.0),
    ];
    let (n, a) = assert_ear_exact("comb", &ring);
    println!("  case E: comb tris={n} area={a}");
    // Sanity: the comb area is far below its convex hull, so a fan fallback
    // over any remainder would be unmasked by the area assertion above.
    assert!((a - ring_area(&ring)).abs() <= 1e-9 * a);
    assert!(convex_hull_area(&ring) > 1.5 * a);
}

// ---------------------------------------------------------------------------
// Fallback audit - stress the ear clipper with many random simple polygons
// ---------------------------------------------------------------------------

fn lcg(state: &mut u64) -> f64 {
    *state = state
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    ((*state >> 11) as f64) / ((1u64 << 53) as f64)
}

/// Star-shaped polygons are simple by construction, arbitrarily concave, and
/// cheap to generate deterministically.
fn random_star_polygon(state: &mut u64, n: usize, r_min: f64, r_max: f64) -> Ring {
    (0..n)
        .map(|k| {
            let a = 2.0 * std::f64::consts::PI * k as f64 / n as f64;
            let r = r_min + (r_max - r_min) * lcg(state);
            (50.0 + r * a.cos(), 50.0 + r * a.sin())
        })
        .collect()
}

#[test]
fn fallback_never_required_for_valid_simple_polygons() {
    // 400 deterministic concave polygons. If the fan fallback ever executed,
    // the n-2 triangle count and/or the exact area would break, so these
    // assertions are also the fallback audit for legitimate input.
    let mut state = 0x51ee_d0d0_u64;
    let mut worst = 0.0f64;
    let mut cases = 0usize;
    for &(n, r_min, r_max) in &[
        (8usize, 20.0, 45.0),
        (12, 10.0, 50.0),
        (16, 2.0, 50.0), // extreme concavity / thin spikes
        (24, 5.0, 40.0),
        (6, 15.0, 45.0),
    ] {
        for _ in 0..80 {
            let ring = random_star_polygon(&mut state, n, r_min, r_max);
            let expected = ring_area(&ring);
            let p = poly(&ring);
            let tris = triangulate_polygon_ear_clipping(&p);
            let mut total = 0.0;
            for t in &tris {
                let (a, b, c) = (p.vertices[t.v[0]], p.vertices[t.v[1]], p.vertices[t.v[2]]);
                total += 0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)).abs();
            }
            let rel = (total - expected).abs() / expected;
            worst = worst.max(rel);
            cases += 1;
            assert_eq!(
                tris.len(),
                n - 2,
                "n={n}: triangle count {} != {} (possible fan fallback)",
                tris.len(),
                n - 2
            );
            assert!(
                rel <= 1e-9,
                "n={n}: area mismatch rel={rel:.3e} (possible fan fallback / convex hull)"
            );
        }
    }
    println!("  fallback audit: {cases} random simple polygons, worst area rel error {worst:.3e}");
    assert!(cases == 400);
}

// ---------------------------------------------------------------------------
// Bridged / keyhole geometry (Phase 10's over-rejection trap)
// ---------------------------------------------------------------------------

#[test]
fn bridged_hole_geometry_is_not_over_rejected() {
    let params = MeshParams {
        target_size: 8.0,
        max_size: 32.0,
        min_size: 1.0,
        quality_threshold: 0.3,
        use_delaunay: true,
        max_iterations: 2,
        max_nodes: 4000,
    };
    let outer: Ring = vec![(0.0, 0.0), (200.0, 0.0), (200.0, 100.0), (0.0, 100.0)];
    let hole: Ring = vec![(6.0, 6.0), (194.0, 6.0), (194.0, 94.0), (6.0, 94.0)];
    let section = Section::new(poly(&outer), vec![poly(&hole)]);
    let mesh = mesh_section(&section, params);
    let area: f64 = mesh
        .elements
        .iter()
        .map(|t| {
            let (a, b, c) = (mesh.nodes[t[0]], mesh.nodes[t[1]], mesh.nodes[t[2]]);
            0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)).abs()
        })
        .sum();
    let expected = ring_area(&outer) - ring_area(&hole);
    assert!(
        (area - expected).abs() <= 1e-9 * expected,
        "bridged keyhole mesh area {area} != expected {expected} (over-rejection of ears)"
    );

    // The same geometry must still produce a physical warping result.
    let props = SectionProperties::from_section(&section);
    let fem = compute_fem_warping_solution(
        &section,
        &props,
        0.3,
        section_properties::mesh::MeshControl::Normal,
    )
    .expect("warping must succeed for a bridged keyhole section");
    assert!(fem.j_raw > 0.0, "J_raw must be positive, got {}", fem.j_raw);
    assert!(
        !fem.used_analytical_fallback,
        "unexpected analytical fallback"
    );
    println!(
        "  bridged keyhole: area={area:.4} (expected {expected:.4}) J_raw={:.6e}",
        fem.j_raw
    );
}

// ---------------------------------------------------------------------------
// Warping domain regression for the representative sections
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
struct GeometryRef {
    outer: Vec<Vec<f64>>,
    #[serde(default)]
    holes: Vec<Vec<Vec<f64>>>,
}

fn load_section(case: &str) -> Section {
    let path = format!("tests/reference/cross_validation/{case}.json");
    let txt =
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("missing reference {path}: {e}"));
    let v: serde_json::Value = serde_json::from_str(&txt).unwrap();
    let g: GeometryRef = serde_json::from_value(v["geometry"].clone()).unwrap();
    let outer = poly(&g.outer.iter().map(|p| (p[0], p[1])).collect::<Ring>());
    let holes: Vec<Polygon> = g
        .holes
        .iter()
        .map(|h| poly(&h.iter().map(|p| (p[0], p[1])).collect::<Ring>()))
        .collect();
    Section::new(outer, holes)
}

#[test]
fn warping_domain_regression() {
    println!(
        "\n  {:<16} {:>13} {:>13} {:>13} {:>13} {:>8}",
        "case", "section A", "mesh A", "J_raw", "J", "fallback"
    );
    for case in [
        "rect_100x50",
        "channel_200x75",
        "angle_100x100",
        "tee_200x100",
        "rhs_200x100",
        "i_300x150",
    ] {
        let section = load_section(case);
        let props = SectionProperties::from_section(&section);
        let fem = compute_fem_warping_solution(
            &section,
            &props,
            0.3,
            section_properties::mesh::MeshControl::Normal,
        )
        .unwrap_or_else(|e| panic!("{case}: warping failed: {e:?}"));

        let mesh_area: f64 = fem
            .elements
            .iter()
            .map(|e| {
                let (x, y) = (&e.coords[0], &e.coords[1]);
                0.5 * ((x[1] - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (y[1] - y[0])).abs()
            })
            .sum();

        println!(
            "  {:<16} {:>13.5e} {:>13.5e} {:>13.5e} {:>13.5e} {:>8}",
            case, props.area, mesh_area, fem.j_raw, fem.j, fem.used_analytical_fallback
        );

        // Primary invariant: the FEM domain equals the requested geometry.
        assert!(
            (mesh_area - props.area).abs() <= 1e-6 * props.area,
            "{case}: mesh area {mesh_area} != section area {}",
            props.area
        );
        assert!(fem.j_raw > 0.0, "{case}: J_raw = {}", fem.j_raw);
        assert!(fem.j > 0.0, "{case}: J = {}", fem.j);
        assert!(fem.iw > 0.0, "{case}: Iw = {}", fem.iw);
    }
}
