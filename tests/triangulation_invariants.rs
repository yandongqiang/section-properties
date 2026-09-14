//! Phase 11 - triangulation and mesh robustness invariants.
//!
//! The fundamental invariant audited here is:
//!
//! ```text
//! triangulated domain == requested geometry domain
//! ```
//!
//! Every mesh produced by `mesh_section` must (a) consist of positively
//! oriented, finite triangles, (b) have every triangle inside the material,
//! (c) have no two triangles overlapping with positive area, and (d) cover
//! exactly the polygon area (outer minus holes). Two overlapping triangles can
//! accidentally reproduce the correct total area, which is why the area
//! invariant alone is not sufficient.

use section_properties::geometry::{Point, Polygon};
use section_properties::mesh::{Mesh, MeshParams, mesh_section};
use section_properties::section::Section;

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

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
    a * 0.5
}

/// Point-in-ring test (even-odd rule; ring may be either winding).
fn point_in_ring(p: (f64, f64), ring: &Ring) -> bool {
    let mut inside = false;
    let n = ring.len();
    for i in 0..n {
        let (xi, yi) = ring[i];
        let (xj, yj) = ring[(i + 1) % n];
        if (yi > p.1) != (yj > p.1) {
            let x_cross = xi + (p.1 - yi) / (yj - yi) * (xj - xi);
            if p.0 < x_cross {
                inside = !inside;
            }
        }
    }
    inside
}

/// Inside the material = inside the outer ring and outside every hole ring.
fn in_material(p: (f64, f64), outer: &Ring, holes: &[Ring]) -> bool {
    point_in_ring(p, outer) && !holes.iter().any(|h| point_in_ring(p, h))
}

// ---------------------------------------------------------------------------
// Mesh helpers
// ---------------------------------------------------------------------------

fn tri_pts(mesh: &Mesh, t: &[usize; 3]) -> [(f64, f64); 3] {
    [
        (mesh.nodes[t[0]].x, mesh.nodes[t[0]].y),
        (mesh.nodes[t[1]].x, mesh.nodes[t[1]].y),
        (mesh.nodes[t[2]].x, mesh.nodes[t[2]].y),
    ]
}

fn tri_area(t: &[(f64, f64); 3]) -> f64 {
    let (a, b, c) = (t[0], t[1], t[2]);
    0.5 * ((b.0 - a.0) * (c.1 - a.1) - (c.0 - a.0) * (b.1 - a.1))
}

fn centroid(t: &[(f64, f64); 3]) -> (f64, f64) {
    (
        (t[0].0 + t[1].0 + t[2].0) / 3.0,
        (t[0].1 + t[1].1 + t[2].1) / 3.0,
    )
}

/// Separating-axis test: true when two convex triangles overlap with an area
/// strictly greater than `eps` (touching along edges/vertices does not count).
fn triangles_overlap(t1: &[(f64, f64); 3], t2: &[(f64, f64); 3], eps: f64) -> bool {
    let axes = |t: &[(f64, f64); 3]| -> Vec<(f64, f64)> {
        let mut v = Vec::new();
        for i in 0..3 {
            let (ax, ay) = t[i];
            let (bx, by) = t[(i + 1) % 3];
            let (ex, ey) = (bx - ax, by - ay);
            // outward normal
            v.push((-ey, ex));
        }
        v
    };
    for (nx, ny) in axes(t1).into_iter().chain(axes(t2)) {
        let norm = (nx * nx + ny * ny).sqrt();
        if norm == 0.0 {
            continue;
        }
        let (nx, ny) = (nx / norm, ny / norm);
        let proj = |t: &[(f64, f64); 3]| {
            t.iter()
                .map(|&(x, y)| x * nx + y * ny)
                .fold((f64::MAX, f64::MIN), |(lo, hi), v| (lo.min(v), hi.max(v)))
        };
        let (lo1, hi1) = proj(t1);
        let (lo2, hi2) = proj(t2);
        let overlap = hi1.min(hi2) - lo1.max(lo2);
        if overlap <= eps {
            return false; // separating axis found (or mere touching)
        }
    }
    true
}

struct Report {
    label: String,
    expected: f64,
    mesh_area: f64,
    tris: usize,
}

fn assert_mesh_matches_geometry(
    label: &str,
    outer: &Ring,
    holes: &[Ring],
    params: MeshParams,
) -> Report {
    let expected = ring_area(outer).abs() - holes.iter().map(|h| ring_area(h).abs()).sum::<f64>();
    let section = Section::new(poly(outer), holes.iter().map(poly).collect());
    let mesh = mesh_section(&section, params);
    assert!(!mesh.elements.is_empty(), "{}: empty mesh", label);

    let mut total = 0.0;
    let mut tris: Vec<[(f64, f64); 3]> = Vec::with_capacity(mesh.elements.len());

    for t in &mesh.elements {
        let tp = tri_pts(&mesh, t);
        // (a) finite and positively oriented
        for &(x, y) in &tp {
            assert!(
                x.is_finite() && y.is_finite(),
                "{}: non-finite vertex",
                label
            );
        }
        let a = tri_area(&tp);
        assert!(
            a > 0.0,
            "{}: non-positive triangle area {} (inverted or degenerate)",
            label,
            a
        );
        total += a;
        // (b) inside the material
        assert!(
            in_material(centroid(&tp), outer, holes),
            "{}: triangle centroid {:?} outside the material",
            label,
            centroid(&tp)
        );
        tris.push(tp);
    }

    // (c) no two triangles overlap with positive area
    let eps = 1e-9 * expected.max(1.0).sqrt().max(1.0);
    for i in 0..tris.len() {
        for j in (i + 1)..tris.len() {
            assert!(
                !triangles_overlap(&tris[i], &tris[j], eps),
                "{}: triangles {} and {} overlap",
                label,
                i,
                j
            );
        }
    }

    // (d) exact area coverage (scale aware: relative, with a tiny absolute floor)
    let err = (total - expected).abs();
    let scale = expected.abs().max(1e-12);
    assert!(
        err <= 1e-9 * scale + 1e-12,
        "{}: mesh area {} != geometry area {} (abs {:.3e}, rel {:.3e}, {} triangles)",
        label,
        total,
        expected,
        err,
        err / scale,
        tris.len()
    );

    Report {
        label: label.to_string(),
        expected,
        mesh_area: total,
        tris: tris.len(),
    }
}

fn params(target: f64) -> MeshParams {
    MeshParams {
        target_size: target,
        max_size: target * 4.0,
        min_size: target / 8.0,
        quality_threshold: 0.3,
        use_delaunay: true,
        max_iterations: 2,
        max_nodes: 4000,
    }
}

// ---------------------------------------------------------------------------
// Corpus
// ---------------------------------------------------------------------------

fn convex_corpus() -> Vec<(&'static str, Ring)> {
    vec![
        (
            "rectangle",
            vec![(0.0, 0.0), (100.0, 0.0), (100.0, 50.0), (0.0, 50.0)],
        ),
        (
            "square",
            vec![(0.0, 0.0), (80.0, 0.0), (80.0, 80.0), (0.0, 80.0)],
        ),
        ("triangle", vec![(0.0, 0.0), (100.0, 0.0), (0.0, 80.0)]),
        ("regular hexagon", {
            (0..6)
                .map(|k| {
                    let a = std::f64::consts::PI / 3.0 * k as f64;
                    (50.0 + 40.0 * a.cos(), 50.0 + 40.0 * a.sin())
                })
                .collect()
        }),
    ]
}

fn concave_corpus() -> Vec<(&'static str, Ring)> {
    vec![
        (
            "L-section",
            vec![
                (0.0, 0.0),
                (100.0, 0.0),
                (100.0, 20.0),
                (20.0, 20.0),
                (20.0, 100.0),
                (0.0, 100.0),
            ],
        ),
        (
            "T-section",
            vec![
                (0.0, 0.0),
                (100.0, 0.0),
                (100.0, 20.0),
                (60.0, 20.0),
                (60.0, 120.0),
                (40.0, 120.0),
                (40.0, 20.0),
                (0.0, 20.0),
            ],
        ),
        (
            "channel",
            vec![
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
            "I-section",
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
            ],
        ),
        (
            "Z-section",
            vec![
                (0.0, 0.0),
                (60.0, 0.0),
                (60.0, 10.0),
                (10.0, 10.0),
                (10.0, 60.0),
                (70.0, 60.0),
                (70.0, 70.0),
                (0.0, 70.0),
            ],
        ),
        (
            "arrow polygon",
            vec![(0.0, 0.0), (50.0, 40.0), (100.0, 0.0), (50.0, 100.0)],
        ),
        (
            "U-shaped polygon",
            vec![
                (0.0, 0.0),
                (80.0, 0.0),
                (80.0, 80.0),
                (60.0, 80.0),
                (60.0, 20.0),
                (20.0, 20.0),
                (20.0, 80.0),
                (0.0, 80.0),
            ],
        ),
        (
            "double-notch",
            vec![
                (0.0, 0.0),
                (100.0, 0.0),
                (100.0, 100.0),
                (70.0, 100.0),
                (70.0, 30.0),
                (30.0, 30.0),
                (30.0, 100.0),
                (0.0, 100.0),
            ],
        ),
        (
            "multiple concavities (comb)",
            vec![
                (0.0, 0.0),
                (100.0, 0.0),
                (100.0, 100.0),
                (80.0, 100.0),
                (80.0, 20.0),
                (60.0, 20.0),
                (60.0, 100.0),
                (40.0, 100.0),
                (40.0, 20.0),
                (20.0, 20.0),
                (20.0, 100.0),
                (0.0, 100.0),
            ],
        ),
    ]
}

/// Collinear-vertex degeneracies: the class that exposed the Phase 10 defect.
fn collinear_corpus() -> Vec<(&'static str, Ring)> {
    vec![
        (
            "vertex on adjacent edge",
            vec![
                (0.0, 0.0),
                (50.0, 0.0),
                (100.0, 0.0),
                (100.0, 100.0),
                (0.0, 100.0),
            ],
        ),
        (
            "multiple consecutive collinear",
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
            "vertex on candidate ear diagonal (Phase 10 trigger)",
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
            ],
        ),
        (
            "collinear vertices at a notch",
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
    ]
}

fn hole_cases() -> Vec<(&'static str, Ring, Vec<Ring>)> {
    vec![
        (
            "centred hole",
            vec![(0.0, 0.0), (200.0, 0.0), (200.0, 100.0), (0.0, 100.0)],
            vec![vec![
                (75.0, 25.0),
                (125.0, 25.0),
                (125.0, 75.0),
                (75.0, 75.0),
            ]],
        ),
        (
            "eccentric hole",
            vec![(0.0, 0.0), (200.0, 0.0), (200.0, 100.0), (0.0, 100.0)],
            vec![vec![(20.0, 20.0), (60.0, 20.0), (60.0, 60.0), (20.0, 60.0)]],
        ),
        (
            "multiple holes",
            vec![(0.0, 0.0), (220.0, 0.0), (220.0, 120.0), (0.0, 120.0)],
            vec![
                vec![(20.0, 20.0), (60.0, 20.0), (60.0, 60.0), (20.0, 60.0)],
                vec![(140.0, 40.0), (190.0, 40.0), (190.0, 90.0), (140.0, 90.0)],
            ],
        ),
        (
            "thin-wall hollow section (bridged keyhole)",
            vec![(0.0, 0.0), (200.0, 0.0), (200.0, 100.0), (0.0, 100.0)],
            vec![vec![(6.0, 6.0), (194.0, 6.0), (194.0, 94.0), (6.0, 94.0)]],
        ),
    ]
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn convex_and_concave_area_invariants() {
    let mut reports = Vec::new();
    for (name, ring) in convex_corpus().into_iter().chain(concave_corpus()) {
        reports.push(assert_mesh_matches_geometry(name, &ring, &[], params(8.0)));
    }
    println!("\n=== convex/concave corpus ===");
    for r in &reports {
        println!(
            "  {:<34} expected={:>12.4} mesh={:>12.4} tris={}",
            r.label, r.expected, r.mesh_area, r.tris
        );
    }
}

#[test]
fn collinear_vertex_degeneracies() {
    let mut reports = Vec::new();
    for (name, ring) in collinear_corpus() {
        reports.push(assert_mesh_matches_geometry(name, &ring, &[], params(8.0)));
    }
    println!("\n=== collinear corpus ===");
    for r in &reports {
        println!(
            "  {:<48} expected={:>12.4} mesh={:>12.4} tris={}",
            r.label, r.expected, r.mesh_area, r.tris
        );
    }
}

#[test]
fn hole_cases_exclude_hole_interiors() {
    let mut reports = Vec::new();
    for (name, outer, holes) in hole_cases() {
        reports.push(assert_mesh_matches_geometry(
            name,
            &outer,
            &holes,
            params(8.0),
        ));
    }
    println!("\n=== hole corpus (hole interiors excluded) ===");
    for r in &reports {
        println!(
            "  {:<40} expected={:>12.4} mesh={:>12.4} tris={}",
            r.label, r.expected, r.mesh_area, r.tris
        );
    }
}

#[test]
fn rotation_invariance_of_concave_meshes() {
    println!("\n=== rotation invariance (mesh area must be preserved) ===");
    for (name, ring) in concave_corpus() {
        // rotate about the ring centroid so the geometry stays in place
        let n = ring.len() as f64;
        let (cx, cy) = (
            ring.iter().map(|p| p.0).sum::<f64>() / n,
            ring.iter().map(|p| p.1).sum::<f64>() / n,
        );
        let base = assert_mesh_matches_geometry(&format!("{name} @0deg"), &ring, &[], params(8.0));
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
            let r =
                assert_mesh_matches_geometry(&format!("{name} @{deg}deg"), &rot, &[], params(8.0));
            let rel = (r.mesh_area - base.expected).abs() / base.expected.abs();
            println!(
                "  {:<36} expected={:>12.4} mesh={:>12.4} rel={:.3e}",
                r.label, r.expected, r.mesh_area, rel
            );
        }
    }
}

#[test]
fn scale_invariance_of_concave_meshes() {
    println!("\n=== scale invariance: mesh area ~ alpha^2 ===");
    for (name, ring) in [
        (
            "L-section",
            vec![
                (0.0, 0.0),
                (100.0, 0.0),
                (100.0, 20.0),
                (20.0, 20.0),
                (20.0, 100.0),
                (0.0, 100.0),
            ],
        ),
        (
            "I-section",
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
            ],
        ),
    ] {
        let a0 = ring_area(&ring).abs();
        for alpha in [1e-6_f64, 1e-3, 1.0, 1e3, 1e6] {
            let scaled: Ring = ring.iter().map(|&(x, y)| (x * alpha, y * alpha)).collect();
            let r = assert_mesh_matches_geometry(
                &format!("{name} alpha={alpha:e}"),
                &scaled,
                &[],
                params(8.0 * alpha),
            );
            let expected = a0 * alpha * alpha;
            let rel = (r.mesh_area - expected).abs() / expected;
            assert!(
                rel <= 1e-6,
                "{name} alpha={alpha:e}: mesh {:.6e} vs expected {:.6e} (rel {:.3e})",
                r.mesh_area,
                expected,
                rel
            );
            println!(
                "  {:<26} expected={:>14.6e} mesh={:>14.6e} rel={:.3e}",
                r.label, expected, r.mesh_area, rel
            );
        }
    }
}

/// Ear clipping (before refinement) must satisfy the exact simple-polygon
/// invariants: n-2 triangles and exact area for every corpus polygon.
#[test]
fn ear_clipping_exact_area_and_triangle_count() {
    use section_properties::mesh::triangulation::triangulate_polygon_ear_clipping;

    println!("\n=== ear clipping: n-2 triangles and exact area ===");
    for (name, ring) in convex_corpus()
        .into_iter()
        .chain(concave_corpus())
        .chain(collinear_corpus())
    {
        let p = poly(&ring);
        let tris = triangulate_polygon_ear_clipping(&p);
        let mut total = 0.0;
        for t in &tris {
            let (a, b, c) = (p.vertices[t.v[0]], p.vertices[t.v[1]], p.vertices[t.v[2]]);
            total += 0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)).abs();
        }
        let expected = ring_area(&ring).abs();
        println!(
            "  {:<48} n={:>3} tris={:>3} (n-2={:>3}) expected={:>10.3} got={:>10.3}",
            name,
            ring.len(),
            tris.len(),
            ring.len() - 2,
            expected,
            total
        );
        assert_eq!(tris.len(), ring.len() - 2, "{name}: triangle count");
        assert!(
            (total - expected).abs() <= 1e-9 * expected.max(1.0),
            "{name}: ear-clip area {total} != polygon area {expected}"
        );
    }
}
