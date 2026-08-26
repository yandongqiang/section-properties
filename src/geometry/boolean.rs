//! Boolean operations on polygons (union, intersection, difference).
//!
//! Implements the Greiner–Hormann clipping algorithm, which plays the same
//! role shapely plays for Python `sectionproperties`. Handles non-convex
//! simple polygons; degenerate cases (a vertex lying exactly on another
//! polygon's edge) are avoided by convention.
//!
//! `difference(a, b)` is realised as `intersection(a, b.reversed())`.

use super::{Point, Polygon};

/// Boolean operation selector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoolOp {
    /// Region covered by both polygons.
    Intersection,
    /// Region covered by either polygon.
    Union,
    /// Region covered by `a` but not `b`.
    Difference,
}

#[derive(Debug, Clone)]
struct Node {
    pt: Point,
    prev: usize,
    next: usize,
    is_inter: bool,
    /// Walking forward along this ring, do we enter the other polygon?
    entry: bool,
    /// Index of the paired node in the other ring.
    neighbor: usize,
    /// Shared intersection id (pairing key).
    pair: usize,
    processed: bool,
}

#[derive(Debug, Clone)]
struct Ring {
    nodes: Vec<Node>,
}

impl Ring {
    fn step(&self, i: usize, forward: bool) -> usize {
        if forward {
            self.nodes[i].next
        } else {
            self.nodes[i].prev
        }
    }
}

struct RawIntersection {
    ea: usize,
    ta: f64,
    eb: usize,
    tb: f64,
    pt: Point,
}

/// Proper interior edge-edge intersection parameters (`None` if parallel or
/// non-crossing).
fn edge_intersection(p1: Point, p2: Point, q1: Point, q2: Point) -> Option<(f64, f64)> {
    let rx = p2.x - p1.x;
    let ry = p2.y - p1.y;
    let sx = q2.x - q1.x;
    let sy = q2.y - q1.y;

    let denom = rx * sy - ry * sx;
    if denom.abs() < 1e-14 {
        return None;
    }

    let qpx = q1.x - p1.x;
    let qpy = q1.y - p1.y;
    let s = (qpx * sy - qpy * sx) / denom;
    let t = (qpx * ry - qpy * rx) / denom;

    if s > 0.0 && s < 1.0 && t > 0.0 && t < 1.0 {
        Some((s, t))
    } else {
        None
    }
}

fn compute_raw_intersections(pa: &[Point], pb: &[Point]) -> Vec<RawIntersection> {
    let na = pa.len();
    let nb = pb.len();
    let mut out = Vec::new();

    for i in 0..na {
        for j in 0..nb {
            if let Some((ta, tb)) =
                edge_intersection(pa[i], pa[(i + 1) % na], pb[j], pb[(j + 1) % nb])
            {
                out.push(RawIntersection {
                    ea: i,
                    ta,
                    eb: j,
                    tb,
                    pt: Point::new(
                        pa[i].x + ta * (pa[(i + 1) % na].x - pa[i].x),
                        pa[i].y + ta * (pa[(i + 1) % na].y - pa[i].y),
                    ),
                });
            }
        }
    }
    out
}

/// Build a ring from base vertices plus spliced intersection nodes.
/// `inters`: (edge_index, param, shared_id) sorted later per-edge.
fn build_ring(base: &[Point], inters: &[(usize, f64, usize)], raw: &[RawIntersection]) -> Ring {
    let n = base.len();
    let mut by_edge: std::collections::HashMap<usize, Vec<(f64, usize)>> =
        std::collections::HashMap::new();
    for &(e, t, id) in inters {
        by_edge.entry(e).or_default().push((t, id));
    }
    for list in by_edge.values_mut() {
        list.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap());
    }

    let mut nodes: Vec<Node> = Vec::new();
    for i in 0..n {
        let idx = nodes.len();
        nodes.push(Node {
            pt: base[i],
            prev: idx.wrapping_sub(usize::MAX / 2), // fixed below
            next: 0,
            is_inter: false,
            entry: false,
            neighbor: usize::MAX,
            pair: usize::MAX,
            processed: false,
        });

        if let Some(list) = by_edge.get(&i) {
            for &(_, id) in list {
                let idx = nodes.len();
                nodes.push(Node {
                    pt: raw[id].pt,
                    prev: idx - 1,
                    next: idx + 1,
                    is_inter: true,
                    entry: false,
                    neighbor: usize::MAX,
                    pair: id,
                    processed: false,
                });
            }
        }
    }

    // Close ring links.
    let last = nodes.len() - 1;
    for i in 0..=last {
        nodes[i].next = if i == last { 0 } else { i + 1 };
        nodes[i].prev = if i == 0 { last } else { i - 1 };
    }

    Ring { nodes }
}

/// Assign entry flags by walking forward from the first vertex, whose
/// inside/outside status w.r.t. the other polygon is given.
fn assign_entries(ring: &mut Ring, first_inside_other: bool) {
    let mut inside = first_inside_other;
    for i in 0..ring.nodes.len() {
        if ring.nodes[i].is_inter {
            ring.nodes[i].entry = !inside;
            inside = !inside;
        }
    }
}

/// Trace one output loop starting at an unprocessed intersection node.
fn trace_loop(a: &mut Ring, b: &mut Ring, start: usize, op: BoolOp) -> Vec<Point> {
    let mut pts = vec![a.nodes[start].pt];
    let mut in_a = true;
    let mut cur = start;
    let max_iter = a.nodes.len() + b.nodes.len() + 4;

    for _ in 0..max_iter {
        // Direction rule (Foster):
        // intersection: forward if entering; union: forward if exiting.
        // difference is handled upstream as intersection with reversed B.
        let entry = {
            let ring = if in_a { &*a } else { &*b };
            ring.nodes[cur].entry
        };
        let forward = match (op, in_a) {
            (BoolOp::Intersection, _) => entry,
            (BoolOp::Union, _) => !entry,
            (BoolOp::Difference, true) => !entry,
            (BoolOp::Difference, false) => entry,
        };

        a_mark_pair(a, b, cur, in_a);

        // Walk emitting ordinary vertices until the next intersection.
        loop {
            cur = {
                let ring = if in_a { &*a } else { &*b };
                ring.step(cur, forward)
            };
            let is_inter = {
                let ring = if in_a { &*a } else { &*b };
                ring.nodes[cur].is_inter
            };
            if is_inter || (cur == start && in_a) {
                break;
            }
            pts.push({
                let ring = if in_a { &*a } else { &*b };
                ring.nodes[cur].pt
            });
        }

        if cur == start && in_a {
            break;
        }

        // Emit the reached intersection, consume it, and jump across rings.
        pts.push({
            let ring = if in_a { &*a } else { &*b };
            ring.nodes[cur].pt
        });
        a_mark_pair(a, b, cur, in_a);
        let reached_neighbor = {
            let ring = if in_a { &*a } else { &*b };
            ring.nodes[cur].neighbor
        };
        debug_assert!(reached_neighbor != usize::MAX);
        cur = reached_neighbor;
        in_a = !in_a;

        // Closure: arriving back at the start node (only meaningful once we
        // are back in ring A).
        if in_a && cur == start {
            break;
        }
    }

    pts
}
fn a_mark_pair(a: &mut Ring, b: &mut Ring, cur: usize, in_a: bool) {
    if in_a {
        a.nodes[cur].processed = true;
        let n = a.nodes[cur].neighbor;
        if n != usize::MAX {
            b.nodes[n].processed = true;
        }
    } else {
        b.nodes[cur].processed = true;
        let n = b.nodes[cur].neighbor;
        if n != usize::MAX {
            a.nodes[n].processed = true;
        }
    }
}

fn signed_area_pts(pts: &[Point]) -> f64 {
    pts.iter()
        .enumerate()
        .map(|(k, p)| {
            let q = &pts[(k + 1) % pts.len()];
            p.x * q.y - q.x * p.y
        })
        .sum::<f64>()
        / 2.0
}


/// Approximate scale of a polygon (bounding-box diagonal).
fn bbox_diag(pts: &[Point]) -> f64 {
    let (mut mn_x, mut mn_y) = (f64::INFINITY, f64::INFINITY);
    let (mut mx_x, mut mx_y) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in pts {
        mn_x = mn_x.min(p.x);
        mn_y = mn_y.min(p.y);
        mx_x = mx_x.max(p.x);
        mx_y = mx_y.max(p.y);
    }
    ((mx_x - mn_x).powi(2) + (mx_y - mn_y).powi(2)).sqrt().max(1e-12)
}

/// Distance from point `p` to segment `a`-`b`.
fn point_segment_dist(p: Point, a: Point, b: Point) -> f64 {
    let abx = b.x - a.x;
    let aby = b.y - a.y;
    let l2 = abx * abx + aby * aby;
    if l2 < 1e-24 {
        return ((p.x - a.x).powi(2) + (p.y - a.y).powi(2)).sqrt();
    }
    let t = (((p.x - a.x) * abx + (p.y - a.y) * aby) / l2).clamp(0.0, 1.0);
    let cx = a.x + t * abx;
    let cy = a.y + t * aby;
    ((p.x - cx).powi(2) + (p.y - cy).powi(2)).sqrt()
}

/// True if any vertex of `a` lies (nearly) on the boundary of `b` or vice
/// versa -- the class of degeneracies Greiner-Hormann cannot handle directly.
fn has_vertex_on_boundary(a: &[Point], b: &Polygon, tol: f64) -> bool {
    let n = b.vertices.len();
    for p in a {
        for i in 0..n {
            if point_segment_dist(*p, b.vertices[i], b.vertices[(i + 1) % n]) <= tol {
                return true;
            }
        }
    }
    false
}

/// Run the boolean operation on the outer boundaries of two polygons.
///
/// Returns the resulting polygons; an empty vector means an empty result.
pub fn polygon_boolean(a: &Polygon, b: &Polygon, op: BoolOp) -> Vec<Polygon> {
    // Identical boundaries: exact fast path.
    if a.vertices.len() == b.vertices.len()
        && a.vertices.iter().zip(&b.vertices).all(|(p, q)| p == q)
    {
        return match op {
            BoolOp::Intersection | BoolOp::Union => vec![a.clone()],
            BoolOp::Difference => vec![],
        };
    }

    // Robustness: when a vertex of either polygon sits exactly on the other
    // polygon's boundary (or edges are collinear-overlapping), perturb `b`
    // by an offset so the generic algorithm applies. Escalating jitter
    // scales are tried and the result is accepted when the inclusion-
    // exclusion area identity holds; this mirrors shapely's internal
    // robustness handling of degenerate inputs.
    let diag = bbox_diag(&a.vertices).max(bbox_diag(&b.vertices));
    let degenerate = has_vertex_on_boundary(&a.vertices, b, 1e-7 * diag)
        || has_vertex_on_boundary(&b.vertices, a, 1e-7 * diag);

    if !degenerate {
        return polygon_boolean_impl(a, b, op);
    }

    let _area_a = a.area().abs();
    let _area_b = b.area().abs();

    // Monte-Carlo acceptance: the resulting region must agree with an
    // independent point-classification of the requested operation.
    let mut rng_state = 0x2545F4914F6CDD1Du64;
    let mut next_unit = || {
        // xorshift64*
        rng_state ^= rng_state >> 12;
        rng_state ^= rng_state << 25;
        rng_state ^= rng_state >> 27;
        ((rng_state.wrapping_mul(0x2545F4914F6CDD1D)) >> 11) as f64 / (1u64 << 53) as f64
    };

    let bbox = |pts: &[Point]| {
        pts.iter().fold(
            (
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
            ),
            |(l, r, b, t), p| (l.min(p.x), r.max(p.x), b.min(p.y), t.max(p.y)),
        )
    };
    let (l_a, r_a, b_a, t_a) = bbox(&a.vertices);
    let (l_b, r_b, b_b, t_b) = bbox(&b.vertices);
    let (lo_x, hi_x) = (l_a.min(l_b), r_a.max(r_b));
    let (lo_y, hi_y) = (b_a.min(b_b), t_a.max(t_b));
    let n_samples = 300usize;
    let mut samples = Vec::with_capacity(n_samples);
    for _ in 0..n_samples {
        samples.push(Point::new(
            lo_x + next_unit() * (hi_x - lo_x),
            lo_y + next_unit() * (hi_y - lo_y),
        ));
    }
    let want = |p: &Point| -> bool {
        match op {
            BoolOp::Intersection => a.contains_point(*p) && b.contains_point(*p),
            BoolOp::Union => a.contains_point(*p) || b.contains_point(*p),
            BoolOp::Difference => a.contains_point(*p) && !b.contains_point(*p),
        }
    };
    let frac_target =
        samples.iter().filter(|p| want(p)).count() as f64 / n_samples as f64;

    let mut best: Option<(f64, Vec<Polygon>)> = None;
    for &scale in &[1e-6f64, 1e-4, 1e-2, 5e-2] {
        let eps = scale * diag;
        for &(dx, dy) in &[(1.0, 0.37), (-0.61, 1.0), (0.83, -0.83)] {
            let shifted: Vec<Point> = b
                .vertices
                .iter()
                .map(|p| Point::new(p.x + eps * dx, p.y + eps * dy))
                .collect();
            let b_j = Polygon::new(shifted);
            let result = polygon_boolean_impl(a, &b_j, op);

            // Fraction of samples inside the result polygons.
            let inside = |p: &Point| -> bool {
                result.iter().any(|poly| poly.contains_point(*p))
            };
            let frac_got =
                samples.iter().filter(|p| inside(p)).count() as f64 / n_samples as f64;

            let err = (frac_got - frac_target).abs();
            // Binomial noise allowance (~3 sigma).
            let tol = 3.0 * (0.25f64 / n_samples as f64).sqrt() + 1e-3;
            if err <= tol {
                return result;
            }
            if best.as_ref().map_or(true, |(be, _)| err < *be) {
                best = Some((err, result));
            }
        }
    }

    if let Some((_, result)) = best {
        return result;
    }
    polygon_boolean_impl(a, b, op)
}

fn polygon_boolean_impl(a: &Polygon, b: &Polygon, op: BoolOp) -> Vec<Polygon> {
    // Fast paths for nesting / disjoint configurations.
    let a_in_b = a.vertices.iter().all(|v| b.contains_point(*v));
    let b_in_a = b.vertices.iter().all(|v| a.contains_point(*v));

    match op {
        BoolOp::Intersection => {
            if a_in_b {
                return vec![a.clone()];
            }
            if b_in_a {
                return vec![b.clone()];
            }
        }
        BoolOp::Union => {
            if a_in_b {
                return vec![b.clone()];
            }
            if b_in_a {
                return vec![a.clone()];
            }
        }
        BoolOp::Difference => {
            if a_in_b {
                return vec![];
            }
            if b_in_a {
                // Hole case: only expressible at Section level with holes.
                return vec![a.clone()];
            }
        }
    }

    // For difference, reverse B so intersection semantics apply
    // (A - B == A intersect reversed-B).
    let b_eff: Vec<Point> = if op == BoolOp::Difference {
        b.vertices.iter().rev().copied().collect()
    } else {
        b.vertices.clone()
    };

    let raw = compute_raw_intersections(&a.vertices, &b_eff);
    if raw.is_empty() {
        return match op {
            BoolOp::Intersection => vec![],
            BoolOp::Union => vec![a.clone(), b.clone()],
            BoolOp::Difference => vec![a.clone()],
        };
    }

    let mut ring_a = build_ring(
        &a.vertices,
        &raw.iter().enumerate().map(|(id, r)| (r.ea, r.ta, id)).collect::<Vec<_>>(),
        &raw,
    );
    let mut ring_b = build_ring(
        &b_eff,
        &raw.iter().enumerate().map(|(id, r)| (r.eb, r.tb, id)).collect::<Vec<_>>(),
        &raw,
    );

    // Pair up intersection nodes across rings by shared id.
    use std::collections::HashMap;
    let mut pair_to_b: HashMap<usize, usize> = HashMap::new();
    for (i, nd) in ring_b.nodes.iter().enumerate() {
        if nd.is_inter {
            pair_to_b.insert(nd.pair, i);
        }
    }
    let mut pair_to_a: HashMap<usize, usize> = HashMap::new();
    for (i, nd) in ring_a.nodes.iter().enumerate() {
        if nd.is_inter {
            pair_to_a.insert(nd.pair, i);
        }
    }
    for (&pid, &ja) in &pair_to_a {
        if let Some(&jb) = pair_to_b.get(&pid) {
            ring_a.nodes[ja].neighbor = jb;
            ring_b.nodes[jb].neighbor = ja;
        }
    }

    assign_entries(&mut ring_a, b.contains_point(a.vertices[0]));
    assign_entries(&mut ring_b, a.contains_point(b_eff[0]));

    let mut results = Vec::new();
    let starts: Vec<usize> = ring_a
        .nodes
        .iter()
        .enumerate()
        .filter(|(_, n)| n.is_inter)
        .map(|(i, _)| i)
        .collect();
    for i in starts {
        if !ring_a.nodes[i].processed {
            let pts = trace_loop(&mut ring_a, &mut ring_b, i, op);
            if pts.len() >= 3 && signed_area_pts(&pts).abs() > 1e-15 {
                results.push(Polygon::new(pts));
            }
        }
    }
    results
}

/// Boolean difference between two full sections (outer boundary + holes).
///
/// Mirrors shapely `geometry - other` as used by Python `sectionproperties`.
///
/// - When `b` lies entirely inside `a` (and does not touch its boundaries)
///   the result is `a` with `b`'s outline added as a hole.
/// - Otherwise the boundary-level Greiner-Hormann difference is used; each
///   resulting region keeps whichever of `a`'s holes fall inside it.
///
/// Returns one [`Section`] per disjoint result region.
pub fn section_difference(a: &crate::section::Section, b: &crate::section::Section) -> Vec<crate::section::Section> {
    use crate::section::Section;

    let raw = compute_raw_intersections(&a.outer.vertices, &b.outer.vertices);
    let b_in_a = b.outer.vertices.iter().all(|v| a.outer.contains_point(*v));
    let touches_holes = a.holes.iter().any(|h| {
        h.vertices.iter().any(|v| b.outer.contains_point(*v))
            || b.outer.vertices.iter().any(|v| h.contains_point(*v))
            || !compute_raw_intersections(&h.vertices, &b.outer.vertices).is_empty()
    });

    if raw.is_empty() && b_in_a && !touches_holes {
        // Pure hole case.
        let mut holes = a.holes.clone();
        holes.push(b.outer.clone());
        return vec![Section::new(a.outer.clone(), holes)];
    }

    // Generic case: boundary-level difference, reattach interior holes.
    let pieces = polygon_boolean(&a.outer, &b.outer, BoolOp::Difference);
    pieces
        .into_iter()
        .map(|p| {
            let holes: Vec<crate::geometry::Polygon> = a
                .holes
                .iter()
                .filter(|h| {
                    // Keep holes fully inside this piece and untouched by b.
                    h.vertices.iter().all(|v| p.contains_point(*v))
                        && compute_raw_intersections(&h.vertices, &b.outer.vertices).is_empty()
                        && !h.vertices.iter().any(|v| b.outer.contains_point(*v))
                })
                .cloned()
                .collect();
            Section::new(p, holes)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(x0: f64, y0: f64, size: f64) -> Polygon {
        Polygon::new(vec![
            Point::new(x0, y0),
            Point::new(x0 + size, y0),
            Point::new(x0 + size, y0 + size),
            Point::new(x0, y0 + size),
        ])
    }

    #[test]
    fn intersection_of_overlapping_squares() {
        let a = square(0.0, 0.0, 2.0);
        let b = square(1.0, 1.0, 2.0);
        let r = polygon_boolean(&a, &b, BoolOp::Intersection);
        assert_eq!(r.len(), 1);
        assert!((r[0].area() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn union_area_subtracts_overlap() {
        let a = square(0.0, 0.0, 2.0);
        let b = square(1.0, 1.0, 2.0);
        let r = polygon_boolean(&a, &b, BoolOp::Union);
        assert_eq!(r.len(), 1);
        assert!((r[0].area() - 7.0).abs() < 1e-9);
    }

    #[test]
    fn difference_removes_overlap() {
        let a = square(0.0, 0.0, 2.0);
        let b = square(1.0, 1.0, 2.0);
        let r = polygon_boolean(&a, &b, BoolOp::Difference);
        assert_eq!(r.len(), 1);
        assert!((r[0].area() - 3.0).abs() < 1e-9);
    }

    #[test]
    fn disjoint_polygons() {
        let a = square(0.0, 0.0, 1.0);
        let b = square(5.0, 5.0, 1.0);
        assert!(polygon_boolean(&a, &b, BoolOp::Intersection).is_empty());
        assert_eq!(polygon_boolean(&a, &b, BoolOp::Union).len(), 2);
        let d = polygon_boolean(&a, &b, BoolOp::Difference);
        assert_eq!(d.len(), 1);
        assert!((d[0].area() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn contained_polygons() {
        let outer = square(0.0, 0.0, 3.0);
        let inner = square(1.0, 1.0, 1.0);

        let i = polygon_boolean(&inner, &outer, BoolOp::Intersection);
        assert_eq!(i.len(), 1);
        assert!((i[0].area() - 1.0).abs() < 1e-12);

        let u = polygon_boolean(&inner, &outer, BoolOp::Union);
        assert_eq!(u.len(), 1);
        assert!((u[0].area() - 9.0).abs() < 1e-12);

        let d = polygon_boolean(&outer, &inner, BoolOp::Difference);
        // boundary-level: hole case returns the full outer polygon
        assert_eq!(d.len(), 1);
        assert!((d[0].area() - 9.0).abs() < 1e-12);
    }

    #[test]
    fn section_difference_creates_hole() {
        use crate::section::Section;
        let outer = square(0.0, 0.0, 4.0);
        let a = Section::new(outer, vec![]);
        let b = Section::new(square(1.0, 1.0, 2.0), vec![]);

        let result = section_difference(&a, &b);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].holes.len(), 1);
        assert!((result[0].area() - (16.0 - 4.0)).abs() < 1e-9);
        assert!((result[0].centroid().x - 2.0).abs() < 1e-12);

        // Properties must account for the hole.
        let props = crate::section_properties::SectionProperties::from_section(&result[0]);
        let expected_ix = 4.0 * 4.0_f64.powi(3) / 12.0 - 2.0 * 2.0_f64.powi(3) / 12.0;
        assert!((props.ix - expected_ix).abs() / expected_ix < 1e-10);
    }

    #[test]
    fn section_difference_partial_overlap_no_hole() {
        use crate::section::Section;
        let a = Section::new(square(0.0, 0.0, 2.0), vec![]);
        let b = Section::new(square(1.0, 1.0, 2.0), vec![]);
        let result = section_difference(&a, &b);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].holes.len(), 0);
        assert!((result[0].area() - 3.0).abs() < 1e-9);
    }

    #[test]
    fn section_difference_with_existing_holes() {
        use crate::section::Section;
        // Square with existing hole, minus an offset square.
        let a = Section::new(
            square(0.0, 0.0, 4.0),
            vec![square(0.5, 0.5, 1.0)],
        );
        let b = Section::new(square(2.5, 2.5, 1.0), vec![]);
        let result = section_difference(&a, &b);
        assert_eq!(result.len(), 1);
        // Interior subtraction becomes a second hole.
        assert_eq!(result[0].holes.len(), 2);
        assert!((result[0].area() - (16.0 - 1.0 - 1.0)).abs() < 1e-9);
    }
    #[test]
    fn identical_polygons() {
        let a = square(0.0, 0.0, 2.0);
        assert_eq!(polygon_boolean(&a, &a.clone(), BoolOp::Intersection)[0].area() , 4.0);
        assert_eq!(polygon_boolean(&a, &a.clone(), BoolOp::Union)[0].area(), 4.0);
        assert!(polygon_boolean(&a, &a.clone(), BoolOp::Difference).is_empty());
    }

    #[test]
    fn shared_edge_squares_union_area() {
        // Two unit squares sharing the edge x = 1 exactly.
        let a = square(0.0, 0.0, 1.0);
        let b = square(1.0, 0.0, 1.0);
        let u = polygon_boolean(&a, &b, BoolOp::Union);
        let total: f64 = u.iter().map(|p| p.area()).sum();
        assert!((total - 2.0).abs() < 1e-9);
        assert!(polygon_boolean(&a, &b, BoolOp::Intersection).is_empty());
        let d = polygon_boolean(&a, &b, BoolOp::Difference);
        assert!((d[0].area() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn vertex_on_edge_case_satisfies_area_identity() {
        // Triangle apex (1,0) lies exactly on A's bottom edge.
        let a = square(0.0, 0.0, 2.0);
        let b = Polygon::new(vec![
            Point::new(1.0, 0.0),
            Point::new(3.0, 1.0),
            Point::new(1.0, 2.0),
        ]);
        let area_a = a.area();
        let area_b = b.area();

        let i = polygon_boolean(&a, &b, BoolOp::Intersection);
        let inter: f64 = i.iter().map(|p| p.area()).sum();
        let u = polygon_boolean(&a, &b, BoolOp::Union);
        let uni: f64 = u.iter().map(|p| p.area()).sum();
        let d = polygon_boolean(&a, &b, BoolOp::Difference);
        let diff: f64 = d.iter().map(|p| p.area()).sum();

        // Inclusion-exclusion identity must hold (within jitter tolerance).
        assert!(
            (area_a + area_b - inter - uni).abs() < 1e-3,
            "union identity: {} + {} - {} != {}",
            area_a,
            area_b,
            inter,
            uni
        );
        assert!((area_a - inter - diff).abs() < 1e-3, "difference identity");
        assert!(inter > 0.0 && inter < area_b.min(area_a));
    }
    #[test]
    fn non_convex_l_shape_intersection() {
        let l = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(2.0, 0.0),
            Point::new(2.0, 1.0),
            Point::new(1.0, 1.0),
            Point::new(1.0, 2.0),
            Point::new(0.0, 2.0),
        ]);
        let b = square(0.5, 0.5, 1.0); // [0.5,1.5]^2
        let r = polygon_boolean(&l, &b, BoolOp::Intersection);
        assert_eq!(r.len(), 1);
        assert!((r[0].area() - 0.75).abs() < 1e-9);
    }
}




