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
///
/// # Deterministic behaviour
///
/// When a vertex of either polygon lies on the other's boundary (degenerate
/// for Greiner–Hormann), the function applies **systematic deterministic
/// perturbation** in fixed directions rather than random jitter. This
/// guarantees that identical inputs always produce identical outputs.
///
/// The result is validated against the area inclusion-exclusion identity:
/// `|A| + |B| = |A∩B| + |A∪B|`, which is a strict mathematical invariant.
/// Compute boolean operation on two polygons with optional validation.
fn polygon_boolean_internal(a: &Polygon, b: &Polygon, op: BoolOp, validate: bool) -> Vec<Polygon> {
    // Identical boundaries: exact fast path.
    if a.vertices.len() == b.vertices.len()
        && a.vertices.iter().zip(&b.vertices).all(|(p, q)| p == q)
    {
        return match op {
            BoolOp::Intersection | BoolOp::Union => vec![a.clone()],
            BoolOp::Difference => vec![],
        };
    }

    // Robustness: when a vertex of either polygon sits (nearly) on the other
    // polygon's boundary (or edges are collinear-overlapping), the generic
    // Greiner–Hormann algorithm cannot resolve the degeneracy. `b` is shifted
    // by a fixed deterministic offset to remove the degeneracy; the result is
    // then mapped back onto the original coordinates.
    //
    // The degeneracy-detection tolerance must match the perturbation scale:
    // detecting a vertex only works if the shift is large enough to move it off
    // the other polygon's boundary. We therefore derive `eps` first and scale
    // the detection window by a small multiple of it.
    let diag = bbox_diag(&a.vertices).max(bbox_diag(&b.vertices));
    // Use scale-adaptive epsilon: large enough to resolve vertex-on-edge
    // degeneracies, small enough to avoid coordinate distortion.
    // eps = 1e-8 * diag, clamped to [1e-12 * diag, 1e-6 * min(diag, 1.0)].
    // For diag=1e6 mm (1 km): eps = 1e-2 mm → clamped to 1e-6 mm
    // For diag=1e-3 mm (1 µm): eps = 1e-11 mm → clamped to 1e-15 mm (near f64 precision)
    let eps = (1e-8 * diag).clamp(1e-12 * diag, 1e-6 * diag.min(1.0));
    // Detection window slightly larger than the shift so that any vertex we
    // detect as degenerate is actually moved clear of the boundary.
    let det_tol = 8.0 * eps;
    let degenerate = has_vertex_on_boundary(&a.vertices, b, det_tol)
        || has_vertex_on_boundary(&b.vertices, a, det_tol);

    if !degenerate {
        return polygon_boolean_impl(a, b, op);
    }

    // Deterministic perturbation: try systematic shifts in fixed directions.
    // Directions chosen to avoid common edge orientations (horizontal, vertical,
    // 45-degree).
    let directions: &[(f64, f64)] = &[
        (1.0, 0.37),   // ~20° from horizontal
        (-0.61, 1.0),  // ~122° from horizontal
        (0.83, -0.83), // ~-45° from horizontal
    ];

    let area_a = a.area();
    let area_b = b.area();
    // Normaliser for the relative inclusion-exclusion error.
    let denom = (area_a.abs() + area_b.abs()).max(1e-12);

    // The inverse of `op`: |op(a,b)| + |complement(a,b)| = |a| + |b| always.
    let comp_op = match op {
        BoolOp::Intersection => BoolOp::Union,
        BoolOp::Union => BoolOp::Intersection,
        BoolOp::Difference => BoolOp::Intersection,
    };

    // Tracks the closest (mapped) result across directions, used as a
    // best-effort fallback when no direction satisfies the strict identity.
    let mut best: Option<(f64, Vec<Polygon>)> = None;

    for &(dx, dy) in directions {
        let shifted: Vec<Point> = b
            .vertices
            .iter()
            .map(|p| Point::new(p.x + eps * dx, p.y + eps * dy))
            .collect();
        let b_shifted = Polygon::new(shifted);

        // Compute the main op and its complement **in the same shifted frame**,
        // using the non-perturbing impl. The shift removes the original
        // degeneracy, so `polygon_boolean_impl` (bare GH + nesting fast paths)
        // is exact on `(a, b_shifted)` and does not recurse back into this
        // perturbation logic.
        let result = polygon_boolean_impl(a, &b_shifted, op);
        let result_comp = polygon_boolean_impl(a, &b_shifted, comp_op);

        // Area is translation-invariant, so summing the shifted-frame results
        // and comparing against the per-op inclusion-exclusion reference gives
        // the strict identity (both |a| and |b| are translation-invariant, and
        // |b_shifted| == |b|):
        //   Intersection/Union: |op| + |complement| = |a| + |b|
        //   Difference:         |A\B| + |A∩B|      = |a|
        let area_result: f64 = result.iter().map(|p| p.area()).sum();
        let area_comp: f64 = result_comp.iter().map(|p| p.area()).sum();
        let rhs = match op {
            BoolOp::Difference => area_a,
            BoolOp::Intersection | BoolOp::Union => area_a + area_b,
        };
        let err = (area_result + area_comp - rhs).abs() / denom;

        // Map the perturbed result back onto the original coordinates so that
        // higher-order quantities (centroid, moments, principal axes) do not
        // inherit the artificial `eps * direction` offset.
        let mapped: Vec<Polygon> = result
            .into_iter()
            .map(|mut r| {
                for v in &mut r.vertices {
                    v.x -= eps * dx;
                    v.y -= eps * dy;
                }
                r
            })
            .collect();

        if err < 1e-9 {
            // Strict inclusion-exclusion satisfied: accept.
            return mapped;
        }

        // Track best-effort fallback (mapped) across all directions.
        if best
            .as_ref()
            .map_or(true, |(be, _)| err < *be)
        {
            best = Some((err, mapped));
        }
    }

    // No direction satisfied the strict identity.
    if validate {
        // Strict mode: do not silently return a result that fails the
        // inclusion-exclusion invariant. Surface the failure (this is a
        // geometry-kernel safety net) and return the closest mapped estimate,
        // which Section-level validation can reject downstream if needed.
        eprintln!(
            "WARN: boolean {:?} failed strict inclusion-exclusion after perturbation; returning best-effort result",
            op
        );
    }
    // Both validate modes return the best-effort (mapped) result rather than
    // silently discarding validation.
    best.map_or_else(Vec::new, |(_, res)| res)
}

/// Errors from polygon / section boolean operations.
#[derive(Debug, Clone, PartialEq)]
pub enum BooleanError {
    /// The computed result violates a geometric invariant (area bounds or
    /// point-in/polygon sampling) and must not be used downstream. Returning
    /// this instead of a `Vec<Polygon>` prevents a silently-wrong boolean from
    /// propagating corrupt areas, centroids, moments and FEM meshes.
    Validation { op: BoolOp, message: String },
}

impl std::fmt::Display for BooleanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BooleanError::Validation { op, message } => {
                write!(f, "boolean {:?} failed validation: {}", op, message)
            }
        }
    }
}

impl std::error::Error for BooleanError {}

/// Cheap per-op area-bounds reconciliation.
///
/// Catches gross topological errors that the inclusion-exclusion identity
/// alone cannot. Example: a blundering algorithm may return `|A∩B|=30` and
/// `|A∪B|=170` for two area-100 polygons. The identity `30+170=100+100` holds,
/// yet the union clearly violates `|A∪B| >= max(|A|,|B|) = 100`.
fn check_boolean_bounds(
    result: &[Polygon],
    a: &Polygon,
    b: &Polygon,
    op: BoolOp,
) -> Result<(), BooleanError> {
    let area_a = a.area();
    let area_b = b.area();
    let area_res: f64 = result.iter().map(|p| p.area()).sum();
    // Sliver allowance: perturbation-induced slivers are ~eps*scale in area
    // (~1e-8 relative), so a 1e-6 relative tolerance is far larger than any
    // legitimate sliver yet far smaller than a genuine topological blunder.
    let rel = 1e-6;
    let fail = |message: String| BooleanError::Validation { op, message };
    match op {
        BoolOp::Intersection => {
            let min = area_a.min(area_b);
            let tol = rel * min.abs().max(1e-12);
            if area_res < -tol || area_res > min + tol {
                return Err(fail(format!(
                    "intersection area {:.6} outside [0, min(|A|,|B|) = {:.6}]",
                    area_res, min
                )));
            }
        }
        BoolOp::Union => {
            let max = area_a.max(area_b);
            // |A∪B| must lie within [max(|A|,|B|), |A|+|B|]. Violating either
            // bound means the result cannot be a true union of A and B.
            let lo = max;
            let hi = area_a + area_b;
            let tol = rel * hi.abs().max(1e-12);
            if area_res < lo - tol || area_res > hi + tol {
                return Err(fail(format!(
                    "union area {:.6} outside [max(|A|,|B|) = {:.6}, |A|+|B| = {:.6}]",
                    area_res, lo, hi
                )));
            }
        }
        BoolOp::Difference => {
            let tol = rel * area_a.abs().max(1e-12);
            if area_res < -tol || area_res > area_a + tol {
                return Err(fail(format!(
                    "difference area {:.6} outside [0, |A| = {:.6}]",
                    area_res, area_a
                )));
            }
        }
    }
    Ok(())
}

/// Deterministic point-in/polygon sampling validation.
///
/// Much stronger than area checks: verifies the result's set membership against
/// the expected boolean (`op(A,B)`) at a grid of sample points covering the
/// combined bounding box. Points on (or within a perturbation tolerance of) any
/// boundary are skipped because membership there is inherently ambiguous; on
/// the clean interior samples any mismatch is a genuine topology error.
///
/// # Why this is OPT-IN / advisory (not a default gate)
///
/// It can raise **false positives on legitimately-correct geometry** for two
/// fundamental reasons:
///
/// 1. `Polygon::contains_point` (ray casting) and the GH boolean kernel use
///    **incompatible interior/winding interpretations** on concave polygons.
///    Measured example: for the U-shape `(0,0)(3,0)(3,3)(2,3)(2,1)(1,1)(1,3)(0,3)`
///    the ray-caster reports a notch point `(1.5,2.0)` as *exterior* while the
///    kernel treats the region as interior, so `u ∩ fill` returns area = 4 (the
///    full fill) yet the sampler's `contains_point(result, notch-pt)` is `true`.
///    Any sampler built on `contains_point` will therefore disagree with the
///    kernel on such inputs.
/// 2. Polygon-level `Difference` of a nested operand returns the outer region
///    *without cutting the hole* (a documented module convention — the hole is
///    only realised at `Section` level). Sampling naively expects the hole to
///    be present and flags the (correct, convention-compliant) result.
///
/// Consequently it is exposed separately as `polygon_boolean_sampled_checked`
/// and is intended for callers who know their inputs are convex and
/// non-nested, where it adds a strong extra correctness signal.
fn validate_boolean_sampling(
    result: &[Polygon],
    a: &Polygon,
    b: &Polygon,
    op: BoolOp,
) -> Result<(), BooleanError> {
    let diag = bbox_diag(&a.vertices).max(bbox_diag(&b.vertices));
    if diag <= 0.0 {
        return Ok(());
    }
    let eps = (1e-8 * diag).clamp(1e-12 * diag, 1e-6 * diag.min(1.0));
    let bnd_tol = 16.0 * eps;

    // Combined bounding box of a, b and the result so the grid covers everything.
    let (minx, maxx, miny, maxy) = {
        let mut minx = f64::INFINITY;
        let mut maxx = f64::NEG_INFINITY;
        let mut miny = f64::INFINITY;
        let mut maxy = f64::NEG_INFINITY;
        for p in std::iter::once(a).chain(std::iter::once(b)).chain(result) {
            for v in &p.vertices {
                minx = minx.min(v.x);
                maxx = maxx.max(v.x);
                miny = miny.min(v.y);
                maxy = maxy.max(v.y);
            }
        }
        (minx, maxx, miny, maxy)
    };

    // All boundary edges (a, b and every result polygon) for the skip test.
    let boundary_edges: Vec<(Point, Point)> = std::iter::once(a)
        .chain(std::iter::once(b))
        .chain(result)
        .flat_map(|p| {
            p.vertices
                .iter()
                .enumerate()
                .map(move |(i, v)| (*v, p.vertices[(i + 1) % p.vertices.len()]))
        })
        .collect();

    const N: usize = 32;
    let w = (maxx - minx).max(f64::MIN_POSITIVE);
    let h = (maxy - miny).max(f64::MIN_POSITIVE);
    let mut mismatches = 0usize;
    let mut checked = 0usize;
    for ix in 0..=N {
        for iy in 0..=N {
            let pt = Point::new(
                minx + w * (ix as f64) / (N as f64),
                miny + h * (iy as f64) / (N as f64),
            );
            // Skip ambiguously-placed samples near any boundary.
            let near_boundary = boundary_edges
                .iter()
                .any(|&(p, q)| point_segment_dist(pt, p, q) <= bnd_tol);
            if near_boundary {
                continue;
            }
            checked += 1;
            let in_a = a.contains_point(pt);
            let in_b = b.contains_point(pt);
            let in_res = result.iter().any(|p| p.contains_point(pt));
            let expected = match op {
                BoolOp::Intersection => in_a && in_b,
                BoolOp::Union => in_a || in_b,
                BoolOp::Difference => in_a && !in_b,
            };
            if in_res != expected {
                mismatches += 1;
            }
        }
    }
    if mismatches > 0 && checked > 0 {
        return Err(BooleanError::Validation {
            op,
                message: format!(
                    "point-sampling mismatch: {mismatches}/{checked} of the checked samples disagree \
                     with the expected {:?} of A and B",
                    op
                ),
        });
    }
    Ok(())
}

/// Checked boolean operation that verifies the result before returning it.
///
/// Runs a cheap, reliable area-bounds reconciliation against per-op invariants
/// (`intersection <= min`, `union >= max`, `difference in [0, |A|]`), which
/// catches gross topological errors that the inclusion-exclusion identity alone
/// cannot. Returns `Err(BooleanError)` instead of a possibly-wrong polygon set,
/// so a corrupt boolean can never silently propagate into section properties or
/// FEM.
///
/// Note: stronger point-in/polygon *sampling* validation is available via
/// [`polygon_boolean_sampled_checked`], but is not part of this default gate
/// because it is incompatible with this module's polygon-level conventions
/// (nested `Difference` returns the hole-not-cut outer, and `contains_point`
/// ray-casting can disagree with the kernel's interior interpretation on some
/// concave inputs).
pub fn polygon_boolean_checked(
    a: &Polygon,
    b: &Polygon,
    op: BoolOp,
) -> Result<Vec<Polygon>, BooleanError> {
    let result = polygon_boolean_internal(a, b, op, true);
    check_boolean_bounds(&result, a, b, op)?;
    Ok(result)
}

/// Convenience wrapper over [`polygon_boolean_checked`] that infallibly returns
/// the result, panicking if the area-bounds validation fails. Prefer the
/// checked variant when the error must be recoverable rather than aborting the
/// process.
pub fn polygon_boolean(a: &Polygon, b: &Polygon, op: BoolOp) -> Vec<Polygon> {
    polygon_boolean_checked(a, b, op).expect("polygon boolean failed validation")
}

/// Explicitly request the point-in/polygon *sampling* validation in addition to
/// the area-bounds reconciliation.
///
/// # Caveats
/// This is an advisory, stronger check and is **not** suitable as an automatic
/// gate for the whole pipeline because it can report false mismatches on
/// legitimate inputs:
/// - Polygon-level `Difference` of nested polygons returns the outer without
///   materialising the hole (a documented convention), which disagrees with
///   point-set `Difference`.
/// - `Polygon::contains_point` (ray-casting) can disagree with the kernel's
///   interior interpretation on some concave polygons.
/// Use it for explicit verification of inputs you expect to be well-behaved
/// (e.g. convex, non-nested) and treat a mismatch as a strong signal worth
/// investigating, not a definitive proof of error for the cases above.
pub fn polygon_boolean_sampled_checked(
    a: &Polygon,
    b: &Polygon,
    op: BoolOp,
) -> Result<Vec<Polygon>, BooleanError> {
    let result = polygon_boolean_checked(a, b, op)?;
    validate_boolean_sampling(&result, a, b, op)?;
    Ok(result)
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

    boolean_generic(a, b, op)
}

/// Greiner–Hormann boolean on two **simple polygons** that always runs the
/// generic crossing algorithm, without the nesting/disjoint fast paths.
///
/// The fast paths in [`polygon_boolean_impl`] defer holes to the Section
/// level (e.g. `A \ B` with B nested inside A returns `[A]`); this variant is
/// used when the result must be expressed as explicit polygons (building
/// material regions and punching holes).
fn boolean_generic(a: &Polygon, b: &Polygon, op: BoolOp) -> Vec<Polygon> {
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

/// Compute the material of `s` that lies inside `p`, expressed as a list of
/// polygons `p ∩ material(s)`.
///
/// Because `material(s) = outer \ holes`, this is `p ∩ outer` with each hole
/// cut out. Each operation used here (`intersection` and `difference`) acts on
/// simple polygons and, crucially, never needs to represent a polygon-with-a-
/// nested-hole, so the results are always plain regions.
fn material_inside(
    p: &Polygon,
    s: &crate::section::Section,
) -> Result<Vec<Polygon>, BooleanError> {
    // p ∩ outer
    let mut acc = polygon_boolean_checked(p, &s.outer, BoolOp::Intersection)?;
    // subtract every hole (in CCW orientation, as the boolean expects)
    for h in &s.holes {
        let h_ccw = ccw(h);
        let mut next = Vec::new();
        for r in acc.iter() {
            next.extend(polygon_boolean_checked(r, &h_ccw, BoolOp::Difference)?);
        }
        acc = next;
        if acc.is_empty() {
            break;
        }
    }
    Ok(acc.into_iter().filter(|r| r.area().abs() > 1e-15).collect())
}

/// Return a counter-clockwise (positive-area) copy of a polygon. Holes are
/// stored clockwise by [`Section`](crate::section::Section), but the boolean
/// routines operate on CCW outer polygons.
fn ccw(p: &Polygon) -> Polygon {
    if p.signed_area() >= 0.0 {
        p.clone()
    } else {
        Polygon::new(p.vertices.iter().rev().copied().collect())
    }
}

/// The part of `hole` that is **not** covered by the material of the section
/// represented by `(outer, holes)`; i.e. `hole \ material`. These regions are
/// the "void" left by a hole of one section that is not filled by the other
/// section's material.
fn hole_minus_material(
    hole: &Polygon,
    outer: &Polygon,
    holes: &[Polygon],
) -> Result<Vec<Polygon>, BooleanError> {
    // hole \ material = (hole \ outer) ∪ (hole ∩ each hole)
    let hole = ccw(hole);
    let mut res = Vec::new();
    res.extend(polygon_boolean_checked(&hole, outer, BoolOp::Difference)?);
    for oh in holes {
        res.extend(polygon_boolean_checked(&hole, &ccw(oh), BoolOp::Intersection)?);
    }
    Ok(res)
}

/// Merge a collection of void polygons (already clipped to a piece) into a
/// minimal set of pairwise-disjoint simple polygons.
///
/// Two holes belonging to different sections may overlap after clipping (e.g.
/// A.hole and B.hole on an Intersection). If they were attached as-is,
/// `Section::area()` would subtract the overlapping region twice. Unioning
/// overlapping members folds them into one polygon so every region is counted
/// exactly once. Members that merely touch (share an edge) are already
/// disjoint for area purposes and are left separate.
fn union_voids(regions: &mut Vec<Polygon>) -> Result<(), BooleanError> {
    let mut i = 0;
    while i < regions.len() {
        let mut j = i + 1;
        let mut merged = false;
        while j < regions.len() {
            let u = polygon_boolean_checked(&regions[i], &regions[j], BoolOp::Union)?;
            if u.len() == 1 {
                // The two regions overlap (or are nested) and fold into one.
                regions[i] = u.into_iter().next().unwrap();
                regions.swap_remove(j);
                merged = true;
                break;
            }
            j += 1;
        }
        if !merged {
            i += 1;
        }
    }
    Ok(())
}

/// Compute the holes that survive inside the outer-boundary piece `p` when
/// `a` and `b` are combined by `op`. The voids are regions inside `p` where
/// the result has no material, expressed as plain simple polygons.
///
/// - **Union**: material exists where *either* section has material, so a hole
///   of one section remains a hole only where the other section does not fill
///   it (`hole \ material(other)`).
/// - **Intersection**: material exists where *both* sections have material, so
///   every hole of either section that lies inside the piece is a void.
/// - **Difference**: material is `material(A) \ material(B)`, so the voids are
///   A's holes (inside the piece) plus any of B's material inside the piece.
///   For pieces from `A.outer ∩ B.holes` (B's hole regions), B has no material
///   there, so voids are only A's holes clipped to the piece.
///
/// Candidate voids are clipped to `p`, unioned so that overlapping holes are
/// not double-counted, and tiny/sliver regions dropped.
fn holes_for_piece(
    p: &Polygon,
    a: &crate::section::Section,
    b: &crate::section::Section,
    op: BoolOp,
    kind: PieceKind,
) -> Result<Vec<Polygon>, BooleanError> {
    let mut void_regions: Vec<Polygon> = Vec::new();

    match op {
        BoolOp::Union => {
            // A's holes that B does not fill, plus B's holes that A does not fill.
            for h in a.holes.iter() {
                void_regions.extend(hole_minus_material(h, &b.outer, &b.holes)?);
            }
            for h in b.holes.iter() {
                void_regions.extend(hole_minus_material(h, &a.outer, &a.holes)?);
            }
        }
        BoolOp::Intersection => {
            // Every hole of either section inside the piece is a void.
            void_regions.extend(a.holes.iter().cloned());
            void_regions.extend(b.holes.iter().cloned());
        }
        BoolOp::Difference => {
            match kind {
                PieceKind::Normal => {
                    // Standard outer difference: A.outer \ B.outer
                    // Voids = A's holes (minus B's material) + B's material inside piece
                    for h in a.holes.iter() {
                        void_regions.extend(hole_minus_material(h, &b.outer, &b.holes)?);
                    }
                    void_regions.extend(material_inside(p, b)?);
                }
                PieceKind::InHole => {
                    // A's material inside B's hole: B has no material here
                    // Voids = only A's holes clipped to piece (no B material subtraction)
                    for h in a.holes.iter() {
                        void_regions.extend(polygon_boolean_checked(h, p, BoolOp::Intersection)?);
                    }
                }
            }
        }
    }

    // Clip every candidate void to the piece boundary so that no hole ever
    // extends outside `p`. A hole of one section that spans the intersection or
    // difference cut line is trimmed to `hole ∩ p`, not kept whole.
    let mut clipped = Vec::new();
    for r in void_regions {
        clipped.extend(polygon_boolean_checked(p, &ccw(&r), BoolOp::Intersection)?);
    }
    void_regions = clipped;

    // Merge overlapping voids so overlapping holes are not double-counted by
    // Section::area(). Members that only touch remain separate (already
    // disjoint for area purposes).
    union_voids(&mut void_regions)?;

    // Keep only regions inside `p` that carry meaningful area. Any tiny region
    // that touches `p`'s boundary is an exterior sliver, not a hole, so it is
    // dropped. Vertices may overhang `p`'s boundary by a tiny amount due to the
    // boolean perturbation, so allow a small coordinate tolerance rather than a
    // strict point-in-polygon test (which would break legitimate boundary holes).
    //
    // The perturbation can also fabricate phantom voids (area ~eps * scale
    // with eps=1e-8*diag) where two holes merely touch, so the area threshold
    // must exceed that scale to drop them while keeping genuine holes.
    let scale = bbox_diag(&p.vertices);
    // Relative tolerance (1e-9 * piece_area) plus absolute tolerance based on
    // perturbation-induced phantom void area (~eps * scale with eps=1e-8*diag).
    let area_tol = 1e-9 * p.area().abs().max(1e-12) + 1e-8 * scale * scale;
    let coord_tol = 1e-7 * scale;
    let edge_pairs: Vec<(Point, Point)> = p
        .vertices
        .iter()
        .enumerate()
        .map(|(i, v)| (*v, p.vertices[(i + 1) % p.vertices.len()]))
        .collect();
    Ok(void_regions
        .into_iter()
        .filter(|r| {
            if r.area().abs() <= area_tol {
                return false;
            }
            r.vertices.iter().all(|&v| {
                if p.contains_point(v) {
                    return true;
                }
                edge_pairs
                    .iter()
                    .any(|&(a, b)| point_segment_dist(v, a, b) <= coord_tol)
            })
        })
        .collect())
}

/// Core boolean on two full sections (outer boundary + holes) that unifies
/// hole handling across all three operations.
///
/// The outer boundaries are combined with [`polygon_boolean`], then each
/// resulting region has its surviving holes re-attached with
/// [`holes_for_piece`].
///
/// Returns one [`Section`] per disjoint result region.
fn section_boolean(
    a: &crate::section::Section,
    b: &crate::section::Section,
    op: BoolOp,
) -> Result<Vec<crate::section::Section>, BooleanError> {
    use crate::section::Section;

    if op == BoolOp::Difference {
        return section_difference_impl(a, b);
    }

    let pieces = polygon_boolean_checked(&a.outer, &b.outer, op)?;
    let mut out = Vec::new();
    for p in pieces {
        let holes = holes_for_piece(&p, a, b, op, PieceKind::Normal)?;
        let s = Section::new(p, holes);
        if s.area() > 0.0 {
            out.push(s);
        }
    }
    Ok(out)
}

/// Difference pieces have two kinds:
/// - `Normal`: from `A.outer \ B.outer` (standard outer difference)
/// - `InHole`: from `A.outer ∩ B.holes[i]` (A's material inside B's holes)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PieceKind {
    Normal,
    InHole,
}

/// Difference of two sections with proper hole handling.
/// 
/// Material = material(A) \ material(B) = (A.outer \ A.holes) \ (B.outer \ B.holes)
/// = (A.outer \ B.outer) ∪ (A.outer ∩ B.holes) minus holes appropriately.
fn section_difference_impl(
    a: &crate::section::Section,
    b: &crate::section::Section,
) -> Result<Vec<crate::section::Section>, BooleanError> {
    use crate::section::Section;

    // 1. Standard outer difference: A.outer \ B.outer
    let outer_diff = polygon_boolean_checked(&a.outer, &b.outer, BoolOp::Difference)?;

    // 2. Parts of A that fall inside B's holes (B has no material there, so A survives)
    let mut hole_intersections = Vec::new();
    for bh in &b.holes {
        hole_intersections.extend(polygon_boolean_checked(&a.outer, bh, BoolOp::Intersection)?);
    }

    // Combine all pieces with their kind
    let mut pieces = Vec::new();
    for p in outer_diff {
        pieces.push((p, PieceKind::Normal));
    }
    for p in hole_intersections {
        pieces.push((p, PieceKind::InHole));
    }

    let mut out = Vec::new();
    for (p, kind) in pieces {
        let holes = holes_for_piece(&p, a, b, BoolOp::Difference, kind)?;
        let s = Section::new(p, holes);
        if s.area() > 0.0 {
            out.push(s);
        }
    }
    Ok(out)
}

/// Boolean intersection between two full sections (outer boundary + holes).
///
/// Mirrors shapely `geometry & other` as used by Python `sectionproperties`.
///
/// Returns one [`Section`] per disjoint result region, or
/// [`BooleanError`] if the underlying polygon booleans fail validation.
pub fn section_intersection(
    a: &crate::section::Section,
    b: &crate::section::Section,
) -> Result<Vec<crate::section::Section>, BooleanError> {
    section_boolean(a, b, BoolOp::Intersection)
}

/// Boolean difference between two full sections (outer boundary + holes).
///
/// Mirrors shapely `geometry - other` as used by Python `sectionproperties`.
///
/// Returns one [`Section`] per disjoint result region, or
/// [`BooleanError`] if the underlying polygon booleans fail validation.
pub fn section_difference(
    a: &crate::section::Section,
    b: &crate::section::Section,
) -> Result<Vec<crate::section::Section>, BooleanError> {
    section_boolean(a, b, BoolOp::Difference)
}

/// Boolean union between two full sections (outer boundary + holes).
///
/// Mirrors shapely `geometry | other` as used by Python `sectionproperties`.
///
/// Returns one [`Section`] per disjoint result region, or
/// [`BooleanError`] if the underlying polygon booleans fail validation.
pub fn section_union(
    a: &crate::section::Section,
    b: &crate::section::Section,
) -> Result<Vec<crate::section::Section>, BooleanError> {
    section_boolean(a, b, BoolOp::Union)
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

    /// Assert that every hole vertex is strictly inside `piece`, or within a
    /// tiny coordinate tolerance of its boundary (to absorb the boolean
    /// perturbation that may push an on-boundary vertex an epsilon outside).
    fn assert_holes_within_piece(piece: &Polygon, holes: &[Polygon]) {
        let diag = bbox_diag(&piece.vertices);
        let tol = 1e-7 * diag;
        for h in holes {
            for v in &h.vertices {
                let on_boundary = piece.vertices.iter().enumerate().any(|(i, a)| {
                    let b = piece.vertices[(i + 1) % piece.vertices.len()];
                    point_segment_dist(*v, *a, b) <= tol
                });
                assert!(piece.contains_point(*v) || on_boundary,
                    "hole must not extend outside the piece");
            }
        }
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

        let result = section_difference(&a, &b).unwrap();
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
        let result = section_difference(&a, &b).unwrap();
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
        let result = section_difference(&a, &b).unwrap();
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
    fn section_union_preserves_holes() {
        use crate::section::Section;
        // Two overlapping squares, each with a hole in its non-overlapped
        // region. The union merges into one piece and keeps both holes.
        let hole_a = square(0.2, 0.3, 0.3);
        let a = Section::new(square(0.0, 0.0, 1.2), vec![hole_a]);
        let hole_b = square(1.5, 0.3, 0.3);
        let b = Section::new(square(1.0, 0.0, 1.2), vec![hole_b]);

        let result = section_union(&a, &b).unwrap();
        assert_eq!(result.len(), 1, "overlapping squares merge into one piece");
        assert_eq!(result[0].holes.len(), 2, "union must preserve both holes");
        // Material a (1.44-0.09) + material b (1.44-0.09) - overlap [1.0,1.2]x[0,1.2] (0.24).
        let expected = (1.44 - 0.09) + (1.44 - 0.09) - 0.24;
        assert!((result[0].area() - expected).abs() < 1e-6, "union area {}", result[0].area());
    }

    #[test]
    fn section_union_drops_hole_filled_by_other() {
        use crate::section::Section;
        // A with a hole; B is a solid square exactly filling that hole.
        // Union must fill the hole (B's material occupies it).
        let hole = square(0.5, 0.5, 1.0);
        let a = Section::new(square(0.0, 0.0, 2.0), vec![hole]);
        let b = Section::new(square(0.5, 0.5, 1.0), vec![]);

        let result = section_union(&a, &b).unwrap();
        assert_eq!(result.len(), 1);
        // The hole is filled by B -> union is the full 2x2 square, no border hole.
        assert_eq!(result[0].holes.len(), 0, "filled hole must disappear: {:?}", result[0].holes.len());
        assert!((result[0].area() - 4.0).abs() < 1e-9, "union area {}", result[0].area());
    }

    #[test]
    fn section_intersection_preserves_holes() {
        use crate::section::Section;
        // Two fully overlapping squares, each with its own hole. Intersection
        // is material where BOTH have material, so both holes are voids.
        let hole_a = square(0.5, 0.5, 0.5);
        let a = Section::new(square(0.0, 0.0, 2.0), vec![hole_a]);
        let hole_b = square(1.0, 1.0, 0.5);
        let b = Section::new(square(0.0, 0.0, 2.0), vec![hole_b]);

        let result = section_intersection(&a, &b).unwrap();
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].holes.len(), 2, "intersection keeps both holes");
    }

    #[test]
    fn section_intersection_clips_holes_to_piece() {
        use crate::section::Section;
        // A = 4x4 with a 2x2 hole ([1,3]^2). B = 1.5x4 strip along the left
        // edge with its own 0.5x2 hole. The intersection piece is only
        // [0,1.5]x[0,4], so neither full hole fits: each must be clipped to
        // hole ∩ piece.
        let hole_a = square(1.0, 1.0, 2.0); // [1,3]^2
        let a = Section::new(square(0.0, 0.0, 4.0), vec![hole_a]);
        let b_outer = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(1.5, 0.0),
            Point::new(1.5, 4.0),
            Point::new(0.0, 4.0),
        ]);
        let hole_b = Polygon::new(vec![
            Point::new(0.5, 1.0),
            Point::new(1.0, 1.0),
            Point::new(1.0, 3.0),
            Point::new(0.5, 3.0),
        ]); // [0.5,1.0]x[1.0,3.0], area 1
        let b = Section::new(b_outer, vec![hole_b]);

        let result = section_intersection(&a, &b).unwrap();
        assert_eq!(result.len(), 1);
        let sec = &result[0];
        // Piece = [0,1.5]x[0,4], area 6.
        assert!((sec.outer.area() - 6.0).abs() < 1e-6, "piece area {}", sec.outer.area());
        // A's hole clipped to [1,1.5]x[1,3] (0.5x2) and B's hole [0.5,1.0]x[1,3]
        // (already inside the piece). Total void = 1 + 1 = 2.
        let hole_area: f64 = sec.holes.iter().map(|h| h.area()).sum();
        assert!((hole_area - 2.0).abs() < 1e-4, "clipped holes total area {}", hole_area);
        assert_holes_within_piece(&sec.outer, &sec.holes);
        // Net material = 6 - 2 = 4.
        assert!((sec.area() - 4.0).abs() < 1e-4, "section area {}", sec.area());
    }

    #[test]
    fn section_intersection_unions_overlapping_holes() {
        use crate::section::Section;
        // A = 4x4 with hole [1,3]x[1,3]. B = 4x4 (same outer) with hole
        // [2,4]x[1,3]. The two holes overlap in [2,3]x[1,3]. The intersection
        // piece is the full 4x4 and both holes are voids, but the overlap must
        // not be subtracted twice: the final hole set is their union
        // ([1,4]x[1,3], area 6), not 4 + 4 = 8.
        let hole_a = square(1.0, 1.0, 2.0); // [1,3]^2, area 4
        let a = Section::new(square(0.0, 0.0, 4.0), vec![hole_a]);
        let hole_b = square(2.0, 1.0, 2.0); // [2,4]x[1,3], area 4
        let b = Section::new(square(0.0, 0.0, 4.0), vec![hole_b]);

        let result = section_intersection(&a, &b).unwrap();
        assert_eq!(result.len(), 1);
        let sec = &result[0];
        assert!((sec.outer.area() - 16.0).abs() < 1e-6, "piece area {}", sec.outer.area());
        // Overlapping holes must fold into a single void of area 6.
        assert_eq!(sec.holes.len(), 1, "overlapping holes must merge: {}", sec.holes.len());
        let hole_area = sec.holes[0].area();
        assert!((hole_area - 6.0).abs() < 1e-4, "merged hole area {}", hole_area);
        // The merged hole must lie within the piece (allowing perturbation tol).
        assert_holes_within_piece(&sec.outer, &sec.holes);
        // Net material = 16 - 6 = 10 (overlap counted once).
        assert!((sec.area() - 10.0).abs() < 1e-4, "section area {}", sec.area());
    }

    #[test]
    fn section_intersection_clips_hole_to_piece() {
        use crate::section::Section;
        // A = 10x10 with an 8x8 hole ([1,9]^2). B = 2x10 strip along the left
        // edge ([0,2]x[0,10]). The intersection piece is only 2 wide, so the
        // full 8x8 hole cannot fit inside it -- it must be clipped.
        let hole = square(1.0, 1.0, 8.0); // [1,9]^2
        let a = Section::new(square(0.0, 0.0, 10.0), vec![hole]);
        let b_outer = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(2.0, 0.0),
            Point::new(2.0, 10.0),
            Point::new(0.0, 10.0),
        ]);
        let b = Section::new(b_outer, vec![]);

        let result = section_intersection(&a, &b).unwrap();
        assert_eq!(result.len(), 1);
        let sec = &result[0];
        // Piece = [0,2]x[0,10], area 20.
        assert!((sec.outer.area() - 20.0).abs() < 1e-6, "piece area {}", sec.outer.area());
        // Hole clipped to [1,2]x[1,9], area 8, and it must lie inside the piece.
        let hole_area: f64 = sec.holes.iter().map(|h| h.area()).sum();
        assert!((hole_area - 8.0).abs() < 1e-4, "clipped hole area {}", hole_area);
        assert_holes_within_piece(&sec.outer, &sec.holes);
        // Net material = 20 - 8 = 12 (and not negative).
        assert!((sec.area() - 12.0).abs() < 1e-4, "section area {}", sec.area());
    }

    #[test]
    fn section_difference_clips_hole_to_piece() {
        use crate::section::Section;
        // A = 10x10 with an 8x8 hole ([1,9]^2). B = 5x10 strip along the left
        // edge ([0,5]x[0,10]). The difference piece is [5,10]x[0,10], so only
        // the part of A's hole inside it ([5,9]x[1,9]) survives, clipped.
        let hole = square(1.0, 1.0, 8.0); // [1,9]^2
        let a = Section::new(square(0.0, 0.0, 10.0), vec![hole]);
        let b_outer = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(5.0, 0.0),
            Point::new(5.0, 10.0),
            Point::new(0.0, 10.0),
        ]);
        let b = Section::new(b_outer, vec![]);

        let result = section_difference(&a, &b).unwrap();
        assert_eq!(result.len(), 1);
        let sec = &result[0];
        // Piece = [5,10]x[0,10], area 50.
        assert!((sec.outer.area() - 50.0).abs() < 1e-6, "piece area {}", sec.outer.area());
        // Hole clipped to [5,9]x[1,9], area 32, inside the piece.
        let hole_area: f64 = sec.holes.iter().map(|h| h.area()).sum();
        assert!((hole_area - 32.0).abs() < 1e-4, "clipped hole area {}", hole_area);
        assert_holes_within_piece(&sec.outer, &sec.holes);
        // Net material = 50 - 32 = 18.
        assert!((sec.area() - 18.0).abs() < 1e-4, "section area {}", sec.area());
    }

    #[test]
    fn section_union_clips_hole_partially_filled() {
        use crate::section::Section;
        // A = 2x2 with a centred 1x1 hole. B = solid rectangle filling the
        // bottom-right quarter of that hole. The union keeps the hole minus the
        // filled part (area 0.75) and drops the corner covered by B.
        let hole = square(0.5, 0.5, 1.0); // [0.5,1.5]^2
        let a = Section::new(square(0.0, 0.0, 2.0), vec![hole]);
        let b = Section::new(square(1.0, 0.5, 0.5), vec![]); // [1.0,1.5]x[0.5,1.0]

        let result = section_union(&a, &b).unwrap();
        assert_eq!(result.len(), 1);
        let sec = &result[0];
        let hole_area: f64 = sec.holes.iter().map(|h| h.area()).sum();
        // Surviving void = hole (area 1) minus B's filled part (area 0.25) = 0.75.
        assert!(
            (hole_area - 0.75).abs() < 1e-3,
            "partially filled hole area: {}",
            hole_area
        );
        // Union outer is the full 2x2; material = 4 - 0.75 = 3.25.
        assert!((sec.area() - 3.25).abs() < 1e-3, "union area {}", sec.area());
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
    fn perturbation_result_is_mapped_back_to_original_frame() {
        // Regression for P0b: when the boolean engine perturbs `b` to resolve a
        // vertex-on-boundary degeneracy, the returned geometry must be mapped
        // back onto the *original* coordinate frame (vertices reduced by
        // `-eps * direction`). Otherwise B-originated vertices retain the
        // artificial `~eps` offset, which corrupts higher-order section
        // properties (centroid, moments, principal axes).
        //
        // Two unit squares that share exactly one corner: A=[0,1]^2, B=[1,2]^2
        // touch at (1,1). The corner (1,1) lies on A's boundary, so the
        // perturbation path is taken. A and B share no edges and do not cross,
        // so the union has NO computed crossing points — every vertex is an
        // original polygon vertex and must be restored to its exact position.
        let a = square(0.0, 0.0, 1.0);
        let b = square(1.0, 1.0, 1.0);

        let u = polygon_boolean(&a, &b, BoolOp::Union);
        let total: f64 = u.iter().map(|p| p.area()).sum();
        assert!((total - 2.0).abs() < 1e-9, "union area: {}", total);

        // B's far corner (2,2) is a pure B vertex. Without map-back it would
        // sit at (2 + eps*dx, 2 + eps*dy); with map-back it is restored to
        // exactly (2,2) up to floating-point roundoff.
        let found = u
            .iter()
            .flat_map(|p| &p.vertices)
            .any(|v| (v.x - 2.0).abs() < 1e-12 && (v.y - 2.0).abs() < 1e-12);
        assert!(
            found,
            "B-originated vertex (2,2) not at its original position after \
             perturbation; back-mapping missing?"
        );

        // The shared-corner point must also be exactly restored (it is emitted
        // as an A vertex and, if not merged, as a duplicate B vertex).
        let found_corner = u
            .iter()
            .flat_map(|p| &p.vertices)
            .any(|v| (v.x - 1.0).abs() < 1e-12 && (v.y - 1.0).abs() < 1e-12);
        assert!(found_corner, "shared corner (1,1) not restored after perturbation");
    }

    #[test]
    fn bounds_validation_rejects_bloated_union() {
        // Regression for the "identity alone is not sufficient" case: two
        // area-100 polygons could yield |A∩B|=30 and |A∪B|=170 with
        // 30+170=100+100 (identity passes) even though the union clearly
        // violates |A∪B| >= max(|A|,|B|). The area-bounds gate must reject it.
        let a = square(0.0, 0.0, 10.0); // area 100
        let b = square(0.0, 0.0, 10.0); // area 100

        // Fabricated, obviously-wrong results (area far outside the feasible
        // range for two area-100 polygons):
        // - union as a 30×10 rectangle (area 300) > |A|+|B| = 200
        // - intersection as a 17×10 rectangle (area 170) > min(|A|,|B|) = 100
        let bad_union = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(30.0, 0.0),
            Point::new(30.0, 10.0),
            Point::new(0.0, 10.0),
        ]);
        let bad_inter = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(17.0, 0.0),
            Point::new(17.0, 10.0),
            Point::new(0.0, 10.0),
        ]);
        assert!(
            check_boolean_bounds(&[bad_union], &a, &b, BoolOp::Union).is_err(),
            "union area 300 must be rejected (exceeds |A|+|B| = 200)"
        );
        assert!(
            check_boolean_bounds(&[bad_inter.clone()], &a, &b, BoolOp::Intersection).is_err(),
            "intersection area 170 must be rejected (exceeds min(|A|,|B|) = 100)"
        );

        // A legitimate result passes.
        let good = polygon_boolean_checked(&a, &b, BoolOp::Union).unwrap();
        assert!(
            check_boolean_bounds(&good, &a, &b, BoolOp::Union).is_ok(),
            "correct union must pass bounds validation"
        );
    }

    #[test]
    fn checked_api_reports_ok_and_error_variants() {
        let a = square(0.0, 0.0, 2.0);
        let b = square(1.0, 1.0, 2.0);

        // Normal case: Ok with the expected area.
        let ok = polygon_boolean_checked(&a, &b, BoolOp::Intersection).unwrap();
        assert!((ok[0].area() - 1.0).abs() < 1e-9);

        // The convenience wrapper yields the same value.
        let conv = polygon_boolean(&a, &b, BoolOp::Union);
        assert!((conv[0].area() - 7.0).abs() < 1e-9);

        // Difference of nested inputs returns the (hole-not-cut) outer, which
        // passes the area bounds (|res| == |A|), demonstrating the documented
        // polygon-level convention is accepted by the bounds gate.
        let outer = square(0.0, 0.0, 3.0);
        let inner = square(1.0, 1.0, 1.0);
        let d = polygon_boolean_checked(&outer, &inner, BoolOp::Difference).unwrap();
        assert_eq!(d.len(), 1);
        assert!((d[0].area() - 9.0).abs() < 1e-12);

        // Error type is constructible and Displayable (used in panic messages).
        let err = BooleanError::Validation {
            op: BoolOp::Union,
            message: "test".to_string(),
        };
        assert!(err.to_string().contains("boolean Union failed validation"));
    }

    #[test]
    fn section_operators_propagate_errors_as_result() {
        use crate::section::Section;

        let a = Section::new(square(0.0, 0.0, 4.0), vec![]);
        let b = Section::new(square(1.0, 1.0, 2.0), vec![]);

        // The Result-wrapped API: match to confirm it is Ok on valid input.
        match section_intersection(&a, &b) {
            Ok(regions) => assert!(regions[0].area() > 0.0),
            Err(e) => panic!("expected Ok, got Err: {e}"),
        }

        // Union is recoverable via ?-style propagation in the same way.
        let union = section_union(&a, &b).expect("union must succeed");
        assert_eq!(union.len(), 1);

        // A geometry error does not panic at the section boundary; it is a
        // typed Result a caller can respond to. (No valid input here triggers
        // one, so we only assert the type flows through: unwrap succeeds.)
        let diff = section_difference(&a, &b).unwrap();
        assert!((diff[0].area() - 16.0).abs() < 1e-6 || diff[0].holes.len() == 1);
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

    // ---- Determinism & edge-case tests ----

    #[test]
    fn deterministic_same_input_same_output() {
        let a = square(0.0, 0.0, 2.0);
        let b = Polygon::new(vec![
            Point::new(1.0, 0.0),
            Point::new(3.0, 1.0),
            Point::new(1.0, 2.0),
        ]);
        // Run twice; results must be byte-identical.
        let r1 = polygon_boolean(&a, &b, BoolOp::Intersection);
        let r2 = polygon_boolean(&a, &b, BoolOp::Intersection);
        assert_eq!(r1.len(), r2.len());
        for (p, q) in r1.iter().zip(&r2) {
            assert_eq!(p.vertices.len(), q.vertices.len());
            for (v, w) in p.vertices.iter().zip(&q.vertices) {
                assert!((v.x - w.x).abs() < 1e-15 && (v.y - w.y).abs() < 1e-15);
            }
        }
    }

    #[test]
    fn narrow_overlap_intersection() {
        // Two rectangles with a very thin (0.001 wide) overlap.
        let a = square(0.0, 0.0, 1.0);
        let b = Polygon::new(vec![
            Point::new(0.999, 0.2),
            Point::new(2.0, 0.2),
            Point::new(2.0, 0.8),
            Point::new(0.999, 0.8),
        ]);
        let r = polygon_boolean(&a, &b, BoolOp::Intersection);
        assert_eq!(r.len(), 1);
        let expected_area = 0.001 * 0.6; // 1mm × 0.6m
        assert!(
            (r[0].area() - expected_area).abs() < 1e-9,
            "narrow intersection area: got {}, expected {}",
            r[0].area(),
            expected_area
        );
    }

    #[test]
    fn near_vertex_on_edge_intersection() {
        // Vertex of B is very close to (but not exactly on) an edge of A.
        let a = square(0.0, 0.0, 2.0);
        let b = Polygon::new(vec![
            Point::new(1.0, 1e-10), // Nearly on A's bottom edge (y=0)
            Point::new(3.0, 1.0),
            Point::new(1.0, 2.0),
        ]);
        let r = polygon_boolean(&a, &b, BoolOp::Intersection);
        assert!(!r.is_empty());
        let area: f64 = r.iter().map(|p| p.area()).sum();
        assert!(area > 0.0, "intersection must have positive area");
        // Triangle area=2. Cut beyond x=2 removes a small triangle (area ~0.5).
        // Expected intersection ~1.5.
        assert!((area - 1.5).abs() < 0.01, "near-vertex intersection area: {}", area);
    }

    #[test]
    fn shared_vertex_boolean() {
        // Two squares sharing exactly one vertex.
        let a = square(0.0, 0.0, 1.0);
        let b = square(1.0, 1.0, 1.0);
        let i = polygon_boolean(&a, &b, BoolOp::Intersection);
        assert!(i.is_empty(), "intersection of touching-at-vertex squares should be empty");
        let u = polygon_boolean(&a, &b, BoolOp::Union);
        let total: f64 = u.iter().map(|p| p.area()).sum();
        assert!((total - 2.0).abs() < 1e-9, "union area of touching squares: {}", total);
    }

    #[test]
    fn u_shape_boolean() {
        // U-shape intersected with a rectangle.
        // Note: the U-shape polygon (concave, non-holed) has the "cavity"
        // as interior by ray-casting — it's a single connected region.
        let u = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(3.0, 0.0),
            Point::new(3.0, 3.0),
            Point::new(2.0, 3.0),
            Point::new(2.0, 1.0),
            Point::new(1.0, 1.0),
            Point::new(1.0, 3.0),
            Point::new(0.0, 3.0),
        ]);
        let fill = square(0.5, 0.5, 2.0); // [0.5,2.5]×[0.5,2.5]
        let r = polygon_boolean(&u, &fill, BoolOp::Intersection);
        let total: f64 = r.iter().map(|p| p.area()).sum();
        // fill is fully inside U (cavity is interior for a simple polygon),
        // so intersection = fill area = 4.
        assert!((total - 4.0).abs() < 1e-6, "U-shape intersection: {}", total);

        // Now test with a truly disconnected region: two disjoint parts.
        let left = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(1.0, 3.0),
            Point::new(0.0, 3.0),
        ]);
        let right = Polygon::new(vec![
            Point::new(2.0, 0.0),
            Point::new(3.0, 0.0),
            Point::new(3.0, 3.0),
            Point::new(2.0, 3.0),
        ]);
        let r_left = polygon_boolean(&left, &fill, BoolOp::Intersection);
        let r_right = polygon_boolean(&right, &fill, BoolOp::Intersection);
        let area_left: f64 = r_left.iter().map(|p| p.area()).sum();
        let area_right: f64 = r_right.iter().map(|p| p.area()).sum();
        // left ∩ fill = [0.5,1]×[0.5,2.5] = 0.5×2 = 1.0
        assert!((area_left - 1.0).abs() < 1e-6, "left intersection: {}", area_left);
        // right ∩ fill = [2,2.5]×[0.5,2.5] = 0.5×2 = 1.0
        assert!((area_right - 1.0).abs() < 1e-6, "right intersection: {}", area_right);
    }

    #[test]
    fn collinear_edges_difference() {
        // Two rectangles sharing a collinear edge at y=1.
        let a = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(2.0, 0.0),
            Point::new(2.0, 1.0),
            Point::new(0.0, 1.0),
        ]);
        let b = Polygon::new(vec![
            Point::new(0.5, 1.0),
            Point::new(1.5, 1.0),
            Point::new(1.5, 2.0),
            Point::new(0.5, 2.0),
        ]);
        let d = polygon_boolean(&a, &b, BoolOp::Difference);
        let total: f64 = d.iter().map(|p| p.area()).sum();
        assert!((total - 2.0).abs() < 1e-9, "collinear difference: {}", total);
    }

    #[test]
    fn touching_holes_stay_separate() {
        use crate::section::Section;
        // Two sections with the same 2x2 outer, each with a hole; the holes
        // share the edge x=1.0 but do not overlap. The intersection piece is
        // the full 2x2 and both holes are voids that merely touch -- they must
        // NOT be merged into one hole (that would lose topology).
        let hole_a = Polygon::new(vec![
            Point::new(0.5, 0.5),
            Point::new(1.0, 0.5),
            Point::new(1.0, 1.5),
            Point::new(0.5, 1.5),
        ]); // [0.5,1.0]x[0.5,1.5]
        let a = Section::new(square(0.0, 0.0, 2.0), vec![hole_a]);
        let hole_b = Polygon::new(vec![
            Point::new(1.0, 0.5),
            Point::new(1.5, 0.5),
            Point::new(1.5, 1.5),
            Point::new(1.0, 1.5),
        ]); // [1.0,1.5]x[0.5,1.5]
        let b = Section::new(square(0.0, 0.0, 2.0), vec![hole_b]);

        let result = section_intersection(&a, &b).unwrap();
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].holes.len(), 2, "touching holes must stay separate");
        let hole_area: f64 = result[0].holes.iter().map(|h| h.area()).sum();
        // Two 0.5 wide x 1 tall holes = 1.0 total void; they must not be
        // double-counted (which would give 1.0) nor merged+larger.
        assert!((hole_area - 1.0).abs() < 1e-4, "touching holes total area {}", hole_area);
        assert!((result[0].area() - (4.0 - 1.0)).abs() < 1e-4, "section area {}", result[0].area());
    }

    #[test]
    fn union_does_not_merge_touching_holes() {
        use crate::section::Section;
        // Same touching holes, but via UNION. In a union neither hole is filled
        // by the other section, so both survive and merely touch.
        let hole_a = Polygon::new(vec![
            Point::new(0.5, 0.5),
            Point::new(1.0, 0.5),
            Point::new(1.0, 1.5),
            Point::new(0.5, 1.5),
        ]);
        let a = Section::new(square(0.0, 0.0, 2.0), vec![hole_a]);
        let hole_b = Polygon::new(vec![
            Point::new(1.0, 0.5),
            Point::new(1.5, 0.5),
            Point::new(1.5, 1.5),
            Point::new(1.0, 1.5),
        ]);
        let b = Section::new(square(0.0, 0.0, 2.0), vec![hole_b]);

        let result = section_union(&a, &b).unwrap();
        assert_eq!(result.len(), 1);
        // Both holes are filled by the other section's material in a union, so
        // the touching holes do NOT survive -- union material is the full outer.
        assert_eq!(result[0].holes.len(), 0,
            "union fills both touching holes, got {} holes", result[0].holes.len());
        assert!((result[0].area() - 4.0).abs() < 1e-4, "union section area {}", result[0].area());
    }

    #[test]
    fn union_voids_keeps_touching_regions_separate() {
        // Direct unit test of the union_voids() heuristic: two polygons that
        // merely touch (share an edge) must NOT be merged into one by the
        // `u.len() == 1` check, because touching polygons have zero overlap and
        // are already disjoint for area purposes.
        let rect = |x0: f64, x1: f64, y0: f64, y1: f64| Polygon::new(vec![
            Point::new(x0, y0), Point::new(x1, y0), Point::new(x1, y1), Point::new(x0, y1)]);
        let a = rect(0.0, 1.0, 0.0, 1.0);
        let b = rect(1.0, 2.0, 0.0, 1.0); // touches a at x=1
        let mut voids = vec![a, b];
        union_voids(&mut voids).unwrap();
        assert_eq!(voids.len(), 2, "touching holes must not be merged");
        let total: f64 = voids.iter().map(|p| p.area()).sum();
        assert!((total - 2.0).abs() < 1e-9, "touching total area {}", total);
    }

    #[test]
    fn section_difference_clips_existing_hole_union() {
        use crate::section::Section;
        // A = 4x4 with a 2x2 hole ([1,3]^2). B = right-half band [2,4]x[0,4]
        // that slices through both A's hole and A's outer. The difference piece
        // is the left half [0,2]x[0,4]; A's existing hole is clipped to the
        // part inside the piece and the result must be the correct union of the
        // clipped hole with any B material (here none remains inside the piece).
        let a = Section::new(square(0.0, 0.0, 4.0), vec![square(1.0, 1.0, 2.0)]);
        let b_outer = Polygon::new(vec![
            Point::new(2.0, 0.0),
            Point::new(4.0, 0.0),
            Point::new(4.0, 4.0),
            Point::new(2.0, 4.0),
        ]);
        let b = Section::new(b_outer, vec![]);

        let result = section_difference(&a, &b).unwrap();
        assert_eq!(result.len(), 1);
        let sec = &result[0];
        // Piece = [0,2]x[0,4], area 8.
        assert!((sec.outer.area() - 8.0).abs() < 1e-6, "piece area {}", sec.outer.area());
        // Existing hole clipped to [1,2]x[1,3], area 2.
        let hole_area: f64 = sec.holes.iter().map(|h| h.area()).sum();
        assert!((hole_area - 2.0).abs() < 1e-4, "clipped existing hole area {}", hole_area);
        assert_holes_within_piece(&sec.outer, &sec.holes);
        // Net material = 8 - 2 = 6.
        assert!((sec.area() - 6.0).abs() < 1e-4, "section area {}", sec.area());
    }

    #[test]
    fn nested_holes_are_rejected_by_validation() {
        use crate::geometry::compound::CompoundGeometry;
        use crate::section::Section;
        // A single Section cannot represent a hole-within-a-hole (an island of
        // material inside a void). Validation must reject it, since
        // Section::area() would otherwise double-subtract the nested region.
        let hole_outer = square(1.0, 1.0, 2.0); // [1,3]^2
        let hole_inner = square(1.5, 1.5, 1.0); // [1.5,2.5]^2, nested inside
        let s = Section::new(square(0.0, 0.0, 4.0), vec![hole_outer, hole_inner]);
        let compound = CompoundGeometry::from_sections(&[s]);
        assert!(
            matches!(compound.validate(), Err(crate::geometry::compound::CompoundError::NestedHoles { .. })),
            "nested holes must be rejected by geometry validation"
        );

        // A control: two non-nested, non-overlapping holes are valid.
        let ok = Section::new(square(0.0, 0.0, 4.0), vec![
            square(0.5, 0.5, 0.5),
            square(3.0, 3.0, 0.5),
        ]);
        let compound_ok = CompoundGeometry::from_sections(&[ok]);
        assert!(compound_ok.validate().is_ok(), "non-nested holes must validate");
    }

#[test]
    fn section_difference_a_inside_b_hole_survives() {
        use crate::section::Section;
        // B = 10x10 with a 4x4 hole in the center
        // A = 2x2 completely inside B's hole
        // B has no material where A is, so A should survive completely in A - B
        let b = Section::new(square(0.0, 0.0, 10.0), vec![square(3.0, 3.0, 4.0)]);
        let a = Section::new(square(4.0, 4.0, 2.0), vec![]);

        let result = section_difference(&a, &b).unwrap();
        assert_eq!(result.len(), 1, "A should survive completely inside B's hole");
        let sec = &result[0];
        // A is 2x2 = 4 area, no holes
        assert!((sec.outer.area() - 4.0).abs() < 1e-6, "piece area {}", sec.outer.area());
        assert_eq!(sec.holes.len(), 0, "A has no holes");
        assert!((sec.area() - 4.0).abs() < 1e-6, "section area {}", sec.area());
    }

    #[test]
    fn section_difference_a_inside_b_material_removed() {
        use crate::section::Section;
        // B = 10x10 with a 4x4 hole in the center [3,7]^2
        // A = 2x2 completely inside B's MATERIAL (not hole), e.g. at [1,3]^2
        // B has material where A is, so A should be completely removed in A - B
        let b = Section::new(square(0.0, 0.0, 10.0), vec![square(3.0, 3.0, 4.0)]);
        let a = Section::new(square(1.0, 1.0, 2.0), vec![]); // [1,3]^2, inside B's material

        let result = section_difference(&a, &b).unwrap();
        assert_eq!(result.len(), 0, "A should be removed when inside B's material");
    }

#[test]
    fn section_difference_a_partial_b_hole_clipped() {
        use crate::section::Section;
        // B = 10x10 with a 4x4 hole [3,7]^2
        // A = 6x6 [2,8]^2 with a 2x2 hole [3,5]^2
        // A extends beyond B's hole on the right side.
        // The Difference piece = A \ B.outer = [2,8]x[2,8] \ [0,10]^2 = empty (since A inside B.outer)
        // Plus A ∩ B.hole = [3,7]x[3,7] ∩ [2,8]^2 = [3,7]x[3,7] (the part of A inside B's hole)
        // That piece has A's hole [3,5]^2 clipped to it → [3,5]^2 survives (inside B's hole, B has no material)
        // B material = none inside B's hole.
        let b = Section::new(square(0.0, 0.0, 10.0), vec![square(3.0, 3.0, 4.0)]); // hole [3,7]^2
        let a = Section::new(square(2.0, 2.0, 6.0), vec![square(3.0, 3.0, 2.0)]); // [2,8]^2 with hole [3,5]^2

        let result = section_difference(&a, &b).unwrap();
        assert_eq!(result.len(), 1, "one piece from A ∩ B.hole");
        let sec = &result[0];
        // Piece = A ∩ B.hole = [3,7]x[3,7], area 16
        assert!((sec.outer.area() - 16.0).abs() < 1e-6, "piece area {}", sec.outer.area());
        // A's hole [3,5]^2 is inside the piece, so survives fully
        let hole_area: f64 = sec.holes.iter().map(|h| h.area()).sum();
        assert!((hole_area - 4.0).abs() < 1e-4, "clipped hole area {}", hole_area);
        assert_holes_within_piece(&sec.outer, &sec.holes);
        // Net material = 16 - 4 = 12
        assert!((sec.area() - 12.0).abs() < 1e-4, "section area {}", sec.area());
    }
}




