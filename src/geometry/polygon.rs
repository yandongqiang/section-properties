use super::Point;

#[derive(Debug, Clone, PartialEq)]
pub enum PolygonError {
    /// Polygon has fewer than 3 vertices.
    TooFewVertices,
    /// After deduplication, fewer than 3 distinct vertices remain.
    TooFewDistinctVertices,
    /// A vertex has non-finite coordinates.
    NonFiniteVertex(usize),
    /// Polygon has zero or near-zero area.
    ZeroArea,
    /// Polygon has self-intersections (non-adjacent edges cross).
    SelfIntersection,
}

impl std::fmt::Display for PolygonError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PolygonError::TooFewVertices => write!(f, "polygon needs at least 3 vertices"),
            PolygonError::TooFewDistinctVertices => write!(f, "polygon needs at least 3 distinct vertices"),
            PolygonError::NonFiniteVertex(i) => write!(f, "vertex {} has non-finite coordinates", i),
            PolygonError::ZeroArea => write!(f, "polygon has zero or near-zero area"),
            PolygonError::SelfIntersection => write!(f, "polygon has self-intersections"),
        }
    }
}

impl std::error::Error for PolygonError {}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum JoinStyle {
    Miter,
    Round,
    Bevel,
}

impl Default for JoinStyle {
    fn default() -> Self {
        JoinStyle::Miter
    }
}

#[derive(Debug, Clone)]
pub struct Polygon {
    pub vertices: Vec<Point>,
}

impl Polygon {
    pub fn new(vertices: Vec<Point>) -> Self {
        assert!(vertices.len() >= 3, "Polygon needs at least 3 vertices");

        for (i, v) in vertices.iter().enumerate() {
            assert!(
                v.x.is_finite() && v.y.is_finite(),
                "Polygon vertex {} is not finite: ({}, {})",
                i,
                v.x,
                v.y
            );
        }

        // Remove consecutive duplicate vertices (zero-length edges) and a
        // trailing vertex that closes back on the first (closed-ring convention).
        let mut dedup: Vec<Point> = Vec::with_capacity(vertices.len());
        for v in &vertices {
            if dedup.last().map_or(true, |p| *p != *v) {
                dedup.push(*v);
            }
        }
        if dedup.len() > 1 && dedup[0] == *dedup.last().unwrap() {
            dedup.pop();
        }

        assert!(
            dedup.len() >= 3,
            "Polygon needs at least 3 distinct vertices"
        );

        let poly = Self { vertices: dedup };
        assert!(
            poly.signed_area().abs() > f64::EPSILON,
            "Polygon has (near-)zero area"
        );

        poly
    }

    /// Offset the polygon by `distance` using mitred corners.
    ///
    /// Positive distances grow a CCW polygon outward; negative distances shrink
    /// it. Returns `None` if the offset polygon degenerates (e.g. an inward
    /// offset larger than the polygon size).
    ///
    /// Mirrors shapely's `buffer(distance, join_style=mitre)` used by Python
    /// `sectionproperties`' `Geometry.offset()`.
    pub fn offset(&self, distance: f64) -> Option<Polygon> {
        self.offset_with_style(distance, JoinStyle::Miter, 2.0)
    }

    /// Offset the polygon with explicit join style and mitre limit.
    ///
    /// `join_style` controls corner treatment:
    /// - `Miter`: Extend offset edges until they intersect (default)
    /// - `Round`: Arc of radius |distance| (approximated with segments)
    /// - `Bevel`: Connect offset edges with a single segment
    ///
    /// `mitre_limit` (only for Miter) caps the mitre length to `mitre_limit * |distance|`.
    /// Exceeding corners are bevelled instead. Typical value: 2.0.
    pub fn offset_with_style(
        &self,
        distance: f64,
        join_style: JoinStyle,
        mitre_limit: f64,
    ) -> Option<Polygon> {
        // Work with CCW orientation so that positive = grow.
        let ccw: Vec<Point> = if self.signed_area() < 0.0 {
            self.vertices.iter().rev().copied().collect()
        } else {
            self.vertices.clone()
        };
        let n = ccw.len();
        if n < 3 {
            return None;
        }

        // Quick rejection: inward offset larger than inradius -> empty.
        if distance < 0.0 && self.area() < (distance * distance) * (n as f64) {
            // Cheap heuristic; actual degeneracy caught later.
        }

        let mut out: Vec<Point> = Vec::with_capacity(n * 4); // room for round/bevel
        let abs_d = distance.abs();

        for i in 0..n {
            let p0 = ccw[(i + n - 1) % n];
            let p1 = ccw[i];
            let p2 = ccw[(i + 1) % n];

            let d1x = p1.x - p0.x;
            let d1y = p1.y - p0.y;
            let d2x = p2.x - p1.x;
            let d2y = p2.y - p1.y;

            let l1 = (d1x * d1x + d1y * d1y).sqrt();
            let l2 = (d2x * d2x + d2y * d2y).sqrt();
            if l1 < f64::EPSILON || l2 < f64::EPSILON {
                continue;
            }

            // Outward unit normals (right side of travel direction for CCW).
            let n1 = (d1y / l1, -d1x / l1);
            let n2 = (d2y / l2, -d2x / l2);

            // Turn angle: positive = left turn (convex outward for CCW polygon).
            let cross = d1x * d2y - d1y * d2x;
            let dot = d1x * d2x + d1y * d2y;
            let turn = cross.atan2(dot); // (-PI, PI]

            if turn > 0.0 {
                // Convex corner -> mitre / round / bevel outwards
                let m1 = Point::new(p1.x + distance * n1.0, p1.y + distance * n1.1);
                let m2 = Point::new(p1.x + distance * n2.0, p1.y + distance * n2.1);

                match join_style {
                    JoinStyle::Miter => {
                        // Mitre length = |distance| / cos(turn/2).
                        // If > mitre_limit * |distance|, fall back to bevel.
                        let half = turn * 0.5;
                        let mitre_len = abs_d / half.cos().max(1e-12);
                        if mitre_len > mitre_limit * abs_d {
                            // Bevel: emit m1 then m2.
                            out.push(m1);
                            out.push(m2);
                        } else {
                            // Proper mitre: intersection of offset lines.
                            let denom = d1x * d2y - d1y * d2x;
                            if denom.abs() < 1e-14 {
                                out.push(m2);
                            } else {
                                let ax = p0.x + distance * n1.0;
                                let ay = p0.y + distance * n1.1;
                                let bx = p1.x + distance * n2.0;
                                let by = p1.y + distance * n2.1;
                                let t = ((bx - ax) * d2y - (by - ay) * d2x) / denom;
                                out.push(Point::new(ax + t * d1x, ay + t * d1y));
                            }
                        }
                    }
                    JoinStyle::Bevel => {
                        out.push(m1);
                        out.push(m2);
                    }
                    JoinStyle::Round => {
                        // Approximate arc with chord segments.
                        // Emit points along arc from m1 to m2.
                        let cx = p1.x;
                        let cy = p1.y;
                        let ang1 = (m1.y - cy).atan2(m1.x - cx);
                        let ang2 = (m2.y - cy).atan2(m2.x - cx);
                        // CCW arc from ang1 to ang2.
                        let diff = if ang2 > ang1 { ang2 - ang1 } else { ang2 - ang1 + 2.0 * std::f64::consts::PI };
                        // Use chord error to determine segment count: e = r * (1 - cos(dθ/2))
                        // For small dθ: 1 - cos(dθ/2) ≈ dθ²/8, so dθ ≈ sqrt(8*e/r)
                        // Use max_chord_error = 1e-6 (adjustable for precision needs).
                        const MAX_CHORD_ERROR: f64 = 1e-6;
                        let dtheta_max = (8.0 * MAX_CHORD_ERROR / abs_d).sqrt().min(std::f64::consts::PI / 6.0); // cap at 30°
                        let segs = ((diff / dtheta_max).ceil() as usize).max(3);
                        out.push(m1);
                        for s in 1..segs {
                            let a = ang1 + diff * (s as f64) / (segs as f64);
                            out.push(Point::new(cx + abs_d * a.cos(), cy + abs_d * a.sin()));
                        }
                        // m2 will be emitted by next iteration or manually here:
                        // Skip to avoid duplicate; next iteration emits its m1.
                    }
                }
            } else {
                // Concave corner (or straight) -> simple mitre intersection or bevel.
                let denom = d1x * d2y - d1y * d2x;
                if denom.abs() < 1e-14 {
                    // Collinear edges.
                    out.push(Point::new(p1.x + distance * n2.0, p1.y + distance * n2.1));
                } else {
                    let ax = p0.x + distance * n1.0;
                    let ay = p0.y + distance * n1.1;
                    let bx = p1.x + distance * n2.0;
                    let by = p1.y + distance * n2.1;
                    let t = ((bx - ax) * d2y - (by - ay) * d2x) / denom;
                    out.push(Point::new(ax + t * d1x, ay + t * d1y));
                }
            }
        }

        // ---- Self-intersection cleanup & ring reconstruction ----
        let clean = cleanup_offset_ring(out, distance > 0.0);
        if clean.is_empty() || clean.len() < 3 {
            return None;
        }

        let poly_area = signed_area_raw(&clean);
        // For outward offset, expect positive area; inward, expect smaller positive area.
        // Negative area indicates self-intersection (bowtie).
        if poly_area.abs() < f64::EPSILON || poly_area < 0.0 {
            return None;
        }

        // Ensure final polygon has correct orientation (CCW for positive area).
        let final_vertices = if poly_area > 0.0 { clean } else { clean.into_iter().rev().collect() };

        Some(Polygon::new(final_vertices))
    }

    /// Calculate the signed area of the polygon.
    ///
    /// Counter-clockwise polygon -> positive
    /// Clockwise polygon -> negative
    pub fn signed_area(&self) -> f64 {
        let mut sum = 0.0;

        for i in 0..self.vertices.len() {
            let p1 = self.vertices[i];
            let p2 = self.vertices[(i + 1) % self.vertices.len()];

            sum += p1.x * p2.y - p2.x * p1.y;
        }

        0.5 * sum
    }

    /// Calculate the absolute area of the polygon.
    pub fn area(&self) -> f64 {
        self.signed_area().abs()
    }

    /// Calculate the centroid of the polygon.
    pub fn centroid(&self) -> Point {
        let signed_area = self.signed_area();

        assert!(
            signed_area.abs() > f64::EPSILON,
            "Cannot calculate centroid of a degenerate polygon"
        );

        let mut cx = 0.0;
        let mut cy = 0.0;

        for i in 0..self.vertices.len() {
            let p1 = self.vertices[i];
            let p2 = self.vertices[(i + 1) % self.vertices.len()];

            let cross = p1.x * p2.y - p2.x * p1.y;

            cx += (p1.x + p2.x) * cross;
            cy += (p1.y + p2.y) * cross;
        }

        Point::new(cx / (6.0 * signed_area), cy / (6.0 * signed_area))
    }

    /// Calculate the second moment of area about the global x-axis.
    pub fn moment_of_inertia_x(&self) -> f64 {
        let mut sum = 0.0;

        for i in 0..self.vertices.len() {
            let p1 = self.vertices[i];
            let p2 = self.vertices[(i + 1) % self.vertices.len()];

            let cross = p1.x * p2.y - p2.x * p1.y;

            sum += (p1.y.powi(2) + p1.y * p2.y + p2.y.powi(2)) * cross;
        }

        sum / 12.0
    }

    /// Calculate the second moment of area about the global y-axis.
    pub fn moment_of_inertia_y(&self) -> f64 {
        let mut sum = 0.0;

        for i in 0..self.vertices.len() {
            let p1 = self.vertices[i];
            let p2 = self.vertices[(i + 1) % self.vertices.len()];

            let cross = p1.x * p2.y - p2.x * p1.y;

            sum += (p1.x.powi(2) + p1.x * p2.x + p2.x.powi(2)) * cross;
        }

        sum / 12.0
    }

    /// Calculate the product of area about the global axes.
    pub fn product_of_inertia_xy(&self) -> f64 {
        let mut sum = 0.0;

        for i in 0..self.vertices.len() {
            let p1 = self.vertices[i];
            let p2 = self.vertices[(i + 1) % self.vertices.len()];

            let cross = p1.x * p2.y - p2.x * p1.y;

            sum += (p1.x * p2.y + 2.0 * p1.x * p1.y + 2.0 * p2.x * p2.y + p2.x * p1.y) * cross;
        }

        sum / 24.0
    }

    /// Calculate the second moment of area about the centroidal x-axis.
    pub fn centroidal_moment_of_inertia_x(&self) -> f64 {
        let area = self.area();
        let centroid = self.centroid();

        self.moment_of_inertia_x() - area * centroid.y.powi(2)
    }

    /// Calculate the second moment of area about the centroidal y-axis.
    pub fn centroidal_moment_of_inertia_y(&self) -> f64 {
        let area = self.area();
        let centroid = self.centroid();

        self.moment_of_inertia_y() - area * centroid.x.powi(2)
    }

    /// Calculate the product of area about the centroidal axes.
    pub fn centroidal_product_of_inertia_xy(&self) -> f64 {
        let area = self.area();
        let centroid = self.centroid();

        self.product_of_inertia_xy() - area * centroid.x * centroid.y
    }

    /// Rotate all vertices about the origin by `angle` radians (CCW positive).
    pub fn rotate(&self, angle: f64) -> Self {
        Polygon::new(
            self.vertices
                .iter()
                .map(|v| {
                    Point::new(
                        v.x * angle.cos() - v.y * angle.sin(),
                        v.x * angle.sin() + v.y * angle.cos(),
                    )
                })
                .collect(),
        )
    }

    /// Check if a point is inside the polygon (ray casting algorithm).
    pub fn contains_point(&self, point: Point) -> bool {
        let mut inside = false;
        let n = self.vertices.len();

        for i in 0..n {
            let p1 = self.vertices[i];
            let p2 = self.vertices[(i + 1) % n];

            // Check if point is on the edge
            if point_on_segment(point, p1, p2) {
                return true;
            }

            // Ray casting algorithm
            if ((p1.y > point.y) != (p2.y > point.y))
                && (point.x < (p2.x - p1.x) * (point.y - p1.y) / (p2.y - p1.y) + p1.x)
            {
                inside = !inside;
            }
        }

        inside
    }

    /// Calculate the perimeter of the polygon.
    pub fn perimeter(&self) -> f64 {
        let mut sum = 0.0;
        let n = self.vertices.len();

        for i in 0..n {
            let p1 = self.vertices[i];
            let p2 = self.vertices[(i + 1) % n];
            let dx = p2.x - p1.x;
            let dy = p2.y - p1.y;
            sum += (dx * dx + dy * dy).sqrt();
        }

        sum
    }

    /// Maximum fiber distance from centroid in Y direction (for section modulus).
    pub fn max_fiber_distance_y(&self) -> f64 {
        let centroid = self.centroid();
        self.vertices
            .iter()
            .map(|v| (v.y - centroid.y).abs())
            .fold(0.0, f64::max)
    }

    /// Maximum fiber distance from centroid in X direction (for section modulus).
    pub fn max_fiber_distance_x(&self) -> f64 {
        let centroid = self.centroid();
        self.vertices
            .iter()
            .map(|v| (v.x - centroid.x).abs())
            .fold(0.0, f64::max)
    }
    /// Clip the polygon against the half-plane `nx*x + ny*y <= c`
    /// (`below = true`) or `>= c` (`below = false`), using
    /// Sutherland-Hodgman clipping. Returns `None` when nothing remains.
    pub fn clip_halfspace(&self, n_x: f64, n_y: f64, c: f64, below: bool) -> Option<Polygon> {
        let verts = &self.vertices;
        if verts.is_empty() {
            return None;
        }

        let side = |p: &Point| -> f64 { n_x * p.x + n_y * p.y - c };

        let mut out: Vec<Point> = Vec::new();
        let s = verts.len();
        let mut prev = verts[s - 1];
        let mut prev_inside = if below {
            side(&prev) <= 0.0
        } else {
            side(&prev) >= 0.0
        };

        for cur in verts.iter() {
            let cur_inside = if below {
                side(cur) <= 0.0
            } else {
                side(cur) >= 0.0
            };
            if prev_inside != cur_inside {
                // Intersection with the cutting line.
                let dp = side(&prev);
                let dc = side(cur);
                let t = dp / (dp - dc);
                out.push(Point::new(
                    prev.x + t * (cur.x - prev.x),
                    prev.y + t * (cur.y - prev.y),
                ));
            }
            if cur_inside {
                out.push(*cur);
            }
            prev = *cur;
            prev_inside = cur_inside;
        }

        if out.len() < 3 {
            return None;
        }

        // Drop degenerate slivers safely.
        let area: f64 = out
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let q = &out[(i + 1) % out.len()];
                p.x * q.y - q.x * p.y
            })
            .sum::<f64>()
            / 2.0;
        if area.abs() < 1e-14 {
            return None;
        }

        Some(Polygon::new(out))
    }

    /// Split the polygon into the two halves lying either side of the line
    /// through points `a` and `b`.
    ///
    /// Returns `(below, above)`; either side may be `None` when the line does
    /// not cut the polygon. Mirrors Python `bisect_section` helpers used by
    /// `Section.split_section`.
    pub fn split_by_line(&self, a: Point, b: Point) -> (Option<Polygon>, Option<Polygon>) {
        let dx = b.x - a.x;
        let dy = b.y - a.y;
        let len = (dx * dx + dy * dy).sqrt().max(1e-15);
        let n_x = dy / len;
        let n_y = -dx / len;
        let c = n_x * a.x + n_y * a.y;
        (
            self.clip_halfspace(n_x, n_y, c, true),
            self.clip_halfspace(n_x, n_y, c, false),
        )
    }

    /// Check if the polygon has self-intersections (non-adjacent edges crossing).
    ///
    /// Returns `true` if any two non-adjacent edges intersect in their interiors.
    /// This does not count shared vertices (adjacent edges) as intersections.
    pub fn has_self_intersections(&self) -> bool {
        let n = self.vertices.len();
        if n < 4 {
            return false;
        }

        for i in 0..n {
            let a1 = self.vertices[i];
            let a2 = self.vertices[(i + 1) % n];
            for j in (i + 2)..n {
                // Skip adjacent edges (share a vertex) and first/last pair.
                if i == 0 && j == n - 1 {
                    continue;
                }
                let b1 = self.vertices[j];
                let b2 = self.vertices[(j + 1) % n];

                if let Some((t_a, t_b)) = segment_intersection(a1, a2, b1, b2) {
                    // Exclude intersections at endpoints (already shared vertices).
                    if t_a > 1e-10 && t_a < 1.0 - 1e-10 && t_b > 1e-10 && t_b < 1.0 - 1e-10 {
                        return true;
                    }
                }
            }
        }
        false
    }

    /// Validating constructor: returns an error if the polygon has
    /// self-intersections or is otherwise invalid.
    ///
    /// Checks:
    /// - At least 3 distinct vertices
    /// - All vertices are finite
    /// - Non-zero area
    /// - No self-intersections
    pub fn try_new(vertices: Vec<Point>) -> Result<Self, PolygonError> {
        if vertices.len() < 3 {
            return Err(PolygonError::TooFewVertices);
        }

        for (i, v) in vertices.iter().enumerate() {
            if !v.x.is_finite() || !v.y.is_finite() {
                return Err(PolygonError::NonFiniteVertex(i));
            }
        }

        // Remove consecutive duplicate vertices and trailing closed-ring vertex.
        let mut dedup: Vec<Point> = Vec::with_capacity(vertices.len());
        for v in &vertices {
            if dedup.last().map_or(true, |p| *p != *v) {
                dedup.push(*v);
            }
        }
        if dedup.len() > 1 && dedup[0] == *dedup.last().unwrap() {
            dedup.pop();
        }

        if dedup.len() < 3 {
            return Err(PolygonError::TooFewDistinctVertices);
        }

        let poly = Self { vertices: dedup };

        if poly.signed_area().abs() <= f64::EPSILON {
            return Err(PolygonError::ZeroArea);
        }

        if poly.has_self_intersections() {
            return Err(PolygonError::SelfIntersection);
        }

        Ok(poly)
    }
}

impl super::boundary::BoundaryExtrema for Polygon {
    fn extreme_distances(&self, center: Point, direction: Point) -> (f64, f64) {
        self.vertices
            .iter()
            .map(|v| {
                let dx = v.x - center.x;
                let dy = v.y - center.y;
                dx * direction.x + dy * direction.y
            })
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(mn, mx), d| {
                (mn.min(d), mx.max(d))
            })
    }
}

/// Compute signed area of a raw vertex list (assumed closed).
fn signed_area_raw(verts: &[Point]) -> f64 {
    let mut sum = 0.0;
    let n = verts.len();
    for i in 0..n {
        let p1 = verts[i];
        let p2 = verts[(i + 1) % n];
        sum += p1.x * p2.y - p2.x * p1.y;
    }
    0.5 * sum
}

/// Distance squared between two points.
fn distance_sq(a: Point, b: Point) -> f64 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    dx * dx + dy * dy
}

/// Internal intersection representation for self-intersection cleanup.
#[allow(dead_code)]
struct Intersection {
    edge_a: usize,
    edge_b: usize,
    t_a: f64,
    t_b: f64,
    point: Point,
}

/// Remove zero-length edges, near-duplicate vertices, and fix self-intersections
/// in the offset ring. Returns a cleaned vertex list (still CCW if possible).
fn cleanup_offset_ring(verts: Vec<Point>, outward: bool) -> Vec<Point> {
    if verts.len() < 3 {
        return Vec::new();
    }

    // 1. Remove consecutive near-duplicate vertices.
    let eps = 1e-12;
    let mut dedup: Vec<Point> = Vec::with_capacity(verts.len());
    for v in &verts {
        if dedup.last().map_or(true, |p| distance_sq(*p, *v) > eps * eps) {
            dedup.push(*v);
        }
    }
    // Close check: first vs last
    if dedup.len() > 1 && distance_sq(dedup[0], *dedup.last().unwrap()) < eps * eps {
        dedup.pop();
    }
    if dedup.len() < 3 {
        return Vec::new();
    }

    // 2. Detect and remove self-intersections by finding simple polygon loops.
    // TODO: Current implementation is a pragmatic fallback, not topologically correct.
    // split_into_loops() is a stub; "largest loop" heuristic fails for complex
    // concave sections (cold-formed C/Z/Σ) where true offset has multiple valid regions.
    // Proper fix: implement Vatti/Greiner-Hormann clipping or integrate Clipper/GEOS
    // for true polygon buffer / ring reconstruction.
    if let Some(clean) = remove_self_intersections(&dedup, outward) {
        return clean;
    }

    dedup
}

/// Find self-intersections and return the largest valid simple loop.
fn remove_self_intersections(verts: &[Point], outward: bool) -> Option<Vec<Point>> {
    let n = verts.len();
    if n < 4 {
        return Some(verts.to_vec());
    }

    // Find all intersection points between non-adjacent edges.
    let mut intersections = Vec::new();

    for i in 0..n {
        let a1 = verts[i];
        let a2 = verts[(i + 1) % n];
        for j in (i + 2)..n {
            // Skip adjacent edges (share a vertex) and first/last pair.
            if i == 0 && j == n - 1 {
                continue;
            }
            let b1 = verts[j];
            let b2 = verts[(j + 1) % n];

            if let Some((t_a, t_b)) = segment_intersection(a1, a2, b1, b2) {
                // Exclude intersections at endpoints (already shared vertices).
                if t_a > 1e-10 && t_a < 1.0 - 1e-10 && t_b > 1e-10 && t_b < 1.0 - 1e-10 {
                    let px = a1.x + t_a * (a2.x - a1.x);
                    let py = a1.y + t_a * (a2.y - a1.y);
                    intersections.push(Intersection {
                        edge_a: i,
                        edge_b: j,
                        t_a,
                        t_b,
                        point: Point::new(px, py),
                    });
                }
            }
        }
    }

    if intersections.is_empty() {
        return Some(verts.to_vec());
    }

    // Sort intersections by edge_a, then t_a.
    intersections.sort_by(|a, b| {
        a.edge_a
            .cmp(&b.edge_a)
            .then(a.t_a.partial_cmp(&b.t_a).unwrap())
    });

    // Build split points per edge.
    // We'll reconstruct the polygon by walking along edges and inserting intersection vertices.
    // For simplicity and robustness in engineering contexts, we use a different approach:
    // sample the polygon and use winding to keep the "correct" region.
    // But here we just keep the largest simple sub-loop by area.
    // This is a pragmatic fix for typical offset self-intersections.

    // Split into loops at intersections.
    let loops = split_into_loops(verts, &intersections);
    if loops.is_empty() {
        return Some(verts.to_vec());
    }

    // Pick the loop with largest absolute area that has correct orientation.
    let mut best: Option<Vec<Point>> = None;
    let mut best_area = 0.0;
    for loop_verts in loops {
        if loop_verts.len() < 3 {
            continue;
        }
        let area = signed_area_raw(&loop_verts);
        let abs_area = area.abs();
        // Outward offset should be CCW (positive); inward offset also positive but smaller.
        let correct_orient = if outward { area > 0.0 } else { area > 0.0 };
        if correct_orient && abs_area > best_area {
            best_area = abs_area;
            best = Some(loop_verts);
        }
    }

    best
}

/// Check if two segments intersect and return (t_a, t_b) parameters.
fn segment_intersection(a1: Point, a2: Point, b1: Point, b2: Point) -> Option<(f64, f64)> {
    let dx1 = a2.x - a1.x;
    let dy1 = a2.y - a1.y;
    let dx2 = b2.x - b1.x;
    let dy2 = b2.y - b1.y;

    let denom = dx1 * dy2 - dy1 * dx2;
    if denom.abs() < 1e-14 {
        return None; // Parallel or collinear
    }

    let dx3 = b1.x - a1.x;
    let dy3 = b1.y - a1.y;
    let t_a = (dx3 * dy2 - dy3 * dx2) / denom;
    let t_b = (dx3 * dy1 - dy3 * dx1) / denom;

    if t_a >= 0.0 && t_a <= 1.0 && t_b >= 0.0 && t_b <= 1.0 {
        Some((t_a, t_b))
    } else {
        None
    }
}

/// Split polygon into simple loops at intersections.
/// TODO: Currently a stub - does not actually split at intersections.
/// Proper implementation needs to:
/// 1. Insert intersection vertices into edge sequences
/// 2. Build graph of edge segments between intersections
/// 3. Extract all simple cycles (loops) via graph traversal
/// 4. Filter by winding number / orientation for true offset semantics
/// This is equivalent to polygon clipping (Vatti algorithm) or
/// using a robust library (Clipper2, GEOS).
fn split_into_loops(verts: &[Point], _intersections: &[Intersection]) -> Vec<Vec<Point>> {
    vec![verts.to_vec()]
}

/// Helper function to check if a point is on a line segment.
fn point_on_segment(p: Point, a: Point, b: Point) -> bool {
    let cross = (p.x - a.x) * (b.y - a.y) - (p.y - a.y) * (b.x - a.x);
    if cross.abs() > 1e-10 {
        return false;
    }
    let dot = (p.x - a.x) * (p.x - b.x) + (p.y - a.y) * (p.y - b.y);
    dot <= 1e-10
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rectangle_area() {
        let polygon = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 5.0),
            Point::new(0.0, 5.0),
        ]);

        assert_eq!(polygon.area(), 50.0);
    }

    #[test]
    fn triangle_area() {
        let polygon = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(4.0, 0.0),
            Point::new(0.0, 3.0),
        ]);

        assert_eq!(polygon.area(), 6.0);
    }

    #[test]
    fn rectangle_centroid() {
        let polygon = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 5.0),
            Point::new(0.0, 5.0),
        ]);

        let centroid = polygon.centroid();

        assert!((centroid.x - 5.0).abs() < 1e-12);
        assert!((centroid.y - 2.5).abs() < 1e-12);
    }

    #[test]
    fn triangle_centroid() {
        let polygon = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(4.0, 0.0),
            Point::new(0.0, 3.0),
        ]);

        let centroid = polygon.centroid();

        assert!((centroid.x - 4.0 / 3.0).abs() < 1e-12);
        assert!((centroid.y - 1.0).abs() < 1e-12);
    }

    #[test]
    fn clockwise_polygon_has_same_centroid() {
        let polygon = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(0.0, 5.0),
            Point::new(10.0, 5.0),
            Point::new(10.0, 0.0),
        ]);

        let centroid = polygon.centroid();

        assert!((centroid.x - 5.0).abs() < 1e-12);
        assert!((centroid.y - 2.5).abs() < 1e-12);
    }

    #[test]
    fn rectangle_global_moments_of_inertia() {
        let polygon = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 5.0),
            Point::new(0.0, 5.0),
        ]);

        let ix = polygon.moment_of_inertia_x();
        let iy = polygon.moment_of_inertia_y();
        let ixy = polygon.product_of_inertia_xy();

        assert!((ix - 416.6666666666667).abs() < 1e-10);
        assert!((iy - 1666.6666666666667).abs() < 1e-10);
        assert!((ixy - 625.0).abs() < 1e-10);
    }

    #[test]
    fn rectangle_centroidal_moments_of_inertia() {
        let polygon = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 5.0),
            Point::new(0.0, 5.0),
        ]);

        let ix = polygon.centroidal_moment_of_inertia_x();
        let iy = polygon.centroidal_moment_of_inertia_y();
        let ixy = polygon.centroidal_product_of_inertia_xy();

        assert!((ix - 104.16666666666667).abs() < 1e-10);
        assert!((iy - 416.6666666666667).abs() < 1e-10);
        assert!(ixy.abs() < 1e-10);
    }

    #[test]
    fn triangle_centroidal_moments_of_inertia() {
        let polygon = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(4.0, 0.0),
            Point::new(0.0, 3.0),
        ]);

        let ix = polygon.centroidal_moment_of_inertia_x();
        let iy = polygon.centroidal_moment_of_inertia_y();
        let ixy = polygon.centroidal_product_of_inertia_xy();

        assert!((ix - 3.0).abs() < 1e-10);
        assert!((iy - 5.333333333333333).abs() < 1e-10);
        assert!((ixy + 2.0).abs() < 1e-10);
    }

    #[test]
    fn boundary_extrema_rectangle() {
        use super::super::boundary::BoundaryExtrema;

        // 10×5 rectangle at origin.
        let rect = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 5.0),
            Point::new(0.0, 5.0),
        ]);
        let center = Point::new(5.0, 2.5);

        let (y_min, y_max) = rect.extreme_y(center);
        assert!((y_min + 2.5).abs() < 1e-12);
        assert!((y_max - 2.5).abs() < 1e-12);

        let (x_min, x_max) = rect.extreme_x(center);
        assert!((x_min + 5.0).abs() < 1e-12);
        assert!((x_max - 5.0).abs() < 1e-12);
    }

    #[test]
    fn boundary_extrema_rotated_direction() {
        use super::super::boundary::BoundaryExtrema;

        // Unit square at origin.
        let sq = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(1.0, 1.0),
            Point::new(0.0, 1.0),
        ]);
        let center = Point::new(0.5, 0.5);

        // 45-degree direction: (cos45, sin45).
        let dir = Point::new(std::f64::consts::FRAC_1_SQRT_2, std::f64::consts::FRAC_1_SQRT_2);
        let (lo, hi) = sq.extreme_distances(center, dir);
        // Extreme points are (0,0) and (1,1); distances from center are ±√2/2.
        let expected = std::f64::consts::FRAC_1_SQRT_2;
        assert!((lo + expected).abs() < 1e-12);
        assert!((hi - expected).abs() < 1e-12);
    }

    #[test]
    fn has_self_intersections_simple_rect() {
        let rect = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 5.0),
            Point::new(0.0, 5.0),
        ]);
        assert!(!rect.has_self_intersections());
    }

    #[test]
    fn has_self_intersections_bowtie() {
        // Bow-tie with non-zero area: (0,0)-(10,0)-(0,10)-(8,8)
        // Edge 1 (10,0)-(0,10) crosses Edge 3 (8,8)-(0,0)
        let bowtie = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(0.0, 10.0),
            Point::new(8.0, 8.0),
        ]);
        assert!(bowtie.has_self_intersections());
    }

    #[test]
    fn has_self_intersections_hourglass() {
        // Hourglass: (0,0)-(10,0)-(5,5)-(10,10)-(0,10)
        // Edge 1 (10,0)-(5,5) crosses Edge 4 (0,10)-(0,0) ? No
        // Actually: Edge 0 (0,0)-(10,0) and Edge 2 (5,5)-(10,10) are parallel
        // Edge 1 (10,0)-(5,5) and Edge 3 (10,10)-(0,10) don't cross in interiors
        // This hourglass doesn't self-intersect in interior.
        // Use a proper crossing: (0,0)-(10,0)-(5,5)-(10,10)-(0,10)
        // Wait, this doesn't cross. Let me use a proper self-intersecting pentagon.
        // (0,0) -> (10,0) -> (5,10) -> (10,10) -> (0,10)
        // Edges: 0:(0,0)-(10,0), 1:(10,0)-(5,10), 2:(5,10)-(10,10), 3:(10,10)-(0,10), 4:(0,10)-(0,0)
        // Check 0 vs 2: (0,0)-(10,0) and (5,10)-(10,10) - parallel
        // Check 1 vs 3: (10,0)-(5,10) and (10,10)-(0,10) - cross?
        // dx1=-5, dy1=10; dx2=-10, dy2=0
        // denom = 50; t_a=2 (outside)
        // Hmm, let's use a simpler self-intersecting shape.
        // Triangle with a "spike" that crosses: (0,0)-(10,0)-(0,10)-(5,5)
        // Wait that's 4 vertices. Let's use: (0,0)-(10,0)-(0,10)-(8,8) which we know works.
        let hourglass = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(0.0, 10.0),
            Point::new(8.0, 8.0),
        ]);
        assert!(hourglass.has_self_intersections());
    }

    #[test]
    fn try_new_valid_polygon() {
        let verts = vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 5.0),
            Point::new(0.0, 5.0),
        ];
        let poly = Polygon::try_new(verts).unwrap();
        assert_eq!(poly.area(), 50.0);
        assert!(!poly.has_self_intersections());
    }

#[test]
    fn try_new_rejects_bowtie() {
        // Same bow-tie as above: (0,0)-(10,0)-(0,10)-(8,8)
        let verts = vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(0.0, 10.0),
            Point::new(8.0, 8.0),
        ];
        let err = Polygon::try_new(verts).unwrap_err();
        assert!(matches!(err, PolygonError::SelfIntersection));
    }

    #[test]
    fn try_new_rejects_too_few_vertices() {
        let err = Polygon::try_new(vec![Point::new(0.0, 0.0), Point::new(1.0, 0.0)]).unwrap_err();
        assert!(matches!(err, PolygonError::TooFewVertices));
    }

    #[test]
    fn try_new_rejects_zero_area() {
        let verts = vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(2.0, 0.0), // Collinear = zero area
        ];
        let err = Polygon::try_new(verts).unwrap_err();
        assert!(matches!(err, PolygonError::ZeroArea));
    }

    #[test]
    fn try_new_rejects_non_finite() {
        let verts = vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(f64::NAN, 1.0),
        ];
        let err = Polygon::try_new(verts).unwrap_err();
        assert!(matches!(err, PolygonError::NonFiniteVertex(2)));
    }

    #[test]
    fn try_new_rejects_duplicate_vertices() {
        let verts = vec![
            Point::new(0.0, 0.0),
            Point::new(0.0, 0.0), // Duplicate
            Point::new(0.0, 0.0), // Another duplicate
            Point::new(0.0, 0.0), // Another duplicate
        ];
        let err = Polygon::try_new(verts).unwrap_err();
        assert!(matches!(err, PolygonError::TooFewDistinctVertices));
    }
}
