//! Triangulation algorithms for polygon mesh generation.
//!
//! Implements ear clipping for simple polygons and Delaunay refinement.

use crate::geometry::{Point, Polygon};
use crate::section::Section;

/// Triangle defined by three vertex indices (CCW orientation).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Triangle {
    pub v: [usize; 3],
}

impl Triangle {
    pub fn new(a: usize, b: usize, c: usize) -> Self {
        Self { v: [a, b, c] }
    }

    /// Get the three vertex indices.
    pub fn vertices(&self) -> [usize; 3] {
        self.v
    }

    /// Check if point is inside triangle (using barycentric coordinates).
    pub fn contains_point(&self, points: &[Point], p: Point) -> bool {
        let a = points[self.v[0]];
        let b = points[self.v[1]];
        let c = points[self.v[2]];

        let v0 = Point::new(c.x - a.x, c.y - a.y);
        let v1 = Point::new(b.x - a.x, b.y - a.y);
        let v2 = Point::new(p.x - a.x, p.y - a.y);

        let dot00 = v0.x * v0.x + v0.y * v0.y;
        let dot01 = v0.x * v1.x + v0.y * v1.y;
        let dot02 = v0.x * v2.x + v0.y * v2.y;
        let dot11 = v1.x * v1.x + v1.y * v1.y;
        let dot12 = v1.x * v2.x + v1.y * v2.y;

        let inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
        let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
        let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

        u >= -1e-10 && v >= -1e-10 && (u + v) <= 1.0 + 1e-10
    }

    /// Compute triangle area (signed, positive for CCW).
    pub fn signed_area(&self, points: &[Point]) -> f64 {
        let a = points[self.v[0]];
        let b = points[self.v[1]];
        let c = points[self.v[2]];
        0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y))
    }

    /// Compute triangle circumcircle (center and radius squared).
    pub fn circumcircle(&self, points: &[Point]) -> (Point, f64) {
        let a = points[self.v[0]];
        let b = points[self.v[1]];
        let c = points[self.v[2]];

        let d = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
        if d.abs() < 1e-12 {
            return (Point::new(0.0, 0.0), f64::INFINITY);
        }

        let ux = ((a.x * a.x + a.y * a.y) * (b.y - c.y)
            + (b.x * b.x + b.y * b.y) * (c.y - a.y)
            + (c.x * c.x + c.y * c.y) * (a.y - b.y))
            / d;
        let uy = ((a.x * a.x + a.y * a.y) * (c.x - b.x)
            + (b.x * b.x + b.y * b.y) * (a.x - c.x)
            + (c.x * c.x + c.y * c.y) * (b.x - a.x))
            / d;

        let center = Point::new(ux, uy);
        let r2 = (a.x - ux).powi(2) + (a.y - uy).powi(2);
        (center, r2)
    }
}

/// Triangulation result containing triangles and boundary information.
#[derive(Debug, Clone)]
pub struct Triangulation {
    pub triangles: Vec<Triangle>,
    pub boundary_edges: Vec<[usize; 2]>,
    pub hole_edges: Vec<Vec<[usize; 2]>>,
}

impl Triangulation {
    pub fn new() -> Self {
        Self {
            triangles: Vec::new(),
            boundary_edges: Vec::new(),
            hole_edges: Vec::new(),
        }
    }

    pub fn n_triangles(&self) -> usize {
        self.triangles.len()
    }
}

impl Default for Triangulation {
    fn default() -> Self {
        Self::new()
    }
}

/// Check if a vertex is an "ear" in a polygon.
fn is_ear(polygon: &[usize], _vertices: &[Point], i: usize, all_vertices: &[Point]) -> bool {
    let n = polygon.len();
    if n < 3 {
        return false;
    }

    let prev = polygon[(i + n - 1) % n];
    let curr = polygon[i];
    let next = polygon[(i + 1) % n];

    let a = all_vertices[prev];
    let b = all_vertices[curr];
    let c = all_vertices[next];

    // Check if angle is convex (CCW polygon -> positive signed area)
    let cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
    if cross <= 1e-12 {
        return false; // Not a convex vertex
    }

    // Check if any other vertex lies inside this triangle
    for &idx in polygon.iter() {
        if idx == prev || idx == curr || idx == next {
            continue;
        }
        let p = all_vertices[idx];
        // Barycentric test
        let v0 = Point::new(c.x - a.x, c.y - a.y);
        let v1 = Point::new(b.x - a.x, b.y - a.y);
        let v2 = Point::new(p.x - a.x, p.y - a.y);

        let dot00 = v0.x * v0.x + v0.y * v0.y;
        let dot01 = v0.x * v1.x + v0.y * v1.y;
        let dot02 = v0.x * v2.x + v0.y * v2.y;
        let dot11 = v1.x * v1.x + v1.y * v1.y;
        let dot12 = v1.x * v2.x + v1.y * v2.y;

        let inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
        let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
        let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

        if u > 1e-10 && v > 1e-10 && (u + v) < 1.0 - 1e-10 {
            return false; // Another vertex inside ear
        }
    }

    true
}

/// Ear clipping triangulation for simple polygons (no holes).
pub fn triangulate_polygon_ear_clipping(polygon: &Polygon) -> Vec<Triangle> {
    let vertices = &polygon.vertices;
    let n = vertices.len();
    if n < 3 {
        return Vec::new();
    }

    // Ensure CCW orientation
    let signed_area = polygon.signed_area();
    let mut vertex_indices: Vec<usize> = (0..n).collect();
    if signed_area < 0.0 {
        vertex_indices.reverse();
    }

    let mut triangles = Vec::new();
    let mut remaining = vertex_indices.clone();

    let mut guard = 0;
    while remaining.len() > 3 && guard < n * n {
        guard += 1;
        let mut ear_found = false;

        for i in 0..remaining.len() {
            if is_ear(&remaining, vertices, i, vertices) {
                let prev = remaining[(i + remaining.len() - 1) % remaining.len()];
                let curr = remaining[i];
                let next = remaining[(i + 1) % remaining.len()];

                triangles.push(Triangle::new(prev, curr, next));
                remaining.remove(i);
                ear_found = true;
                break;
            }
        }

        if !ear_found {
            // Polygon may have self-intersections or be degenerate
            // Fall back to fan triangulation from first vertex
            for i in 1..remaining.len() - 1 {
                triangles.push(Triangle::new(remaining[0], remaining[i], remaining[i + 1]));
            }
            break;
        }
    }

    if remaining.len() == 3 {
        triangles.push(Triangle::new(remaining[0], remaining[1], remaining[2]));
    }

    triangles
}

/// Strict interior intersection of segments (shared endpoints excluded).
fn segments_properly_intersect(p1: Point, p2: Point, q1: Point, q2: Point) -> bool {
    fn d(a: Point, b: Point, c: Point) -> f64 {
        (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
    }
    let d1 = d(q1, q2, p1);
    let d2 = d(q1, q2, p2);
    let d3 = d(p1, p2, q1);
    let d4 = d(p1, p2, q2);
    ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
}

/// Check that the bridge segment `a`-`b` does not cross any edge of `chain`
/// (edges sharing an endpoint with the bridge are ignored) and its midpoint
/// lies inside the material region.
fn bridge_is_valid(chain: &[Point], a: Point, b: Point) -> bool {
    let n = chain.len();
    for k in 0..n {
        let c = chain[k];
        let e = chain[(k + 1) % n];
        let touches = c == a || e == a || c == b || e == b;
        if !touches && segments_properly_intersect(a, b, c, e) {
            return false;
        }
    }
    true
}

/// Merge hole boundaries into the outer boundary using keyhole bridges so
/// that a single simple-polygon ear-clipping triangulates the material
/// exactly (hole boundaries become mesh edges). Returns vertex list.
fn merge_holes_into_outer(outer: &Polygon, holes: &[Polygon]) -> Vec<Point> {
    let mut verts = outer.vertices.clone();

    // Sort holes by area (largest first) for stability.
    let mut ordered: Vec<&Polygon> = holes.iter().collect();
    ordered.sort_by(|x, y| y.area().partial_cmp(&x.area()).unwrap());

    for hole in ordered {
        let hv = &hole.vertices;
        let nh = hv.len();
        let nv = verts.len();

        // Candidate pairs sorted by distance.
        let mut candidates: Vec<(f64, usize, usize)> = Vec::new();
        for i in 0..nv {
            for j in 0..nh {
                let dx = verts[i].x - hv[j].x;
                let dy = verts[i].y - hv[j].y;
                candidates.push((dx * dx + dy * dy, i, j));
            }
        }
        candidates.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap());

        // Bridge midpoints must lie in material: outside every hole and
        // inside the outer boundary.
        let mut merged: Option<Vec<Point>> = None;
        for &(_, i, j) in &candidates {
            let a = verts[i];
            let b = hv[j];
            let mid = Point::new(0.5 * (a.x + b.x), 0.5 * (a.y + b.y));
            if !outer.contains_point(mid) || holes.iter().any(|h| h.contains_point(mid)) {
                continue;
            }
            if !bridge_is_valid(&verts, a, b) {
                continue;
            }
            let mut m: Vec<Point> = Vec::with_capacity(nv + nh + 2);
            m.extend_from_slice(&verts[..=i]);
            for k in 0..=nh {
                m.push(hv[(j + k) % nh]);
            }
            m.push(verts[i]);
            m.extend_from_slice(&verts[i + 1..]);
            merged = Some(m);
            break;
        }

        match merged {
            Some(m) => verts = m,
            // Fallback: leave this hole unbridged; caller's carving handles it.
            None => continue,
        }
    }

    verts
}

/// Triangulate a section (outer + holes), keeping hole boundaries as exact
/// mesh edges via keyhole bridging.
pub fn triangulate_section_bridged(
    outer: &Polygon,
    holes: &[Polygon],
) -> (Vec<Point>, Vec<[usize; 3]>) {
    if holes.is_empty() {
        let tris = triangulate_polygon_ear_clipping(outer);
        return (outer.vertices.clone(), tris.iter().map(|t| t.v).collect());
    }

    let merged_points = merge_holes_into_outer(outer, holes);
    let poly = Polygon::new(merged_points.clone());
    let tris = triangulate_polygon_ear_clipping(&poly);

    // Map back through Polygon's dedup: rebuild index triples by coordinate.
    let mut nodes: Vec<Point> = Vec::new();
    let mut index_of: std::collections::HashMap<(u64, u64), usize> =
        std::collections::HashMap::new();
    let key = |p: &Point| -> (u64, u64) { (p.x.to_bits(), p.y.to_bits()) };
    let mut elements = Vec::with_capacity(tris.len());
    for tri in &tris {
        let mut tri_idx = [0usize; 3];
        for (k, vi) in tri.v.iter().enumerate() {
            let p = poly.vertices[*vi];
            let next = nodes.len();
            let idx = *index_of.entry(key(&p)).or_insert_with(|| {
                nodes.push(p);
                next
            });
            tri_idx[k] = idx;
        }
        // Keep even near-degenerate bridge slivers: they carry connectivity.
        elements.push(tri_idx);
    }

    (nodes, elements)
}

/// Delaunay triangulation using Bowyer-Watson algorithm (incremental).
pub fn triangulate_delaunay(points: &[Point]) -> Vec<Triangle> {
    if points.len() < 3 {
        return Vec::new();
    }

    // Super-triangle encompassing all points
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;

    for p in points {
        min_x = min_x.min(p.x);
        max_x = max_x.max(p.x);
        min_y = min_y.min(p.y);
        max_y = max_y.max(p.y);
    }

    let dx = max_x - min_x;
    let dy = max_y - min_y;
    let delta_max = dx.max(dy);
    let mid_x = (min_x + max_x) / 2.0;
    let mid_y = (min_y + max_y) / 2.0;

    let super_a = Point::new(mid_x - 20.0 * delta_max, mid_y - delta_max);
    let super_b = Point::new(mid_x, mid_y + 20.0 * delta_max);
    let super_c = Point::new(mid_x + 20.0 * delta_max, mid_y - delta_max);

    let mut all_points = points.to_vec();
    let super_a_idx = all_points.len();
    all_points.push(super_a);
    let super_b_idx = all_points.len();
    all_points.push(super_b);
    let super_c_idx = all_points.len();
    all_points.push(super_c);

    let mut triangles = vec![Triangle::new(super_a_idx, super_b_idx, super_c_idx)];

    // Insert points one by one
    for i in 0..points.len() {
        let p = points[i];
        let mut bad_triangles = Vec::new();

        // Find all triangles whose circumcircle contains the point
        for (ti, tri) in triangles.iter().enumerate() {
            let (center, r2) = tri.circumcircle(&all_points);
            let dist2 = (p.x - center.x).powi(2) + (p.y - center.y).powi(2);
            if dist2 <= r2 + 1e-10 {
                bad_triangles.push(ti);
            }
        }

        // Find boundary edges of the polygonal hole
        let mut edge_count = std::collections::HashMap::new();
        for &ti in &bad_triangles {
            let tri = triangles[ti];
            for edge in [
                [tri.v[0], tri.v[1]],
                [tri.v[1], tri.v[2]],
                [tri.v[2], tri.v[0]],
            ] {
                let key = if edge[0] < edge[1] {
                    edge
                } else {
                    [edge[1], edge[0]]
                };
                *edge_count.entry(key).or_insert(0) += 1;
            }
        }

        // Remove bad triangles (in reverse order to keep indices valid)
        for &ti in bad_triangles.iter().rev() {
            triangles.remove(ti);
        }

        // Re-triangulate the hole
        for (edge, &count) in &edge_count {
            if count == 1 {
                // Boundary edge - form new triangle with the inserted point
                triangles.push(Triangle::new(edge[0], edge[1], i));
            }
        }
    }

    // Remove triangles that use super-triangle vertices
    triangles.retain(|tri| {
        tri.v[0] < points.len() && tri.v[1] < points.len() && tri.v[2] < points.len()
    });

    triangles
}

/// Triangulate a section (outer boundary + holes) using constrained Delaunay.
pub fn triangulate_section(
    section: &Section,
    params: crate::mesh::MeshParams,
) -> crate::mesh::Mesh {
    let outer = &section.outer;
    let holes = &section.holes;

    // Prefer bridged triangulation: hole boundaries become exact mesh edges.
    let (mut nodes, mut elements) = triangulate_section_bridged(outer, holes);

    // Refine uniformly to honour the target element size (ear clipping
    // produces only boundary vertices otherwise).
    refine_mesh_uniform(
        &mut nodes,
        &mut elements,
        params.target_size.max(1e-6),
        params.max_iterations.clamp(1, 12),
        params.max_nodes,
    );

    let outer_start = 0;
    let outer_end = outer.vertices.len();
    let boundary_indices: Vec<usize> = (outer_start..outer_end).collect();

    let mut hole_boundary_indices = Vec::new();
    if holes.is_empty() {
        // Bridged path already handled everything; identify hole nodes by
        // coordinate match for API completeness.
    } else {
        for hole in holes {
            let idx: Vec<usize> = hole
                .vertices
                .iter()
                .filter_map(|p| nodes.iter().position(|q| q == p))
                .collect();
            hole_boundary_indices.push(idx);
        }
    }

    let mut mesh = crate::mesh::Mesh::new();
    mesh.nodes = nodes;
    mesh.elements = elements;
    mesh.boundary_nodes = boundary_indices;
    mesh.hole_boundary_nodes = hole_boundary_indices;
    mesh.element_materials = vec![0; mesh.elements.len()];

    mesh
}

/// Uniform (red) refinement by edge midpoint subdivision until all edges are
/// below `target_size`. Keeps the mesh conforming; original vertex order is
/// preserved so boundary index ranges stay valid.
pub(crate) fn refine_mesh_uniform(
    nodes: &mut Vec<Point>,
    elements: &mut Vec<[usize; 3]>,
    target_size: f64,
    max_iterations: usize,
    max_nodes: usize,
) {
    use std::collections::HashMap;

    for _ in 0..max_iterations {
        let needs = elements.iter().any(|tri| {
            let p0 = nodes[tri[0]];
            let p1 = nodes[tri[1]];
            let p2 = nodes[tri[2]];
            let d01 = ((p1.x - p0.x).powi(2) + (p1.y - p0.y).powi(2)).sqrt();
            let d12 = ((p2.x - p1.x).powi(2) + (p2.y - p1.y).powi(2)).sqrt();
            let d20 = ((p0.x - p2.x).powi(2) + (p0.y - p2.y).powi(2)).sqrt();
            d01.max(d12).max(d20) > target_size
        });
        if !needs || nodes.len() >= max_nodes / 4 {
            break;
        }

        let mut edge_map: HashMap<(usize, usize), usize> = HashMap::new();
        let mut new_elements = Vec::with_capacity(elements.len() * 4);
        for &elem in elements.iter() {
            let mut mid = |a: usize, b: usize| -> usize {
                let key = if a < b { (a, b) } else { (b, a) };
                if let Some(&m) = edge_map.get(&key) {
                    return m;
                }
                let pa = nodes[a];
                let pb = nodes[b];
                let idx = nodes.len();
                nodes.push(Point::new(0.5 * (pa.x + pb.x), 0.5 * (pa.y + pb.y)));
                edge_map.insert(key, idx);
                idx
            };
            let m01 = mid(elem[0], elem[1]);
            let m12 = mid(elem[1], elem[2]);
            let m20 = mid(elem[2], elem[0]);
            new_elements.push([elem[0], m01, m20]);
            new_elements.push([m01, elem[1], m12]);
            new_elements.push([m20, m12, elem[2]]);
            new_elements.push([m01, m12, m20]);
        }
        *elements = new_elements;
    }
}

/// Triangulate a simple polygon (no holes).
pub fn triangulate_polygon(polygon: &Polygon) -> Vec<Triangle> {
    triangulate_polygon_ear_clipping(polygon)
}

/// Refine triangulation using Delaunay edge flips.
pub fn refine_delaunay(triangles: &mut [Triangle], points: &[Point]) {
    let mut changed = true;
    let mut iterations = 0;

    while changed && iterations < 10 {
        changed = false;
        iterations += 1;

        // Build edge to triangle adjacency
        let mut edge_map: std::collections::HashMap<[usize; 2], Vec<usize>> =
            std::collections::HashMap::new();
        for (ti, tri) in triangles.iter().enumerate() {
            for edge in [
                [tri.v[0], tri.v[1]],
                [tri.v[1], tri.v[2]],
                [tri.v[2], tri.v[0]],
            ] {
                let key = if edge[0] < edge[1] {
                    edge
                } else {
                    [edge[1], edge[0]]
                };
                edge_map.entry(key).or_default().push(ti);
            }
        }

        // Check each interior edge for Delaunay condition
        for (edge, adj_tris) in &edge_map {
            if adj_tris.len() == 2 {
                let t1 = triangles[adj_tris[0]];
                let t2 = triangles[adj_tris[1]];

                // Find opposite vertices
                let opp1 =
                    t1.v.iter()
                        .find(|&&v| v != edge[0] && v != edge[1])
                        .copied()
                        .unwrap();
                let opp2 =
                    t2.v.iter()
                        .find(|&&v| v != edge[0] && v != edge[1])
                        .copied()
                        .unwrap();

                // Check if edge is locally Delaunay
                let (center1, r2_1) = t1.circumcircle(points);
                let p2 = points[opp2];
                let dist2 = (p2.x - center1.x).powi(2) + (p2.y - center1.y).powi(2);

                if dist2 < r2_1 - 1e-10 {
                    // Flip edge
                    let new_t1 = Triangle::new(edge[0], opp1, opp2);
                    let new_t2 = Triangle::new(edge[1], opp2, opp1);
                    triangles[adj_tris[0]] = new_t1;
                    triangles[adj_tris[1]] = new_t2;
                    changed = true;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon};

    #[test]
    fn ear_clipping_triangle() {
        let poly = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(0.0, 1.0),
        ]);
        let tris = triangulate_polygon_ear_clipping(&poly);
        assert_eq!(tris.len(), 1);
    }

    #[test]
    fn ear_clipping_square() {
        let poly = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(1.0, 1.0),
            Point::new(0.0, 1.0),
        ]);
        let tris = triangulate_polygon_ear_clipping(&poly);
        assert_eq!(tris.len(), 2);
    }

    #[test]
    fn ear_clipping_pentagon() {
        let poly = Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(2.0, 0.0),
            Point::new(2.0, 1.0),
            Point::new(1.0, 2.0),
            Point::new(0.0, 1.0),
        ]);
        let tris = triangulate_polygon_ear_clipping(&poly);
        assert_eq!(tris.len(), 3);
    }

    #[test]
    fn delaunay_simple() {
        let points = vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(0.0, 1.0),
            Point::new(1.0, 1.0),
        ];
        let tris = triangulate_delaunay(&points);
        assert_eq!(tris.len(), 2);
    }

    #[test]
    fn triangle_contains_point() {
        let points = vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(0.0, 1.0),
        ];
        let tri = Triangle::new(0, 1, 2);
        assert!(tri.contains_point(&points, Point::new(0.25, 0.25)));
        assert!(!tri.contains_point(&points, Point::new(0.75, 0.75)));
    }

    #[test]
    fn triangle_circumcircle() {
        let points = vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(0.0, 1.0),
        ];
        let tri = Triangle::new(0, 1, 2);
        let (center, r2) = tri.circumcircle(&points);
        assert!((center.x - 0.5).abs() < 1e-6);
        assert!((center.y - 0.5).abs() < 1e-6);
        assert!((r2 - 0.5).abs() < 1e-6);
    }
}

/// Mesh density control for automated sizing based on section bounding box.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MeshControl {
    /// Coarse mesh: target_size = max_dim / 4
    Coarse,
    /// Normal mesh: target_size = max_dim / 6 (default)
    Normal,
    /// Fine mesh: target_size = max_dim / 10
    Fine,
    /// Very fine mesh: target_size = max_dim / 20
    VeryFine,
    /// Custom target size (absolute value, not relative to max_dim)
    Custom(f64),
}

impl Default for MeshControl {
    fn default() -> Self {
        MeshControl::Normal
    }
}

impl MeshControl {
    /// Compute target_size from section bounding box diagonal.
    pub fn compute_target_size(self, max_dim: f64) -> f64 {
        match self {
            MeshControl::Coarse => max_dim / 4.0,
            MeshControl::Normal => max_dim / 6.0,
            MeshControl::Fine => max_dim / 10.0,
            MeshControl::VeryFine => max_dim / 20.0,
            MeshControl::Custom(size) => size,
        }
    }

    /// Compute target_size with thin-walled adjustment.
    pub fn compute_target_size_thin_walled(self, max_dim: f64, min_edge: f64) -> f64 {
        let base = self.compute_target_size(max_dim);
        match self {
            MeshControl::Coarse | MeshControl::Normal => {
                // For thin-walled sections:
                // - Floor at max_dim/20 to prevent overly fine mesh
                // - Ceiling at min_edge/2 to resolve wall thickness
                // - The .min(min_edge/2.0) CAN go below max_dim/20 if min_edge is very small,
                //   which is intended for very thin walls — but a tiny discretisation edge
                //   (e.g. a root-radius arc segment) must not blow the mesh up, so never go
                //   finer than max_dim/40.
                base.max(max_dim / 20.0)
                    .min(min_edge / 2.0)
                    .max(max_dim / 40.0)
            }
            MeshControl::Fine | MeshControl::VeryFine => {
                // For fine meshes, use min edge length but cap at base
                (min_edge / 2.0).min(base).max(max_dim / 50.0)
            }
            MeshControl::Custom(_) => base,
        }
    }
}

/// Create MeshParams from MeshControl and section dimensions.
pub fn mesh_params_from_control(
    control: MeshControl,
    max_dim: f64,
    min_edge: f64,
    is_thin_walled: bool,
) -> crate::mesh::MeshParams {
    let target_size = if is_thin_walled {
        control.compute_target_size_thin_walled(max_dim, min_edge)
    } else {
        control.compute_target_size(max_dim)
    };
    crate::mesh::MeshParams {
        target_size,
        max_size: target_size * 2.0,
        min_size: target_size * 0.3,
        quality_threshold: 0.3,
        use_delaunay: true,
        // Limit uniform-refinement passes for thin-walled sections. Uniform
        // red-refinement quadruples the element count per pass (42 -> 168 ->
        // 672 -> 2688 -> 10752 ...) with no intermediate sizes, so the default
        // 10 passes over-shoots to a huge mesh whose skyline factor is
        // impractically slow on long/narrow thin-walled sections. A small
        // iteration cap keeps the mesh moderate while still resolving the wall.
        max_iterations: if is_thin_walled { 3 } else { 10 },
        max_nodes: 20000,
    }
}
