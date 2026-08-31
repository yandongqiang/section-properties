use super::{Point, Polygon};
use crate::section::Section;

/// Axis selector for mirroring.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
    /// Reflect about the x-axis.
    X,
    /// Reflect about the y-axis.
    Y,
}

/// A rigid or affine transformation applied to a [`Geometry`]'s vertices.
///
/// Mirrors `sectionproperties.pre.geometry.Transform`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Transform {
    /// Translate by an offset (dx, dy).
    Translate { dx: f64, dy: f64 },
    /// Rotate counter-clockwise about the origin by `angle` radians.
    Rotate { angle: f64 },
    /// Uniform scale about the origin.
    Scale { factor: f64 },
    /// Reflect about the x-axis (y → -y).
    MirrorX,
    /// Reflect about the y-axis (x → -x).
    MirrorY,
}

impl Transform {
    /// Apply this transform to a point.
    pub fn apply(&self, p: Point) -> Point {
        match *self {
            Transform::Translate { dx, dy } => Point::new(p.x + dx, p.y + dy),
            Transform::Rotate { angle } => {
                let (s, c) = angle.sin_cos();
                Point::new(p.x * c - p.y * s, p.x * s + p.y * c)
            }
            Transform::Scale { factor } => Point::new(p.x * factor, p.y * factor),
            Transform::MirrorX => Point::new(p.x, -p.y),
            Transform::MirrorY => Point::new(-p.x, p.y),
        }
    }
}

/// A single connected region: an outer boundary plus optional holes,
/// optionally followed by a list of transforms.
///
/// Mirrors `sectionproperties.pre.geometry.Geometry`.
#[derive(Debug, Clone)]
pub struct Geometry {
    /// Outer boundary (normalised to CCW).
    pub outer: Polygon,
    /// Hole boundaries (each normalised to CW).
    pub holes: Vec<Polygon>,
    /// Geometric transforms applied in order via [`Geometry::geometries`].
    pub transforms: Vec<Transform>,
}

impl Geometry {
    /// Create a new geometry from an outer polygon and optional holes.
    ///
    /// The outer boundary is normalised to counter-clockwise orientation
    /// and each hole to clockwise orientation, matching the convention used
    /// by [`Section`](crate::section::Section).
    pub fn new(mut outer: Polygon, mut holes: Vec<Polygon>) -> Self {
        if outer.signed_area() < 0.0 {
            outer.vertices.reverse();
        }
        for hole in &mut holes {
            if hole.signed_area() > 0.0 {
                hole.vertices.reverse();
            }
        }
        Self {
            outer,
            holes,
            transforms: Vec::new(),
        }
    }

    /// Create a geometry from outer and hole point loops.
    ///
    /// Mirrors `Geometry.from_points(points, holes=...)`.
    pub fn from_points(points: Vec<Point>, holes: Vec<Vec<Point>>) -> Self {
        Self::new(
            Polygon::new(points),
            holes.into_iter().map(Polygon::new).collect(),
        )
    }

    /// Convert a single-boundary [`Section`](crate::section::Section) into a geometry.
    pub fn from_section(section: &crate::section::Section) -> Self {
        Self::new(section.outer.clone(), section.holes.clone())
    }

    /// Rotate counter-clockwise about the origin by `angle_degrees` degrees.
    ///
    /// Mirrors `Geometry.rotate(angle_degrees, rot_point)`.
    pub fn rotate(mut self, angle_degrees: f64) -> Self {
        self.transforms
            .push(Transform::Rotate { angle: angle_degrees.to_radians() });
        self
    }

    /// Rotate about an explicit point.
    pub fn rotate_about(mut self, angle_degrees: f64, point: Point) -> Self {
        self.transforms.push(Transform::Translate { dx: -point.x, dy: -point.y });
        self.transforms
            .push(Transform::Rotate { angle: angle_degrees.to_radians() });
        self.transforms.push(Transform::Translate { dx: point.x, dy: point.y });
        self
    }

    /// Reflect about the x or y axis.
    ///
    /// Mirrors `Geometry.mirror(axis, mirror_point)`; here reflection is always
    /// about the global axes.
    pub fn mirror(mut self, axis: Axis) -> Self {
        self.transforms.push(match axis {
            Axis::X => Transform::MirrorX,
            Axis::Y => Transform::MirrorY,
        });
        self
    }

    /// Translate by `(dx, dy)`.
    ///
    /// Mirrors `Geometry.shift(x_offset, y_offset)`.
    pub fn shift(mut self, dx: f64, dy: f64) -> Self {
        self.transforms.push(Transform::Translate { dx, dy });
        self
    }

    /// Return a copy translated so that the centroid sits at the origin.
    ///
    /// Mirrors `Geometry.align_center()`.
    pub fn align_center(&self) -> Self {
        let c = self.centroid();
        let mut g = self.clone();
        g.transforms.push(Transform::Translate { dx: -c.x, dy: -c.y });
        g
    }

    /// Translate this geometry so its bounding box aligns with `other`.
    ///
    /// For each axis independently the nearest bounding-box edge is moved onto
    /// the corresponding edge of `other`'s box, matching the behaviour of
    /// `Geometry.align_to(other, pt=(0, 0))`.
    pub fn align_to(&self, other: &Self) -> Self {
        let (l1, r1, b1, t1) = self.bounds();
        let (l2, r2, b2, t2) = other.bounds();

        let dx = if (l1 - l2).abs() < (r1 - r2).abs() { l2 - l1 } else { r2 - r1 };
        let dy = if (b1 - b2).abs() < (t1 - t2).abs() { b2 - b1 } else { t2 - t1 };

        if dx == 0.0 && dy == 0.0 {
            return self.clone();
        }
        let mut g = self.clone();
        g.transforms.push(Transform::Translate { dx, dy });
        g
    }

    /// Offset the geometry by `amount` using mitred corners.
    ///
    /// Positive amounts grow the geometry, negative amounts shrink it. The
    /// outer boundary is offset by `+amount` and each hole by `-amount`.
    /// Returns `None` if any boundary degenerates.
    ///
    /// Mirrors `Geometry.offset(amount)`.
    pub fn offset(&self, amount: f64) -> Option<Self> {
        let outer = self.apply_transforms().outer.offset(amount)?;
        let mut holes = Vec::with_capacity(self.holes.len());
        for h in &self.holes {
            holes.push(h.offset(-amount)?);
        }
        Some(Self { outer, holes, transforms: Vec::new() })
    }

    /// Split the geometry into two halves either side of the line through
    /// points `a` and `b`.
    ///
    /// Mirrors `Geometry.split_section(point_a, point_b)`. Returns
    /// `(below, above)`; each side is `None` when empty.
    pub fn split_section(
        &self,
        a: Point,
        b: Point,
    ) -> (Option<Geometry>, Option<Geometry>) {
        let g = self.apply_transforms();
        let (below_outer, above_outer) = g.outer.split_by_line(a, b);

        // Clip each hole and attach the matching half to its side.
        let mut below_holes = Vec::new();
        let mut above_holes = Vec::new();
        for h in &g.holes {
            let (hb, ha) = h.split_by_line(a, b);
            if let Some(hb) = hb {
                below_holes.push(hb);
            }
            if let Some(ha) = ha {
                above_holes.push(ha);
            }
        }

        let below = below_outer
            .map(|o| Geometry { outer: o, holes: below_holes, transforms: Vec::new() });
        let above = above_outer
            .map(|o| Geometry { outer: o, holes: above_holes, transforms: Vec::new() });
        (below, above)
    }
    /// Boolean union with `other`.
    ///
    /// Mirrors `Geometry | other` (shapely `union`). Operates on the outer
    /// Boolean union with `other`.
    pub fn union(&self, other: &Self) -> Option<Self> {
        self.boolean_with_holes(other, super::boolean::BoolOp::Union)
    }

    /// Boolean intersection with `other`.
    ///
    /// Mirrors `Geometry & other` (shapely `intersection`).
    pub fn intersection(&self, other: &Self) -> Option<Self> {
        self.boolean_with_holes(other, super::boolean::BoolOp::Intersection)
    }

    /// Boolean difference: this geometry minus `other`.
    ///
    /// Mirrors `Geometry - other` (shapely `difference`).
    pub fn subtract(&self, other: &Self) -> Option<Self> {
        self.boolean_with_holes(other, super::boolean::BoolOp::Difference)
    }

    fn boolean_with_holes(&self, other: &Self, op: super::boolean::BoolOp) -> Option<Self> {
        let a = Section::new(self.apply_transforms().outer, self.apply_transforms().holes);
        let b = Section::new(other.apply_transforms().outer, other.apply_transforms().holes);

        let results = match op {
            super::boolean::BoolOp::Union => {
                super::boolean::section_union(&a, &b)
            }
            super::boolean::BoolOp::Intersection => {
                super::boolean::section_intersection(&a, &b)
            }
            super::boolean::BoolOp::Difference => {
                super::boolean::section_difference(&a, &b)
            }
        };

        let best = results
            .into_iter()
            .max_by(|x, y| x.area().partial_cmp(&y.area()).unwrap())?;
        Some(Geometry::new(best.outer, best.holes))
    }

    /// Return a copy of this geometry with the transforms applied to all vertices.
    ///
    /// Transforms are applied in the order they appear in [`Geometry::transforms`].
    pub fn apply_transforms(&self) -> Self {
        if self.transforms.is_empty() {
            return self.clone();
        }
        let map = |poly: &Polygon| -> Polygon {
            Polygon::new(
                poly.vertices
                    .iter()
                    .map(|&v| self.transforms.iter().fold(v, |p, t| t.apply(p)))
                    .collect(),
            )
        };
        Self {
            outer: map(&self.outer),
            holes: self.holes.iter().map(map).collect(),
            transforms: Vec::new(),
        }
    }

    /// Net area of the geometry (outer minus holes), after applying transforms.
    pub fn area(&self) -> f64 {
        let g = if self.transforms.is_empty() {
            self
        } else {
            &self.apply_transforms()
        };
        let mut a = g.outer.area();
        for h in &g.holes {
            a -= h.area();
        }
        a
    }

    /// Centroid of the geometry using the composite area formula.
    pub fn centroid(&self) -> Point {
        let g = if self.transforms.is_empty() {
            self
        } else {
            &self.apply_transforms()
        };
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut total = 0.0;

        let outer_c = g.outer.centroid();
        let outer_a = g.outer.area();
        sum_x += outer_c.x * outer_a;
        sum_y += outer_c.y * outer_a;
        total += outer_a;

        for h in &g.holes {
            let a = h.area();
            let c = h.centroid();
            sum_x -= c.x * a;
            sum_y -= c.y * a;
            total -= a;
        }

        Point::new(sum_x / total, sum_y / total)
    }

    /// Bounding box of the geometry: (min_x, max_x, min_y, max_y).
    pub fn bounds(&self) -> (f64, f64, f64, f64) {
        let g = if self.transforms.is_empty() {
            self
        } else {
            &self.apply_transforms()
        };
        let mut min_x = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        for poly in std::iter::once(&g.outer).chain(g.holes.iter()) {
            for v in &poly.vertices {
                min_x = min_x.min(v.x);
                max_x = max_x.max(v.x);
                min_y = min_y.min(v.y);
                max_y = max_y.max(v.y);
            }
        }
        (min_x, max_x, min_y, max_y)
    }

    /// Perimeter of the geometry (outer + holes), after applying transforms.
    pub fn perimeter(&self) -> f64 {
        let g = if self.transforms.is_empty() {
            self
        } else {
            &self.apply_transforms()
        };
        g.outer.perimeter() + g.holes.iter().map(|h| h.perimeter()).sum::<f64>()
    }
}

/// A collection of independent regions, e.g. two parallel rectangles,
/// an angle bolted to a plate, or a built-up girder.
///
/// Mirrors `sectionproperties.pre.geometry.CompoundGeometry`.
///
/// # Invariant
/// Regions are assumed to be **material-disjoint**: no part of one region's
/// material may overlap another region's material. Properties (`area`,
/// inertia, ...) are computed by simple summation, which is only correct
/// under this assumption.
///
/// Use [`CompoundGeometry::validate`] to check the invariant, and
    /// [`CompoundGeometry::dissolved_outer_only`] to merge overlapping inputs into a valid
    /// compound.
#[derive(Debug, Clone)]
pub struct CompoundGeometry {
    /// The individual regions that make up the compound section.
    pub geometries: Vec<Geometry>,
}

/// Errors from compound-geometry topology validation.
#[derive(Debug, Clone, PartialEq)]
pub enum CompoundError {
    /// Material of region `a` overlaps material of region `b`.
    OverlappingRegions { a: usize, b: usize },
    /// Hole `hole` of `region` is not fully inside the region's outer
    /// boundary.
    HoleNotInsideOuter { region: usize, hole: usize },
    /// Hole `b` of `region` is nested inside hole `a` (a hole-within-a-hole).
    /// A single region cannot represent an island of material inside a void,
    /// so nested holes are rejected.
    NestedHoles { region: usize, a: usize, b: usize },
}

impl std::fmt::Display for CompoundError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CompoundError::OverlappingRegions { a, b } => {
                write!(f, "regions {a} and {b} have overlapping material")
            }
            CompoundError::HoleNotInsideOuter { region, hole } => {
                write!(f, "hole {hole} of region {region} is not fully inside its outer boundary")
            }
            CompoundError::NestedHoles { region, a, b } => {
                write!(f, "region {region} has hole {b} nested inside hole {a} (holes cannot nest)")
            }
        }
    }
}

impl std::error::Error for CompoundError {}

/// Bounding-box relation between two polygons for quick rejection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BBoxRelation {
    /// No overlap, not even touching
    Disjoint,
    /// Touching at boundary but no area overlap
    Touch,
    /// Positive area overlap
    Overlap,
}

/// Quick bounding-box relation test for two polygons.
/// Returns Disjoint if separated, Touch if only boundaries touch, Overlap if interiors overlap.
fn bbox_relation(a: &Polygon, b: &Polygon) -> BBoxRelation {
    let bbox = |pts: &[Point]| {
        pts.iter().fold(
            (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY),
            |(l, r, b, t), p| (l.min(p.x), r.max(p.x), b.min(p.y), t.max(p.y)),
        )
    };
    let (al, ar, ab, at) = bbox(&a.vertices);
    let (bl, br, bb, bt) = bbox(&b.vertices);

    // Check for disjoint (no overlap, not even touching)
    if ar < bl || br < al || at < bb || bt < ab {
        return BBoxRelation::Disjoint;
    }

    // Check for touch (boundaries touch but interiors don't overlap)
    // This means at least one dimension has equality at the boundary
    let x_touch = ar == bl || br == al;
    let y_touch = at == bb || bt == ab;
    if x_touch || y_touch {
        return BBoxRelation::Touch;
    }

    // Otherwise there is positive area overlap
    BBoxRelation::Overlap
}

impl CompoundGeometry {
    /// Create a compound geometry from its constituent regions.
    ///
    /// # Invariant
    /// The caller guarantees that region materials are **disjoint**.
    /// Overlapping inputs silently double-count area/inertia in property
    /// computations. Use [`CompoundGeometry::validate`] to check, or
    /// [`CompoundGeometry::dissolved_outer_only`] to merge overlapping inputs.
    pub fn new(geometries: Vec<Geometry>) -> Self {
        Self { geometries }
    }

    /// Validating constructor: returns an error if any two regions have
    /// overlapping material or a hole escapes its outer boundary.
    pub fn new_validated(geometries: Vec<Geometry>) -> Result<Self, CompoundError> {
        let compound = Self { geometries };
        compound.validate()?;
        Ok(compound)
    }

    /// Merge overlapping inputs into a valid (material-disjoint) compound by
    /// dissolving overlaps with the boolean union. **Holes are dropped during
    /// dissolution** (the merged outline is exact; re-derive holes afterwards
    /// if required). This is a union of outer boundaries only.
    ///
    /// Prefer [`Self::dissolved_outer_only`] for explicit naming.
    #[deprecated(since = "0.1.0", note = "Use dissolved_outer_only() for explicit naming; holes are dropped")]
    pub fn dissolved(geometries: Vec<Geometry>) -> Self {
        Self::dissolved_outer_only(geometries)
    }

    /// Merge overlapping inputs into a valid (material-disjoint) compound by
    /// dissolving overlaps with the boolean union. **Holes are dropped** –
    /// only outer boundaries are united. This is a union of outer boundaries
    /// only; holes must be re-derived afterwards if required.
    pub fn dissolved_outer_only(geometries: Vec<Geometry>) -> Self {
        use super::boolean::{polygon_boolean, BoolOp};

        let mut acc: Vec<Polygon> =
            geometries.iter().map(|g| g.apply_transforms().outer).collect();

        loop {
            let mut merged: Option<(usize, usize, Vec<Polygon>)> = None;
            'outer: for i in 0..acc.len() {
                for j in (i + 1)..acc.len() {
                    if matches!(bbox_relation(&acc[i], &acc[j]), BBoxRelation::Disjoint) {
                        continue;
                    }
                    let u = polygon_boolean(&acc[i], &acc[j], BoolOp::Union);
                    let u_area: f64 = u.iter().map(|p| p.area().abs()).sum();
                    let sum_area = acc[i].area().abs() + acc[j].area().abs();
                    if u_area < sum_area - 1e-9 {
                        merged = Some((i, j, u));
                        break 'outer;
                    }
                }
            }
            match merged {
                Some((i, j, u)) => {
                    let last = acc.len() - 1;
                    acc[j] = acc[last].clone();
                    acc.swap_remove(last);
                    acc.splice(i..i + 1, u);
                }
                None => break,
            }
        }

        let geometries = acc
            .into_iter()
            .map(|p| Geometry { outer: p, holes: Vec::new(), transforms: Vec::new() })
            .collect();
        Self { geometries }
    }

    /// Check the material-disjoint invariant.
    ///
    /// * every hole must lie fully inside its own outer boundary,
    /// * no two regions may claim the same material point (verified with
    ///   deterministic edge–edge and containment checks).
    pub fn validate(&self) -> Result<(), CompoundError> {
        // Hole containment within its own outer boundary.
        for (ri, g) in self.geometries.iter().enumerate() {
            let g = g.apply_transforms();
            for (hi, hole) in g.holes.iter().enumerate() {
                // 1. All hole vertices must be inside outer.
                let vertices_inside = hole.vertices.iter().all(|v| g.outer.contains_point(*v));
                if !vertices_inside {
                    return Err(CompoundError::HoleNotInsideOuter { region: ri, hole: hi });
                }
                // 2. No hole edge may cross outer boundary edge.
                for i in 0..hole.vertices.len() {
                    let h1 = hole.vertices[i];
                    let h2 = hole.vertices[(i + 1) % hole.vertices.len()];
                    for j in 0..g.outer.vertices.len() {
                        let o1 = g.outer.vertices[j];
                        let o2 = g.outer.vertices[(j + 1) % g.outer.vertices.len()];
                        if segments_interact(h1, h2, o1, o2) {
                            return Err(CompoundError::HoleNotInsideOuter { region: ri, hole: hi });
                        }
                    }
                }
            }

            // 3. No hole may be nested inside another hole of the same region.
            // A hole-within-a-hole is an island of material inside a void, which
            // a single region (one outer + holes) cannot represent.
            for a in 0..g.holes.len() {
                for b in (a + 1)..g.holes.len() {
                    let ha = &g.holes[a];
                    let hb = &g.holes[b];
                    let a_contains_b = hb.vertices.iter().all(|v| ha.contains_point(*v));
                    let b_contains_a = ha.vertices.iter().all(|v| hb.contains_point(*v));
                    if a_contains_b || b_contains_a {
                        return Err(CompoundError::NestedHoles { region: ri, a, b });
                    }
                }
            }
        }

        // Cross-region material overlap via deterministic checks.
        //
        // For each pair of regions, we verify the "material-disjoint"
        // invariant by checking:
        //  1. Edge–edge intersections between outer boundaries.
        //  2. Vertex containment: any vertex of region A inside region B's
        //     material (outer minus holes) or vice versa.
        //  3. Full containment: one outer fully inside the other, with the
        //     intersection not fully absorbed by holes.
        let n = self.geometries.len();
        if n < 2 {
            return Ok(());
        }

        let applied: Vec<Geometry> =
            self.geometries.iter().map(|g| g.apply_transforms()).collect();

        for i in 0..n {
            for j in (i + 1)..n {
                let gi = &applied[i];
                let gj = &applied[j];
                if regions_overlap_deterministic(gi, gj) {
                    return Err(CompoundError::OverlappingRegions { a: i, b: j });
                }
            }
        }
        Ok(())
    }

    /// Build a compound geometry from a slice of [`Section`](crate::section::Section).
    pub fn from_sections(sections: &[crate::section::Section]) -> Self {
        Self {
            geometries: sections.iter().map(Geometry::from_section).collect(),
        }
    }

    /// Boolean operation across compound geometries.
    ///
    /// Mirrors `CompoundGeometry.combine(other, operation)`:
    /// - Union: all regions of both compounds unioned pairwise into one
    ///   accumulated set (dissolves internal overlaps).
/// - Intersection: every region pair intersected, keeping non-empty
    ///   results.
    /// - Difference: each region of `self` reduced by every region of
    ///   `other` in sequence.
    pub fn boolean(&self, other: &Self, op: super::boolean::BoolOp) -> Self {
        use super::boolean::polygon_boolean;

        match op {
            super::boolean::BoolOp::Intersection => {
                // Intersection: pairwise intersection of all region pairs
                // Use polygon_boolean directly on outers (like original code, which worked)
                // Note: This ignores holes, which is fine for simple cases
                let mut out = Vec::new();
                for ga in &self.geometries {
                    for gb in &other.geometries {
                        out.extend(super::boolean::polygon_boolean(&ga.outer, &gb.outer, op));
                    }
                }
                let geometries = out
                    .into_iter()
                    .map(|p| Geometry { outer: p, holes: Vec::new(), transforms: Vec::new() })
                    .collect();
                Self { geometries }
            }
            super::boolean::BoolOp::Difference => {
                // Start with all self regions
                let mut current: Vec<Geometry> = self.geometries.clone();
                
                // Subtract each other region from all current regions
                for gb in &other.geometries {
                    let mut next = Vec::new();
                    for ga in &current {
                        if let Some(diff) = ga.boolean_with_holes(gb, op) {
                            next.push(diff);
                        }
                    }
                    current = next;
                    if current.is_empty() {
                        break;
                    }
                }
                let geometries = current
                    .into_iter()
                    .map(|g| Geometry { outer: g.outer, holes: g.holes, transforms: Vec::new() })
                    .collect();
                Self { geometries }
            }
            super::boolean::BoolOp::Union => {
                // Collect every region from both compounds
                let mut acc: Vec<Geometry> = self
                    .geometries
                    .iter()
                    .chain(other.geometries.iter())
                    .cloned()
                    .collect();

                // Merge overlapping regions iteratively
                loop {
                    let mut merged: Option<(usize, usize, Geometry)> = None;
                    'outer: for i in 0..acc.len() {
                        for j in (i + 1)..acc.len() {
                            if matches!(bbox_relation(&acc[i].outer, &acc[j].outer), BBoxRelation::Disjoint) {
                                continue;
                            }
                            if let Some(union) = acc[i].boolean_with_holes(&acc[j], super::boolean::BoolOp::Union) {
                                merged = Some((i, j, union));
                                break 'outer;
                            }
                        }
                    }
                    match merged {
                        Some((i, j, union)) => {
                            let last = acc.len() - 1;
                            acc[j] = acc[last].clone();
                            acc.swap_remove(last);
                            acc.splice(i..i + 1, std::iter::once(union));
                        }
                        None => break,
                    }
                }

                let geometries = acc;
                Self { geometries }
            }
        }
    }

    /// Boolean union with `other`.
    pub fn union_compound(&self, other: &Self) -> Self {
        self.boolean(other, super::boolean::BoolOp::Union)
    }

    /// Boolean intersection with `other`.
    pub fn intersection_compound(&self, other: &Self) -> Self {
        self.boolean(other, super::boolean::BoolOp::Intersection)
    }

    /// Boolean difference: this compound minus `other`.
    pub fn subtract_compound(&self, other: &Self) -> Self {
        self.boolean(other, super::boolean::BoolOp::Difference)
    }
    /// Total net area across all regions.
    pub fn area(&self) -> f64 {
        self.geometries.iter().map(|g| g.area()).sum()
    }

    /// Centroid of the compound section via the composite area formula.
    pub fn centroid(&self) -> Point {
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut total = 0.0;
        for g in &self.geometries {
            let a = g.area();
            let c = g.centroid();
            sum_x += c.x * a;
            sum_y += c.y * a;
            total += a;
        }
        assert!(
            total.abs() > f64::EPSILON,
            "CompoundGeometry has zero total area"
        );
        Point::new(sum_x / total, sum_y / total)
    }

    /// Bounding box of the compound geometry: (min_x, max_x, min_y, max_y).
    pub fn bounds(&self) -> (f64, f64, f64, f64) {
        let mut min_x = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        for g in &self.geometries {
            let (l, r, b, t) = g.bounds();
            min_x = min_x.min(l);
            max_x = max_x.max(r);
            min_y = min_y.min(b);
            max_y = max_y.max(t);
        }
        (min_x, max_x, min_y, max_y)
    }

    /// Total perimeter across all regions (outer + holes).
    pub fn perimeter(&self) -> f64 {
        self.geometries.iter().map(|g| g.perimeter()).sum()
    }

    /// Return all polygons (outer first, then holes) across every region,
    /// with any transforms already applied.
    pub fn polygons(&self) -> Vec<Polygon> {
        let mut out = Vec::new();
        for g in &self.geometries {
            let g = g.apply_transforms();
            out.push(g.outer);
            out.extend(g.holes);
        }
        out
    }
}

/// Convert a [`Section`](crate::section::Section) into a single-geometry compound.
impl From<crate::section::Section> for CompoundGeometry {
    fn from(section: crate::section::Section) -> Self {
        Self::new(vec![Geometry::new(section.outer, section.holes)])
    }
}

/// Deterministic check whether two regions share any material area.
///
/// A region's material is its outer boundary minus its holes.
/// Two regions overlap if and only if there exists a point that lies in
/// both materials.  This is checked by:
///
/// 1. **Edge–edge intersection** – if any outer-boundary edge of `a`
///    crosses any outer-boundary edge of `b`, the interiors overlap.
/// 2. **Vertex containment** – if a vertex of one outer lies inside the
///    other region's material (outer minus holes), there is overlap.
/// 3. **Full containment** – if all vertices of one outer lie inside the
///    other outer, the intersection polygon is non-empty *and* not
///    entirely absorbed by the union of all holes.
fn regions_overlap_deterministic(a: &Geometry, b: &Geometry) -> bool {
    let a_outer = &a.outer;
    let b_outer = &b.outer;

    // 1. Edge–edge intersection between the two outer boundaries.
    for i in 0..a_outer.vertices.len() {
        let a1 = a_outer.vertices[i];
        let a2 = a_outer.vertices[(i + 1) % a_outer.vertices.len()];
        for j in 0..b_outer.vertices.len() {
            let b1 = b_outer.vertices[j];
            let b2 = b_outer.vertices[(j + 1) % b_outer.vertices.len()];
            if segments_interact(a1, a2, b1, b2) {
                return true;
            }
        }
    }

    // 2. Vertex containment: any vertex of `a` inside `b`'s material?
    for v in &a_outer.vertices {
        if b_outer.contains_point(*v) && !b.holes.iter().any(|h| h.contains_point(*v)) {
            return true;
        }
    }
    // Any vertex of `b` inside `a`'s material?
    for v in &b_outer.vertices {
        if a_outer.contains_point(*v) && !a.holes.iter().any(|h| h.contains_point(*v)) {
            return true;
        }
    }

    // 3. Full containment: one outer fully contains the other, but no
    //    vertex lies on the wrong side.  In this case the two outers
    //    overlap as regions, but we must verify the overlap is not
    //    entirely absorbed by holes.
    let a_in_b = a_outer.vertices.iter().all(|v| b_outer.contains_point(*v));
    let b_in_a = b_outer.vertices.iter().all(|v| a_outer.contains_point(*v));

    if a_in_b || b_in_a {
        // Compute the boolean intersection of the two outer polygons.
        let inter = super::boolean::polygon_boolean(a_outer, b_outer, super::boolean::BoolOp::Intersection);
        if inter.is_empty() {
            return false;
        }
        // Subtract every hole from both regions.  If anything survives, the
        // regions overlap in material.
        let mut remaining = inter;
        for hole in a.holes.iter().chain(b.holes.iter()) {
            let mut next = Vec::new();
            for p in &remaining {
                let diff = super::boolean::polygon_boolean(p, hole, super::boolean::BoolOp::Difference);
                next.extend(diff);
            }
            remaining = next;
            if remaining.is_empty() {
                break;
            }
        }
        return !remaining.is_empty();
    }

    false
}

/// Segment-segment relation for robust topology checks.
#[derive(Debug, Clone, Copy, PartialEq)]
enum SegmentRelation {
    /// No intersection (disjoint, no shared points)
    Disjoint,
    /// Touching at a single endpoint (shared vertex)
    Touch,
    /// Proper interior intersection (crossing)
    ProperCross,
    /// Collinear and overlapping with positive-length intersection
    CollinearOverlap,
}

/// Determine the topological relation between two segments A1→A2 and B1→B2.
/// Uses orientation tests with epsilon for numerical robustness.
fn segment_relation(a1: Point, a2: Point, b1: Point, b2: Point) -> SegmentRelation {
    const EPS: f64 = 1e-12;

    fn orient(p: Point, q: Point, r: Point) -> f64 {
        (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x)
    }

    fn on_segment(p: Point, q: Point, r: Point) -> bool {
        // q lies on segment pr (assuming collinear)
        q.x >= p.x.min(r.x) - EPS && q.x <= p.x.max(r.x) + EPS &&
        q.y >= p.y.min(r.y) - EPS && q.y <= p.y.max(r.y) + EPS
    }

    fn points_equal(p: Point, q: Point) -> bool {
        (p.x - q.x).abs() <= EPS && (p.y - q.y).abs() <= EPS
    }

    let o1 = orient(a1, a2, b1);
    let o2 = orient(a1, a2, b2);
    let o3 = orient(b1, b2, a1);
    let o4 = orient(b1, b2, a2);

    // General case: proper crossing (endpoints straddle each other's line)
    let o1_pos = o1 > EPS;
    let o1_neg = o1 < -EPS;
    let o2_pos = o2 > EPS;
    let o2_neg = o2 < -EPS;
    let o3_pos = o3 > EPS;
    let o3_neg = o3 < -EPS;
    let o4_pos = o4 > EPS;
    let o4_neg = o4 < -EPS;

    if (o1_pos != o2_pos || o1_neg != o2_neg) &&
       (o3_pos != o4_pos || o3_neg != o4_neg) {
        return SegmentRelation::ProperCross;
    }

    // Special cases: check for collinear overlap
    // All orientations are near zero -> segments are collinear
    if o1.abs() <= EPS && o2.abs() <= EPS && o3.abs() <= EPS && o4.abs() <= EPS {
        // Check if projections on x or y overlap
        let a_min_x = a1.x.min(a2.x);
        let a_max_x = a1.x.max(a2.x);
        let b_min_x = b1.x.min(b2.x);
        let b_max_x = b1.x.max(b2.x);
        let a_min_y = a1.y.min(a2.y);
        let a_max_y = a1.y.max(a2.y);
        let b_min_y = b1.y.min(b2.y);
        let b_max_y = b1.y.max(b2.y);

        let overlap_x = a_max_x >= b_min_x - EPS && b_max_x >= a_min_x - EPS;
        let overlap_y = a_max_y >= b_min_y - EPS && b_max_y >= a_min_y - EPS;

        if overlap_x && overlap_y {
            return SegmentRelation::CollinearOverlap;
        }
        return SegmentRelation::Disjoint;
    }

    // Check for endpoint touching (shared vertex) - this is Touch, not Disjoint
    // b1 on a
    if o1.abs() <= EPS && on_segment(a1, b1, a2) {
        return if points_equal(b1, a1) || points_equal(b1, a2) {
            SegmentRelation::Touch
        } else {
            SegmentRelation::Disjoint
        };
    }
    // b2 on a
    if o2.abs() <= EPS && on_segment(a1, b2, a2) {
        return if points_equal(b2, a1) || points_equal(b2, a2) {
            SegmentRelation::Touch
        } else {
            SegmentRelation::Disjoint
        };
    }
    // a1 on b
    if o3.abs() <= EPS && on_segment(b1, a1, b2) {
        return if points_equal(a1, b1) || points_equal(a1, b2) {
            SegmentRelation::Touch
        } else {
            SegmentRelation::Disjoint
        };
    }
    // a2 on b
    if o4.abs() <= EPS && on_segment(b1, a2, b2) {
        return if points_equal(a2, b1) || points_equal(a2, b2) {
            SegmentRelation::Touch
        } else {
            SegmentRelation::Disjoint
        };
    }

    SegmentRelation::Disjoint
}

fn points_equal(p: Point, q: Point) -> bool {
    const EPS: f64 = 1e-12;
    (p.x - q.x).abs() <= EPS && (p.y - q.y).abs() <= EPS
}

/// Strict edge-crossing test: two segments A1→A2 and B1→B2 cross (share
/// interior points) if and only if the endpoints of each segment straddle
/// the line through the other. Touching at a single endpoint is **not**
/// considered a cross (shared-vertex configurations are allowed).
fn segments_cross(a1: Point, a2: Point, b1: Point, b2: Point) -> bool {
    matches!(segment_relation(a1, a2, b1, b2), SegmentRelation::ProperCross)
}

/// Check if two segments have any topological interaction (crossing, touching, or collinear overlap).
fn segments_interact(a1: Point, a2: Point, b1: Point, b2: Point) -> bool {
    !matches!(segment_relation(a1, a2, b1, b2), SegmentRelation::Disjoint)
}
