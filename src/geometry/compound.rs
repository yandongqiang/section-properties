use super::{Point, Polygon};

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
    /// boundaries; returns the largest resulting region, or `None` if empty.
    pub fn union(&self, other: &Self) -> Option<Self> {
        self.boolean(other, super::boolean::BoolOp::Union)
    }

    /// Boolean intersection with `other`.
    ///
    /// Mirrors `Geometry & other` (shapely `intersection`).
    pub fn intersection(&self, other: &Self) -> Option<Self> {
        self.boolean(other, super::boolean::BoolOp::Intersection)
    }

    /// Boolean difference: this geometry minus `other`.
    ///
    /// Mirrors `Geometry - other` (shapely `difference`).
    pub fn subtract(&self, other: &Self) -> Option<Self> {
        self.boolean(other, super::boolean::BoolOp::Difference)
    }

    fn boolean(&self, other: &Self, op: super::boolean::BoolOp) -> Option<Self> {
        let a = Geometry::from_section(&crate::section::Section::new(
            self.apply_transforms().outer,
            Vec::new(),
        ));
        let b = Geometry::from_section(&crate::section::Section::new(
            other.apply_transforms().outer,
            Vec::new(),
        ));

        let results = super::boolean::polygon_boolean(&a.outer, &b.outer, op);
        let best = results
            .into_iter()
            .max_by(|x, y| x.area().partial_cmp(&y.area()).unwrap())?;
        Some(Geometry::new(best, Vec::new()))
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
#[derive(Debug, Clone)]
pub struct CompoundGeometry {
    /// The individual regions that make up the compound section.
    pub geometries: Vec<Geometry>,
}

/// Quick bounding-box overlap test for two polygons.
fn bbox_overlap(a: &Polygon, b: &Polygon) -> bool {
    let bbox = |pts: &[Point]| {
        pts.iter().fold(
            (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY),
            |(l, r, b, t), p| (l.min(p.x), r.max(p.x), b.min(p.y), t.max(p.y)),
        )
    };
    let (al, ar, ab, at) = bbox(&a.vertices);
    let (bl, br, bb, bt) = bbox(&b.vertices);
    al <= br && bl <= ar && ab <= bt && bb <= at
}
impl CompoundGeometry {
    /// Create a compound geometry from its constituent regions.
    pub fn new(geometries: Vec<Geometry>) -> Self {
        Self { geometries }
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
                let mut out = Vec::new();
                for ga in &self.geometries {
                    for gb in &other.geometries {
                        out.extend(polygon_boolean(&ga.outer, &gb.outer, op));
                    }
                }
                let geometries = out
                    .into_iter()
                    .map(|p| Geometry { outer: p, holes: Vec::new(), transforms: Vec::new() })
                    .collect();
                Self { geometries }
            }
            super::boolean::BoolOp::Difference => {
                let mut acc: Vec<Polygon> =
                    self.geometries.iter().map(|g| g.outer.clone()).collect();
                for gb in &other.geometries {
                    let mut next = Vec::new();
                    for a in &acc {
                        next.extend(polygon_boolean(a, &gb.outer, op));
                    }
                    acc = next;
                    if acc.is_empty() {
                        break;
                    }
                }
                let geometries = acc
                    .into_iter()
                    .map(|p| Geometry { outer: p, holes: Vec::new(), transforms: Vec::new() })
                    .collect();
                Self { geometries }
            }
            super::boolean::BoolOp::Union => {
                // Collect every region then repeatedly merge overlapping
                // pairs until a fixed point. Touching-only regions stay
                // separate (matching Python's compound semantics).
                let mut acc: Vec<Polygon> = self
                    .geometries
                    .iter()
                    .chain(other.geometries.iter())
                    .map(|g| g.outer.clone())
                    .collect();

                loop {
                    let mut merged: Option<(usize, usize, Vec<Polygon>)> = None;
                    'outer: for i in 0..acc.len() {
                        for j in (i + 1)..acc.len() {
                            if !bbox_overlap(&acc[i], &acc[j]) {
                                continue;
                            }
                            let u = polygon_boolean(
                                &acc[i],
                                &acc[j],
                                super::boolean::BoolOp::Union,
                            );
                            let u_area: f64 = u.iter().map(|p| p.area().abs()).sum();
                            let sum_area =
                                acc[i].area().abs() + acc[j].area().abs();
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
