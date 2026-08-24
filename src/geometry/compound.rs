use super::{Point, Polygon};

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

    /// Convert a single-boundary [`Section`](crate::section::Section) into a geometry.
    pub fn from_section(section: &crate::section::Section) -> Self {
        Self::new(section.outer.clone(), section.holes.clone())
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
                    .map(|&v| {
                        self.transforms
                            .iter()
                            .fold(v, |p, t| t.apply(p))
                    })
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