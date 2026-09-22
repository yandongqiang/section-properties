use super::Point;

/// Trait for section boundaries that can report their extreme fiber distances.
///
/// For polygonal (straight-edge) boundaries, extrema live at vertices.
/// For curved boundaries (arcs, Bezier, splines), extrema may lie on the
/// curve between vertices and require analytical or numerical evaluation.
///
/// # Future curve support
///
/// When curve elements are added, each type implements this trait with its
/// own strategy:
///
/// | Type | Strategy |
/// |------|----------|
/// | `Polygon` | Fold over vertices — O(n) |
/// | `Arc` | Analytical: center ± radius along direction |
/// | `Bezier` | Root-finding on derivative Bernstein polynomials |
/// | `CompoundCurve` | max/min across segments |
pub trait BoundaryExtrema {
    /// Signed distances `(min, max)` from `center` projected onto `direction`.
    ///
    /// * `min` — most negative (farthest in the `-direction` sense).
    /// * `max` — most positive (farthest in the `+direction` sense).
    ///
    /// `direction` does not need to be unit-length; the result scales
    /// proportionally, which is harmless for section-modulus ratios.
    fn extreme_distances(&self, center: Point, direction: Point) -> (f64, f64);

    /// Convenience: extreme distances along the global Y axis.
    fn extreme_y(&self, center: Point) -> (f64, f64) {
        self.extreme_distances(center, Point::new(0.0, 1.0))
    }

    /// Convenience: extreme distances along the global X axis.
    fn extreme_x(&self, center: Point) -> (f64, f64) {
        self.extreme_distances(center, Point::new(1.0, 0.0))
    }
}
