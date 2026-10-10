//! Minimal 3-D vector math used by the viewport.
//!
//! All functions operate on `[f64; 3]` and are `const` where possible.

/// 3-D vector.
pub type Vec3 = [f64; 3];

/// Component-wise addition.
pub const fn add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// Component-wise subtraction.
pub const fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// Scalar multiplication.
pub const fn scale(v: Vec3, s: f64) -> Vec3 {
    [v[0] * s, v[1] * s, v[2] * s]
}

/// Dot product.
pub const fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Cross product.
pub const fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Euclidean length.
pub fn length(v: Vec3) -> f64 {
    v[0].hypot(v[1]).hypot(v[2])
}

/// Normalize to unit length.  Returns `[0; 3]` for zero or non-finite input.
pub fn normalize(v: Vec3) -> Vec3 {
    let len = length(v);
    if len == 0.0 || !len.is_finite() {
        return [0.0; 3];
    }
    scale(v, 1.0 / len)
}

/// Linear interpolation: `(1 - t) * a + t * b`.
pub const fn lerp(a: Vec3, b: Vec3, t: f64) -> Vec3 {
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    ]
}

/// Distance from point `p` to the line segment `a → b` (2-D, using x/y).
pub fn point_to_segment_dist_2d(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let ap = [p[0] - a[0], p[1] - a[1]];
    let ab_len_sq = ab[0] * ab[0] + ab[1] * ab[1];
    if ab_len_sq == 0.0 {
        return ap[0].hypot(ap[1]);
    }
    let t = ((ap[0] * ab[0] + ap[1] * ab[1]) / ab_len_sq).clamp(0.0, 1.0);
    let closest = [a[0] + ab[0] * t, a[1] + ab[1] * t];
    (p[0] - closest[0]).hypot(p[1] - closest[1])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vec3_basics() {
        assert_eq!(add([1.0, 2.0, 3.0], [4.0, 5.0, 6.0]), [5.0, 7.0, 9.0]);
        assert_eq!(sub([5.0, 7.0, 9.0], [4.0, 5.0, 6.0]), [1.0, 2.0, 3.0]);
        assert_eq!(scale([1.0, 2.0, 3.0], 2.0), [2.0, 4.0, 6.0]);
        assert_eq!(dot([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]), 0.0);
        assert_eq!(cross([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]), [0.0, 0.0, 1.0]);
    }

    #[test]
    fn normalize_unit() {
        let n = normalize([3.0, 0.0, 0.0]);
        assert!((length(n) - 1.0).abs() < 1e-15);
    }

    #[test]
    fn normalize_zero() {
        assert_eq!(normalize([0.0, 0.0, 0.0]), [0.0, 0.0, 0.0]);
    }

    #[test]
    fn segment_distance() {
        let d = point_to_segment_dist_2d([0.0, 1.0], [0.0, 0.0], [2.0, 0.0]);
        assert!((d - 1.0).abs() < 1e-15);
    }
}
