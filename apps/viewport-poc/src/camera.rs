//! Orbit / pan / zoom camera with perspective projection.

use crate::project::{self, Vec3};

/// Result of projecting a 3-D point to screen space.
#[derive(Debug, Clone, Copy)]
pub struct ScreenPos {
    /// Screen X in pixels.
    pub x: f64,
    /// Screen Y in pixels.
    pub y: f64,
    /// Camera-space depth (positive = in front of camera).
    pub depth: f64,
}

/// Standard preset viewpoints (MIDAS-style view buttons).
///
/// The eye direction is derived from the camera's spherical parameters
/// (`azimuth`, `elevation`); see [`Camera3D::eye`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StandardView {
    /// Isometric: azimuth 45°, elevation 30°.
    Iso,
    /// Front: eye on +Z looking towards −Z (the XZ plane, `+Y` up).
    Front,
    /// Top: eye (almost) directly above, looking down `−Y`.
    Top,
    /// Right: eye on +X looking towards −X (the YZ plane, `+Y` up).
    Right,
}

impl StandardView {
    /// All presets in display order.
    pub const ALL: [StandardView; 4] = [
        StandardView::Iso,
        StandardView::Front,
        StandardView::Top,
        StandardView::Right,
    ];

    /// Short label for GUI buttons.
    pub const fn label(self) -> &'static str {
        match self {
            StandardView::Iso => "Iso",
            StandardView::Front => "Front",
            StandardView::Top => "Top",
            StandardView::Right => "Right",
        }
    }

    /// `(azimuth, elevation)` in radians for this preset.
    ///
    /// `Top` is kept just below `π/2` so that the `forward × world_up` cross
    /// product in [`Camera3D::basis`] stays well-conditioned (it degenerates
    /// when the view direction is exactly parallel to the world up axis).
    pub const fn angles(self) -> (f64, f64) {
        match self {
            StandardView::Iso => (std::f64::consts::FRAC_PI_4, std::f64::consts::FRAC_PI_6),
            StandardView::Front => (std::f64::consts::FRAC_PI_2, 0.0),
            StandardView::Top => (std::f64::consts::FRAC_PI_2, 1.55),
            StandardView::Right => (0.0, 0.0),
        }
    }
}

/// Orbit camera with spherical-coordinate parameterisation.
#[derive(Debug, Clone)]
pub struct Camera3D {
    /// Look-at target.
    pub target: Vec3,
    /// Horizontal angle (radians, 0 = +X).
    pub azimuth: f64,
    /// Vertical angle (radians, 0 = XZ plane, +π/2 = +Y).
    pub elevation: f64,
    /// Distance from target to eye.
    pub distance: f64,
    /// Vertical field-of-view (radians).
    pub fov: f64,
    /// Near clipping plane.
    pub near: f64,
    /// Far clipping plane.
    pub far: f64,
}

impl Default for Camera3D {
    fn default() -> Self {
        Self {
            target: [0.0, 0.0, 0.0],
            azimuth: std::f64::consts::FRAC_PI_4,
            elevation: std::f64::consts::FRAC_PI_6,
            distance: 10.0,
            fov: std::f64::consts::FRAC_PI_3,
            near: 0.01,
            far: 1e4,
        }
    }
}

impl Camera3D {
    /// Create a camera and frame the given bounding box.
    pub fn fit(min: Vec3, max: Vec3) -> Self {
        let mut cam = Self::default();
        cam.frame(min, max);
        cam
    }

    /// Aim at the centre of a bounding box and pull back far enough to see it,
    /// **keeping the current orientation**.  Shared by [`fit`](Self::fit) and
    /// the "Fit / Zoom Extents" command.
    pub fn frame(&mut self, min: Vec3, max: Vec3) {
        let center = [
            (min[0] + max[0]) * 0.5,
            (min[1] + max[1]) * 0.5,
            (min[2] + max[2]) * 0.5,
        ];
        let size = [
            (max[0] - min[0]).max(1e-6),
            (max[1] - min[1]).max(1e-6),
            (max[2] - min[2]).max(1e-6),
        ];
        let diag = size[0].hypot(size[1]).hypot(size[2]).max(1.0);
        self.target = center;
        self.distance = diag * 2.5;
    }

    /// Apply a standard preset orientation, keeping the current target and
    /// distance.
    pub fn set_view(&mut self, view: StandardView) {
        let (az, el) = view.angles();
        self.azimuth = az;
        self.elevation = el;
    }

    /// Reset to default orientation while keeping the target.
    pub fn reset(&mut self) {
        self.azimuth = std::f64::consts::FRAC_PI_4;
        self.elevation = std::f64::consts::FRAC_PI_6;
        self.distance = 10.0;
    }

    /// Eye position in world coordinates.
    pub fn eye(&self) -> Vec3 {
        let ce = self.elevation.cos();
        [
            self.target[0] + self.distance * ce * self.azimuth.cos(),
            self.target[1] + self.distance * self.elevation.sin(),
            self.target[2] + self.distance * ce * self.azimuth.sin(),
        ]
    }

    /// Camera basis vectors: (forward, right, up).
    pub fn basis(&self) -> (Vec3, Vec3, Vec3) {
        let eye = self.eye();
        let forward = project::normalize(project::sub(self.target, eye));
        let world_up = [0.0, 1.0, 0.0];
        let right = project::normalize(project::cross(forward, world_up));
        let up = project::cross(right, forward);
        (forward, right, up)
    }

    /// Orbit: change azimuth / elevation.
    pub fn orbit(&mut self, delta_az: f64, delta_el: f64) {
        self.azimuth += delta_az;
        self.elevation = (self.elevation + delta_el).clamp(-1.55, 1.55);
    }

    /// Pan: move target in the camera plane.
    pub fn pan(&mut self, dx: f64, dy: f64) {
        let (_, right, up) = self.basis();
        self.target = project::sub(self.target, project::scale(right, dx));
        self.target = project::sub(self.target, project::scale(up, dy));
    }

    /// Zoom: scale distance.
    pub fn zoom(&mut self, factor: f64) {
        self.distance = (self.distance * factor).clamp(0.1, 1e5);
    }

    /// Eye position, look-at target and camera up vector, as a single triple.
    ///
    /// This is the world-space look-at frame used to build
    /// [`view_projection`](Self::view_projection); it mirrors the internal
    /// [`basis`](Self::basis) computation so both agree exactly.
    pub fn eye_target_up(&self) -> (Vec3, Vec3, Vec3) {
        let (_, _, up) = self.basis();
        (self.eye(), self.target, up)
    }

    /// Column-major `4×4` view-projection matrix for OpenGL (`mat4` in GLSL).
    ///
    /// Produces the same perspective mapping as [`project`](Self::project):
    /// applying `mvp * vec4(p, 1)` and dividing by `w` yields clip space whose
    /// `x`/`y` map to NDC identically (the `y` axis points up, as in
    /// OpenGL).  The camera looks down its own `−Z` after the view transform,
    /// and clip-space `z` lies in `[-1, 1]` for points between `near` and
    /// `far`.
    ///
    /// `aspect` is `width / height` of the target viewport.
    ///
    /// The matrix is returned as a flat `[f32; 16]` in **column-major** order,
    /// ready for `glUniformMatrix4fv` with `transpose = false`.
    pub fn view_projection(&self, aspect: f64) -> [f32; 16] {
        let (forward, right, up) = self.basis();
        let eye = self.eye();

        // World → camera.  Columns of the 3×3 part are the camera axes; the
        // third row is `-forward` so the camera looks down OpenGL's `−Z`.
        let view: [f64; 16] = [
            right[0],
            up[0],
            -forward[0],
            0.0,
            right[1],
            up[1],
            -forward[1],
            0.0,
            right[2],
            up[2],
            -forward[2],
            0.0,
            -project::dot(right, eye),
            -project::dot(up, eye),
            project::dot(forward, eye),
            1.0,
        ];

        let f = 1.0 / (self.fov * 0.5).tan();
        let n = self.near;
        let far = self.far;
        let proj: [f64; 16] = [
            f / aspect,
            0.0,
            0.0,
            0.0,
            0.0,
            f,
            0.0,
            0.0,
            0.0,
            0.0,
            (far + n) / (n - far),
            -1.0,
            0.0,
            0.0,
            (2.0 * far * n) / (n - far),
            0.0,
        ];

        let mvp = mul4_col_major(&proj, &view);
        let mut out = [0.0_f32; 16];
        for (o, &v) in out.iter_mut().zip(mvp.iter()) {
            *o = v as f32;
        }
        out
    }

    /// Project a world-space point to screen coordinates.
    ///
    /// Returns `None` if the point is behind the camera or outside the
    /// near/far range.
    pub fn project(&self, point: Vec3, width: f64, height: f64) -> Option<ScreenPos> {
        if !point[0].is_finite() || !point[1].is_finite() || !point[2].is_finite() {
            return None;
        }
        let eye = self.eye();
        let (forward, right, up) = self.basis();
        let rel = project::sub(point, eye);
        let z = project::dot(rel, forward);
        if z < self.near || z > self.far {
            return None;
        }
        let x = project::dot(rel, right);
        let y = project::dot(rel, up);
        let aspect = width / height;
        let f = 1.0 / (self.fov * 0.5).tan();
        let x_ndc = f * x / (z * aspect);
        let y_ndc = f * y / z;
        let sx = (x_ndc + 1.0) * 0.5 * width;
        let sy = (1.0 - y_ndc) * 0.5 * height;
        Some(ScreenPos {
            x: sx,
            y: sy,
            depth: z,
        })
    }
}

/// Multiply two column-major `4×4` matrices: `a * b` (`a` applied after `b`).
fn mul4_col_major(a: &[f64; 16], b: &[f64; 16]) -> [f64; 16] {
    let mut out = [0.0_f64; 16];
    for c in 0..4 {
        for r in 0..4 {
            let mut sum = 0.0;
            for k in 0..4 {
                sum += a[k * 4 + r] * b[c * 4 + k];
            }
            out[c * 4 + r] = sum;
        }
    }
    out
}

/// Apply a column-major `4×4` matrix to a point, returning clip-space `(x,y,z,w)`.
#[cfg(test)]
fn apply4(m: &[f32; 16], p: Vec3) -> [f64; 4] {
    let g = |i: usize| m[i] as f64;
    [
        g(0) * p[0] + g(4) * p[1] + g(8) * p[2] + g(12),
        g(1) * p[0] + g(5) * p[1] + g(9) * p[2] + g(13),
        g(2) * p[0] + g(6) * p[1] + g(10) * p[2] + g(14),
        g(3) * p[0] + g(7) * p[1] + g(11) * p[2] + g(15),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn center_projects_to_screen_center() {
        let cam = Camera3D::default();
        let w = 800.0;
        let h = 600.0;
        let p = cam.project(cam.target, w, h).unwrap();
        assert!((p.x - w * 0.5).abs() < 1e-9);
        assert!((p.y - h * 0.5).abs() < 1e-9);
    }

    #[test]
    fn behind_camera_returns_none() {
        let mut cam = Camera3D::default();
        cam.target = [0.0, 0.0, 0.0];
        let eye = cam.eye();
        let behind = project::scale(eye, 2.0);
        assert!(cam.project(behind, 800.0, 600.0).is_none());
    }

    #[test]
    fn non_finite_returns_none() {
        let cam = Camera3D::default();
        assert!(cam.project([f64::NAN, 0.0, 0.0], 800.0, 600.0).is_none());
        assert!(
            cam.project([f64::INFINITY, 0.0, 0.0], 800.0, 600.0)
                .is_none()
        );
    }

    #[test]
    fn orbit_changes_eye() {
        let mut cam = Camera3D::default();
        let eye0 = cam.eye();
        cam.orbit(0.5, 0.0);
        let eye1 = cam.eye();
        assert!(project::sub(eye0, eye1) != [0.0; 3]);
    }

    #[test]
    fn zoom_changes_distance() {
        let mut cam = Camera3D::default();
        let d0 = cam.distance;
        cam.zoom(0.5);
        assert!((cam.distance - d0 * 0.5).abs() < 1e-15);
    }

    #[test]
    fn fit_uses_bounding_box() {
        let cam = Camera3D::fit([0.0, 0.0, 0.0], [4.0, 3.0, 0.0]);
        assert!((cam.target[0] - 2.0).abs() < 1e-15);
        assert!((cam.target[1] - 1.5).abs() < 1e-15);
    }

    #[test]
    fn frame_keeps_orientation() {
        let mut cam = Camera3D::default();
        let (az0, el0) = (cam.azimuth, cam.elevation);
        cam.frame([-1.0, -2.0, -3.0], [1.0, 2.0, 3.0]);
        assert_eq!(cam.azimuth, az0);
        assert_eq!(cam.elevation, el0);
        assert!((cam.target[0]).abs() < 1e-15);
        assert!(cam.distance > 0.0);
    }

    #[test]
    fn frame_matches_fit() {
        let (min, max) = ([0.0, 0.0, 0.0], [4.0, 3.0, 2.0]);
        let fitted = Camera3D::fit(min, max);
        let mut framed = Camera3D::default();
        framed.frame(min, max);
        assert!((fitted.target[0] - framed.target[0]).abs() < 1e-15);
        assert!((fitted.target[1] - framed.target[1]).abs() < 1e-15);
        assert!((fitted.distance - framed.distance).abs() < 1e-15);
    }

    #[test]
    fn set_view_updates_angles_only() {
        let mut cam = Camera3D::default();
        cam.target = [1.0, 2.0, 3.0];
        cam.distance = 42.0;
        cam.set_view(StandardView::Front);
        assert_eq!(cam.azimuth, StandardView::Front.angles().0);
        assert_eq!(cam.elevation, StandardView::Front.angles().1);
        assert_eq!(cam.target, [1.0, 2.0, 3.0]);
        assert_eq!(cam.distance, 42.0);
    }

    #[test]
    fn all_presets_produce_finite_basis() {
        for view in StandardView::ALL {
            let mut cam = Camera3D::default();
            cam.set_view(view);
            let (f, r, u) = cam.basis();
            for v in [f, r, u] {
                assert!(v[0].is_finite() && v[1].is_finite() && v[2].is_finite());
            }
        }
    }

    #[test]
    fn top_view_is_not_degenerate_and_looks_down() {
        let mut cam = Camera3D::default();
        cam.set_view(StandardView::Top);
        let (forward, _, _) = cam.basis();
        // Looking downward: forward points towards −Y.
        assert!(forward[1] < -0.9);
    }

    // ── View-projection matrix ──────────────────────────────────────────

    /// The GL matrix must reproduce the painter's `project()` exactly: apply
    /// `mvp`, divide by `w`, map NDC → screen, and compare.
    #[test]
    fn view_projection_matches_project() {
        let w = 800.0;
        let h = 600.0;
        let aspect = w / h;
        let mvp = Camera3D::default().view_projection(aspect);
        let cam = Camera3D::default();

        for p in [
            [0.0, 0.0, 0.0],
            [1.0, 2.0, 3.0],
            [-2.5, 0.5, 4.0],
            [0.3, -1.2, -0.7],
        ] {
            let clip = apply4(&mvp, p);
            assert!(clip[3] > 0.0, "point {p:?} behind camera");
            let ndc_x = clip[0] / clip[3];
            let ndc_y = clip[1] / clip[3];
            let sx = (ndc_x + 1.0) * 0.5 * w;
            let sy = (1.0 - ndc_y) * 0.5 * h;
            let expected = cam.project(p, w, h).unwrap();
            assert!((sx - expected.x).abs() < 1e-3, "x {sx} vs {}", expected.x);
            assert!((sy - expected.y).abs() < 1e-3, "y {sy} vs {}", expected.y);
            // Camera-space depth is the clip-space w.
            assert!((clip[3] - expected.depth).abs() < 1e-3);
        }
    }

    /// Near/far planes must map to clip-space `z = ∓1`.
    #[test]
    fn view_projection_depth_range() {
        let cam = Camera3D::default();
        let mvp = cam.view_projection(1.0);
        let (forward, _, _) = cam.basis();
        let eye = cam.eye();
        for (dist, want) in [(cam.near, -1.0), (cam.far, 1.0)] {
            let p = project::add(eye, project::scale(forward, dist));
            let clip = apply4(&mvp, p);
            let ndc_z = clip[2] / clip[3];
            assert!(
                (ndc_z - want).abs() < 1e-2,
                "dist {dist}: ndc_z {ndc_z} vs {want}"
            );
        }
    }

    /// `eye_target_up` must agree with the internal basis accessors.
    #[test]
    fn eye_target_up_agrees_with_basis() {
        let cam = Camera3D::default();
        let (eye, target, up) = cam.eye_target_up();
        assert_eq!(eye, cam.eye());
        assert_eq!(target, cam.target);
        let (_, _, up2) = cam.basis();
        assert_eq!(up, up2);
    }

    /// The perspective divide must be finite for every preset view.
    #[test]
    fn view_projection_finite_for_all_presets() {
        for view in StandardView::ALL {
            let mut cam = Camera3D::default();
            cam.set_view(view);
            let m = cam.view_projection(4.0 / 3.0);
            assert!(m.iter().all(|v| v.is_finite()), "{view:?} produced non-finite");
        }
    }
}
