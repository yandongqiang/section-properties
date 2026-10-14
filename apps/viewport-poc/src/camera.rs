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
    /// Back: eye on −Z looking towards +Z.
    Back,
    /// Left: eye on −X looking towards +X.
    Left,
    /// Bottom: eye (almost) directly below, looking up `+Y`.
    Bottom,
    /// Front–top edge: azimuth 90°, elevation 45° (between `Front` and `Top`).
    FrontTop,
    /// Front–right edge: azimuth 45°, elevation 0° (between `Front` and `Right`).
    FrontRight,
    /// Front–left edge: azimuth 135°, elevation 0° (between `Front` and `Left`).
    FrontLeft,
    /// Front–bottom edge: azimuth 90°, elevation −45° (between `Front` and `Bottom`).
    FrontBottom,
    /// Back–top edge: azimuth −90°, elevation 45° (between `Back` and `Top`).
    BackTop,
    /// Back–right edge: azimuth −45°, elevation 0° (between `Back` and `Right`).
    BackRight,
    /// Back–left edge: azimuth −135°, elevation 0° (between `Back` and `Left`).
    BackLeft,
    /// Back–bottom edge: azimuth −90°, elevation −45° (between `Back` and `Bottom`).
    BackBottom,
    /// Top–right edge: azimuth 0°, elevation 45° (between `Top` and `Right`).
    TopRight,
    /// Top–left edge: azimuth 180°, elevation 45° (between `Top` and `Left`).
    TopLeft,
    /// Bottom–right edge: azimuth 0°, elevation −45° (between `Bottom` and `Right`).
    BottomRight,
    /// Bottom–left edge: azimuth 180°, elevation −45° (between `Bottom` and `Left`).
    BottomLeft,
}

impl StandardView {
    /// All presets in display order.
    pub const ALL: [StandardView; 4] = [
        StandardView::Iso,
        StandardView::Front,
        StandardView::Top,
        StandardView::Right,
    ];

    /// The six axis-aligned cube faces, the twelve mid-edge views and `Iso` —
    /// the full set exposed by the view-cube gizmo (faces + edges + corner).
    /// Deliberately a superset of [`ALL`](Self::ALL): the View menu keeps
    /// showing the four primary presets.
    pub const CUBE: [StandardView; 19] = [
        StandardView::Iso,
        StandardView::Front,
        StandardView::Back,
        StandardView::Left,
        StandardView::Right,
        StandardView::Top,
        StandardView::Bottom,
        StandardView::FrontTop,
        StandardView::FrontRight,
        StandardView::FrontLeft,
        StandardView::FrontBottom,
        StandardView::BackTop,
        StandardView::BackRight,
        StandardView::BackLeft,
        StandardView::BackBottom,
        StandardView::TopRight,
        StandardView::TopLeft,
        StandardView::BottomRight,
        StandardView::BottomLeft,
    ];

    /// The twelve mid-edge views (Phase P7-2): 45° two-axis views, one per
    /// cube edge, reachable by clicking an edge of the view-cube gizmo.
    pub const EDGES: [StandardView; 12] = [
        StandardView::FrontTop,
        StandardView::FrontRight,
        StandardView::FrontLeft,
        StandardView::FrontBottom,
        StandardView::BackTop,
        StandardView::BackRight,
        StandardView::BackLeft,
        StandardView::BackBottom,
        StandardView::TopRight,
        StandardView::TopLeft,
        StandardView::BottomRight,
        StandardView::BottomLeft,
    ];

    /// Short label for GUI buttons.
    pub const fn label(self) -> &'static str {
        match self {
            StandardView::Iso => "Iso",
            StandardView::Front => "Front",
            StandardView::Top => "Top",
            StandardView::Right => "Right",
            StandardView::Back => "Back",
            StandardView::Left => "Left",
            StandardView::Bottom => "Bottom",
            StandardView::FrontTop => "Front-Top",
            StandardView::FrontRight => "Front-Right",
            StandardView::FrontLeft => "Front-Left",
            StandardView::FrontBottom => "Front-Bottom",
            StandardView::BackTop => "Back-Top",
            StandardView::BackRight => "Back-Right",
            StandardView::BackLeft => "Back-Left",
            StandardView::BackBottom => "Back-Bottom",
            StandardView::TopRight => "Top-Right",
            StandardView::TopLeft => "Top-Left",
            StandardView::BottomRight => "Bottom-Right",
            StandardView::BottomLeft => "Bottom-Left",
        }
    }

    /// `(azimuth, elevation)` in radians for this preset.
    ///
    /// `Top` / `Bottom` are kept just inside `±π/2` so that the
    /// `forward × world_up` cross product in [`Camera3D::basis`] stays
    /// well-conditioned (it degenerates when the view direction is exactly
    /// parallel to the world up axis).
    ///
    /// The twelve mid-edge views use exact `±45°` steps: their eye direction
    /// has two non-zero components of equal magnitude (see
    /// [`StandardView::eye_dir`]).
    pub const fn angles(self) -> (f64, f64) {
        match self {
            StandardView::Iso => (std::f64::consts::FRAC_PI_4, std::f64::consts::FRAC_PI_6),
            StandardView::Front => (std::f64::consts::FRAC_PI_2, 0.0),
            StandardView::Back => (-std::f64::consts::FRAC_PI_2, 0.0),
            StandardView::Right => (0.0, 0.0),
            StandardView::Left => (std::f64::consts::PI, 0.0),
            StandardView::Top => (std::f64::consts::FRAC_PI_2, 1.55),
            StandardView::Bottom => (std::f64::consts::FRAC_PI_2, -1.55),
            StandardView::FrontTop => (std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_4),
            StandardView::FrontRight => (std::f64::consts::FRAC_PI_4, 0.0),
            StandardView::FrontLeft => (3.0 * std::f64::consts::FRAC_PI_4, 0.0),
            StandardView::FrontBottom => (std::f64::consts::FRAC_PI_2, -std::f64::consts::FRAC_PI_4),
            StandardView::BackTop => (-std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_4),
            StandardView::BackRight => (-std::f64::consts::FRAC_PI_4, 0.0),
            StandardView::BackLeft => (-3.0 * std::f64::consts::FRAC_PI_4, 0.0),
            StandardView::BackBottom => (-std::f64::consts::FRAC_PI_2, -std::f64::consts::FRAC_PI_4),
            StandardView::TopRight => (0.0, std::f64::consts::FRAC_PI_4),
            StandardView::TopLeft => (std::f64::consts::PI, std::f64::consts::FRAC_PI_4),
            StandardView::BottomRight => (0.0, -std::f64::consts::FRAC_PI_4),
            StandardView::BottomLeft => (std::f64::consts::PI, -std::f64::consts::FRAC_PI_4),
        }
    }

    /// Unit eye direction (target → eye) for this preset.
    ///
    /// Mirrors the spherical-to-Cartesian mapping in [`Camera3D::eye`], so a
    /// face preset has exactly one non-zero component and a mid-edge preset
    /// (see [`EDGES`](Self::EDGES)) has exactly two, each `±1/√2`.
    pub fn eye_dir(self) -> Vec3 {
        let (az, el) = self.angles();
        let ce = el.cos();
        [ce * az.cos(), el.sin(), ce * az.sin()]
    }
}

/// Primary-drag interaction mode for the viewport.
///
/// The ribbon's View group exposes **Pan** and **Zoom** as radio buttons; the
/// historical primary-drag behaviour is [`ViewMode::Orbit`], which stays the
/// default.  The mode only decides what a *plain* primary drag does — the
/// Shift/Ctrl box-select drag, the Alt section-cut drag and the right-button
/// pan drag are unaffected.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ViewMode {
    /// Primary drag orbits the camera (azimuth / elevation).
    #[default]
    Orbit,
    /// Primary drag pans the camera target in the camera plane.
    Pan,
    /// Primary drag dollies the camera (vertical drag = zoom).
    Zoom,
}

impl ViewMode {
    /// All modes, in ribbon order.
    pub const ALL: [ViewMode; 3] = [ViewMode::Orbit, ViewMode::Pan, ViewMode::Zoom];

    /// Short label for the status bar / ribbon tooltips.
    pub const fn label(self) -> &'static str {
        match self {
            ViewMode::Orbit => "Orbit",
            ViewMode::Pan => "Pan",
            ViewMode::Zoom => "Zoom",
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
    fn view_cube_presets_are_finite_and_distinct() {
        assert_eq!(StandardView::CUBE.len(), 19);
        let mut seen: Vec<[i64; 2]> = Vec::new();
        for view in StandardView::CUBE {
            let mut cam = Camera3D::default();
            cam.set_view(view);
            let (f, r, u) = cam.basis();
            for v in [f, r, u] {
                assert!(v.iter().all(|c| c.is_finite()), "{view:?} non-finite basis");
            }
            // Each preset must point somewhere distinct (angles rounded).
            let (az, el) = view.angles();
            let key = [(az * 1000.0) as i64, (el * 1000.0) as i64];
            assert!(!seen.contains(&key), "duplicate angles for {view:?}");
            seen.push(key);
        }
    }

    #[test]
    fn top_and_bottom_look_oppositely() {
        let mut top = Camera3D::default();
        top.set_view(StandardView::Top);
        let mut bottom = Camera3D::default();
        bottom.set_view(StandardView::Bottom);
        let (ft, _, _) = top.basis();
        let (fb, _, _) = bottom.basis();
        assert!(ft[1] < -0.9, "Top should look down, got {ft:?}");
        assert!(fb[1] > 0.9, "Bottom should look up, got {fb:?}");
    }

    #[test]
    fn top_view_is_not_degenerate_and_looks_down() {
        let mut cam = Camera3D::default();
        cam.set_view(StandardView::Top);
        let (forward, _, _) = cam.basis();
        // Looking downward: forward points towards −Y.
        assert!(forward[1] < -0.9);
    }

    // ── Mid-edge view snaps (Phase P7-2) ─────────────────────────────────

    /// The six face presets look along exactly one world axis.
    ///
    /// `Top` / `Bottom` are deliberately kept just inside `±π/2` (elevation
    /// 1.55) so the camera basis stays well-conditioned; their off-axis
    /// component is therefore small (~2.6°) but not exactly zero.
    #[test]
    fn face_views_look_along_one_axis() {
        for view in [
            StandardView::Front,
            StandardView::Back,
            StandardView::Left,
            StandardView::Right,
            StandardView::Top,
            StandardView::Bottom,
        ] {
            let d = view.eye_dir();
            let big = d.iter().filter(|c| c.abs() > 0.9).count();
            assert_eq!(big, 1, "{view:?} should have one dominant axis: {d:?}");
            for c in d {
                assert!(
                    c.abs() > 0.9 || c.abs() < 0.05,
                    "{view:?} stray component {c} in {d:?}"
                );
            }
        }
    }

    /// Every mid-edge preset is a 45° two-axis view: the unit eye direction
    /// has exactly two non-zero components, each of magnitude `1/√2`, and the
    /// third is (up to floating-point) zero.
    #[test]
    fn edge_views_are_45_degree_two_axis_views() {
        assert_eq!(StandardView::EDGES.len(), 12);
        let half = std::f64::consts::FRAC_PI_4.cos(); // 1/√2
        for view in StandardView::EDGES {
            let d = view.eye_dir();
            let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
            assert!((len - 1.0).abs() < 1e-12, "{view:?} not a unit vector: {d:?}");
            let big = d.iter().filter(|c| c.abs() > 0.5).count();
            assert_eq!(big, 2, "{view:?} should have two non-zero axes: {d:?}");
            let zero = d.iter().filter(|c| c.abs() < 1e-9).count();
            assert_eq!(zero, 1, "{view:?} should have one zero axis: {d:?}");
            for c in d {
                assert!(
                    c.abs() < 1e-9 || (c.abs() - half).abs() < 1e-9,
                    "{view:?} component {c} is not 0 or ±1/√2"
                );
            }
        }
    }

    /// No edge preset coincides with a face preset or `Iso`, and all twelve
    /// are mutually distinct — so every cube edge snaps somewhere new.
    #[test]
    fn edge_views_are_distinct_from_faces_and_each_other() {
        let mut seen: Vec<[i64; 2]> = Vec::new();
        for edge in StandardView::EDGES {
            for other in StandardView::CUBE {
                if matches!(
                    other,
                    StandardView::Iso
                        | StandardView::Front
                        | StandardView::Back
                        | StandardView::Left
                        | StandardView::Right
                        | StandardView::Top
                        | StandardView::Bottom
                ) {
                    assert_ne!(
                        edge.angles(),
                        other.angles(),
                        "{edge:?} collides with face {other:?}"
                    );
                }
            }
            let (az, el) = edge.angles();
            let key = [(az * 1000.0) as i64, (el * 1000.0) as i64];
            assert!(!seen.contains(&key), "duplicate edge angles for {edge:?}");
            seen.push(key);
        }
        assert_eq!(seen.len(), 12);
    }

    /// `set_view` applies an edge preset's angles (target/distance untouched),
    /// and the resulting basis is well-conditioned.
    #[test]
    fn set_view_applies_edge_presets() {
        for view in StandardView::EDGES {
            let mut cam = Camera3D::default();
            cam.target = [1.0, 2.0, 3.0];
            cam.distance = 42.0;
            cam.set_view(view);
            assert_eq!(cam.azimuth, view.angles().0);
            assert_eq!(cam.elevation, view.angles().1);
            assert_eq!(cam.target, [1.0, 2.0, 3.0]);
            assert_eq!(cam.distance, 42.0);
            let (f, r, u) = cam.basis();
            for v in [f, r, u] {
                assert!(v.iter().all(|c| c.is_finite()), "{view:?} non-finite basis");
            }
        }
    }

    // ── ViewMode (Phase B2: ribbon Pan / Zoom wiring) ─────────────────────

    #[test]
    fn view_mode_defaults_to_orbit() {
        assert_eq!(ViewMode::default(), ViewMode::Orbit);
    }

    #[test]
    fn view_mode_labels_are_distinct_and_nonempty() {
        let labels: Vec<&str> = ViewMode::ALL.iter().map(|m| m.label()).collect();
        assert_eq!(labels, vec!["Orbit", "Pan", "Zoom"]);
        for l in labels {
            assert!(!l.is_empty());
        }
    }

    #[test]
    fn view_mode_all_covers_every_variant_once() {
        assert_eq!(ViewMode::ALL.len(), 3);
        assert!(ViewMode::ALL.contains(&ViewMode::Orbit));
        assert!(ViewMode::ALL.contains(&ViewMode::Pan));
        assert!(ViewMode::ALL.contains(&ViewMode::Zoom));
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
