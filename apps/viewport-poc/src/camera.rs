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
        Self {
            target: center,
            distance: diag * 2.5,
            ..Default::default()
        }
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
}
