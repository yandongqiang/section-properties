//! GPU 3-D viewport renderer (Phases P1 + P2) built on eframe's `glow`
//! backend.  No extra crates are used — only `glow` via `eframe`.
//!
//! # Why an offscreen framebuffer?
//!
//! `egui_glow` paints into the eframe window framebuffer, and eframe creates
//! that surface with `NativeOptions::depth_buffer == 0` (the default — see
//! `eframe::epi`: "egui doesn't need the depth buffer, so the default value is
//! 0").  Its `prepare_painting` additionally calls `glDisable(GL_DEPTH_TEST)`.
//! So the egui target has **no depth buffer**, and depth testing against it
//! would be silently ignored.
//!
//! We therefore render the 3-D scene into our **own** offscreen framebuffer
//! object, then blit the resolved colour texture over the egui viewport rect
//! with a second, trivial shader program.  (P2) When MSAA is available the
//! scene is drawn into a **multisampled** renderbuffer FBO and resolved into
//! the colour texture via `blit_framebuffer`; otherwise we draw straight into
//! the single-sample colour texture.  The FBO(s) are completeness-checked when
//! (re)created and resized alongside the viewport, so depth correctness never
//! depends on the window configuration.
//!
//! # What the GPU pass draws (P2)
//!
//! * **Members** as closed extruded rectangular prisms (12 triangles each).
//!   The cross-section is an *equivalent rectangle* derived from the member's
//!   stiffness (`A`, `Iy`, `Iz`) — see
//!   [`viewport_poc::scene::equivalent_rectangle`] — laid out along the
//!   member's local axes.  Triangles carry flat per-face normals and are
//!   Lambert-shaded by a single world-space directional light + ambient.
//! * **Nodes** as low-poly (icosahedron) shaded spheres.
//! * **Ground grid** as blended lines that alpha-fade with distance from the
//!   camera (a soft horizon), plus the opaque **global axes**.
//!
//! Back-face culling is enabled for the solids; all prisms/spheres are wound
//! counter-clockwise as seen from outside.
//!
//! # Shared state
//!
//! GL objects must outlive the per-frame [`egui_glow::CallbackFn`] closures
//! and are created while a GL context is current.  Because the
//! [`ViewportApp`](crate::ViewportApp) struct (and its constructor in
//! `ui/actions.rs`) intentionally lies outside this module's ownership, the
//! resources are parked in a module-level singleton guarded by a [`Mutex`]
//! instead of on the app struct.  The Shading / MSAA toggles live here too.

use std::sync::{Arc, Mutex, OnceLock};

use eframe::egui_glow;
use eframe::glow::{self, HasContext};

use viewport_poc::camera::Camera3D;
use viewport_poc::display::ResultDisplayState;
use viewport_poc::scene::{equivalent_rectangle, ViewportScene};
use viewport_poc::selection::SelectionState;

use super::theme::{AXIS_X, AXIS_Y, AXIS_Z, BG, GRID_C, MEMBER_C, NODE_C, SELECTED_C};

/// Floats per interleaved scene vertex: position(3) + normal(3) + colour(4).
const FLOATS_PER_VERTEX: usize = 10;

/// Vertex count of one extruded member prism (6 quads = 12 triangles).
#[cfg(test)]
const PRISM_VERTS: usize = 12 * 3;
/// Vertex count of one icosahedron node sphere (20 triangles).
#[cfg(test)]
const SPHERE_VERTS: usize = 20 * 3;

/// World-space direction the directional light comes *from* (normalised in the
/// fragment shader).  Fixed in world space, so faces brighten/darken as the
/// camera orbits — which is exactly what makes the solids read as 3-D.
const LIGHT_DIR: [f32; 3] = [0.35, 0.85, 0.45];

/// MSAA sample count requested for the offscreen target (clamped to the
/// context's `MAX_SAMPLES`).
const DESIRED_SAMPLES: i32 = 4;

/// Node sphere radius as a fraction of the scene bounding-box diagonal, so
/// nodes stay a sensible size regardless of model scale.
const NODE_RADIUS_FRACTION: f64 = 0.015;
/// Selected nodes are drawn this much larger than unselected ones.
const SELECTED_NODE_SCALE: f64 = 1.4;

/// Which viewport renderer is active.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RenderMode {
    /// Legacy software projection via `egui::Painter` (always available).
    Software,
    /// GPU rendering through an offscreen depth-tested framebuffer.
    Gpu,
}

impl RenderMode {
    /// Short label for the menu.
    pub const fn label(self) -> &'static str {
        match self {
            RenderMode::Software => "Software",
            RenderMode::Gpu => "GPU",
        }
    }
}

/// A batch of line segments sharing one line width.
struct LineBatch {
    width: f32,
    /// Whether this batch is alpha-blended (used by the fading ground grid).
    blend: bool,
    verts: Vec<f32>,
}

/// CPU-side scene description handed to the paint callback each frame.
///
/// Everything is in world space plus the view-projection matrix; the GPU does
/// the projection so depth testing is exact.
pub struct GlGeometry {
    /// Column-major view-projection matrix.
    mvp: [f32; 16],
    /// Line primitives, grouped by width (axes first, then the faded grid).
    line_batches: Vec<LineBatch>,
    /// Shaded triangles (member prisms + node spheres), interleaved per
    /// [`FLOATS_PER_VERTEX`].
    solids: Vec<f32>,
    /// Clear colour (the viewport background).
    clear: [f32; 4],
}

/// Persistent GL resources plus the current render mode and options.
struct Inner {
    mode: RenderMode,
    available: bool,
    renderer: Option<GlRenderer>,
    /// One-shot message (e.g. a runtime failure) to surface in the dock.
    pending_message: Option<String>,
    /// Per-face Lambert lighting on solids (`View ▸ Shading / lighting`).
    shading: bool,
    /// Request MSAA on the offscreen target (`View ▸ MSAA (4×)`).
    msaa: bool,
}

fn state() -> &'static Mutex<Inner> {
    static STATE: OnceLock<Mutex<Inner>> = OnceLock::new();
    STATE.get_or_init(|| {
        Mutex::new(Inner {
            mode: RenderMode::Software,
            available: false,
            renderer: None,
            pending_message: None,
            shading: true,
            msaa: true,
        })
    })
}

/// Create GL resources against the app's GL context (called once at startup).
///
/// On success the default mode becomes [`RenderMode::Gpu`]; on any failure the
/// mode stays [`RenderMode::Software`] and the error is returned for the
/// caller to surface.  Passing `None` (no glow context) is a failure too.
pub fn init(gl: Option<Arc<glow::Context>>) -> Result<(), String> {
    let mut inner = state().lock().unwrap_or_else(|e| e.into_inner());
    let Some(gl) = gl else {
        inner.available = false;
        inner.mode = RenderMode::Software;
        return Err("no OpenGL context (eframe was not built with the glow backend)".to_string());
    };
    match GlRenderer::new(&gl) {
        Ok(renderer) => {
            let v = gl.version();
            eprintln!(
                "[viewport-poc] GPU renderer ready (OpenGL {}.{}{}, MSAA max {} samples).",
                v.major,
                v.minor,
                if v.is_embedded { " ES" } else { "" },
                renderer.max_samples,
            );
            inner.renderer = Some(renderer);
            inner.available = true;
            inner.mode = RenderMode::Gpu;
            Ok(())
        }
        Err(e) => {
            eprintln!("[viewport-poc] GPU renderer init failed: {e}; falling back to software.");
            inner.renderer = None;
            inner.available = false;
            inner.mode = RenderMode::Software;
            Err(e)
        }
    }
}

/// Whether the GPU renderer initialised successfully.
pub fn available() -> bool {
    state().lock().unwrap_or_else(|e| e.into_inner()).available
}

/// Current render mode.
pub fn mode() -> RenderMode {
    state().lock().unwrap_or_else(|e| e.into_inner()).mode
}

/// Switch render mode.  Selecting [`RenderMode::Gpu`] when GPU init failed is
/// rejected (returns `false`) so the software path stays usable.
pub fn set_mode(mode: RenderMode) -> bool {
    let mut inner = state().lock().unwrap_or_else(|e| e.into_inner());
    match mode {
        RenderMode::Gpu if !inner.available => false,
        _ => {
            inner.mode = mode;
            true
        }
    }
}

/// Whether per-face lighting is on (GPU solids only).
pub fn shading_enabled() -> bool {
    state().lock().unwrap_or_else(|e| e.into_inner()).shading
}

/// Toggle per-face lighting.  A geometry rebuild isn't needed — lighting is a
/// draw-time uniform.
pub fn set_shading(on: bool) {
    state().lock().unwrap_or_else(|e| e.into_inner()).shading = on;
}

/// Whether MSAA is requested for the offscreen GPU target.
pub fn msaa_enabled() -> bool {
    state().lock().unwrap_or_else(|e| e.into_inner()).msaa
}

/// Toggle MSAA.  The FBO is rebuilt automatically on the next draw because the
/// requested sample state no longer matches the allocated one.
pub fn set_msaa(on: bool) {
    state().lock().unwrap_or_else(|e| e.into_inner()).msaa = on;
}

/// Take a pending one-shot message (if any) for display in the dock.
pub fn take_message() -> Option<String> {
    state()
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .pending_message
        .take()
}

/// Render `geom` into the offscreen framebuffer and blit it over `info`'s rect.
///
/// Invoked from within the [`egui_glow::CallbackFn`]; the mutex is locked only
/// for the duration of the draw.
pub fn paint(painter: &egui_glow::Painter, info: &egui::PaintCallbackInfo, geom: &GlGeometry) {
    let gl = painter.gl().clone();
    let restore = painter.intermediate_fbo();

    let vp = info.viewport_in_pixels();
    let rect = [
        vp.left_px,
        vp.from_bottom_px,
        vp.width_px.max(1),
        vp.height_px.max(1),
    ];

    let mut inner = state().lock().unwrap_or_else(|e| e.into_inner());
    let want_msaa = inner.msaa;
    let want_shade = inner.shading;

    let outcome = {
        let Some(renderer) = inner.renderer.as_mut() else {
            return;
        };
        let res = renderer.draw(&gl, restore, geom, rect, want_msaa, want_shade);
        let warn = renderer.take_warning();
        (res, warn)
    };

    match outcome.0 {
        Ok(()) => {
            if let Some(w) = outcome.1 {
                inner.pending_message = Some(w);
            }
        }
        Err(e) => {
            // Runtime failure: drop back to software and report once.
            eprintln!("[viewport-poc] GPU renderer runtime failure: {e}; falling back to software.");
            inner.pending_message = Some(format!(
                "GPU renderer failed ({e}); switched to Software renderer."
            ));
            inner.renderer = None;
            inner.available = false;
            inner.mode = RenderMode::Software;
        }
    }
}

fn rgba(c: egui::Color32) -> [f32; 4] {
    [
        c.r() as f32 / 255.0,
        c.g() as f32 / 255.0,
        c.b() as f32 / 255.0,
        1.0,
    ]
}

impl GlGeometry {
    /// Build the frame's geometry from the current camera / scene / display
    /// state.  Line widths are given in physical pixels.
    pub fn build(
        camera: &Camera3D,
        scene: &ViewportScene,
        display: &ResultDisplayState,
        selection: SelectionState,
        show_base: bool,
        aspect: f64,
        pixels_per_point: f32,
    ) -> Self {
        let ppp = pixels_per_point.max(0.1);
        let mut grid = LineBatch {
            width: 1.0 * ppp,
            blend: true,
            verts: Vec::new(),
        };
        let mut axes = LineBatch {
            width: 2.5 * ppp,
            blend: false,
            verts: Vec::new(),
        };
        let mut solids: Vec<f32> = Vec::new();

        // ── Ground grid (y = 0) with distance fade ──
        // Alpha falls off with distance from the camera so the grid dissolves
        // toward the horizon instead of stopping abruptly.  The fade window
        // scales with the camera distance, so it looks the same at any zoom.
        if display.show_grid {
            let eye = camera.eye();
            let near = camera.distance.max(1e-6);
            let span = (near * 1.5).max(1e-6);
            let col = rgba(GRID_C);
            let fade_alpha = |p: [f64; 3]| -> f32 {
                let d = ((p[0] - eye[0]).powi(2)
                    + (p[1] - eye[1]).powi(2)
                    + (p[2] - eye[2]).powi(2))
                .sqrt();
                let t = ((d - near) / span).clamp(0.0, 1.0);
                (1.0 - t * t * (3.0 - 2.0 * t)) as f32
            };
            let (gmin, gmax, step) = (-2.0_f64, 8.0_f64, 1.0_f64);
            let mut i = gmin;
            while i <= gmax {
                let a = [gmin, 0.0, i];
                let b = [gmax, 0.0, i];
                push_line_fade(&mut grid.verts, a, b, col, fade_alpha(a), fade_alpha(b));
                let c = [i, 0.0, gmin];
                let d = [i, 0.0, gmax];
                push_line_fade(&mut grid.verts, c, d, col, fade_alpha(c), fade_alpha(d));
                i += step;
            }
        }

        // ── Global axes ──
        // Drawn *before* the blended grid so the axis lines (which lie exactly
        // in the ground plane) win the depth test where they overlap a grid
        // line instead of being z-fought away.
        if display.show_global_axes {
            let o = [0.0, 0.0, 0.0];
            push_line(&mut axes.verts, o, [1.0, 0.0, 0.0], rgba(AXIS_X));
            push_line(&mut axes.verts, o, [0.0, 1.0, 0.0], rgba(AXIS_Y));
            push_line(&mut axes.verts, o, [0.0, 0.0, 1.0], rgba(AXIS_Z));
        }

        // ── Members (extruded prisms) and nodes (spheres) ──
        if show_base {
            let (bmin, bmax) = scene.bounding_box();
            let diag = ((bmax[0] - bmin[0]).powi(2)
                + (bmax[1] - bmin[1]).powi(2)
                + (bmax[2] - bmin[2]).powi(2))
            .sqrt();
            let node_r = (NODE_RADIUS_FRACTION * diag).max(1e-6);

            for mi in 0..scene.n_members() {
                let Some(&(ni, nj)) = scene.member_nodes.get(mi) else {
                    continue;
                };
                let (Some(&pi), Some(&pj)) = (
                    scene.node_positions.get(ni),
                    scene.node_positions.get(nj),
                ) else {
                    continue;
                };
                let Some((x_hat, y_hat, z_hat)) =
                    scene.member_local_axes(mi).or_else(|| fallback_local_axes(pi, pj))
                else {
                    continue;
                };
                let len = scene.member_length(mi).unwrap_or(0.0);
                let sec = scene.member_section(mi).unwrap_or_default();
                let (dim_y, dim_z) = equivalent_rectangle(sec.area, sec.iy, sec.iz, len);
                let is_sel = scene
                    .member_ids
                    .get(mi)
                    .is_some_and(|&id| selection == SelectionState::Member(id));
                let colour = if is_sel { rgba(SELECTED_C) } else { rgba(MEMBER_C) };
                push_prism(
                    &mut solids,
                    pi,
                    pj,
                    x_hat,
                    y_hat,
                    z_hat,
                    dim_y * 0.5,
                    dim_z * 0.5,
                    colour,
                );
            }

            for (i, &pos) in scene.node_positions.iter().enumerate() {
                let is_sel = scene
                    .node_ids
                    .get(i)
                    .is_some_and(|&id| selection == SelectionState::Node(id));
                let (colour, r) = if is_sel {
                    (rgba(SELECTED_C), node_r * SELECTED_NODE_SCALE)
                } else {
                    (rgba(NODE_C), node_r)
                };
                push_sphere(&mut solids, pos, r, colour);
            }
        }

        // Axes first (opaque, wins coincident depth), then the blended grid.
        let mut line_batches = Vec::new();
        for b in [axes, grid] {
            if !b.verts.is_empty() {
                line_batches.push(b);
            }
        }

        Self {
            mvp: camera.view_projection(aspect),
            line_batches,
            solids,
            clear: rgba(BG),
        }
    }
}

// ── Vertex emitters ─────────────────────────────────────────────────────

fn push_vertex(buf: &mut Vec<f32>, p: [f64; 3], n: [f64; 3], colour: [f32; 4]) {
    buf.extend_from_slice(&[
        p[0] as f32,
        p[1] as f32,
        p[2] as f32,
        n[0] as f32,
        n[1] as f32,
        n[2] as f32,
        colour[0],
        colour[1],
        colour[2],
        colour[3],
    ]);
}

fn push_line(buf: &mut Vec<f32>, a: [f64; 3], b: [f64; 3], colour: [f32; 4]) {
    push_vertex(buf, a, [0.0; 3], colour);
    push_vertex(buf, b, [0.0; 3], colour);
}

fn push_line_fade(
    buf: &mut Vec<f32>,
    a: [f64; 3],
    b: [f64; 3],
    colour: [f32; 4],
    alpha_a: f32,
    alpha_b: f32,
) {
    push_vertex(buf, a, [0.0; 3], [colour[0], colour[1], colour[2], alpha_a]);
    push_vertex(buf, b, [0.0; 3], [colour[0], colour[1], colour[2], alpha_b]);
}

fn push_tri(buf: &mut Vec<f32>, a: [f64; 3], b: [f64; 3], c: [f64; 3], n: [f64; 3], colour: [f32; 4]) {
    push_vertex(buf, a, n, colour);
    push_vertex(buf, b, n, colour);
    push_vertex(buf, c, n, colour);
}

fn push_quad(
    buf: &mut Vec<f32>,
    a: [f64; 3],
    b: [f64; 3],
    c: [f64; 3],
    d: [f64; 3],
    n: [f64; 3],
    colour: [f32; 4],
) {
    push_tri(buf, a, b, c, n, colour);
    push_tri(buf, a, c, d, n, colour);
}

/// Append a closed rectangular prism from `pi` to `pj`.
///
/// The cross-section spans `±hy` along local `y_hat` and `±hz` along local
/// `z_hat`.  Every face is flat-shaded with its outward normal and wound
/// **counter-clockwise as seen from outside**, so `GL_CULL_FACE`/`GL_BACK`
/// with the default `CCW` front face keeps the outside and culls the inside.
///
/// The winding was derived with `(b−a)×(c−a)` pointing along the face normal;
/// `member_local_axes` returns a right-handed `(x_hat, y_hat, z_hat)` with
/// `x × y = z`.
fn push_prism(
    buf: &mut Vec<f32>,
    pi: [f64; 3],
    pj: [f64; 3],
    x_hat: [f64; 3],
    y_hat: [f64; 3],
    z_hat: [f64; 3],
    hy: f64,
    hz: f64,
    colour: [f32; 4],
) {
    let off = |base: [f64; 3], oy: f64, oz: f64| -> [f64; 3] {
        [
            base[0] + oy * y_hat[0] + oz * z_hat[0],
            base[1] + oy * y_hat[1] + oz * z_hat[1],
            base[2] + oy * y_hat[2] + oz * z_hat[2],
        ]
    };
    // Cross-section corners, in (y, z): 0=(+,+), 1=(−,+), 2=(−,−), 3=(+,−).
    let i0 = off(pi, hy, hz);
    let i1 = off(pi, -hy, hz);
    let i2 = off(pi, -hy, -hz);
    let i3 = off(pi, hy, -hz);
    let j0 = off(pj, hy, hz);
    let j1 = off(pj, -hy, hz);
    let j2 = off(pj, -hy, -hz);
    let j3 = off(pj, hy, -hz);

    let px = x_hat;
    let nx = [-px[0], -px[1], -px[2]];
    let py = y_hat;
    let ny = [-py[0], -py[1], -py[2]];
    let pz = z_hat;
    let nz = [-pz[0], -pz[1], -pz[2]];

    // End caps: j end faces +x, i end faces −x.
    push_tri(buf, j0, j1, j2, px, colour);
    push_tri(buf, j0, j2, j3, px, colour);
    push_tri(buf, i0, i2, i1, nx, colour);
    push_tri(buf, i0, i3, i2, nx, colour);

    // Sides, one per perimeter edge (outward normal in brackets):
    //   0→1 (+z), 1→2 (−y), 2→3 (−z), 3→0 (+y).
    push_quad(buf, i0, i1, j1, j0, pz, colour);
    push_quad(buf, i1, i2, j2, j1, ny, colour);
    push_quad(buf, i2, i3, j3, j2, nz, colour);
    push_quad(buf, i3, i0, j0, j3, py, colour);
}

/// Append a low-poly (icosahedron) shaded sphere of radius `r`.
///
/// Smooth normals (the normalised vertex direction) make the 20 faces read as
/// a round ball; the faces are wound CCW outward.
fn push_sphere(buf: &mut Vec<f32>, center: [f64; 3], r: f64, colour: [f32; 4]) {
    let (verts, faces) = icosahedron();
    for face in faces {
        for idx in face {
            let d = verts[idx];
            let p = [center[0] + r * d[0], center[1] + r * d[1], center[2] + r * d[2]];
            push_vertex(buf, p, d, colour);
        }
    }
}

/// Unit icosahedron: 12 normalised vertices and 20 outward-CCW triangles.
fn icosahedron() -> ([[f64; 3]; 12], [[usize; 3]; 20]) {
    let t = (1.0 + 5.0_f64.sqrt()) * 0.5;
    let raw = [
        [-1.0, t, 0.0],
        [1.0, t, 0.0],
        [-1.0, -t, 0.0],
        [1.0, -t, 0.0],
        [0.0, -1.0, t],
        [0.0, 1.0, t],
        [0.0, -1.0, -t],
        [0.0, 1.0, -t],
        [t, 0.0, -1.0],
        [t, 0.0, 1.0],
        [-t, 0.0, -1.0],
        [-t, 0.0, 1.0],
    ];
    let mut verts = [[0.0_f64; 3]; 12];
    for (out, &v) in verts.iter_mut().zip(raw.iter()) {
        *out = normalize3(v);
    }
    let faces = [
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];
    (verts, faces)
}

fn normalize3(v: [f64; 3]) -> [f64; 3] {
    let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if n > 0.0 && n.is_finite() {
        [v[0] / n, v[1] / n, v[2] / n]
    } else {
        [0.0, 0.0, 0.0]
    }
}

fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Build an arbitrary right-handed local frame from a member's axis, used when
/// [`ViewportScene::member_local_axes`] fails (e.g. a `ref_vec` parallel to the
/// member).  This keeps such members visible as prisms instead of vanishing.
/// Returns `None` only for a zero-length / non-finite member.
fn fallback_local_axes(pi: [f64; 3], pj: [f64; 3]) -> Option<([f64; 3], [f64; 3], [f64; 3])> {
    let d = [pj[0] - pi[0], pj[1] - pi[1], pj[2] - pi[2]];
    let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    if !l.is_finite() || l <= 1e-12 {
        return None;
    }
    let x = [d[0] / l, d[1] / l, d[2] / l];
    // Reference not parallel to the axis (y unless the axis is near-vertical).
    let up = if x[1].abs() < 0.9 {
        [0.0, 1.0, 0.0]
    } else {
        [1.0, 0.0, 0.0]
    };
    let z = normalize3(cross3(x, up));
    if z == [0.0, 0.0, 0.0] {
        return None;
    }
    let y = cross3(z, x); // unit, and x × y = z (right-handed).
    Some((x, y, z))
}

/// All live GL objects for the 3-D pass and the blit.
struct GlRenderer {
    scene_program: glow::Program,
    blit_program: glow::Program,
    scene_vao: glow::VertexArray,
    vbo: glow::Buffer,
    blit_vao: glow::VertexArray,
    #[allow(dead_code)]
    blit_vbo: glow::Buffer,

    u_mvp: Option<glow::UniformLocation>,
    u_shade: Option<glow::UniformLocation>,
    u_light_dir: Option<glow::UniformLocation>,
    u_blit_tex: Option<glow::UniformLocation>,

    /// Single-sample resolve target: colour texture (blitted to the screen) +
    /// depth renderbuffer.  Also the direct render target when MSAA is off.
    fbo: glow::Framebuffer,
    color_tex: glow::Texture,
    depth_rb: glow::Renderbuffer,

    /// Multisample scene target (colour + depth renderbuffers), resolved into
    /// `fbo` via `blit_framebuffer`.
    msaa_fbo: glow::Framebuffer,
    msaa_color_rb: glow::Renderbuffer,
    msaa_depth_rb: glow::Renderbuffer,

    /// Current attachment size; `(0, 0)` forces (re)creation.
    target_size: (i32, i32),
    /// Whether the multisample attachments are currently valid.
    msaa_built: bool,
    /// Set when single-sample FBO creation failed; the renderer then draws
    /// nothing.
    broken: bool,
    /// `GL_MAX_SAMPLES` for the live context (0 = query failed).
    max_samples: i32,
    /// Last MSAA sample count logged (avoids per-resize log spam).
    reported_samples: i32,
    /// A one-shot warning (e.g. MSAA degraded) to surface in the dock.
    warning: Option<String>,

    max_line_width: f32,
}

impl GlRenderer {
    fn new(gl: &glow::Context) -> Result<Self, String> {
        unsafe {
            let header = shader_header(gl);

            let scene_vs = format!(
                "{header}\
in vec3 a_pos;\n\
in vec3 a_normal;\n\
in vec4 a_color;\n\
uniform mat4 u_mvp;\n\
out vec4 v_color;\n\
out vec3 v_normal;\n\
void main() {{\n\
    v_color = a_color;\n\
    v_normal = a_normal;\n\
    gl_Position = u_mvp * vec4(a_pos, 1.0);\n\
}}\n"
            );
            let scene_fs = format!(
                "{header}\
in vec4 v_color;\n\
in vec3 v_normal;\n\
uniform float u_shade;\n\
uniform vec3 u_light_dir;\n\
out vec4 frag_color;\n\
void main() {{\n\
    if (u_shade < 0.5) {{\n\
        frag_color = v_color;\n\
        return;\n\
    }}\n\
    vec3 n = normalize(v_normal);\n\
    float diff = max(dot(n, normalize(u_light_dir)), 0.0);\n\
    float lit = 0.35 + 0.65 * diff;\n\
    frag_color = vec4(v_color.rgb * lit, v_color.a);\n\
}}\n"
            );
            let blit_vs = format!(
                "{header}\
in vec2 a_pos;\n\
out vec2 v_uv;\n\
void main() {{\n\
    v_uv = a_pos * 0.5 + 0.5;\n\
    gl_Position = vec4(a_pos, 0.0, 1.0);\n\
}}\n"
            );
            let blit_fs = format!(
                "{header}\
in vec2 v_uv;\n\
uniform sampler2D u_tex;\n\
out vec4 frag_color;\n\
void main() {{\n\
    frag_color = texture(u_tex, v_uv);\n\
}}\n"
            );

            let scene_program = link_program(
                gl,
                &scene_vs,
                &scene_fs,
                &[(0, "a_pos"), (1, "a_normal"), (2, "a_color")],
            )?;
            let blit_program = link_program(gl, &blit_vs, &blit_fs, &[(0, "a_pos")])?;

            let u_mvp = gl.get_uniform_location(scene_program, "u_mvp");
            let u_shade = gl.get_uniform_location(scene_program, "u_shade");
            let u_light_dir = gl.get_uniform_location(scene_program, "u_light_dir");
            let u_blit_tex = gl.get_uniform_location(blit_program, "u_tex");

            // Scene geometry buffer + VAO (pos3 + normal3 + colour4).
            let scene_vao = gl
                .create_vertex_array()
                .map_err(|e| format!("create VAO failed: {e}"))?;
            let vbo = gl
                .create_buffer()
                .map_err(|e| format!("create VBO failed: {e}"))?;
            gl.bind_vertex_array(Some(scene_vao));
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));
            let stride = (FLOATS_PER_VERTEX * std::mem::size_of::<f32>()) as i32;
            gl.enable_vertex_attrib_array(0);
            gl.vertex_attrib_pointer_f32(0, 3, glow::FLOAT, false, stride, 0);
            gl.enable_vertex_attrib_array(1);
            gl.vertex_attrib_pointer_f32(1, 3, glow::FLOAT, false, stride, 12);
            gl.enable_vertex_attrib_array(2);
            gl.vertex_attrib_pointer_f32(2, 4, glow::FLOAT, false, stride, 24);
            gl.bind_vertex_array(None);

            // Blit quad buffer + VAO (two NDC triangles => a fullscreen quad).
            let blit_vao = gl
                .create_vertex_array()
                .map_err(|e| format!("create blit VAO failed: {e}"))?;
            let blit_vbo = gl
                .create_buffer()
                .map_err(|e| format!("create blit VBO failed: {e}"))?;
            let quad: [f32; 12] = [
                -1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0, 1.0,
            ];
            gl.bind_vertex_array(Some(blit_vao));
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(blit_vbo));
            gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, as_bytes(&quad), glow::STATIC_DRAW);
            gl.enable_vertex_attrib_array(0);
            gl.vertex_attrib_pointer_f32(0, 2, glow::FLOAT, false, 0, 0);
            gl.bind_vertex_array(None);

            let fbo = gl
                .create_framebuffer()
                .map_err(|e| format!("create FBO failed: {e}"))?;
            let color_tex = gl
                .create_texture()
                .map_err(|e| format!("create colour texture failed: {e}"))?;
            let depth_rb = gl
                .create_renderbuffer()
                .map_err(|e| format!("create depth renderbuffer failed: {e}"))?;

            let msaa_fbo = gl
                .create_framebuffer()
                .map_err(|e| format!("create MSAA FBO failed: {e}"))?;
            let msaa_color_rb = gl
                .create_renderbuffer()
                .map_err(|e| format!("create MSAA colour renderbuffer failed: {e}"))?;
            let msaa_depth_rb = gl
                .create_renderbuffer()
                .map_err(|e| format!("create MSAA depth renderbuffer failed: {e}"))?;

            let max_samples = gl.get_parameter_i32(glow::MAX_SAMPLES);

            let mut max_line_width = 1.0_f32;
            let mut range = [1.0_f32, 1.0_f32];
            gl.get_parameter_f32_slice(glow::ALIASED_LINE_WIDTH_RANGE, &mut range);
            if range[1].is_finite() && range[1] >= 1.0 {
                max_line_width = range[1];
            }

            Ok(Self {
                scene_program,
                blit_program,
                scene_vao,
                vbo,
                blit_vao,
                blit_vbo,
                u_mvp,
                u_shade,
                u_light_dir,
                u_blit_tex,
                fbo,
                color_tex,
                depth_rb,
                msaa_fbo,
                msaa_color_rb,
                msaa_depth_rb,
                target_size: (0, 0),
                msaa_built: false,
                broken: false,
                max_samples,
                reported_samples: 0,
                warning: None,
                max_line_width,
            })
        }
    }

    /// Record a one-shot warning (kept only if none is pending).
    fn warn_once(&mut self, msg: String) {
        if self.warning.is_none() {
            eprintln!("[viewport-poc] {msg}");
            self.warning = Some(msg);
        }
    }

    /// Take the pending warning, if any.
    fn take_warning(&mut self) -> Option<String> {
        self.warning.take()
    }

    /// (Re)create the colour/depth (and, when requested, multisample)
    /// attachments if the viewport size or MSAA request changed.
    fn ensure_target(
        &mut self,
        gl: &glow::Context,
        w: i32,
        h: i32,
        want_msaa: bool,
    ) -> Result<(), String> {
        let want_msaa = want_msaa && self.max_samples >= 2;
        if self.target_size == (w, h) && self.msaa_built == want_msaa && !self.broken {
            return Ok(());
        }
        self.target_size = (w, h);

        unsafe {
            // ── Single-sample resolve target: colour texture + depth RB ──
            gl.bind_texture(glow::TEXTURE_2D, Some(self.color_tex));
            gl.tex_image_2d(
                glow::TEXTURE_2D,
                0,
                glow::RGBA8 as i32,
                w,
                h,
                0,
                glow::RGBA,
                glow::UNSIGNED_BYTE,
                None,
            );
            gl.tex_parameter_i32(
                glow::TEXTURE_2D,
                glow::TEXTURE_MIN_FILTER,
                glow::LINEAR as i32,
            );
            gl.tex_parameter_i32(
                glow::TEXTURE_2D,
                glow::TEXTURE_MAG_FILTER,
                glow::LINEAR as i32,
            );
            gl.tex_parameter_i32(
                glow::TEXTURE_2D,
                glow::TEXTURE_WRAP_S,
                glow::CLAMP_TO_EDGE as i32,
            );
            gl.tex_parameter_i32(
                glow::TEXTURE_2D,
                glow::TEXTURE_WRAP_T,
                glow::CLAMP_TO_EDGE as i32,
            );

            gl.bind_renderbuffer(glow::RENDERBUFFER, Some(self.depth_rb));
            gl.renderbuffer_storage(glow::RENDERBUFFER, glow::DEPTH_COMPONENT24, w, h);

            gl.bind_framebuffer(glow::FRAMEBUFFER, Some(self.fbo));
            gl.framebuffer_texture_2d(
                glow::FRAMEBUFFER,
                glow::COLOR_ATTACHMENT0,
                glow::TEXTURE_2D,
                Some(self.color_tex),
                0,
            );
            gl.framebuffer_renderbuffer(
                glow::FRAMEBUFFER,
                glow::DEPTH_ATTACHMENT,
                glow::RENDERBUFFER,
                Some(self.depth_rb),
            );

            let status = gl.check_framebuffer_status(glow::FRAMEBUFFER);
            if status != glow::FRAMEBUFFER_COMPLETE {
                self.broken = true;
                self.msaa_built = false;
                return Err(format!("framebuffer incomplete (0x{status:x})"));
            }
            self.broken = false;
            self.msaa_built = false;

            // ── Optional multisample scene target ──
            if want_msaa {
                let samples = self.max_samples.min(DESIRED_SAMPLES).max(2);
                gl.bind_renderbuffer(glow::RENDERBUFFER, Some(self.msaa_color_rb));
                gl.renderbuffer_storage_multisample(
                    glow::RENDERBUFFER,
                    samples,
                    glow::RGBA8,
                    w,
                    h,
                );
                gl.bind_renderbuffer(glow::RENDERBUFFER, Some(self.msaa_depth_rb));
                gl.renderbuffer_storage_multisample(
                    glow::RENDERBUFFER,
                    samples,
                    glow::DEPTH_COMPONENT24,
                    w,
                    h,
                );
                gl.bind_framebuffer(glow::FRAMEBUFFER, Some(self.msaa_fbo));
                gl.framebuffer_renderbuffer(
                    glow::FRAMEBUFFER,
                    glow::COLOR_ATTACHMENT0,
                    glow::RENDERBUFFER,
                    Some(self.msaa_color_rb),
                );
                gl.framebuffer_renderbuffer(
                    glow::FRAMEBUFFER,
                    glow::DEPTH_ATTACHMENT,
                    glow::RENDERBUFFER,
                    Some(self.msaa_depth_rb),
                );
                let ms_status = gl.check_framebuffer_status(glow::FRAMEBUFFER);
                if ms_status == glow::FRAMEBUFFER_COMPLETE {
                    self.msaa_built = true;
                    if self.reported_samples != samples {
                        eprintln!(
                            "[viewport-poc] GPU offscreen target: {samples}× MSAA ({w}×{h})."
                        );
                        self.reported_samples = samples;
                    }
                } else {
                    self.warn_once(format!(
                        "MSAA {samples}× target incomplete (0x{ms_status:x}); \
                         using a non-MSAA offscreen target."
                    ));
                }
            }
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn draw(
        &mut self,
        gl: &glow::Context,
        restore: Option<glow::Framebuffer>,
        geom: &GlGeometry,
        rect: [i32; 4],
        want_msaa: bool,
        want_shade: bool,
    ) -> Result<(), String> {
        let [vx, vy, vw, vh] = rect;
        let (w, h) = (vw.max(1), vh.max(1));
        self.ensure_target(gl, w, h, want_msaa)?;
        let use_msaa = self.msaa_built;

        unsafe {
            // ── Pass 1: scene into the offscreen target, with depth ──
            let scene_fbo = if use_msaa { self.msaa_fbo } else { self.fbo };
            gl.bind_framebuffer(glow::FRAMEBUFFER, Some(scene_fbo));
            gl.viewport(0, 0, w, h);
            // egui leaves a scissor rect in *screen* pixels enabled; it would
            // clip the offscreen target, so disable it for the whole callback.
            gl.disable(glow::SCISSOR_TEST);
            gl.enable(glow::DEPTH_TEST);
            gl.depth_func(glow::LESS);
            gl.depth_mask(true);
            gl.disable(glow::CULL_FACE);
            gl.disable(glow::BLEND);
            gl.clear_color(geom.clear[0], geom.clear[1], geom.clear[2], geom.clear[3]);
            gl.clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);

            gl.use_program(Some(self.scene_program));
            gl.uniform_matrix_4_f32_slice(self.u_mvp.as_ref(), false, &geom.mvp);
            gl.uniform_3_f32(
                self.u_light_dir.as_ref(),
                LIGHT_DIR[0],
                LIGHT_DIR[1],
                LIGHT_DIR[2],
            );
            gl.bind_vertex_array(Some(self.scene_vao));
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(self.vbo));

            // ── Lines (unshaded) ──
            gl.uniform_1_f32(self.u_shade.as_ref(), 0.0);
            for batch in &geom.line_batches {
                let count = (batch.verts.len() / FLOATS_PER_VERTEX) as i32;
                if count == 0 {
                    continue;
                }
                if batch.blend {
                    gl.enable(glow::BLEND);
                    gl.blend_func(glow::SRC_ALPHA, glow::ONE_MINUS_SRC_ALPHA);
                    gl.blend_equation(glow::FUNC_ADD);
                } else {
                    gl.disable(glow::BLEND);
                }
                gl.buffer_data_u8_slice(
                    glow::ARRAY_BUFFER,
                    as_bytes(&batch.verts),
                    glow::DYNAMIC_DRAW,
                );
                gl.line_width(batch.width.clamp(1.0, self.max_line_width));
                gl.draw_arrays(glow::LINES, 0, count);
            }
            gl.disable(glow::BLEND);

            // ── Solids: member prisms + node spheres (lit, back-face culled) ──
            if !geom.solids.is_empty() {
                gl.enable(glow::CULL_FACE);
                gl.cull_face(glow::BACK);
                gl.front_face(glow::CCW);
                gl.uniform_1_f32(
                    self.u_shade.as_ref(),
                    if want_shade { 1.0 } else { 0.0 },
                );
                gl.buffer_data_u8_slice(
                    glow::ARRAY_BUFFER,
                    as_bytes(&geom.solids),
                    glow::DYNAMIC_DRAW,
                );
                gl.draw_arrays(
                    glow::TRIANGLES,
                    0,
                    (geom.solids.len() / FLOATS_PER_VERTEX) as i32,
                );
                gl.disable(glow::CULL_FACE);
            }

            // ── Resolve the multisample target into the colour texture ──
            if use_msaa {
                gl.bind_framebuffer(glow::READ_FRAMEBUFFER, Some(self.msaa_fbo));
                gl.bind_framebuffer(glow::DRAW_FRAMEBUFFER, Some(self.fbo));
                gl.blit_framebuffer(
                    0,
                    0,
                    w,
                    h,
                    0,
                    0,
                    w,
                    h,
                    glow::COLOR_BUFFER_BIT,
                    glow::NEAREST,
                );
                gl.bind_framebuffer(glow::FRAMEBUFFER, Some(self.fbo));
            }

            // ── Pass 2: blit the texture over the egui viewport rect ──
            gl.bind_framebuffer(glow::FRAMEBUFFER, restore);
            gl.viewport(vx, vy, w, h);
            gl.disable(glow::DEPTH_TEST);
            gl.depth_mask(false);
            gl.disable(glow::CULL_FACE);
            gl.use_program(Some(self.blit_program));
            gl.active_texture(glow::TEXTURE0);
            gl.bind_texture(glow::TEXTURE_2D, Some(self.color_tex));
            gl.uniform_1_i32(self.u_blit_tex.as_ref(), 0);
            gl.bind_vertex_array(Some(self.blit_vao));
            gl.draw_arrays(glow::TRIANGLES, 0, 6);

            // Leave tidy handles; egui restores its own program/VAO/scissor.
            gl.bind_texture(glow::TEXTURE_2D, None);
            gl.bind_vertex_array(None);
        }
        Ok(())
    }
}

/// Build a `#version` header matching the live context, mirroring the logic
/// egui_glow uses for its own shaders.
fn shader_header(gl: &glow::Context) -> String {
    let v = gl.version();
    if v.is_embedded {
        "#version 300 es\nprecision mediump float;\n".to_string()
    } else if v.major > 3 || (v.major == 3 && v.minor >= 3) {
        "#version 330 core\n".to_string()
    } else {
        "#version 140\n".to_string()
    }
}

fn compile_shader(gl: &glow::Context, kind: u32, source: &str) -> Result<glow::Shader, String> {
    unsafe {
        let shader = gl.create_shader(kind)?;
        gl.shader_source(shader, source);
        gl.compile_shader(shader);
        if !gl.get_shader_compile_status(shader) {
            let log = gl.get_shader_info_log(shader);
            gl.delete_shader(shader);
            return Err(format!("shader compile error: {log}"));
        }
        Ok(shader)
    }
}

fn link_program(
    gl: &glow::Context,
    vs_src: &str,
    fs_src: &str,
    attribs: &[(u32, &str)],
) -> Result<glow::Program, String> {
    let vs = compile_shader(gl, glow::VERTEX_SHADER, vs_src)?;
    let fs = compile_shader(gl, glow::FRAGMENT_SHADER, fs_src)?;
    unsafe {
        let program = gl.create_program()?;
        gl.attach_shader(program, vs);
        gl.attach_shader(program, fs);
        for &(index, name) in attribs {
            gl.bind_attrib_location(program, index, name);
        }
        gl.link_program(program);
        let ok = gl.get_program_link_status(program);
        gl.delete_shader(vs);
        gl.delete_shader(fs);
        if !ok {
            let log = gl.get_program_info_log(program);
            gl.delete_program(program);
            return Err(format!("program link error: {log}"));
        }
        Ok(program)
    }
}

/// Reinterpret a slice of `f32` as bytes for `glBufferData`.
fn as_bytes(data: &[f32]) -> &[u8] {
    unsafe { std::slice::from_raw_parts(data.as_ptr() as *const u8, std::mem::size_of_val(data)) }
}

#[cfg(test)]
mod tests {
    use viewport_poc::display::ResultViewMode;
    use viewport_poc::edit_model::{EditModel, NodalLoadSpec};

    use super::*;

    fn scene() -> ViewportScene {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        m.add_member(a, b).unwrap();
        m.set_nodal_load(b, NodalLoadSpec::new(0.0, -1.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        ViewportScene::from_edit_model(&m)
    }

    /// Whether any vertex in `buf` carries colour `c` (RGBA at offset 6..10).
    fn contains_colour(buf: &[f32], c: [f32; 4]) -> bool {
        buf.chunks_exact(FLOATS_PER_VERTEX).any(|v| {
            v[6] == c[0] && v[7] == c[1] && v[8] == c[2] && v[9] == c[3]
        })
    }

    #[test]
    fn geometry_has_axes_grid_and_solid_members() {
        let cam = Camera3D::fit([0.0, 0.0, 0.0], [3.0, 1.0, 1.0]);
        let s = scene();
        let d = ResultDisplayState::default();
        let geom = GlGeometry::build(&cam, &s, &d, SelectionState::None, true, 4.0 / 3.0, 1.0);
        // Axes + grid lines.
        assert_eq!(geom.line_batches.len(), 2);
        // One prism + two node spheres worth of triangles.
        assert_eq!(
            geom.solids.len(),
            (PRISM_VERTS + 2 * SPHERE_VERTS) * FLOATS_PER_VERTEX
        );
        // Every solid vertex must be finite, and normals unit-ish.
        assert!(geom.solids.iter().all(|v| v.is_finite()));
        assert!(geom.mvp.iter().all(|v| v.is_finite()));
        // Member colour present, selected colour not.
        assert!(contains_colour(&geom.solids, rgba(MEMBER_C)));
        assert!(!contains_colour(&geom.solids, rgba(SELECTED_C)));
    }

    #[test]
    fn deformed_mode_without_undeformed_skips_base_geometry() {
        let cam = Camera3D::default();
        let s = scene();
        let mut d = ResultDisplayState::default();
        d.mode = ResultViewMode::Deformed;
        d.show_undeformed = false;
        let geom = GlGeometry::build(&cam, &s, &d, SelectionState::None, false, 1.0, 1.0);
        // No members/nodes; grid and axes remain (both enabled by default).
        assert!(geom.solids.is_empty());
        assert_eq!(geom.line_batches.len(), 2);
    }

    #[test]
    fn selected_member_uses_highlight_colour() {
        let cam = Camera3D::default();
        let s = scene();
        let d = ResultDisplayState::default();
        let sel = SelectionState::Member(s.member_ids[0]);
        let geom = GlGeometry::build(&cam, &s, &d, sel, true, 1.0, 1.0);
        assert!(contains_colour(&geom.solids, rgba(SELECTED_C)));
        // The plain member colour is gone; nodes keep their own colour.
        assert!(!contains_colour(&geom.solids, rgba(MEMBER_C)));
        assert!(contains_colour(&geom.solids, rgba(NODE_C)));
    }

    #[test]
    fn empty_scene_still_produces_grid_axes_and_node_sphere() {
        let cam = Camera3D::default();
        let mut m = EditModel::new();
        let _ = m.add_node(0.0, 0.0, 0.0).unwrap();
        let s = ViewportScene::from_edit_model(&m);
        let d = ResultDisplayState::default();
        let geom = GlGeometry::build(&cam, &s, &d, SelectionState::None, true, 1.0, 1.0);
        assert_eq!(geom.line_batches.len(), 2);
        assert_eq!(geom.solids.len(), SPHERE_VERTS * FLOATS_PER_VERTEX);
    }

    /// The prism is closed (12 triangles) and its +x cap is wound CCW as seen
    /// from +x, i.e. `(b−a)×(c−a)` points along the outward face normal.
    #[test]
    fn prism_is_closed_with_outward_ccw_caps() {
        let mut buf = Vec::new();
        push_prism(
            &mut buf,
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            0.1,
            0.2,
            [1.0, 1.0, 1.0, 1.0],
        );
        assert_eq!(buf.len(), PRISM_VERTS * FLOATS_PER_VERTEX);

        // First triangle = +x end cap; normal attribute must be +x…
        assert_eq!(&buf[3..6], &[1.0, 0.0, 0.0]);
        // …and the winding must agree with it.
        let v0 = [buf[0], buf[1], buf[2]];
        let v1 = [buf[10], buf[11], buf[12]];
        let v2 = [buf[20], buf[21], buf[22]];
        let e1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
        let e2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];
        let n = [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ];
        assert!(n[0] > 0.0, "cap winding should face +x, got {n:?}");
    }

    /// A low-poly sphere is closed and its normals are unit vectors.
    #[test]
    fn sphere_normals_are_unit() {
        let mut buf = Vec::new();
        push_sphere(&mut buf, [1.0, 2.0, 3.0], 0.5, [0.5, 0.5, 0.5, 1.0]);
        assert_eq!(buf.len(), SPHERE_VERTS * FLOATS_PER_VERTEX);
        for v in buf.chunks_exact(FLOATS_PER_VERTEX) {
            let len = (v[3] * v[3] + v[4] * v[4] + v[5] * v[5]).sqrt();
            assert!((len - 1.0).abs() < 1e-5, "normal length {len}");
        }
    }

    /// The grid batch is blended and its far vertices are more transparent
    /// than the near ones (distance fade).
    #[test]
    fn grid_alpha_fades_with_distance() {
        let cam = Camera3D::default();
        let s = scene();
        let d = ResultDisplayState::default();
        let geom = GlGeometry::build(&cam, &s, &d, SelectionState::None, false, 1.0, 1.0);
        let grid = geom
            .line_batches
            .iter()
            .find(|b| b.blend)
            .expect("grid batch present");
        assert!(grid.verts.iter().all(|v| v.is_finite()));
        let alphas: Vec<f32> = grid.verts.chunks_exact(FLOATS_PER_VERTEX).map(|v| v[9]).collect();
        let min = alphas.iter().cloned().fold(f32::INFINITY, f32::min);
        let max = alphas.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        assert!(min < max, "alpha should vary across the grid");
        assert!(min >= 0.0 && max <= 1.0);
    }

    #[test]
    fn shading_and_msaa_defaults_toggle() {
        // Defaults are on; setters round-trip (uses the shared singleton).
        set_shading(true);
        assert!(shading_enabled());
        set_shading(false);
        assert!(!shading_enabled());
        set_shading(true);

        set_msaa(true);
        assert!(msaa_enabled());
        set_msaa(false);
        assert!(!msaa_enabled());
        set_msaa(true);
    }

    /// The fallback frame (used when a member's `ref_vec` is parallel to its
    /// axis) must be orthonormal and right-handed: `x × y = z`.
    #[test]
    fn fallback_local_axes_is_orthonormal() {
        let len = |v: [f64; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

        for (pi, pj) in [
            ([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]), // vertical (up ref would be parallel)
            ([0.0, 0.0, 0.0], [3.0, 0.0, 0.0]),
            ([1.0, 2.0, 3.0], [-1.0, 2.0, 6.0]),
        ] {
            let (x, y, z) = fallback_local_axes(pi, pj).unwrap();
            for v in [x, y, z] {
                assert!((len(v) - 1.0).abs() < 1e-12, "not unit: {v:?}");
            }
            assert!(dot(x, y).abs() < 1e-12);
            assert!(dot(x, z).abs() < 1e-12);
            assert!(dot(y, z).abs() < 1e-12);
            let c = cross3(x, y);
            assert!(
                (c[0] - z[0]).abs() < 1e-12
                    && (c[1] - z[1]).abs() < 1e-12
                    && (c[2] - z[2]).abs() < 1e-12,
                "not right-handed"
            );
        }

        assert!(fallback_local_axes([1.0, 1.0, 1.0], [1.0, 1.0, 1.0]).is_none());
    }

    #[test]
    fn mode_labels() {
        assert_eq!(RenderMode::Software.label(), "Software");
        assert_eq!(RenderMode::Gpu.label(), "GPU");
    }
}
