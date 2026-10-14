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
//! # Capped section cut (P7-1)
//!
//! When the section cut is active, the solids are drawn twice inside the same
//! offscreen pass: the normal front-face pass (clipped to the kept half), then
//! a **cap pass** that re-draws the same solids with front faces culled and
//! the **same** clip plane in a flat "cut material" colour (`u_cap_mode`).
//! The hole the cut leaves exposes the *kept* half's interior (the removed
//! half's shell is clipped away entirely), so re-drawing the kept half's
//! interior back faces fills the opening and the cut reads as solid.  The
//! depth test stays `LESS`, so the cap only shows where the front pass
//! removed geometry.  See [`GlGeometry::cap_plane`] and the draw code in
//! [`GlRenderer::draw`]; the sense is verified for every axis and both `flip`
//! states by `cap_pass_sense_same_plane_fills_kept_interior`.
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

use structural_analysis::FrameAnalysisResult3D;

use viewport_poc::camera::Camera3D;
use viewport_poc::display::{
    displacement_magnitudes, magnitude_color, max_abs, normalize, ResultDisplayState,
    ResultViewMode,
};
use viewport_poc::edit_model::SupportSpec;
use viewport_poc::scene::{equivalent_rectangle, ViewportScene};
use viewport_poc::selection::SelectionSet;

use super::theme::{
    AXIS_X, AXIS_Y, AXIS_Z, BG, DEF_MEMBER, DEF_NODE, GRID_C, LOAD_F_C, LOAD_M_C, MEMBER_C,
    MEMBER_LOAD_C, NODE_C, SELECTED_C, SUPPORT_C,
};

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

/// Samples per member for the deformed-shape curve (matches the painter path
/// in [`super::viewport`]).
const DEFORM_SAMPLES: usize = 12;
/// World-space length of the nodal-load force arrow (matches the painter).
const NODAL_ARROW_LEN: f64 = 0.5;
/// Length of the local-axis triad drawn for the selected member.
const TRIAD_LEN: f64 = 0.5;
/// Sides used for round tube cross-sections (moment arcs, arrow shafts).
const TUBE_SIDES: usize = 8;
/// Sides used for arrow-head cones.
const CONE_SIDES: usize = 10;

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
    /// Whether the section-cut clip distance applies to this batch.  Reference
    /// geometry (grid, global axes) is deliberately **exempt** so it stays a
    /// stable backdrop; entity / result overlays are clipped.
    clip: bool,
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
    /// Section-cut plane as `[n.x, n.y, n.z, offset]`, pre-flipped so a
    /// positive `gl_ClipDistance[0]` means "keep".  `None` = no cut.
    clip_plane: Option<[f32; 4]>,
    /// **Cap pass** (Phase P7-1) plane — deliberately the *same* keep-plane
    /// as `clip_plane`: the opening the cut leaves exposes the **kept**
    /// half's interior (the removed half's shell is clipped away entirely),
    /// so the cap must re-draw the kept half's interior back faces.  Keeping
    /// the removed half instead would paint grey geometry in front of the
    /// model and leave the opening unfilled.  `None` = no cap (cut off, or
    /// the user disabled `SectionCut::cap`).
    cap_plane: Option<[f32; 4]>,
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
    ///
    /// Everything that lives in 3-D space — members/nodes, result symbols
    /// (supports, loads, reactions, end forces), the deformed shape, the
    /// displacement contour and the local-axis triads — is emitted as
    /// depth-tested GPU geometry.  Only text annotations (labels, legend,
    /// help, axis letters) stay on the painter.
    #[allow(clippy::too_many_arguments)]
    pub fn build(
        camera: &Camera3D,
        scene: &ViewportScene,
        display: &ResultDisplayState,
        selected: &SelectionSet,
        result: Option<&FrameAnalysisResult3D>,
        show_base: bool,
        aspect: f64,
        pixels_per_point: f32,
    ) -> Self {
        let ppp = pixels_per_point.max(0.1);
        let mut grid = LineBatch {
            width: 1.0 * ppp,
            blend: true,
            clip: false,
            verts: Vec::new(),
        };
        let mut axes = LineBatch {
            width: 2.5 * ppp,
            blend: false,
            clip: false,
            verts: Vec::new(),
        };
        // Depth-tested 3-D overlay lines (local triads, support hatching).
        let mut overlay = LineBatch {
            width: 2.0 * ppp,
            blend: false,
            clip: true,
            verts: Vec::new(),
        };
        let mut solids: Vec<f32> = Vec::new();

        // ── Section cut (Phase P5) ──
        // A single user clip plane, applied via `gl_ClipDistance[0]`.  The
        // grid and global axes are deliberately exempt (see their batch's
        // `clip` flag); the solids pass — members, nodes, deformed/contour
        // geometry, load/support/reaction/end-force glyphs — and the overlay
        // lines are clipped so nothing floats in front of the cut.
        let clip_plane = if display.section_cut.enabled {
            display.section_cut.gpu_plane()
        } else {
            None
        };
        // Phase P7-1 cap pass: only when the cut is on AND the user left
        // "Cap cut faces" enabled.  The plane is the **same** keep-plane as
        // `clip_plane` — the opening exposes the kept half's interior, so
        // the interior back faces that fill it live in the kept half (see
        // `cap_pass_sense_same_plane_fills_kept_interior`).  `None` disables
        // the cap pass entirely.
        let cap_plane = if display.section_cut.enabled && display.section_cut.cap {
            clip_plane
        } else {
            None
        };

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

        // ── Scene-derived scale for the 3-D result symbols ──
        // Node radius and a slightly larger "glyph" size for supports / loads,
        // both fractions of the model's bounding-box diagonal so symbols stay
        // legible at any model scale.
        let (bmin, bmax) = scene.bounding_box();
        let diag = ((bmax[0] - bmin[0]).powi(2)
            + (bmax[1] - bmin[1]).powi(2)
            + (bmax[2] - bmin[2]).powi(2))
        .sqrt();
        let node_r = (NODE_RADIUS_FRACTION * diag).max(1e-6);
        let glyph = (node_r * 2.0).max(1e-6);

        // ── Displacement magnitudes for the contour colouring (item 5) ──
        let contour = if display.mode == ResultViewMode::Displacement {
            result.map(|r| {
                let mags = displacement_magnitudes(&r.displacements, scene.n_nodes());
                let max_mag = max_abs(&mags);
                (mags, max_mag)
            })
        } else {
            None
        };

        // ── Members (extruded prisms) and nodes (spheres) ──
        if show_base {
            for mi in 0..scene.n_members() {
                if scene
                    .member_ids
                    .get(mi)
                    .is_some_and(|&id| display.is_member_hidden(id))
                {
                    continue;
                }
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
                    .is_some_and(|&id| selected.contains_member(id));
                // Contour mode colours the prism along its length by the nodal
                // displacement magnitude; otherwise selection/member colour.
                let (ci, cj) = match &contour {
                    Some((mags, max_mag)) if *max_mag > 0.0 => {
                        let ti = mags.get(ni).map_or(0.0, |&m| normalize(m, *max_mag));
                        let tj = mags.get(nj).map_or(0.0, |&m| normalize(m, *max_mag));
                        (contour_colour(ti), contour_colour(tj))
                    }
                    _ => {
                        let c = if is_sel { rgba(SELECTED_C) } else { rgba(MEMBER_C) };
                        (c, c)
                    }
                };
                push_prism_grad(
                    &mut solids,
                    pi,
                    pj,
                    x_hat,
                    y_hat,
                    z_hat,
                    dim_y * 0.5,
                    dim_z * 0.5,
                    ci,
                    cj,
                );
            }

            for (i, &pos) in scene.node_positions.iter().enumerate() {
                if scene
                    .node_ids
                    .get(i)
                    .is_some_and(|&id| display.is_node_hidden(id))
                {
                    continue;
                }
                let is_sel = scene
                    .node_ids
                    .get(i)
                    .is_some_and(|&id| selected.contains_node(id));
                let (colour, r) = if is_sel {
                    (rgba(SELECTED_C), node_r * SELECTED_NODE_SCALE)
                } else {
                    (rgba(NODE_C), node_r)
                };
                push_sphere(&mut solids, pos, r, colour);
            }
        }

        // ── Support glyphs (item 1) ──
        if show_base {
            for (i, &pos) in scene.node_positions.iter().enumerate() {
                if scene
                    .node_ids
                    .get(i)
                    .is_some_and(|&id| display.is_node_hidden(id))
                {
                    continue;
                }
                if let Some(spec) = scene.node_supports.get(i).copied().flatten() {
                    push_support(&mut solids, &mut overlay.verts, pos, spec, glyph);
                }
            }
        }

        // ── Nodal load glyphs (item 2) ──
        if show_base {
            for (i, &pos) in scene.node_positions.iter().enumerate() {
                if scene
                    .node_ids
                    .get(i)
                    .is_some_and(|&id| display.is_node_hidden(id))
                {
                    continue;
                }
                let Some(load) = scene.node_loads.get(i).copied().flatten() else {
                    continue;
                };
                if load.is_zero() || !load.is_finite() {
                    continue;
                }
                let fmag = load.force_magnitude();
                if fmag > 1e-12 {
                    let dir = [load.fx / fmag, load.fy / fmag, load.fz / fmag];
                    push_arrow(
                        &mut solids,
                        pos,
                        dir,
                        NODAL_ARROW_LEN,
                        glyph * 0.22,
                        NODAL_ARROW_LEN * 0.35,
                        glyph * 0.55,
                        rgba(LOAD_F_C),
                    );
                }
                let mmag = load.moment_magnitude();
                if mmag > 1e-12 {
                    let axis = [load.mx / mmag, load.my / mmag, load.mz / mmag];
                    push_moment_arc(&mut solids, pos, axis, glyph * 3.0, glyph * 0.18, rgba(LOAD_M_C));
                }
            }
        }

        // ── Member uniform (distributed) load arrows ──
        if show_base {
            for (mi, &_) in scene.member_nodes.iter().enumerate() {
                if scene
                    .member_ids
                    .get(mi)
                    .is_some_and(|&id| display.is_member_hidden(id))
                {
                    continue;
                }
                let Some(load) = scene.member_loads.get(mi).copied().flatten() else {
                    continue;
                };
                if load.is_zero() || !load.is_finite() {
                    continue;
                }
                let (ni, nj) = scene.member_nodes[mi];
                let pi = scene.node_positions[ni];
                let pj = scene.node_positions[nj];
                let Some((x_hat, y_hat, z_hat)) =
                    scene.member_local_axes(mi).or_else(|| fallback_local_axes(pi, pj))
                else {
                    continue;
                };
                let len = scene.member_length(mi).unwrap_or(0.0);
                if len < 1e-12 {
                    continue;
                }
                let qg = [
                    load.qx * x_hat[0] + load.qy * y_hat[0] + load.qz * z_hat[0],
                    load.qx * x_hat[1] + load.qy * y_hat[1] + load.qz * z_hat[1],
                    load.qx * x_hat[2] + load.qy * y_hat[2] + load.qz * z_hat[2],
                ];
                let qmag = qg[0].hypot(qg[1]).hypot(qg[2]);
                if qmag < 1e-12 {
                    continue;
                }
                let dir = [qg[0] / qmag, qg[1] / qmag, qg[2] / qmag];
                let arrow_len = (0.3 * len.min(1.0)).max(1e-6);
                let n_arrows = 5;
                for k in 1..n_arrows {
                    let t = k as f64 / n_arrows as f64;
                    let base = [
                        pi[0] + t * (pj[0] - pi[0]),
                        pi[1] + t * (pj[1] - pi[1]),
                        pi[2] + t * (pj[2] - pi[2]),
                    ];
                    push_arrow(
                        &mut solids,
                        base,
                        dir,
                        arrow_len,
                        glyph * 0.16,
                        arrow_len * 0.35,
                        glyph * 0.42,
                        rgba(MEMBER_LOAD_C),
                    );
                }
            }
        }

        // ── Member point load arrows ──
        if show_base {
            for (mi, &_) in scene.member_nodes.iter().enumerate() {
                if scene
                    .member_ids
                    .get(mi)
                    .is_some_and(|&id| display.is_member_hidden(id))
                {
                    continue;
                }
                let Some(loads) = scene.member_point_loads.get(mi) else {
                    continue;
                };
                if loads.is_empty() {
                    continue;
                }
                let (ni, nj) = scene.member_nodes[mi];
                let pi = scene.node_positions[ni];
                let pj = scene.node_positions[nj];
                let Some((x_hat, y_hat, z_hat)) =
                    scene.member_local_axes(mi).or_else(|| fallback_local_axes(pi, pj))
                else {
                    continue;
                };
                let len = scene.member_length(mi).unwrap_or(0.0);
                if len < 1e-12 {
                    continue;
                }
                for pl in loads {
                    if pl.is_zero() || !pl.is_finite() {
                        continue;
                    }
                    let fg = [
                        pl.fx * x_hat[0] + pl.fy * y_hat[0] + pl.fz * z_hat[0],
                        pl.fx * x_hat[1] + pl.fy * y_hat[1] + pl.fz * z_hat[1],
                        pl.fx * x_hat[2] + pl.fy * y_hat[2] + pl.fz * z_hat[2],
                    ];
                    let fmag = fg[0].hypot(fg[1]).hypot(fg[2]);
                    if fmag < 1e-12 {
                        continue;
                    }
                    let dir = [fg[0] / fmag, fg[1] / fmag, fg[2] / fmag];
                    let base = [
                        pi[0] + pl.xi * (pj[0] - pi[0]),
                        pi[1] + pl.xi * (pj[1] - pi[1]),
                        pi[2] + pl.xi * (pj[2] - pi[2]),
                    ];
                    let arrow_len = (0.25 * len.min(1.0)).max(1e-6);
                    push_arrow(
                        &mut solids,
                        base,
                        dir,
                        arrow_len,
                        glyph * 0.18,
                        arrow_len * 0.4,
                        glyph * 0.5,
                        rgba(MEMBER_LOAD_C),
                    );
                }
            }
        }

        // ── Deformed shape (item 4) ──
        // The sampled Hermite curve is swept as a GPU solid (extruded with the
        // member's equivalent rectangular section).  In deformed mode this
        // *replaces* the undeformed members unless "show undeformed" is on.
        if display.mode == ResultViewMode::Deformed {
            if let Some(res) = result {
                let scale = display.deformation_scale;
                let deformed = scene.deformed_positions(&res.displacements, scale);
                for mi in 0..scene.n_members() {
                    if scene
                        .member_ids
                        .get(mi)
                        .is_some_and(|&id| display.is_member_hidden(id))
                    {
                        continue;
                    }
                    let len = scene.member_length(mi).unwrap_or(0.0);
                    let sec = scene.member_section(mi).unwrap_or_default();
                    let (dy, dz) = equivalent_rectangle(sec.area, sec.iy, sec.iz, len);
                    let section = rect_section(dy * 0.5, dz * 0.5);
                    let mut pts: Vec<[f64; 3]> = Vec::with_capacity(DEFORM_SAMPLES);
                    for s in 0..DEFORM_SAMPLES {
                        let xi = s as f64 / (DEFORM_SAMPLES - 1) as f64;
                        match scene.member_deformed_point(mi, xi, &res.displacements, scale) {
                            Some(p) => pts.push(p),
                            None => break,
                        }
                    }
                    if pts.len() >= 2 {
                        push_sweep(&mut solids, &pts, &section, rgba(DEF_MEMBER), true);
                    }
                }
                for (i, &pos) in deformed.iter().enumerate() {
                    if scene
                        .node_ids
                        .get(i)
                        .is_some_and(|&id| display.is_node_hidden(id))
                    {
                        continue;
                    }
                    push_sphere(&mut solids, pos, node_r * 0.9, rgba(DEF_NODE));
                }
            }
        }

        // ── Reactions (item 3) ──
        if display.mode == ResultViewMode::Reaction {
            if let Some(res) = result {
                for (i, &pos) in scene.node_positions.iter().enumerate() {
                    if scene
                        .node_ids
                        .get(i)
                        .is_some_and(|&id| display.is_node_hidden(id))
                    {
                        continue;
                    }
                    if scene.node_supports.get(i).copied().flatten().is_none() {
                        continue;
                    }
                    let g = |k: usize| res.reactions.get(i * 6 + k).copied().unwrap_or(0.0);
                    let (fx, fy, fz) = (g(0), g(1), g(2));
                    let (mx, my, mz) = (g(3), g(4), g(5));

                    let fmag = (fx * fx + fy * fy + fz * fz).sqrt();
                    if fmag > 1e-9 {
                        let dir = [fx / fmag, fy / fmag, fz / fmag];
                        let scale = (0.3_f64.min(fmag * 1e-3)).max(glyph);
                        push_arrow(
                            &mut solids,
                            pos,
                            dir,
                            scale,
                            glyph * 0.22,
                            scale * 0.35,
                            glyph * 0.55,
                            rgba(SUPPORT_C),
                        );
                    }

                    let mmag = (mx * mx + my * my + mz * mz).sqrt();
                    if mmag > 1e-9 {
                        let axis = [mx / mmag, my / mmag, mz / mmag];
                        push_moment_arc(
                            &mut solids,
                            pos,
                            axis,
                            glyph * 3.0,
                            glyph * 0.18,
                            rgba(LOAD_M_C),
                        );
                    }
                }
            }
        }

        // ── Member end-force arrows (item 3) ──
        // Result overlays follow the *primary* (last-clicked) member so a
        // multi-selection still has well-defined end-force arrows.
        if display.mode == ResultViewMode::MemberEndForces {
            if let (Some(mid), Some(res)) = (selected.primary().member_id(), result) {
                if let Some(mi) = scene.member_index(mid) {
                    if !display.is_member_hidden(mid) {
                        if let Ok(f_global) = res.member_end_forces_global(mi) {
                            let (ni, nj) = scene.member_nodes[mi];
                            let pi = scene.node_positions[ni];
                            let pj = scene.node_positions[nj];
                            for (end, f) in [(pi, &f_global[0..3]), (pj, &f_global[6..9])] {
                                let (fx, fy, fz) = (f[0], f[1], f[2]);
                                let fmag = (fx * fx + fy * fy + fz * fz).sqrt();
                                if fmag <= 1e-9 {
                                    continue;
                                }
                                let dir = [fx / fmag, fy / fmag, fz / fmag];
                                let scale = (0.3_f64.min(fmag * 1e-3)).max(glyph);
                                push_arrow(
                                    &mut solids,
                                    end,
                                    dir,
                                    scale,
                                    glyph * 0.22,
                                    scale * 0.35,
                                    glyph * 0.55,
                                    rgba(SELECTED_C),
                                );
                            }
                        }
                    }
                }
            }
        }

        // ── Local member axes (item 6) ──
        if display.show_local_axes && result.is_some() {
            if let Some(mid) = selected.primary().member_id() {
                if let Some(mi) = scene.member_index(mid) {
                    if !display.is_member_hidden(mid) {
                        if let (Some(mpos), Some((xh, yh, zh))) =
                            (scene.member_midpoint(mi), scene.member_local_axes(mi))
                        {
                            push_triad(&mut overlay.verts, mpos, xh, yh, zh, TRIAD_LEN);
                        }
                    }
                }
            }
        }

        // ── Internal-force diagrams as depth-tested 3-D line strips (item 7) ──
        // The M/V/N strips are already computed in the member's local y/z; we
        // emit the same world-space segments the painter path uses, so they
        // occlude / are occluded correctly.  Labels + legend stay on the
        // painter (see `diagrams::show_diagram_labels`).
        if display.show_diagrams {
            if let Some(res) = result {
                if let Some(d) = super::diagrams::compute(scene, display, res, selected.primary()) {
                    for l in &d.lines {
                        push_line(&mut overlay.verts, l.a, l.b, rgb(l.color));
                    }
                }
            }
        }

        // Axes first (opaque, wins coincident depth), then the depth-tested
        // overlay lines, then the blended grid.
        let mut line_batches = Vec::new();
        for b in [axes, overlay, grid] {
            if !b.verts.is_empty() {
                line_batches.push(b);
            }
        }

        Self {
            mvp: camera.view_projection(aspect),
            line_batches,
            solids,
            clear: rgba(BG),
            clip_plane,
            cap_plane,
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
    push_tri_c(buf, a, b, c, n, colour, colour, colour);
}

/// Like [`push_tri`] but with a per-vertex colour (used for the displacement
/// contour gradient along a member).
#[allow(clippy::too_many_arguments)]
fn push_tri_c(
    buf: &mut Vec<f32>,
    a: [f64; 3],
    b: [f64; 3],
    c: [f64; 3],
    n: [f64; 3],
    ca: [f32; 4],
    cb: [f32; 4],
    cc: [f32; 4],
) {
    push_vertex(buf, a, n, ca);
    push_vertex(buf, b, n, cb);
    push_vertex(buf, c, n, cc);
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
    push_tri_c(buf, a, b, c, n, colour, colour, colour);
    push_tri_c(buf, a, c, d, n, colour, colour, colour);
}

/// Like [`push_quad`] but with a per-vertex colour (contour gradient).
#[allow(clippy::too_many_arguments)]
fn push_quad_c(
    buf: &mut Vec<f32>,
    a: [f64; 3],
    b: [f64; 3],
    c: [f64; 3],
    d: [f64; 3],
    n: [f64; 3],
    ca: [f32; 4],
    cb: [f32; 4],
    cc: [f32; 4],
    cd: [f32; 4],
) {
    push_tri_c(buf, a, b, c, n, ca, cb, cc);
    push_tri_c(buf, a, c, d, n, ca, cc, cd);
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
    push_prism_grad(buf, pi, pj, x_hat, y_hat, z_hat, hy, hz, colour, colour);
}

/// Like [`push_prism`] but with an independent colour for the `i` and `j` ends,
/// interpolated along the prism — used for the displacement contour gradient.
#[allow(clippy::too_many_arguments)]
fn push_prism_grad(
    buf: &mut Vec<f32>,
    pi: [f64; 3],
    pj: [f64; 3],
    x_hat: [f64; 3],
    y_hat: [f64; 3],
    z_hat: [f64; 3],
    hy: f64,
    hz: f64,
    ci: [f32; 4],
    cj: [f32; 4],
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

    // End caps: j end faces +x (colour `cj`), i end faces −x (colour `ci`).
    push_tri_c(buf, j0, j1, j2, px, cj, cj, cj);
    push_tri_c(buf, j0, j2, j3, px, cj, cj, cj);
    push_tri_c(buf, i0, i2, i1, nx, ci, ci, ci);
    push_tri_c(buf, i0, i3, i2, nx, ci, ci, ci);

    // Sides, one per perimeter edge (outward normal in brackets):
    //   0→1 (+z), 1→2 (−y), 2→3 (−z), 3→0 (+y).  The `i` corners carry `ci`
    // and the `j` corners `cj`, giving a linear gradient along the member.
    push_quad_c(buf, i0, i1, j1, j0, pz, ci, ci, cj, cj);
    push_quad_c(buf, i1, i2, j2, j1, ny, ci, ci, cj, cj);
    push_quad_c(buf, i2, i3, j3, j2, nz, ci, ci, cj, cj);
    push_quad_c(buf, i3, i0, j0, j3, py, ci, ci, cj, cj);
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

// ── 3-D result-symbol builders (Phase P3) ───────────────────────────────
//
// All of these emit the same interleaved `pos + normal + colour` vertices as
// the member prisms / node spheres, so they ride the existing shaded,
// depth-tested, back-face-culled solids pass (or the unshaded line pass for
// line triads).  Faces are wound CCW as seen from outside, matching the
// `GL_CULL_FACE` / `GL_BACK` / `CCW` state used by the renderer.

/// An opaque RGBA colour from an 8-bit RGB triple.
fn rgb(c: [u8; 3]) -> [f32; 4] {
    [
        c[0] as f32 / 255.0,
        c[1] as f32 / 255.0,
        c[2] as f32 / 255.0,
        1.0,
    ]
}

/// Displacement-contour colour at normalised magnitude `t ∈ [0, 1]`.
///
/// Delegates to [`viewport_poc::display::magnitude_color`] so the GPU
/// member colouring and the painter legend agree exactly.
fn contour_colour(t: f64) -> [f32; 4] {
    rgb(magnitude_color(t))
}

fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn add3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale3(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn len3(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

/// Outward (well, `(b−a)×(c−a)`) unit normal of a triangle, or zero for a
/// degenerate triangle.  Used so swept solids always get a correct flat normal
/// regardless of how the frame twisted.
fn tri_normal(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> [f64; 3] {
    normalize3(cross3(sub3(b, a), sub3(c, a)))
}

/// A right-handed frame `(u, v)` with `u × v = t`, where `t` is the unit
/// axis.  Used to lay out round cross-sections (cones, tubes, moment arcs).
fn frame_for_axis(t: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let t = normalize3(t);
    let reference = if t[1].abs() < 0.9 {
        [0.0, 1.0, 0.0]
    } else {
        [1.0, 0.0, 0.0]
    };
    let v = normalize3(cross3(t, reference));
    let u = cross3(v, t);
    (u, v)
}

/// Append a cone with its apex at `apex` and a circular base (radius `radius`)
/// centred at `base_center`.  Side triangles plus a base cap, all CCW outward.
fn push_cone(
    buf: &mut Vec<f32>,
    apex: [f64; 3],
    base_center: [f64; 3],
    radius: f64,
    sides: usize,
    colour: [f32; 4],
) {
    let axis = sub3(apex, base_center);
    let h = len3(axis);
    if !h.is_finite() || h <= 1e-12 || !(radius > 0.0) || !radius.is_finite() {
        return;
    }
    let t = scale3(axis, 1.0 / h);
    let (u, v) = frame_for_axis(t);
    let n = sides.max(3);
    let ring = |k: usize| -> [f64; 3] {
        let a = std::f64::consts::TAU * k as f64 / n as f64;
        let (s, c) = a.sin_cos();
        add3(base_center, add3(scale3(u, radius * c), scale3(v, radius * s)))
    };
    for k in 0..n {
        let p0 = ring(k);
        let p1 = ring(k + 1);
        // p0 → p1 → apex is CCW outward (derived from (b−a)×(c−a)).
        push_tri(buf, p0, p1, apex, tri_normal(p0, p1, apex), colour);
    }
    for k in 0..n {
        let p0 = ring(k);
        let p1 = ring(k + 1);
        // Base cap faces away from the apex.
        push_tri(
            buf,
            base_center,
            p1,
            p0,
            tri_normal(base_center, p1, p0),
            colour,
        );
    }
}

/// Circular cross-section of `sides` points (radius `r`), CCW in the
/// `(u, v)` plane of a sweep frame.
fn circle_section(sides: usize, r: f64) -> Vec<[f64; 2]> {
    let n = sides.max(3);
    (0..n)
        .map(|k| {
            let a = std::f64::consts::TAU * k as f64 / n as f64;
            [r * a.cos(), r * a.sin()]
        })
        .collect()
}

/// Rectangular cross-section `±hy × ±hz`, CCW in the `(u, v)` plane.
fn rect_section(hy: f64, hz: f64) -> [[f64; 2]; 4] {
    [[-hy, -hz], [hy, -hz], [hy, hz], [-hy, hz]]
}

/// Sweep a flat, CCW cross-section (given as local `(u, v)` offsets) along a
/// world-space polyline, producing a closed solid.
///
/// Frames are propagated by projecting the previous `u` onto the plane
/// perpendicular to the new tangent (rotation-minimising frames), so swept
/// solids don't flip or twist wildly.  Caps are added at both ends when `cap`.
/// Returns nothing for a degenerate polyline.
fn push_sweep(
    buf: &mut Vec<f32>,
    pts: &[[f64; 3]],
    section: &[[f64; 2]],
    colour: [f32; 4],
    cap: bool,
) {
    let m = pts.len();
    if m < 2 || section.len() < 3 {
        return;
    }
    // Segment tangents (averaged at interior points).
    let mut tangents = Vec::with_capacity(m);
    for i in 0..m {
        let d = if i == 0 {
            sub3(pts[1], pts[0])
        } else if i == m - 1 {
            sub3(pts[m - 1], pts[m - 2])
        } else {
            sub3(pts[i + 1], pts[i - 1])
        };
        let t = normalize3(d);
        if t == [0.0, 0.0, 0.0] {
            return; // degenerate / coincident samples
        }
        tangents.push(t);
    }
    // Rotation-minimising frames by parallel transport.
    let mut frames: Vec<([f64; 3], [f64; 3])> = Vec::with_capacity(m);
    let mut prev = frame_for_axis(tangents[0]).0;
    frames.push((prev, cross3(tangents[0], prev)));
    for i in 1..m {
        let t = tangents[i];
        let projected = sub3(prev, scale3(t, dot3(prev, t)));
        let l = len3(projected);
        let u = if l > 1e-9 {
            scale3(projected, 1.0 / l)
        } else {
            frame_for_axis(t).0
        };
        frames.push((u, cross3(t, u)));
        prev = u;
    }
    // Ring vertex in (u, v).
    let ring = |i: usize, s: [f64; 2]| -> [f64; 3] {
        let (u, v) = frames[i];
        add3(pts[i], add3(scale3(u, s[0]), scale3(v, s[1])))
    };
    let poly = section.len();
    for i in 0..m - 1 {
        for k in 0..poly {
            let k1 = (k + 1) % poly;
            let a = ring(i, section[k]);
            let b = ring(i, section[k1]);
            let c = ring(i + 1, section[k1]);
            let d = ring(i + 1, section[k]);
            push_quad(buf, a, b, c, d, tri_normal(a, b, c), colour);
        }
    }
    if cap {
        for k in 0..poly {
            let k1 = (k + 1) % poly;
            let a = ring(0, section[k]);
            let b = ring(0, section[k1]);
            push_tri(buf, pts[0], b, a, tri_normal(pts[0], b, a), colour);
            let d = ring(m - 1, section[k]);
            let e = ring(m - 1, section[k1]);
            push_tri(buf, pts[m - 1], d, e, tri_normal(pts[m - 1], d, e), colour);
        }
    }
}

/// A round tube of radius `r` along a polyline.
fn push_tube(buf: &mut Vec<f32>, pts: &[[f64; 3]], r: f64, sides: usize, colour: [f32; 4], cap: bool) {
    if !(r > 0.0) || !r.is_finite() {
        return;
    }
    let section = circle_section(sides, r);
    push_sweep(buf, pts, &section, colour, cap);
}

/// A straight cylinder between `a` and `b`.
fn push_cylinder(buf: &mut Vec<f32>, a: [f64; 3], b: [f64; 3], r: f64, sides: usize, colour: [f32; 4]) {
    push_tube(buf, &[a, b], r, sides, colour, true);
}

/// An arrow: a cylindrical shaft from `base` along `dir` with a conical head.
///
/// `length` is the full world-space length; `head_len` / `head_r` size the
/// cone.  No-op for a degenerate direction / length.
#[allow(clippy::too_many_arguments)]
fn push_arrow(
    buf: &mut Vec<f32>,
    base: [f64; 3],
    dir: [f64; 3],
    length: f64,
    shaft_r: f64,
    head_len: f64,
    head_r: f64,
    colour: [f32; 4],
) {
    let d = normalize3(dir);
    if d == [0.0, 0.0, 0.0] || !length.is_finite() || length <= 1e-12 {
        return;
    }
    if !base.iter().all(|v| v.is_finite()) || !shaft_r.is_finite() || !head_r.is_finite() {
        return;
    }
    let head_len = head_len.clamp(1e-9, length * 0.9);
    let tip = add3(base, scale3(d, length));
    let neck = add3(base, scale3(d, length - head_len));
    let sr = shaft_r.max(1e-9);
    push_cylinder(buf, base, neck, sr, TUBE_SIDES, colour);
    push_cone(buf, tip, neck, head_r.max(sr), CONE_SIDES, colour);
}

/// A moment glyph: a partial ring (arc) of tube around `axis` centred at
/// `center`, capped with a small cone to indicate the rotation sense.
fn push_moment_arc(
    buf: &mut Vec<f32>,
    center: [f64; 3],
    axis: [f64; 3],
    radius: f64,
    tube_r: f64,
    colour: [f32; 4],
) {
    let a = normalize3(axis);
    if a == [0.0, 0.0, 0.0]
        || !(radius > 0.0)
        || !radius.is_finite()
        || !(tube_r > 0.0)
        || !center.iter().all(|v| v.is_finite())
    {
        return;
    }
    let (u, v) = frame_for_axis(a);
    let n = 24;
    let span = std::f64::consts::TAU * 0.8;
    let mut pts = Vec::with_capacity(n + 1);
    for k in 0..=n {
        let th = span * k as f64 / n as f64;
        let (s, c) = th.sin_cos();
        pts.push(add3(
            center,
            add3(scale3(u, radius * c), scale3(v, radius * s)),
        ));
    }
    push_tube(buf, &pts, tube_r, 6, colour, false);
    let last = pts[n];
    let tan = normalize3(sub3(last, pts[n - 1]));
    if tan != [0.0, 0.0, 0.0] {
        push_cone(
            buf,
            add3(last, scale3(tan, tube_r * 4.0)),
            last,
            tube_r * 2.2,
            8,
            colour,
        );
    }
}

/// A support glyph at `pos`, sized by `s`:
/// * fixed → a flat hatched block (thin slab + diagonal hatch lines),
/// * pinned → a cone (apex at the node, base below),
/// * otherwise (custom / free) → a small sphere.
fn push_support(
    solids: &mut Vec<f32>,
    lines: &mut Vec<f32>,
    pos: [f64; 3],
    spec: SupportSpec,
    s: f64,
) {
    let colour = rgba(SUPPORT_C);
    let s = s.max(1e-9);
    if spec.is_fixed() {
        let half_y = (s * 0.35).max(1e-9);
        // A thin slab aligned to the world axes (x–z plane, thin along y).
        push_prism(
            solids,
            [pos[0], pos[1] - half_y, pos[2]],
            [pos[0], pos[1] + half_y, pos[2]],
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            s,
            s,
            colour,
        );
        // Diagonal hatch lines across the underside.
        let y = pos[1] - half_y;
        let n = 4;
        let step = 2.0 * s / n as f64;
        for k in 0..=n {
            let off = -s + step * k as f64;
            push_line(
                lines,
                [pos[0] + off, y, pos[2] - s],
                [pos[0] - s, y, pos[2] + off],
                colour,
            );
        }
    } else if spec.is_pinned() {
        let apex = pos;
        let base = [pos[0], pos[1] - 2.0 * s, pos[2]];
        push_cone(solids, apex, base, s, CONE_SIDES, colour);
    } else {
        push_sphere(solids, pos, (s * 0.8).max(1e-9), colour);
    }
}

/// A 3-D line triad `origin → origin + len·axis` for each local axis.
fn push_triad(lines: &mut Vec<f32>, origin: [f64; 3], xh: [f64; 3], yh: [f64; 3], zh: [f64; 3], len: f64) {
    for (axis, col) in [(xh, AXIS_X), (yh, AXIS_Y), (zh, AXIS_Z)] {
        let end = add3(origin, scale3(axis, len));
        push_line(lines, origin, end, rgba(col));
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
    u_clip_enabled: Option<glow::UniformLocation>,
    u_clip_plane: Option<glow::UniformLocation>,
    u_cap_mode: Option<glow::UniformLocation>,
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
uniform float u_clip_enabled;\n\
uniform vec4 u_clip_plane;\n\
out vec4 v_color;\n\
out vec3 v_normal;\n\
void main() {{\n\
    v_color = a_color;\n\
    v_normal = a_normal;\n\
    // Section cut (Phase P5): user clip plane 0.  Indexing with the constant\n\
    // 0 sizes the predeclared gl_ClipDistance array to one plane.  A large\n\
    // positive value means \"keep\" for batches that are exempt from the cut.\n\
    if (u_clip_enabled > 0.5) {{\n\
        gl_ClipDistance[0] = dot(a_pos, u_clip_plane.xyz) - u_clip_plane.w;\n\
    }} else {{\n\
        gl_ClipDistance[0] = 1.0e30;\n\
    }}\n\
    gl_Position = u_mvp * vec4(a_pos, 1.0);\n\
}}\n"
            );
            let scene_fs = format!(
                "{header}\
in vec4 v_color;\n\
in vec3 v_normal;\n\
uniform float u_shade;\n\
uniform float u_cap_mode;\n\
uniform vec3 u_light_dir;\n\
out vec4 frag_color;\n\
void main() {{\n\
    // Phase P7-1 cap pass: replace the vertex colour with a flat \"cut\n\
    // material\" tone (steel grey + a faint screen-space diagonal hatch) so\n\
    // the interior back faces drawn at the cut read as a solid cap rather\n\
    // than as lit member surfaces.\n\
    if (u_cap_mode > 0.5) {{\n\
        vec3 steel = vec3(0.60, 0.63, 0.68);\n\
        float stripe = mod(floor(gl_FragCoord.x + gl_FragCoord.y), 8.0);\n\
        frag_color = vec4(mix(steel, steel * 0.78, step(4.0, stripe)), 1.0);\n\
        return;\n\
    }}\n\
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
            let u_clip_enabled = gl.get_uniform_location(scene_program, "u_clip_enabled");
            let u_clip_plane = gl.get_uniform_location(scene_program, "u_clip_plane");
            let u_cap_mode = gl.get_uniform_location(scene_program, "u_cap_mode");
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
                u_clip_enabled,
                u_clip_plane,
                u_cap_mode,
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
            // Section cut: enable user clip plane 0 for the whole scene pass and
            // pick the plane per draw via `u_clip_enabled`.  A degenerate plane
            // is treated as "no cut".
            let plane = geom.clip_plane.unwrap_or([0.0, 1.0, 0.0, 0.0]);
            gl.uniform_4_f32(
                self.u_clip_plane.as_ref(),
                plane[0],
                plane[1],
                plane[2],
                plane[3],
            );
            let cut_on = geom.clip_plane.is_some();
            gl.enable(glow::CLIP_DISTANCE0);
            // Cap mode is only turned on for the cap pass below; make sure the
            // normal (front) passes start with it off.
            gl.uniform_1_f32(self.u_cap_mode.as_ref(), 0.0);
            gl.bind_vertex_array(Some(self.scene_vao));
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(self.vbo));

            // ── Lines (unshaded) ──
            gl.uniform_1_f32(self.u_shade.as_ref(), 0.0);
            for batch in &geom.line_batches {
                let count = (batch.verts.len() / FLOATS_PER_VERTEX) as i32;
                if count == 0 {
                    continue;
                }
                // Reference geometry (grid / global axes) is exempt from the cut.
                let clip = if cut_on && batch.clip { 1.0 } else { 0.0 };
                gl.uniform_1_f32(self.u_clip_enabled.as_ref(), clip);
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
                gl.uniform_1_f32(self.u_clip_enabled.as_ref(), if cut_on { 1.0 } else { 0.0 });
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

            // ── Cap pass (Phase P7-1): make the cut look solid ──
            // Re-draw the solids with **front faces culled** (keep the
            // interior back faces) under the **same** clip plane (the kept
            // half).  The opening the cut leaves exposes the kept half's
            // interior — its front faces were culled or clipped away, so
            // those back faces now fill the hole and the cut reads as solid.
            // `LESS` (the frame default) keeps the cap strictly behind the
            // front geometry that survived, so it only shows where the main
            // pass removed geometry.  Flat "cut material" shading is selected
            // by `u_cap_mode`.  Draw order: opaque front geometry first
            // (above), then this fill — same offscreen FBO + depth buffer,
            // so nothing is drawn in front of visible geometry.
            if let Some(cap) = geom.cap_plane {
                if !geom.solids.is_empty() {
                    gl.enable(glow::CULL_FACE);
                    gl.cull_face(glow::FRONT);
                    gl.front_face(glow::CCW);
                    gl.depth_func(glow::LESS);
                    gl.uniform_1_f32(self.u_clip_enabled.as_ref(), 1.0);
                    gl.uniform_4_f32(
                        self.u_clip_plane.as_ref(),
                        cap[0],
                        cap[1],
                        cap[2],
                        cap[3],
                    );
                    gl.uniform_1_f32(self.u_cap_mode.as_ref(), 1.0);
                    // The VBO still holds the solids buffer from the draw just
                    // above (no line batch touched it since) — no re-upload.
                    gl.draw_arrays(
                        glow::TRIANGLES,
                        0,
                        (geom.solids.len() / FLOATS_PER_VERTEX) as i32,
                    );
                    gl.uniform_1_f32(self.u_cap_mode.as_ref(), 0.0);
                    gl.uniform_1_f32(self.u_clip_enabled.as_ref(), 0.0);
                    gl.depth_func(glow::LESS);
                    gl.disable(glow::CULL_FACE);
                }
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

            // Section cut is only for the 3-D scene: the blit program does not
            // write gl_ClipDistance, so leaving the plane enabled would clip
            // the fullscreen quad with undefined distances.
            gl.disable(glow::CLIP_DISTANCE0);

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
    use viewport_poc::selection::SelectionState;

    use super::*;

    fn scene() -> ViewportScene {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        m.add_member(a, b).unwrap();
        ViewportScene::from_edit_model(&m)
    }

    /// A scene carrying a nodal force + moment and a member uniform + point
    /// load, plus a fixed support, to exercise the 3-D symbol builders.
    fn scene_with_loads() -> ViewportScene {
        use viewport_poc::edit_model::{MemberPointLoad3D, MemberUniformLoad3D};

        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        let mem = m.add_member(a, b).unwrap();
        m.fix_node(a).unwrap();
        m.set_nodal_load(b, NodalLoadSpec::new(0.0, -1.0, 0.0, 0.0, 0.0, 1.0))
            .unwrap();
        m.set_member_uniform_load(mem, MemberUniformLoad3D::new(0.0, -1.0, 0.0))
            .unwrap();
        m.add_member_point_load(mem, MemberPointLoad3D::new(0.5, 0.0, -1.0, 0.0))
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
        let geom = GlGeometry::build(&cam, &s, &d, &SelectionSet::default(), None, true, 4.0 / 3.0, 1.0);
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
        let geom = GlGeometry::build(&cam, &s, &d, &SelectionSet::default(), None, false, 1.0, 1.0);
        // No members/nodes; grid and axes remain (both enabled by default).
        assert!(geom.solids.is_empty());
        assert_eq!(geom.line_batches.len(), 2);
    }

    #[test]
    fn selected_member_uses_highlight_colour() {
        let cam = Camera3D::default();
        let s = scene();
        let d = ResultDisplayState::default();
        let sel = SelectionSet::from_single(SelectionState::Member(s.member_ids[0]));
        let geom = GlGeometry::build(&cam, &s, &d, &sel, None, true, 1.0, 1.0);
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
        let geom = GlGeometry::build(&cam, &s, &d, &SelectionSet::default(), None, true, 1.0, 1.0);
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
        let geom = GlGeometry::build(&cam, &s, &d, &SelectionSet::default(), None, false, 1.0, 1.0);
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

    // ── Phase P3: 3-D result-symbol builders ────────────────────────────

    /// Number of vertices a swept solid emits: `6·p·m` (side quads plus two
    /// capped ends) for `p` section points over `m` polyline points.
    fn sweep_verts(section_pts: usize, polyline_pts: usize) -> usize {
        6 * section_pts * polyline_pts
    }

    #[test]
    fn tube_vertex_count() {
        let mut buf = Vec::new();
        push_tube(
            &mut buf,
            &[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            0.05,
            8,
            [1.0, 1.0, 1.0, 1.0],
            true,
        );
        assert_eq!(buf.len(), sweep_verts(8, 2) * FLOATS_PER_VERTEX);
        assert!(buf.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn sweep_rect_vertex_count() {
        let mut buf = Vec::new();
        let pts = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.3, 0.0]];
        let section = rect_section(0.1, 0.05);
        push_sweep(&mut buf, &pts, &section, [1.0, 0.0, 0.0, 1.0], true);
        assert_eq!(buf.len(), sweep_verts(4, 3) * FLOATS_PER_VERTEX);
        assert!(buf.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn arrow_is_a_shaft_plus_a_cone() {
        let mut buf = Vec::new();
        push_arrow(
            &mut buf,
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            1.0,
            0.02,
            0.3,
            0.06,
            [1.0, 1.0, 1.0, 1.0],
        );
        // Shaft = capped 2-point tube; head = cone.
        let expected = sweep_verts(TUBE_SIDES, 2) + 6 * CONE_SIDES;
        assert_eq!(buf.len(), expected * FLOATS_PER_VERTEX);
        assert!(buf.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn degenerate_arrow_and_cone_emit_nothing() {
        let mut buf = Vec::new();
        push_arrow(&mut buf, [0.0; 3], [0.0; 3], 1.0, 0.02, 0.3, 0.06, [1.0; 4]);
        push_cone(&mut buf, [0.0; 3], [0.0; 3], 0.1, 8, [1.0; 4]);
        assert!(buf.is_empty());
    }

    /// The cone's side faces must be wound outward: for an apex above the base
    /// at the origin, the first side normal points outwards (+x) and upwards.
    #[test]
    fn cone_side_normals_point_outward() {
        let mut buf = Vec::new();
        push_cone(
            &mut buf,
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0],
            0.5,
            6,
            [1.0, 1.0, 1.0, 1.0],
        );
        assert_eq!(buf.len(), (6 * 6) * FLOATS_PER_VERTEX);
        // Normal attribute of the first vertex.
        assert!(buf[3] > 0.0, "expected outward +x normal, got {}", buf[3]);
        assert!(buf[4] > 0.0, "expected upward +y normal, got {}", buf[4]);
    }

    #[test]
    fn moment_arc_builds_finite_geometry() {
        let mut buf = Vec::new();
        push_moment_arc(&mut buf, [1.0, 2.0, 3.0], [0.0, 0.0, 1.0], 0.4, 0.05, [1.0; 4]);
        assert!(!buf.is_empty());
        assert!(buf.iter().all(|v| v.is_finite()));
    }

    /// The contour ramp must be identical to the painter's `magnitude_color`
    /// so the GPU members and the legend agree.
    #[test]
    fn contour_colour_matches_display_ramp() {
        for t in [0.0, 0.25, 0.5, 0.75, 1.0] {
            assert_eq!(contour_colour(t), rgb(magnitude_color(t)), "t = {t}");
        }
        // Endpoints: blue → red per the documented ramp.
        assert_eq!(&contour_colour(0.0)[..3], &[80.0 / 255.0, 1.0, 1.0]);
        assert_eq!(&contour_colour(1.0)[..3], &[1.0, 80.0 / 255.0, 80.0 / 255.0]);
    }

    /// A scene with loads and a fixed support emits solid symbols (arrow,
    /// moment arc, support block) and depth-tested hatch lines.
    #[test]
    fn loads_and_supports_emit_solid_symbols() {
        let cam = Camera3D::fit([0.0, 0.0, 0.0], [3.0, 1.0, 1.0]);
        let s = scene_with_loads();
        let d = ResultDisplayState::default();
        let geom = GlGeometry::build(
            &cam,
            &s,
            &d,
            &SelectionSet::default(),
            None,
            true,
            4.0 / 3.0,
            1.0,
        );
        // Axes + support hatching + grid.
        assert_eq!(geom.line_batches.len(), 3);
        assert!(geom.solids.iter().all(|v| v.is_finite()));
        assert!(contains_colour(&geom.solids, rgba(SUPPORT_C)));
        assert!(contains_colour(&geom.solids, rgba(LOAD_F_C)));
        assert!(contains_colour(&geom.solids, rgba(LOAD_M_C)));
        assert!(contains_colour(&geom.solids, rgba(MEMBER_LOAD_C)));
    }

    /// A solved single-member cantilever, used to exercise the result-shape
    /// builders (deformed / contour / reactions / end forces / local axes).
    fn solved_scene() -> (ViewportScene, FrameAnalysisResult3D) {
        let mut m = EditModel::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(3.0, 0.0, 0.0).unwrap();
        m.add_member(a, b).unwrap();
        m.fix_node(a).unwrap();
        m.set_nodal_load(b, NodalLoadSpec::new(0.0, -1000.0, 0.0, 0.0, 0.0, 0.0))
            .unwrap();
        let scene = ViewportScene::from_edit_model(&m);
        let res = m.to_frame_model().unwrap().solve().unwrap();
        (scene, res)
    }

    #[test]
    fn deformed_mode_emits_swept_solids() {
        let cam = Camera3D::default();
        let (s, res) = solved_scene();
        let mut d = ResultDisplayState::default();
        d.mode = ResultViewMode::Deformed;
        d.show_undeformed = false;
        let geom = GlGeometry::build(
            &cam,
            &s,
            &d,
            &SelectionSet::default(),
            Some(&res),
            false,
            1.0,
            1.0,
        );
        assert!(!geom.solids.is_empty());
        assert!(geom.solids.iter().all(|v| v.is_finite()));
        assert!(contains_colour(&geom.solids, rgba(DEF_MEMBER)));
        assert!(contains_colour(&geom.solids, rgba(DEF_NODE)));
        assert_eq!(geom.line_batches.len(), 2); // axes + grid only
    }

    #[test]
    fn displacement_mode_contours_member_prisms() {
        let cam = Camera3D::default();
        let (s, res) = solved_scene();
        let mut d = ResultDisplayState::default();
        d.mode = ResultViewMode::Displacement;
        let geom = GlGeometry::build(
            &cam,
            &s,
            &d,
            &SelectionSet::default(),
            Some(&res),
            true,
            1.0,
            1.0,
        );
        // The plain member colour is replaced by the contour ramp; the fixed
        // end has zero displacement, so its end matches the ramp's blue stop.
        assert!(!contains_colour(&geom.solids, rgba(MEMBER_C)));
        assert!(contains_colour(&geom.solids, contour_colour(0.0)));
        assert!(geom.solids.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn diagrams_and_local_axes_emit_depth_tested_lines() {
        let cam = Camera3D::default();
        let (s, res) = solved_scene();
        let mut d = ResultDisplayState::default();
        d.show_diagrams = true;
        let sel = SelectionSet::from_single(SelectionState::Member(s.member_ids[0]));
        let geom = GlGeometry::build(&cam, &s, &d, &sel, Some(&res), true, 1.0, 1.0);
        // axes + overlay (local triad + diagram strips) + grid.
        assert_eq!(geom.line_batches.len(), 3);
        let overlay = &geom.line_batches[1];
        assert!(!overlay.blend);
        assert!(overlay.verts.len() > 6 * FLOATS_PER_VERTEX);
        assert!(overlay.verts.iter().all(|v| v.is_finite()));
    }

    // ── Phase P5: section cut ───────────────────────────────────────────

    /// Disabled cut ⇒ no plane, and reference batches are marked exempt.
    #[test]
    fn section_cut_disabled_leaves_clip_plane_none() {
        let cam = Camera3D::default();
        let s = scene();
        let d = ResultDisplayState::default();
        assert!(!d.section_cut.enabled);
        let geom = GlGeometry::build(&cam, &s, &d, &SelectionSet::default(), None, true, 1.0, 1.0);
        assert!(geom.clip_plane.is_none());
        // axes (no clip) then grid (no clip).
        assert_eq!(geom.line_batches.len(), 2);
        assert!(geom.line_batches.iter().all(|b| !b.clip));
    }

    /// Enabled cut ⇒ plane present; solids + overlay clipped, grid/axes exempt.
    #[test]
    fn section_cut_marks_batches_and_sets_plane() {
        let cam = Camera3D::default();
        let (s, res) = solved_scene();
        let mut d = ResultDisplayState::default();
        d.show_diagrams = true;
        d.section_cut.enabled = true;
        d.section_cut.axis = viewport_poc::display::CutAxis::X;
        d.section_cut.offset = 1.5;
        let sel = SelectionSet::from_single(SelectionState::Member(s.member_ids[0]));
        let geom = GlGeometry::build(&cam, &s, &d, &sel, Some(&res), true, 1.0, 1.0);

        let plane = geom.clip_plane.expect("plane present when enabled");
        assert_eq!(&plane[..3], &[1.0, 0.0, 0.0]);
        assert_eq!(plane[3], 1.5);

        // axes[0] and grid[2] are exempt; overlay[1] is clipped.
        assert_eq!(geom.line_batches.len(), 3);
        assert!(!geom.line_batches[0].clip, "axes should be exempt");
        assert!(geom.line_batches[1].clip, "overlay should be clipped");
        assert!(!geom.line_batches[2].clip, "grid should be exempt");
    }

    /// A flipped cut passes a negated plane so a positive clip distance still
    /// means "keep".
    #[test]
    fn section_cut_gpu_plane_reflects_flip() {
        let cam = Camera3D::default();
        let s = scene();
        let mut d = ResultDisplayState::default();
        d.section_cut.enabled = true;
        d.section_cut.axis = viewport_poc::display::CutAxis::Y;
        d.section_cut.offset = 2.0;
        d.section_cut.flip = true;
        let geom = GlGeometry::build(&cam, &s, &d, &SelectionSet::default(), None, true, 1.0, 1.0);
        let plane = geom.clip_plane.unwrap();
        assert_eq!(&plane[..3], &[0.0, -1.0, 0.0]);
        assert_eq!(plane[3], -2.0);
    }

    // ── Phase P7-1: capped section cut ──────────────────────────────────

    /// Cut on (cap defaults on) ⇒ a cap plane is present and is exactly the
    /// same keep-plane as the clip plane, for every axis and both flip
    /// states.  (The cap pass must keep the *kept* half — the opening
    /// exposes the kept half's interior; see
    /// `cap_pass_sense_same_plane_fills_kept_interior`.)
    #[test]
    fn cap_plane_is_the_same_keep_plane() {
        use viewport_poc::display::CutAxis;
        let cam = Camera3D::default();
        let s = scene();
        for axis in CutAxis::ALL {
            for flip in [false, true] {
                let mut d = ResultDisplayState::default();
                d.section_cut.enabled = true;
                d.section_cut.axis = axis;
                d.section_cut.offset = 1.25;
                d.section_cut.flip = flip;
                let geom =
                    GlGeometry::build(&cam, &s, &d, &SelectionSet::default(), None, true, 1.0, 1.0);
                let clip = geom.clip_plane.expect("clip plane present");
                let cap = geom.cap_plane.expect("cap plane present when cut is on");
                assert_eq!(cap, clip, "axis {axis:?} flip {flip}");
            }
        }
    }

    /// No cut ⇒ no cap; cut on but cap disabled ⇒ no cap (geometry unchanged).
    #[test]
    fn cap_plane_absent_without_cut_or_when_disabled() {
        let cam = Camera3D::default();
        let s = scene();

        // Cut off.
        let d = ResultDisplayState::default();
        let geom = GlGeometry::build(&cam, &s, &d, &SelectionSet::default(), None, true, 1.0, 1.0);
        assert!(geom.clip_plane.is_none());
        assert!(geom.cap_plane.is_none());

        // Cut on, cap off.
        let mut d = ResultDisplayState::default();
        d.section_cut.enabled = true;
        d.section_cut.cap = false;
        let geom = GlGeometry::build(&cam, &s, &d, &SelectionSet::default(), None, true, 1.0, 1.0);
        assert!(geom.clip_plane.is_some());
        assert!(geom.cap_plane.is_none());
    }

    /// The cap pass re-draws the *existing* solids buffer — it adds no
    /// vertices, so the scene vertex counts are unaffected by the cap toggle.
    #[test]
    fn cap_toggle_does_not_change_geometry() {
        let cam = Camera3D::default();
        let s = scene();
        let mut d = ResultDisplayState::default();
        d.section_cut.enabled = true;
        d.section_cut.cap = true;
        let with_cap =
            GlGeometry::build(&cam, &s, &d, &SelectionSet::default(), None, true, 1.0, 1.0);
        d.section_cut.cap = false;
        let without_cap =
            GlGeometry::build(&cam, &s, &d, &SelectionSet::default(), None, true, 1.0, 1.0);
        assert_eq!(with_cap.solids.len(), without_cap.solids.len());
        assert_eq!(with_cap.solids, without_cap.solids);
    }

    // ── Phase P7-1: cap-pass clip-sense verification (CPU replay) ──────
    //
    // Which half-space must the cap pass keep?  The hole the main pass
    // leaves at the cut exposes the **kept** half's interior — the removed
    // half's shell is clipped away in its entirety — so the fill has to
    // re-draw the kept half's interior back faces under the **same** clip
    // plane.  Keeping the *removed* half (the negated plane) instead paints
    // grey geometry inside the removed region, between the camera and the
    // kept half, and leaves the true opening unfilled.
    //
    // The replay below is faithful to the GL passes: for a triangle wound
    // CCW as seen from outside, front-facing is exactly
    // `dot(normal, eye − centroid) > 0`, and a fragment survives the clip
    // iff `dot(p, plane.xyz) − plane.w >= 0` (the scene vertex shader's
    // formula).  The prism is convex, so a ray meets its surface at most
    // twice and "nearest hit" is exact.

    /// A scene triangle parsed out of a solids buffer (flat per-face normal).
    struct SimTri {
        v: [[f64; 3]; 3],
        n: [f64; 3],
        c: [f64; 3],
    }

    fn sim_sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
        [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
    }

    fn sim_dot(a: [f64; 3], b: [f64; 3]) -> f64 {
        a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    }

    fn sim_cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    }

    fn sim_len(v: [f64; 3]) -> f64 {
        sim_dot(v, v).sqrt()
    }

    /// In-plane orthonormal basis `(u, v)` for a unit normal `n`.
    fn sim_plane_basis(n: [f64; 3]) -> ([f64; 3], [f64; 3]) {
        let reference = if n[0].abs() < 0.9 {
            [1.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0]
        };
        let u = sim_cross(n, reference);
        let ul = sim_len(u);
        let u = [u[0] / ul, u[1] / ul, u[2] / ul];
        (u, sim_cross(n, u))
    }

    fn sim_tris(buf: &[f32]) -> Vec<SimTri> {
        buf.chunks_exact(3 * FLOATS_PER_VERTEX)
            .map(|t| {
                let p = |k: usize| {
                    let v = &t[k * FLOATS_PER_VERTEX..k * FLOATS_PER_VERTEX + 3];
                    [v[0] as f64, v[1] as f64, v[2] as f64]
                };
                let (a, b, c) = (p(0), p(1), p(2));
                SimTri {
                    v: [a, b, c],
                    n: [t[3] as f64, t[4] as f64, t[5] as f64],
                    c: [
                        (a[0] + b[0] + c[0]) / 3.0,
                        (a[1] + b[1] + c[1]) / 3.0,
                        (a[2] + b[2] + c[2]) / 3.0,
                    ],
                }
            })
            .collect()
    }

    /// Ray/triangle distance (Möller–Trumbore); `None` when missed/behind.
    fn sim_ray_tri(t: &SimTri, eye: [f64; 3], dir: [f64; 3]) -> Option<f64> {
        const EPS: f64 = 1e-12;
        let [a, b, c] = t.v;
        let e1 = sim_sub(b, a);
        let e2 = sim_sub(c, a);
        let h = sim_cross(dir, e2);
        let det = sim_dot(e1, h);
        if det.abs() < EPS {
            return None;
        }
        let inv = 1.0 / det;
        let s = sim_sub(eye, a);
        let u = inv * sim_dot(s, h);
        if !(0.0..=1.0).contains(&u) {
            return None;
        }
        let q = sim_cross(s, e1);
        let v = inv * sim_dot(dir, q);
        if v < 0.0 || u + v > 1.0 {
            return None;
        }
        let dist = inv * sim_dot(e2, q);
        (dist > EPS).then_some(dist)
    }

    /// Replay one GL solids pass: nearest hit among the triangles the pass
    /// would actually draw — those on the plane's kept side whose winding
    /// is front-facing (`cull_front == false`, the main pass) or
    /// back-facing (`cull_front == true`, the cap pass).
    fn sim_pass(
        tris: &[SimTri],
        eye: [f64; 3],
        dir: [f64; 3],
        cull_front: bool,
        plane: [f32; 4],
    ) -> Option<f64> {
        let mut best: Option<f64> = None;
        for t in tris {
            let front = sim_dot(t.n, sim_sub(eye, t.c)) > 1e-9;
            if front == cull_front {
                continue; // culled by this pass
            }
            let Some(dist) = sim_ray_tri(t, eye, dir) else {
                continue;
            };
            let p = [
                eye[0] + dir[0] * dist,
                eye[1] + dir[1] * dist,
                eye[2] + dir[2] * dist,
            ];
            // Fragment-level clip, exactly as the scene vertex shader.
            let clip = p[0] * plane[0] as f64 + p[1] * plane[1] as f64 + p[2] * plane[2] as f64
                - plane[3] as f64;
            if clip < 0.0 {
                continue;
            }
            if best.is_none_or(|d| dist < d) {
                best = Some(dist);
            }
        }
        best
    }

    /// Phase P7-1: the cap pass must keep the **same** half-space as the
    /// main pass.  Replaying both passes on the CPU proves the sense for
    /// every axis and both `flip` states:
    ///
    /// * every interior (back-facing) face of the **kept** half that the
    ///   main pass leaves exposed is a hole, and the same-plane cap pass
    ///   fills exactly it;
    /// * neither the main pass nor the same-plane cap pass ever paints the
    ///   **removed** half;
    /// * the negated plane (the rejected sense) would paint the removed
    ///   half's interior — grey geometry between the camera and the model.
    #[test]
    fn cap_pass_sense_same_plane_fills_kept_interior() {
        use viewport_poc::display::{CutAxis, SectionCut};

        // One chunky prism along +x (convex ⇒ exact nearest-hit replay).
        let mut buf = Vec::new();
        push_prism(
            &mut buf,
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            0.3,
            0.3,
            [1.0, 1.0, 1.0, 1.0],
        );
        let tris = sim_tris(&buf);
        let center = [1.0, 0.0, 0.0];

        for axis in CutAxis::ALL {
            for flip in [false, true] {
                let cut = SectionCut {
                    enabled: true,
                    axis,
                    normal: None,
                    // Cut through the prism's middle for every axis.
                    offset: sim_dot(center, axis.normal()),
                    flip,
                    positioned: true,
                    cap: true,
                };
                let keep = cut.gpu_plane().unwrap();
                let removed = [-keep[0], -keep[1], -keep[2], -keep[3]];
                let n = cut.unit_normal().unwrap();
                let s = if flip { -1.0 } else { 1.0 };
                let keep_dir = [s * n[0], s * n[1], s * n[2]];

                // Eye on the removed side, offset inside the plane so the
                // opening is seen at an angle (a straight-on view sees only
                // the far cap, never the side interiors).
                let (u, v) = sim_plane_basis(n);
                let eye = [
                    center[0] - keep_dir[0] * 6.0 + 2.0 * u[0] + 2.0 * v[0],
                    center[1] - keep_dir[1] * 6.0 + 2.0 * u[1] + 2.0 * v[1],
                    center[2] - keep_dir[2] * 6.0 + 2.0 * u[2] + 2.0 * v[2],
                ];

                let mut filled_holes = 0usize;
                let mut mispainted_removed = 0usize;

                for t in &tris {
                    let front = sim_dot(t.n, sim_sub(eye, t.c)) > 1e-9;
                    if front {
                        continue; // only interior (back) faces can be holes
                    }
                    let to_c = sim_sub(t.c, eye);
                    let dt = sim_len(to_c);
                    if dt < 1e-9 {
                        continue;
                    }
                    let dir = [to_c[0] / dt, to_c[1] / dt, to_c[2] / dt];
                    // Which half is this interior face in?  (The ray passes
                    // through the centroid, so the hit point is the
                    // centroid and the clip verdict is the same.)
                    let clip_c = sim_dot(t.c, [keep[0] as f64, keep[1] as f64, keep[2] as f64])
                        - keep[3] as f64;

                    if clip_c >= 0.0 {
                        // ── Kept-half interior face ──
                        // A hole unless front-facing kept geometry blocks
                        // the view of it; the same-plane cap must then fill
                        // exactly this face.
                        let main = sim_pass(&tris, eye, dir, false, keep);
                        if main.is_some_and(|d| d < dt - 1e-9) {
                            continue; // occluded by front geometry
                        }
                        assert!(
                            main.is_none(),
                            "axis {axis:?} flip {flip}: main pass drew something at a hole"
                        );
                        let cap = sim_pass(&tris, eye, dir, true, keep);
                        assert!(
                            cap.is_some_and(|d| (d - dt).abs() < 1e-6),
                            "axis {axis:?} flip {flip}: same-plane cap must fill the hole"
                        );
                        // The negated plane can never fill it: its clip
                        // excludes the kept half, so nothing at the hole's
                        // depth survives (the halves are disjoint).
                        let cap_inv = sim_pass(&tris, eye, dir, true, removed);
                        assert!(
                            !cap_inv.is_some_and(|d| (d - dt).abs() < 1e-6),
                            "axis {axis:?} flip {flip}: negated plane must not fill kept-half holes"
                        );
                        filled_holes += 1;
                    } else {
                        // ── Removed-half interior face ──
                        // The correct cap must not paint it (same plane
                        // clips it away) and neither does the main pass —
                        // but the negated plane would paint grey geometry
                        // here, inside the removed half, in front of the
                        // kept half.  That is the sense to reject.
                        let main = sim_pass(&tris, eye, dir, false, keep);
                        assert!(
                            main.is_none_or(|d| d > dt + 1e-9),
                            "axis {axis:?} flip {flip}: main pass must not draw the removed half"
                        );
                        let cap = sim_pass(&tris, eye, dir, true, keep);
                        assert!(
                            cap.is_none_or(|d| d > dt + 1e-9),
                            "axis {axis:?} flip {flip}: same-plane cap must not paint the removed half"
                        );
                        let cap_inv = sim_pass(&tris, eye, dir, true, removed);
                        assert!(
                            cap_inv.is_some_and(|d| (d - dt).abs() < 1e-6),
                            "axis {axis:?} flip {flip}: negated plane would paint the removed half"
                        );
                        mispainted_removed += 1;
                    }
                }

                // Non-degenerate setup: the cut really is open, the cap
                // really fills it, and the negated plane really mispaints.
                assert!(
                    filled_holes > 0,
                    "axis {axis:?} flip {flip}: no holes were filled"
                );
                assert!(
                    mispainted_removed > 0,
                    "axis {axis:?} flip {flip}: negated plane painted nothing"
                );
            }
        }
    }
}
