# Phase 139 — 3D Viewport Technology Proof of Concept

## Baseline and Final Commit

- **Baseline:** `44aef1b` (Phase 138)
- **Final:** _to be filled after commit_

## Prototype Location

`apps/viewport-poc/` — a standalone binary + library crate added to the
workspace.

### Dependency Choices

| Dependency | Version | Purpose |
|------------|---------|---------|
| `eframe` | 0.29 | Windowed app framework (bundles `egui` + rendering backend) |
| `egui` | 0.29 | Immediate-mode UI (panels, widgets, 2D painter) |
| `structural-analysis` | path | Analysis API (`FrameModel3D`, `FrameAnalysisResult3D`, etc.) |

**Rendering approach:** Software 3D-to-2D projection using `egui`'s built-in
2D `Painter`. No custom `wgpu` shaders — the 3D math (camera, projection) runs
on CPU and the results are drawn as 2D line segments and circles via egui's
painter, which is GPU-accelerated for compositing.

This is the simplest implementation that meets all acceptance criteria. Custom
`wgpu` rendering would add complexity (shader pipeline, mesh buffers, depth
buffer) without clear benefit at the PoC scale.

### Workspace Integration

- `Cargo.toml` (workspace): added `"apps/viewport-poc"` to `members`.
- `section-properties` and `structural-analysis` are **untouched** — no GUI or
  graphics dependencies added to the analysis crates.
- The app crate has both `lib.rs` (testable modules) and `main.rs` (egui app).

## Architecture

### Module Structure

```
apps/viewport-poc/
├── Cargo.toml
├── src/
│   ├── lib.rs          # Testable modules (no GUI deps)
│   ├── main.rs         # eframe app + entry point
│   ├── project.rs      # Vec3 math, point-to-segment distance
│   ├── camera.rs       # Camera3D: orbit/pan/zoom, perspective projection
│   ├── scene.rs        # ViewportScene: display geometry from results
│   ├── selection.rs    # SelectionState: Node/Member/None
│   ├── display.rs      # ResultDisplayState: scale, toggles
│   └── model.rs        # Deterministic test models + synthetic generators
├── tests/
│   ├── geometry.rs     # Coordinate mapping consistency
│   ├── selection.rs    # Viewport ID → source member resolution
│   ├── deformation.rs  # Scaling does not modify stored results
│   ├── robustness.rs   # Non-finite, empty, zero-length bounds
│   └── performance.rs  # Projection cost for small/medium models
```

### Rendering Data Independence

The rendering data model is **fully independent** of the solver's internal
representation:

- `ViewportScene` stores only `Vec<Vec3>` for positions and `Vec<(usize, usize)>`
  for connectivity. It is built from `FrameAnalysisResult3D` via
  `ViewportScene::from_result()`, which copies the public fields.
- The camera, selection, and display state never hold references to the solver
  or model.
- The model is solved **once** on startup (or on user action). The scene is
  rebuilt only when the result changes. Per-frame work is limited to projection
  and drawing.

### Key Abstractions

| Abstraction | Responsibility |
|-------------|----------------|
| `Camera3D` | Orbit/pan/zoom camera, perspective projection, `project()` |
| `ViewportScene` | Node positions, member connectivity, deformed positions |
| `SelectionState` | Selected entity (Node/Member/None) |
| `ResultDisplayState` | Deformation scale, show/hide toggles |
| `ViewportApp` | eframe app: UI layout, input handling, drawing |

## Features Implemented

### Viewport

- ✅ 3D nodes (yellow circles) and frame members (light blue lines)
- ✅ Global X/Y/Z axes (red/green/blue) from origin
- ✅ Ground grid on XZ plane (dark gray)
- ✅ Member highlighting (orange, thicker stroke)
- ✅ Node highlighting (orange, larger circle)
- ✅ Orbit camera (left-drag: azimuth/elevation)
- ✅ Pan camera (right-drag: move target)
- ✅ Zoom camera (scroll wheel: scale distance)
- ✅ Camera reset (R key or button)
- ✅ Clear selection (Escape key)
- ✅ Node labels (N0, N1, ...)

### Analysis Integration

- ✅ Deterministic portal frame model (4 nodes, 3 members, fixed bases, horizontal load)
- ✅ Solve trigger (automatic on startup, "Re-solve" button)
- ✅ Undeformed structure display
- ✅ Deformed structure display with configurable scale (slider, 0–2000×)
- ✅ Scale clearly labeled as "× actual displacement" (display multiplier, not physical)
- ✅ Node displacement values (ux, uy, uz, rx, ry, rz) in selection panel
- ✅ Node reaction values (Rx, Ry, Rz, Mx, My, Mz) in selection panel
- ✅ Member end forces — local coordinates (12 values, labeled by end and component)
- ✅ Member end forces — global coordinates (via `member_end_forces_global()`)
- ✅ Member local axes (x̂, ŷ, ẑ in global coordinates, via `member_local_axes()`)
- ✅ Local axes visualization at selected member midpoint
- ✅ Local vs. global axes visually distinguished (global at origin, local at member)

### Correctness Safeguards

- ✅ Deformed positions computed as `original + scale × displacement` (read-only)
- ✅ Non-finite displacements → original position (no NaN propagation)
- ✅ Non-finite node positions → projection returns `None` (skipped)
- ✅ Empty scene → bounding box `[0;3]`, no crash
- ✅ Coincident nodes → camera uses minimum distance 1.0
- ✅ Camera elevation clamped to ±1.55 rad (avoids gimbal lock)
- ✅ Camera distance clamped to [0.1, 1e5]
- ✅ Depth sorting for undeformed members (far-to-near draw order)

## Known Limitations

- **No custom GPU rendering:** 3D projection is CPU-based. For very large models
  (>10k elements), a `wgpu` render pipeline would be needed.
- **No model editor:** The model is hardcoded. Switching models requires code
  changes.
- **No persistence:** Model and results are not saved/loaded.
- **No multi-window:** Single viewport, single model.
- **Picking is brute-force:** O(n_nodes + n_members) per click. Fine for PoC,
  but a spatial index would be needed for large models.
- **Deformed positions recomputed each frame:** Could be cached when scale is
  unchanged.
- **No screenshots:** Manual visual verification required (see below).

## Manual Verification Procedure

The app requires a desktop session. To verify visually:

```bash
cargo run -p viewport-poc --release
```

Expected behavior:
1. Window opens (1200×800) titled "3D Viewport PoC — Phase 139".
2. Portal frame visible: 2 yellow columns, 1 yellow beam, on a dark background.
3. Ground grid visible (dark gray lines on XZ plane).
4. Global axes visible (red X, green Y, blue Z from origin).
5. Deformed shape visible (purple/magenta, displaced from undeformed).
6. Left-drag orbits the camera. Right-drag pans. Scroll zooms.
7. Click a node → side panel shows position, displacements, reactions.
8. Click a member → side panel shows length, end forces (local + global), local axes.
9. Local axes drawn at selected member midpoint (red x̂, green ŷ, blue ẑ).
10. "R" resets camera. "Escape" clears selection.
11. Deformation scale slider adjusts deformed shape exaggeration.

## Performance Methodology and Results

### Environment

- **OS:** Windows (win32)
- **Rust:** 1.97.1
- **Mode:** `--release`
- **Rendering:** Software projection (CPU) + egui 2D painter (GPU compositing)

### Methodology

Measure the time to project all node positions and member endpoints from 3D
world coordinates to 2D screen coordinates, averaged over 1000 iterations.
This is the viewport-specific portion of the render loop; full frame time also
includes egui layout, widget painting, and GPU compositing.

### Results

| Model | Nodes | Members | Projection µs/frame | Projection fps |
|-------|-------|---------|---------------------|----------------|
| 1×1 grid | 8 | 8 | 1.4 | 698,812 |
| 2×2 grid | 18 | 21 | 3.6 | 281,373 |
| 3×3 grid | 32 | 40 | 6.7 | 149,611 |
| 5×5 grid | 72 | 96 | 15.8 | 63,464 |
| 8×8 grid | 162 | 225 | 37.1 | 26,965 |
| 10×10 grid | 242 | 341 | 54.6 | 18,331 |

**Conclusion:** Projection cost is negligible (< 55 µs for 242 nodes + 341
members). The rendering bottleneck, if any, will be in egui's painting and
compositing, not in the 3D math. For the PoC scale (4–5 nodes), performance
is a non-issue.

## Build / Test / Lint Outcomes

| Check | Result |
|-------|--------|
| `cargo fmt --all -- --check` | ✅ clean |
| `cargo check -p structural-analysis` | ✅ passes |
| `cargo test -p structural-analysis --release` | ✅ all pass, 0 failures |
| `cargo test -p viewport-poc` | ✅ 55 tests pass (26 lib + 6 geometry + 4 selection + 6 deformation + 12 robustness + 1 performance) |
| `cargo doc -p structural-analysis --no-deps` | ✅ 3 pre-existing warnings (truss3d.rs, not from this change) |
| `cargo clippy -p structural-analysis --all-targets -- -D warnings` | ❌ 133 pre-existing errors in `section-properties` (not from this change); 0 errors in `structural-analysis` |
| `cargo clippy -p viewport-poc` | ⚠️ 17 warnings (style lints in PoC code + pre-existing deps); 0 errors |
| Prototype compilation (`cargo check -p viewport-poc`) | ✅ passes, 0 warnings |

## API Gaps

No API gaps were encountered. The `FrameAnalysisResult3D` API (including
`member_local_axes()` and `member_end_forces_global()` from Phase 138) provides
all data needed for the viewport.

## Technology Evaluation: `egui + wgpu`

### Assessment

**egui + eframe is suitable** for the structural-analysis desktop application.

**Pros:**
- Immediate-mode API is simple and productive for engineering UIs.
- Built-in widgets (sliders, checkboxes, grids, collapsing headers) cover
  property-panel needs.
- 2D painter is sufficient for 3D viewport rendering at PoC scale.
- `wgpu` backend available as a feature flag for future GPU-accelerated rendering.
- Pure Rust, no C/C++ dependencies (unlike Qt or GTK).
- Active development, good documentation.

**Cons:**
- No built-in 3D rendering (must implement projection or use `wgpu` directly).
- Immediate-mode can be less efficient for complex UIs (re-built every frame).
- Limited charting/plotting (would need `egui_plot` or custom for result diagrams).

### Recommendation

**Keep `egui + eframe` as the primary candidate** for the desktop application.
For production 3D rendering, add a `wgpu` render pipeline for the viewport
(keeping egui for panels/controls). The PoC validates the architecture; the
next step is a `wgpu` render pipeline for larger models.

## Comparison Plan: Tauri + Three.js

| Criterion | egui + eframe | Tauri + Three.js |
|-----------|---------------|-------------------|
| Language | Pure Rust | Rust (backend) + JS/TS (frontend) |
| 3D rendering | Custom (wgpu or CPU projection) | Three.js (mature WebGL) |
| UI framework | egui (immediate mode) | Web technologies (HTML/CSS/React/Vue) |
| Binary size | ~15 MB | ~5 MB (Tauri) + JS bundle |
| Startup time | Fast (< 100 ms) | Slower (webview init) |
| Cross-platform | Native | Native (via webview) |
| Development speed | Fast for Rust devs | Fast for web devs |
| Ecosystem | Growing | Very mature (npm) |
| 3D ecosystem | Limited | Excellent (Three.js, plugins) |
| Hot reload | No | Yes (frontend) |

**When to choose Tauri + Three.js:**
- If the team has strong web/TypeScript expertise.
- If complex 3D visualization (mesh rendering, stress contours, animations) is
  a core requirement.
- If hot-reload during UI development is important.

**When to choose egui + eframe:**
- If the team prefers pure Rust.
- If the UI is primarily property panels and simple 3D viewport.
- If minimal dependencies and fast startup are priorities.

**Recommendation:** Build the next PoC with Tauri + Three.js to compare
development experience and rendering quality before making a final decision.

## Recommended Next Phase

**Phase 140: Tauri + Three.js viewport PoC**

Build an equivalent viewport using Tauri + Three.js to enable a direct
comparison. Reuse the same `structural-analysis` Rust API via Tauri commands.
Compare:
- Development effort and code complexity.
- Rendering quality (anti-aliasing, depth buffer, transparency).
- Performance for medium models (100–500 elements).
- Developer experience (tooling, hot reload, debugging).
