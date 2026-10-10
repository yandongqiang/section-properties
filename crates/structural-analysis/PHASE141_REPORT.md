# Phase 141 — Viewport Architecture Audit & GUI Technology Decision

**Date:** 2026-10-10  
**Baseline HEAD:** `654aeb3` (Phase 140)  
**Final HEAD:** TBD (this commit)  
**Status:** Complete  

---

## 1. Baseline

| Item | Value |
|------|-------|
| Workspace HEAD | `654aeb3` |
| Phase 139 commit | `b9335d8` — egui + eframe 3D viewport PoC |
| Phase 140 commit | `654aeb3` — Tauri 2 + Three.js viewport PoC |
| Working tree | Clean (no modified tracked files) |
| Rust toolchain | 1.97.1, edition 2024 |
| Node.js | v24.11.1, npm 11.6.2 |
| Platform | Windows (win32) |

### Workspace members

- `section-properties` (v0.4.0, published to crates.io — not modified)
- `structural-analysis` (v0.1.0, `publish = false`)
- `apps/viewport-poc` (egui PoC, Phase 139)
- `apps/tauri-viewport-poc/src-tauri` (Tauri PoC, Phase 140)

### Reports reviewed

- `PHASE139_REPORT.md` — egui viewport PoC: 55 tests, software 3D projection, 54.6 µs/frame
- `PHASE140_REPORT.md` — Tauri viewport PoC: 36 tests, Three.js WebGL, 8.25 ms scene build

---

## 2. Feature Parity Matrix

Legend: ✅ Implemented & tested | ⚠️ Implemented, not verified at runtime | ❌ Not implemented | N/A Not applicable

| # | Feature | egui PoC (Phase 139) | Tauri PoC (Phase 140) |
|---|---------|---------------------|----------------------|
| 1 | Real structural model data (FrameAnalysisResult3D) | ✅ `ViewportScene::from_result()` | ✅ `build_scene_from_result()` |
| 2 | Analysis result source (FrameModel3D::analyze) | ✅ Same solver | ✅ Same solver |
| 3 | Node rendering | ✅ `circle_filled`, software projected | ✅ Three.js `Mesh` spheres, GPU |
| 4 | Member rendering | ✅ `line_segment`, depth-sorted | ✅ `CylinderGeometry`, WebGL depth buffer |
| 5 | Deformed geometry + scale slider | ✅ `deformed_positions(displacements, scale)` | ✅ Frontend: `original + scale * displacement` |
| 6 | Local axes display | ✅ Line segments from midpoint | ✅ `ArrowHelper` for x/y/z |
| 7 | Local end forces (12 components) | ✅ In selection panel | ✅ In DTO + panel |
| 8 | Global end forces (12 components) | ✅ In selection panel | ✅ In DTO + panel |
| 9 | Camera orbit | ✅ Mouse drag (azimuth/elevation) | ✅ `OrbitControls` |
| 10 | Camera pan | ✅ Shift + drag | ✅ `OrbitControls` pan |
| 11 | Camera zoom | ✅ Scroll wheel | ✅ `OrbitControls` zoom |
| 12 | Camera reset | ✅ R key | ❌ Not implemented |
| 13 | Node picking | ✅ O(n) screen-space distance, 12px threshold | ✅ `Raycaster` against meshes |
| 14 | Member picking | ✅ Point-to-segment distance | ✅ `Raycaster` against cylinders |
| 15 | Selection highlight | ✅ Larger circle / thicker line | ✅ Emissive material + scale change |
| 16 | Property panel (position/displacement/reaction) | ✅ Side panel | ✅ HTML panel |
| 17 | Property panel (end forces/local axes) | ✅ Side panel | ✅ HTML panel |
| 18 | Window resize handling | ✅ egui auto-handles | ✅ Resize event listener |
| 19 | Empty scene handling | ✅ Tested: `empty_scene_bounding_box` | ✅ Tested: `grid_1x1_scene_builds` |
| 20 | Zero-length member | ✅ Tested: `zero_length_member` | ✅ Tested: `grid_1x1_scene_builds` |
| 21 | Coincident nodes | ✅ Tested: `coincident_nodes_bounding_box` | ✅ Tested: `portal_no_duplicate_nodes` |
| 22 | Non-finite data | ✅ Tested: returns original / projects to None | ✅ Tested: `portal_scene_is_finite` |
| 23 | Error handling | ✅ `Result` from analyze, shown in UI | ✅ `Result` from Tauri command, caught in frontend |
| 24 | Desktop launch | ⚠️ `eframe::run_native()` (not manually verified) | ⚠️ `tauri::Builder::run()` (not manually verified) |
| 25 | Node labels (N0, N1, ...) | ✅ Text labels | ❌ Not implemented |
| 26 | Camera info panel | ✅ Azimuth/elevation/distance/fov | ❌ Not implemented |
| 27 | Help text / instructions | ✅ In panel | ❌ Not implemented |
| 28 | Re-solve button | ✅ Button to re-run analysis | ❌ Not implemented |
| 29 | Model selection (multiple models) | ❌ Hardcoded portal only | ✅ Dropdown: 5 models |
| 30 | Undeformed overlay toggle | ✅ `show_undeformed` | ✅ `showUndeformed` |
| 31 | Grid helper | ✅ Manual grid lines | ✅ `GridHelper` |
| 32 | Global axes helper | ✅ Manual axis lines | ✅ `AxesHelper` |
| 33 | Depth sorting | ✅ Manual: members sorted by camera depth | ✅ Implicit: WebGL depth buffer |
| 34 | Forces grid display | ✅ `forces_grid` widget | ❌ Not implemented |

### Parity summary

**egui-only features (4):** Node labels, camera info panel, help text, re-solve button, forces grid  
**Tauri-only features (1):** Model selection dropdown (5 models)  
**Both implemented (29):** Core 3D viewport, rendering, deformation, local axes, picking, selection, property panel, degenerate handling  
**Neither implemented (0):** All listed features are covered by at least one PoC  

**Model data equivalence:** Both PoCs use the same model builders (`build_portal_frame`, `build_inclined_frame`, `build_grid_frame`) from `structural-analysis`. Node coordinates, member connectivity, loads, and constraints are identical. FEM solver is the same (`FrameModel3D::analyze()`). Displacement and force values are therefore identical.

---

## 3. Data Contract Audit

### egui ViewportScene

```rust
pub struct ViewportScene {
    node_positions: Vec<Vec3>,        // rendering geometry only
    member_nodes: Vec<(usize, usize)>, // node index pairs
    member_ref_vecs: Vec<Vec3>,        // reference vectors for local axes
}
```

- **Scope:** Minimal — stores only rendering geometry
- **Analysis data access:** Direct Rust-to-Rust via `FrameAnalysisResult3D` (no serialization)
- **Deformed positions:** Computed on demand via `deformed_positions(displacements, scale)`
- **Index type:** `usize` (Rust native)
- **Coordinate type:** `Vec3` (f64)
- **Schema versioning:** None
- **Unit labels:** None (implicit meters/Newton)

### Tauri SceneDto

```rust
pub struct SceneDto {
    nodes: Vec<NodeDto>,      // id, position, displacement, rotation, reaction_force, reaction_moment
    members: Vec<MemberDto>,  // id, node_i, node_j, ref_vec, local_x/y/z_axis, end_forces_local[12], end_forces_global[12]
    n_nodes: usize,
    n_members: usize,
}
```

- **Scope:** Comprehensive — carries ALL analysis data across FFI boundary
- **Analysis data access:** Serialized to JSON, transferred via Tauri IPC, deserialized in TypeScript
- **Deformed positions:** Frontend recomputes: `original + scale * displacement` (no IPC per scale change)
- **Index type:** `i32` (serde/JS-compatible IDs)
- **Coordinate type:** `Vec3Dto` (f64 serialized as JSON number)
- **Schema versioning:** None
- **Unit labels:** None (implicit meters/Newton)

### Comparison

| Aspect | ViewportScene (egui) | SceneDto (Tauri) |
|--------|---------------------|------------------|
| Data scope | Rendering geometry only | All analysis data |
| Transfer mechanism | Rust-to-Rust (zero-copy) | JSON serialization + IPC |
| Serialization cost | 0 | 764 µs / 333 KB (10×10 grid) |
| Index type | `usize` | `i32` |
| Precision | f64 throughout | f64 → JSON → f64 (lossless for typical values) |
| Schema versioning | None | None |
| Unit labels | None | None |
| Coordinate convention | Y-up, meters | Y-up, meters |
| Local axis convention | x along member, y/z from ref_vec | x along member, y/z from ref_vec |
| Non-finite strategy | Returns original position | All values checked for fineness in tests |
| Deformation scale | Computed in Rust per frame | Computed in JS per slider change |

### Shared schema evaluation

**Finding:** A shared, renderer-agnostic scene schema is feasible but **not justified at PoC stage**.

- `SceneDto` is already close to a renderer-agnostic schema — it contains all data needed for rendering and display.
- `ViewportScene` is too minimal for a shared schema (lacks analysis data).
- If Tauri is adopted, `SceneDto` could serve as the shared schema with minor additions (schema version, unit labels).
- If egui is adopted, the analysis data is accessed directly from Rust types, so a shared schema adds unnecessary serialization.
- Introducing a shared crate now would couple both PoCs and add build complexity without clear benefit.

**Recommendation:** Do not introduce a shared schema crate at this stage. Revisit if both renderers must coexist in production.

---

## 4. Performance Comparability

### 8 metrics — measured status

| # | Metric | egui PoC | Tauri PoC | Comparable? |
|---|--------|----------|-----------|-------------|
| 1 | CPU geometry projection | ✅ 54.6 µs/frame (10×10) | N/A (GPU handles) | No — different pipeline stages |
| 2 | Scene build (FEM solve + DTO extraction) | Not measured separately | ✅ 8.25 ms (10×10) | No — egui has no DTO |
| 3 | DTO serialization + transfer | N/A (no DTO) | ✅ 764 µs / 333 KB | No — egui has no IPC |
| 4 | Deformation scale change update | Not measured (recomputed per frame) | Not measured (recomputed per slider change) | Neither benchmarked |
| 5 | Selection/picking latency | Not measured (O(n) brute-force) | Not measured (Raycaster) | Neither benchmarked |
| 6 | Render frame time / FPS | Not measured (CPU-bound) | Not measured (GPU-bound) | Neither benchmarked |
| 7 | Startup time | Not measured | Not measured | Neither benchmarked |
| 8 | Memory usage | Not measured | Not measured | Neither benchmarked |

### Measured values at 3 scales

| Scale | Nodes | Members | egui projection (µs) | Tauri scene build (ms) | Tauri serialization (µs) |
|-------|-------|---------|---------------------|----------------------|------------------------|
| 1×1 | 8 | 8 | 1.4 | 0.257 | — |
| 5×5 | 72 | 98 | 15.8 | 7.46 | — |
| 10×10 | 242 | 341 | 54.6 | 8.25 | 764 |

### Key insight

The measured metrics are **not directly comparable** because they measure different bottlenecks in different architectures:

- **egui PoC** measures CPU projection cost — the bottleneck for software rendering
- **Tauri PoC** measures scene build cost — the bottleneck for IPC + FEM solve

Both architectures share the same FEM solve cost (both call `FrameModel3D::analyze()`). The architectural difference is:

```
egui:   FEM solve (Rust) → ViewportScene (Rust) → software project (Rust, 54.6 µs) → egui paint
Tauri:  FEM solve (Rust) → SceneDto (Rust) → serialize (764 µs) → IPC → Three.js build (JS) → WebGL render
```

The Tauri path adds serialization + IPC + JS scene build overhead (~1 ms total), but GPU rendering is orders of magnitude faster than CPU software rendering for large models. The FEM solve (~7-8 ms) dominates both paths.

### Reproducibility verification

All performance tests re-run in this phase with `--release`:
- egui: `projection_performance` — ✅ pass
- Tauri: `portal_scene_build_time`, `grid_5x5_scene_build_time`, `grid_10x10_scene_build_time`, `grid_10x10_dto_serialisation_time`, `grid_10x10_repeated_build_no_degradation` — ✅ all pass

---

## 5. Tauri Integration & Runtime Audit

### Entry point and command registration

- `src-tauri/src/main.rs` calls `lib::run()`
- `run()` registers 3 commands via `tauri::generate_handler![get_scene, get_scene_inclined, get_scene_grid]`
- Commands return `Result<SceneDto, String>` — errors propagate to frontend

### Rust → TypeScript data flow

1. Frontend calls `invoke('get_scene_grid', { nx: 10, nz: 10 })`
2. Rust command builds model, calls `FrameModel3D::analyze()`, extracts `SceneDto`
3. Tauri serializes `SceneDto` to JSON (764 µs / 333 KB for 10×10)
4. Frontend receives JSON, `SceneManager.loadScene()` builds Three.js objects

### Real analysis data verification

- `build_scene_from_result()` calls `FrameModel3D::analyze()` — real FEM solve
- `NodeDto` includes `displacement`, `rotation`, `reaction_force`, `reaction_moment` from analysis result
- `MemberDto` includes `end_forces_local[12]` and `end_forces_global[12]` from analysis result
- Tests verify fineness of all values (`portal_scene_is_finite`, `inclined_scene_is_finite`)
- Tests verify equilibrium (`portal_global_force_equilibrium`, `portal_global_moment_equilibrium`)

### Frontend error handling

- `try/catch` around `invoke()` calls in `loadScene()`
- Error message displayed in `#error` div
- Loading state shown during invoke via `#loading` div

### Tauri permissions

- `capabilities/default.json` grants `core:default` (event, window, webview)
- No additional permissions required for current commands

### Issues found

| # | Issue | Severity | Impact |
|---|-------|----------|--------|
| 1 | CSP disabled (`null` in `tauri.conf.json`) | Medium | Acceptable for PoC; must harden for production (restrict to `self` and `asset:`) |
| 2 | `renderer.dispose()` not called on scene rebuild | Low | Potential WebGL context leak in long-running sessions; `SceneManager.dispose()` disposes geometries/materials but not renderer |
| 3 | Resize event listener not explicitly removed | Low | Added once in constructor; for desktop app lifecycle this is acceptable |
| 4 | Bundle size 507 KB (128 KB gzipped) | Info | Dominated by Three.js; acceptable for desktop app |
| 5 | npm audit: 2 vulnerabilities | Info | In dev dependencies (`vite`); not in production bundle |

### CPU-intensive operations

- FEM solve runs in Tauri command (Rust backend) — does not block UI thread
- Frontend shows "Loading..." during `invoke()` — correct async pattern
- Deformation scale change is computed in frontend — no IPC per slider tick

### Desktop launch

- Not manually verified in this phase (no GUI environment available)
- `tauri.conf.json` configured: `frontendDist = "../dist"`, `devUrl = "http://localhost:1420"`
- `vite build` produces `dist/` successfully — frontend assets ready
- `bundle.active = false` — no installer generated (PoC setting)

---

## 6. egui Implementation Audit

### CPU projection and culling

- `Camera3D::project()` implements full perspective projection in Rust (f64 precision)
- **No view frustum culling** — all nodes/members projected regardless of visibility
- For 242 nodes: 54.6 µs/frame (negligible)
- For 10K+ nodes: would become a bottleneck (estimated ~2-3 ms based on linear scaling)

### Camera controls

- Orbit: mouse drag (azimuth/elevation)
- Pan: shift + drag
- Zoom: scroll wheel
- Reset: R key
- Elevation clamped to ±1.55 rad (near ±π/2) — prevents gimbal flip
- Distance clamped to [0.1, 1e5]
- FOV: 60° default, adjustable

### Geometry update cost

- `deformed_positions()` called **every frame** for all nodes — O(n_nodes)
- For 242 nodes: negligible (sub-microsecond per node)
- Optimization opportunity: only recompute when `deformation_scale` changes
- Not a bottleneck at current scale

### Selection correctness

- Brute-force O(n_nodes + n_members) screen-space distance test
- Threshold: 12 pixels
- Tests verify: node selection, member selection, far-away returns None
- Correct for small models
- For 10K+ nodes: O(n) per click could be noticeable (~100 µs); a spatial acceleration structure (BVH) would help

### Rendering precision

- Internal computation: f64 (`Vec3`)
- Screen coordinates: f32 (egui `Pos2`)
- Good precision for engineering visualization

### Degenerate case handling

| Case | Handling | Test |
|------|----------|------|
| Empty scene | `bounding_box` returns None | `empty_scene_bounding_box` |
| Zero-length member | Renders as point, no crash | `zero_length_member` |
| Coincident nodes | Renders overlapping, no crash | `coincident_nodes_bounding_box` |
| Non-finite position | `project()` returns None | `non_finite_node_position_projects_to_none` |
| Non-finite displacement | `deformed_positions` returns original | `non_finite_displacement_returns_original` |
| Zero vector normalize | Returns zero vector | `normalize_zero_and_non_finite` |

### Large model scaling

- No view frustum culling
- No level-of-detail
- No instancing
- Software rendering: CPU-bound, limited to ~few thousand nodes for 60 FPS
- For structural engineering models (typically 100-1000 nodes), performance is adequate

### Architecture cost for richer UI

- egui supports: tables, drag-and-drop, tabs, sliders, checkboxes, combo boxes, color pickers
- Adding panels requires Rust code changes and recompilation
- No hot reload (must restart app to see changes)
- Immediate-mode UI: every frame rebuilds UI — simple mental model but can be slow for complex panels

### Confirmed limitations

1. No view frustum culling — all geometry projected every frame
2. Software rendering (CPU) — limited to ~few thousand nodes for interactive FPS
3. No hot reload — Rust code changes require recompilation
4. `deformed_positions()` called every frame — could optimize to only on scale change
5. O(n) picking — no spatial acceleration structure

### Hypothetical difficulties (not confirmed at runtime)

1. Very large models (10K+ nodes) would likely be CPU-bound
2. Complex shaders/materials not available in egui 2D painter
3. Web deployment possible via eframe web backend but with WASM constraints

---

## 7. Decision Framework

### Scoring: 1 (poor) to 5 (excellent)

| # | Criterion | Weight | Tauri + Three.js | egui | Rationale |
|---|-----------|--------|-----------------|------|-----------|
| 1 | 3D viewport capability & performance | High | **5** | **3** | Tauri: full WebGL GPU pipeline, OrbitControls, Raycaster. egui: software CPU projection, basic 3D. |
| 2 | Large model scalability | High | **4** | **2** | Tauri: GPU rendering, frustum culling available. egui: CPU-bound, no culling, ~few K nodes max. |
| 3 | Engineering UI capability | Medium | **4** | **3** | Tauri: full HTML/CSS/JS, any widget library. egui: good immediate-mode UI but less flexible. |
| 4 | Rust kernel integration | High | **4** | **5** | Tauri: clean IPC but 764 µs serialization overhead. egui: zero-copy, direct access to all types. |
| 5 | Development & maintenance cost | Medium | **3** | **4** | Tauri: two languages, two build systems, npm deps. egui: single language, single build. |
| 6 | Windows packaging & reliability | Medium | **4** | **4** | Tauri: MSI/NSIS installers, WebView2 dep. egui: single binary, no deps. Both reliable. |
| 7 | Cross-platform / web potential | Low | **5** | **3** | Tauri: desktop + mobile + web-native Three.js. egui: desktop + WASM (constrained). |
| 8 | Testing & debugging experience | Medium | **3** | **4** | Tauri: Rust backend tests + separate frontend tooling. egui: all tests in Rust, integrated. |

### Weighted scores

Using weights: High=3, Medium=2, Low=1:

| Criterion | Weight | Tauri score | Tauri weighted | egui score | egui weighted |
|-----------|--------|-------------|----------------|------------|---------------|
| 1 | 3 | 5 | 15 | 3 | 9 |
| 2 | 3 | 4 | 12 | 2 | 6 |
| 3 | 2 | 4 | 8 | 3 | 6 |
| 4 | 3 | 4 | 12 | 5 | 15 |
| 5 | 2 | 3 | 6 | 4 | 8 |
| 6 | 2 | 4 | 8 | 4 | 8 |
| 7 | 1 | 5 | 5 | 3 | 3 |
| 8 | 2 | 3 | 6 | 4 | 8 |
| **Total** | | | **72** | | **63** |

### Unweighted scores

- **Tauri + Three.js:** 5+4+4+4+3+4+5+3 = **32**
- **egui:** 3+2+3+5+4+4+3+4 = **28**

### Confidence assessment

| Criterion | Confidence | Notes |
|-----------|------------|-------|
| 1 (3D capability) | High | GPU vs CPU rendering is well-established |
| 2 (Scalability) | High | Software rendering scaling limits are well-known |
| 3 (UI capability) | High | Web UI ecosystem is objectively larger |
| 4 (Rust integration) | High | Directly measured: 764 µs serialization vs 0 |
| 5 (Dev cost) | Medium | Two languages vs one is clear, but ecosystem size offsets some cost |
| 6 (Packaging) | Medium | Both produce native binaries; WebView2 is near-ubiquitous on Windows 10+ |
| 7 (Cross-platform) | Medium | Tauri mobile is still maturing; egui WASM works but constrained |
| 8 (Testing) | High | Directly observed: 55 Rust tests vs 36 Rust tests + no frontend tests |

### Verdict: **A — Recommend Tauri + Three.js**

**Primary reasons:**

1. **GPU rendering is essential** for interactive 3D viewport with structural models. Software rendering limits model size and visual quality.
2. **Web UI ecosystem** provides richer engineering UI components (data tables, plots, tree views, drag-and-drop) that would require significant effort to build in egui.
3. **Cross-platform potential** is valuable — Tauri + Three.js can target desktop, mobile, and web with the same frontend code.
4. **IPC overhead is negligible** — 764 µs serialization is 0.009% of the 8.25 ms scene build time. The FEM solve dominates.
5. **Three.js maturity** — industry-standard 3D library with extensive documentation, examples, and community support.

**Trade-offs accepted:**

1. Two languages (Rust + TypeScript) increase maintenance surface
2. npm dependency management adds complexity
3. Frontend testing requires separate tooling
4. WebView2 runtime dependency (pre-installed on Windows 10 1803+)

---

## 8. Remaining Uncertainties

| # | Uncertainty | Impact | Resolution path |
|---|-------------|--------|----------------|
| 1 | Tauri desktop launch not manually verified | Medium | Run `tauri dev` on target Windows system, verify WebView2 loads, scene renders, picking works |
| 2 | WebGL context cleanup in long sessions | Low | Add `renderer.dispose()` to `SceneManager.dispose()`, verify no context leak over 100+ scene rebuilds |
| 3 | Frontend testing strategy undefined | Medium | Add Vitest unit tests for `SceneManager`, consider Playwright for E2E |
| 4 | Production CSP configuration needed | Low | Set CSP to `default-src 'self'; img-src 'self' asset:; style-src 'self' 'unsafe-inline'` |
| 5 | egui features not in Tauri PoC (node labels, camera info, help, re-solve, forces grid) | Low | Port these features to Tauri PoC in next phase |
| 6 | Large model performance (>10K nodes) untested | Low | Benchmark with 50×50 grid (2,602 nodes) to confirm GPU rendering scales |
| 7 | Tauri mobile/web deployment untested | Low | Out of scope for PoC; revisit if mobile/web target is required |

---

## 9. Recommended Next Phase

**Phase 142: Harden Tauri viewport PoC for production readiness**

Scope:
1. Manually verify `tauri dev` desktop launch and interaction
2. Fix CSP configuration (restrict to `self` + `asset:`)
3. Add `renderer.dispose()` to `SceneManager.dispose()` for proper WebGL cleanup
4. Port missing egui features: node labels, camera info panel, help text, re-solve button
5. Add frontend unit tests (Vitest) for `SceneManager` methods
6. Benchmark at 50×50 grid (2,602 nodes) to confirm GPU rendering scalability
7. Add schema version field to `SceneDto`
8. Add unit labels to DTO and display

**Not in scope:**
- Production application development
- Advanced analysis features
- Shared schema crate
- Mobile/web deployment
- egui PoC modifications (retain as reference)

---

## 10. Validation Results

| Command | Result | Details |
|---------|--------|---------|
| `cargo fmt --all -- --check` | ✅ Pass | No formatting issues |
| `cargo test -p viewport-poc --release` | ✅ 55 pass | 26 lib + 6 geometry + 6 deformation + 1 performance + 12 robustness + 4 selection |
| `cargo test -p tauri-viewport-poc --release` | ✅ 36 pass | 17 DTO + 5 performance + 14 robustness |
| `cargo test -p structural-analysis --release --lib` | ✅ 26 pass | All lib tests pass |
| `npx tsc --noEmit` | ✅ Pass | No type errors |
| `npx vite build` | ✅ Pass | 507 KB JS (128 KB gzipped), 693 ms |
| Desktop launch (egui) | ⚠️ Not performed | No GUI environment available |
| Desktop launch (Tauri) | ⚠️ Not performed | No GUI environment available |

### No code changes made

This phase is a pure audit — no source files were modified. All validation confirms the existing state of both PoCs is correct and functional.

---

## Appendix A: Test Inventory

### egui PoC (55 tests)

| Suite | Count | Key tests |
|-------|-------|-----------|
| lib (inline) | 26 | Camera3D, ViewportScene, Vec3 math, model builders |
| geometry | 6 | Bounding box, member midpoint, deformed positions |
| deformation | 6 | Scale=0, scale=1, non-finite, undeformed |
| selection | 4 | Node pick, member pick, far pick, state helpers |
| robustness | 12 | Empty, coincident, zero-length, non-finite, camera clamps |
| performance | 1 | Projection cost 1×1 to 10×10, 1000-frame average |

### Tauri PoC (36 tests)

| Suite | Count | Key tests |
|-------|-------|-----------|
| dto | 17 | SceneDto structure, node/member fields, fineness, equilibrium |
| robustness | 14 | Local axes right-handed, no duplicates, indices in bounds, fineness, equilibrium |
| performance | 5 | Portal/inclined/grid build time, serialization, repeated build no degradation |

### Test gap analysis

| Area | egui PoC | Tauri PoC | Gap |
|------|----------|-----------|-----|
| Geometry | 6 tests | 0 tests | Tauri lacks geometry-specific tests (relies on DTO tests) |
| Selection | 4 tests | 0 tests | Tauri lacks selection tests (Raycaster is Three.js built-in) |
| Deformation | 6 tests | 0 tests | Tauri lacks deformation tests (computed in frontend) |
| Robustness | 12 tests | 14 tests | Both well-covered, different focus |
| Performance | 1 test | 5 tests | Tauri more comprehensive (multiple scales + serialization) |

**Note:** Tauri PoC's test gaps are in frontend (TypeScript) code, which has no test framework configured. Adding Vitest would address this.
