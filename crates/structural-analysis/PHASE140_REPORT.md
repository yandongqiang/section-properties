# Phase 140 — Tauri + Three.js Viewport Comparison PoC

## Baseline

| Item | Value |
|------|-------|
| Starting HEAD | `b9335d8` (Phase 139) |
| Rust edition | 2024 |
| Rust toolchain | 1.97.1 |
| Node.js | v24.11.1 |
| npm | 11.6.2 |

## Prototype Location

```
apps/tauri-viewport-poc/
├── .gitignore
├── package.json
├── tsconfig.json
├── vite.config.ts
├── index.html
├── src/                        # TypeScript frontend
│   ├── main.ts                 # App entry, Tauri IPC, UI panel
│   ├── scene-manager.ts        # Three.js scene management
│   ├── types.ts                # DTO type definitions
│   └── style.css               # Panel styling
├── dist/                       # Built frontend (Vite output)
│   ├── index.html
│   └── assets/
└── src-tauri/                  # Rust backend
    ├── Cargo.toml
    ├── build.rs
    ├── tauri.conf.json
    ├── capabilities/default.json
    ├── icons/icon.ico
    ├── src/
    │   ├── lib.rs              # Scene DTO + Tauri commands
    │   └── main.rs             # Entry point
    └── tests/
        ├── dto.rs              # 17 DTO correctness tests
        ├── robustness.rs       # 14 robustness tests
        └── performance.rs      # 5 performance tests
```

## Dependency Versions

### Rust (src-tauri/Cargo.toml)
| Dependency | Version |
|------------|---------|
| tauri | 2.12.2 |
| tauri-build | 2.7.1 |
| serde | 1.0.229 |
| serde_json | 1.0.151 |
| structural-analysis | path (workspace) |

### Frontend (package.json)
| Dependency | Version |
|------------|---------|
| @tauri-apps/api | ^2 |
| three | 0.170.0 |
| @types/three | (dev) |
| typescript | ^5.6 |
| vite | 5.4.21 |

## Architecture

### Rust/JS Boundary — Option A: Rust Computes, Exports DTO

The Rust backend builds the structural model, solves the FEM analysis, and
serialises the complete result into a `SceneDto` struct. The TypeScript frontend
receives the DTO via Tauri's IPC and renders it with Three.js. The frontend never
calls solver internals.

```
FrameModel3D → solve() → FrameAnalysisResult3D → build_scene() → SceneDto
    → serde JSON → Tauri IPC → TypeScript → Three.js
```

### Tauri Commands

| Command | Model | Nodes | Members |
|---------|-------|-------|---------|
| `get_scene` | Portal frame | 4 | 3 |
| `get_scene_inclined` | Inclined frame | 5 | 4 |
| `get_scene_grid(nx, nz)` | Grid frame | (nx+1)(nz+1)×2 | varies |

### Scene DTO Structure

```rust
SceneDto {
    nodes: Vec<NodeDto>,      // id, position, displacement, rotation, reactions
    members: Vec<MemberDto>,  // id, connectivity, ref_vec, local_axes, end_forces
    n_nodes: usize,
    n_members: usize,
}
```

Each `NodeDto` carries position, displacement, rotation, reaction force, and
reaction moment. Each `MemberDto` carries connectivity, reference vector,
orthonormal local axes (x̂, ŷ, ẑ), and 12-component end forces in both local
and global coordinates.

### Frontend Architecture

- **SceneManager** class encapsulates all Three.js state (scene, camera,
  renderer, OrbitControls, raycaster, object groups).
- **main.ts** handles Tauri IPC, UI panel construction, and event wiring.
- **types.ts** defines DTO interfaces mirroring the Rust structs.
- Rendering uses `THREE.Mesh` spheres for nodes, `CylinderGeometry` for members,
  `ArrowHelper` for local axes, `GridHelper` / `AxesHelper` for reference.
- Picking uses `THREE.Raycaster` against node and member meshes.

## Feature Implementation

| Feature | egui PoC (Phase 139) | Tauri PoC (Phase 140) |
|---------|----------------------|----------------------|
| 3D nodes | ✅ Software projection | ✅ Three.js spheres |
| 3D members | ✅ Software lines | ✅ Three.js cylinders |
| Global axes | ✅ | ✅ AxesHelper |
| Grid | ✅ | ✅ GridHelper |
| Orbit / pan / zoom | ✅ Custom camera | ✅ OrbitControls |
| Node picking | ✅ Distance test | ✅ Raycaster |
| Member picking | ✅ Distance test | ✅ Raycaster |
| Selection highlight | ✅ Color change | ✅ Color change |
| Local member axes | ✅ Drawn lines | ✅ ArrowHelper |
| Undeformed shape | ✅ | ✅ LineSegments wireframe |
| Deformed shape | ✅ Scale slider | ✅ Scale slider |
| Node displacement values | ✅ Panel | ✅ Panel |
| End forces (local + global) | ✅ Panel | ✅ Panel |
| Model selector | ✅ Hardcoded | ✅ Dropdown (5 models) |
| Property panel | ✅ | ✅ |

## Test Results

### Rust Tests (36 total, all passing)

| Suite | Tests | Status |
|-------|-------|--------|
| DTO correctness (`tests/dto.rs`) | 17 | ✅ all pass |
| Robustness (`tests/robustness.rs`) | 14 | ✅ all pass |
| Performance (`tests/performance.rs`) | 5 | ✅ all pass |

**DTO tests** verify: node/member counts, sequential IDs, position matching,
connectivity, reference vectors, orthonormality of local axes, right-handedness,
nonzero displacements/reactions, 12-component end forces, JSON serialisation
structure, and `build_scene_from_result` consistency.

**Robustness tests** verify: all values finite, node indices in bounds, no
duplicate IDs, x̂ aligned with member direction, right-handed coordinate systems,
grid frames build correctly, repeated builds produce identical results, and
global force/moment equilibrium (ΣR = −F_applied).

### Frontend Verification

| Check | Result |
|-------|--------|
| TypeScript type check (`tsc --noEmit`) | ✅ pass |
| Vite production build | ✅ 728ms, 510KB JS |
| `cargo check` (whole workspace) | ✅ pass |
| `cargo clippy` (tauri-viewport-poc) | ✅ 0 warnings |
| `cargo fmt --check` | ✅ pass |

## Performance Measurements

### Methodology

All Rust measurements use `cargo test --release` with `std::time::Instant`.
The scene build time includes: model construction, FEM solve, DTO extraction,
and local axes / global force computation. Serialisation time is measured
separately with `serde_json::to_string`.

### Rust Backend (release mode)

| Model | Nodes | Members | Scene Build | Serialisation |
|-------|-------|---------|-------------|---------------|
| 1×1 grid | 8 | 7 | 257 µs | — |
| 5×5 grid | 72 | 91 | 7.46 ms | — |
| 10×10 grid | 242 | 331 | 8.25 ms | 764 µs (333 KB) |

Repeated 10×10 builds show no degradation: 5.2–7.2 ms per build.

### Frontend Build

| Metric | Value |
|--------|-------|
| Vite build time | 728 ms |
| Bundle size (JS) | 507.67 KB (128.48 KB gzip) |
| Bundle size (CSS) | 1.58 KB (0.67 KB gzip) |

### Comparison with egui PoC (Phase 139)

| Metric | egui PoC | Tauri PoC |
|--------|----------|-----------|
| Rendering | CPU 3D→2D projection + 2D painter | WebGL via Three.js |
| Frame projection (242 nodes) | 54.6 µs/frame | GPU-accelerated (not measured) |
| Scene DTO build (242 nodes) | N/A (direct) | 8.25 ms (includes solve + DTO) |
| DTO serialisation (242 nodes) | N/A | 764 µs (333 KB JSON) |
| Dependency count | 2 (egui, eframe) | 3+ (tauri, three, vite, …) |
| Binary size | Small (native) | Larger (WebView2 + JS bundle) |
| GPU acceleration | ❌ | ✅ |
| Ecosystem / plugins | Rust-native | Web ecosystem (npm) |
| Hot reload | ❌ | ✅ (Vite HMR) |

**Note:** The egui PoC's 54.6 µs/frame is the CPU projection cost only — it
does not include FEM solve or data extraction. The Tauri PoC's 8.25 ms includes
model construction, FEM solve, local axes computation, global force
transformation, and DTO serialisation. The rendering itself is GPU-accelerated
via WebGL and not directly comparable to CPU projection.

## Limitations and Risks

1. **WebView2 dependency**: Tauri on Windows requires WebView2 (pre-installed on
   Windows 11, bundled on Windows 10). This adds a runtime dependency not present
   with egui.

2. **Bundle size**: The Three.js bundle is ~510 KB (128 KB gzip). For a
   structural engineering desktop app, this is acceptable but larger than a
   pure-native solution.

3. **IPC overhead**: Each `get_scene` call serialises the entire scene to JSON
   and crosses the Rust→JS boundary. For the 10×10 grid (242 nodes, 331 members),
   this is 333 KB / 764 µs. For interactive deformation scale changes, the
   frontend recomputes positions locally without IPC, so this is a one-time cost
   per model load.

4. **No `tauri dev` runtime test**: The PoC was verified via `cargo check`,
   `cargo test`, `tsc --noEmit`, and `vite build`. A full `tauri dev` desktop
   launch was not performed in this phase (requires interactive GUI validation).

5. **Icon file**: A minimal placeholder `icon.ico` was created for Windows
   resource compilation. A proper icon should be created for production.

6. **Security**: CSP is disabled (`"csp": null`) for the PoC. Production should
   enable CSP.

7. **npm vulnerabilities**: 2 vulnerabilities reported by `npm audit` in
   dev dependencies. Not relevant for the built frontend but should be addressed
   for production.

## Recommendation

For the structural analysis viewport, **both approaches are viable** but serve
different priorities:

- **egui (Phase 139)** is superior for: minimal dependencies, pure-native
  binary, predictable performance, Rust-only codebase, and simpler deployment.

- **Tauri + Three.js (Phase 140)** is superior for: GPU-accelerated rendering,
  web ecosystem access (npm packages, CSS, WebGL shaders), hot reload during
  development, richer UI tooling, and team scalability (TS/JS frontend skills).

**Recommendation**: Continue parallel evaluation. For the next phase, implement
a medium-complexity feature (e.g., member load visualization or result
contours) in both PoCs and compare development effort, performance, and user
experience. The final choice should depend on:
1. Whether GPU-accelerated rendering is needed for the target model sizes
2. Team expertise (Rust vs. TypeScript)
3. Deployment constraints (single binary vs. WebView2 dependency)
4. UI complexity roadmap

## Unresolved Criteria

- [ ] Full `tauri dev` desktop launch verification
- [ ] Large model performance (1000+ nodes) comparison
- [ ] Memory usage measurement (not captured in this phase)
- [ ] Startup time measurement (not captured in this phase)
- [ ] Production bundling (`tauri build`) verification
- [ ] Feature parity test with identical model data side-by-side
