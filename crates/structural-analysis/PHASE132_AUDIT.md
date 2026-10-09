# Phase 132 — 3D Frame Architecture Readiness Audit

**Date:** 2026-10-09  
**Auditor:** GLM-5.2  
**Base HEAD:** `0f52382` (Phase 131)  
**Status:** Complete — no production changes  

## 1. Verified baseline

```
HEAD: 0f52382 Phase 131: audit truss batch solver correctness
```

Working tree: clean (only untracked audit docs from prior phases). No production code changes made during this audit.

## 2. Scope and files inspected

| File | Lines | Role |
|------|-------|------|
| `src/beam_fem.rs` | 4489 | 2D Euler-Bernoulli beam element, solver, section forces |
| `src/frame.rs` | 2602 | Frame façade (typed handles, supports, results) |
| `src/truss.rs` | 1375 | 2D truss element and solver |
| `src/truss3d.rs` | 1560 | 3D truss element and solver |
| `src/load.rs` | 636 | LoadCase, Load(Combination, LoadSource |
| `src/mechanism.rs` | 578 | Mechanism detection and rigid-body classification |
| `src/postprocessing.rs` | 1037 | Envelope/post-processing for frame and truss |
| `src/lib.rs` | 151 | Public exports |
| `section-properties` | — | SparseMatrix, LinearSolver, Material, SectionProperties, WarpingProperties |

## 3. Current architecture summary

### Module structure

```text
structural-analysis
├── beam_fem.rs     ← 2D beam element (3 DOF/node: ux, uy, rz)
├── frame.rs        ← Frame façade (typed handles, wraps beam_fem)
├── truss.rs        ← 2D truss (2 DOF/node: ux, uy)
├── truss3d.rs      ← 3D truss (3 DOF/node: ux, uy, uz)
├── load.rs         ← LoadCase/LoadCombination (DOF-agnostic container)
�, mechanism.rs     ← Mechanism detection (matrix-level, DOF-agnostic)
├── postprocessing.rs ← Envelopes (per element type)
└── lib.rs          ← Public exports
```

### DOF conventions

| Element type | DOF/node | DOF layout | `dof_index` | File |
|-------------|----------|------------|-------------|------|
| 2D beam/frame | 3 | `[ux, uy, rz]` | `3·node + d` | `beam_fem.rs:2166` |
| 2D truss | 2 | `[ux, uy]` | `2·node + d` | `truss.rs:420` |
| 3D truss | 3 | `[ux, uy, uz]` | `3·node + d` | `truss3d.rs:420` |

### Key architectural patterns

1. **Separate types per element kind** — no shared `Element` trait. Each element type (beam, truss+3D, truss3D) has its own model, solver, and result types. They reuse only DOF-agnostic infrastructure (`SparseMatrix`, `LinearSolver`, `diagnose_reduced`).

2. **Façade pattern** — `FrameModel` wraps `BeamModel`, adding typed handles (`NodeHandle`, `MemberHandle`), support vocabulary, and validation. All mechanics delegated to `beam_fem`.

3. **Static condensation** — boundary conditions applied by partitioning free/constrained DOFs and reducing `K_ff`. Same algorithm in all solvers (`beam_fem.rs:2788`, `truss.rs:909`, `truss3d.rs:965`).

4. **Factorization lifecycle** — `LinearSolver::factor()` + `LinearSolver::solve()` separated. `solve_configured` factors once; `solve_pre_factored` / `solve_cases` reuse factorization.

5. **Result provenance** — `LoadSource` enum tracks whether results came from model loads, a `LoadCase`, or a `LoadCombination`.

### Section properties available in `section-properties`

| Property | Source | Available |
|----------|--------|-----------|
| Area `A` | `GeometricProperties.area` | ✅ |
| `I_x`, `I_y` | `GeometricProperties.ix`, `.iy` | ✅ |
| Principal `I_11`, `I_22` | `PrincipalProperties.i11`, `.i22` | ✅ |
| Torsion `J` | `WarpingProperties.j` | ✅ |
| Warping `I_w` | `WarpingProperties.iw` | ✅ |
| Shear areas `A_y`, `A_z` | `WarpingProperties.ay`, `.az` | ✅ |
| `E` | `Material.youngs_modulus` | ✅ |
| `G` | `Material.shear_modulus` | ✅ |
| `ν` | `Material.poissons_ratio` | ✅ |
| `ρ` | `Material.density` | ✅ |

**Current `BeamSection`** (`beam_fem.rs:21`): only `{ area, second_moment }` — insufficient for 3D.

## 4. 3D Frame readiness assessment

### 4.1 DOF mapping (6 DOF/node)

3D frame requires 6 DOF per node: `[ux, uy, uz, rx, ry, rz]`. The existing `Dof` enum (`beam_fem.rs:1487`) has 3 variants (`Ux`, `Uy`, `Rz`). A new `Dof3D` enum* enum with 6 variants is needed.

The `dof_index` multiplier (currently hardcoded 3 for beam, 2/3 for truss) would be 6 for 3D frame. This does not conflict with existing conventions — each element type has its own `dof_index`.

### 4.2 Element stiffness (12×12)

A conventional 3D frame element has 12 DOFs (6 per node). The local stiffness matrix combines:

- **Axial** (EA/L): same as 2D, 2×2 block
- **Torsion** (GJ/L): 2×2 block for rotations about member axis
- **Bending about local y** (EI_y/L³): 4×4 block (transverse disp + rotation)
- **Bending about local z** (EI_z/L³): 4×4 block (transverse disp + rotation)

The existing `local_stiffness` (`beam_fem.rs:634`) returns `[[f64; 6]; 6]`. A 3D version returns `[[f64; 12]; 12]`. No conflict — separate function in a new module.

### 4.3 Coordinate transformation (12×12)

2D transformation (`beam_fem.rs:692`) uses a single rotation angle (c, s). 3D requires a **direction cosine matrix** (3×3 rotation) based on member orientation vector + a reference "up" vector for the local y-axis.

The 12×12 transformation is block-diagonal with four 3×3 rotation blocks. This is a standard FEM construction, well-documented in textbooks (e.g., Bathe, Cook). The existing 2D transformation pattern (`Tᵀ · K_local · T`) extends directly.

**Orientation convention:** The member direction is0 = (node_j - node_i) / L defines the local x-axis. A user-supplied reference vector (typically global Z or a custom "up" direction) defines the local y-axis via: y = (ref × x) / |ref × x|, z = x × y. Degenerate cases (member parallel to reference) must be rejected.

### 4.4 Section properties

A new `FrameSection3D` struct is needed:

```rust
pub struct FrameSection3D {
    pub area: f64,      // A
    pub iy: f64,        // Second moment about local y
    pub iz: f64,        // Second moment about local z
    pub j: f64,         // Torsion constant
}
```

All four properties are available from `section-properties`:
- `A` ← `GeometricProperties.area`
- `Iy`, `Iz` ← `GeometricProperties.ix`, `.iy` (or `PrincipalProperties.i11`, `.i22`)
- `J` ← `WarpingProperties.j`

**Shear deformation (Timoshenko)** is a later enhancement. MVP uses Euler-Bernoulli (no shear areas needed).

### 4.5 End releases

Current `EndRelease` (`beam_fem.rs:495`) has 2 boolean fields (`start_rotation`, `end_rotation`). 3D needs up to 6 releases (3 rotations per end: `rx`, `ry`, `rz`). The static condensation framework (`condense_matrix`, `beam_fem.rs:793`) is dimension-agnostic but `invert_small` (`beam_fem.rs:1252`) only supports 1×1/2×2. For 3D with up to 6 released DOFs, a general small matrix inverse (≤6×6) is needed.

**MVP recommendation:** Start with no end releases (fully rigid joints). Add releases in a later phase.

### 4.6 Boundary conditions and supports

The static condensation framework (`apply_boundary_conditions`) is DOF-agnostic — it operates on `fixed_dofs: Vec<bool>` and `prescribed_values: Vec<Option<f64>>`. It works for any DOF count. ✅

Support types for 3D frame:
- `fix(node)`: constrain all 6 DOFs ✅ (pattern exists)
- `pin(node)`: constrain 3 translations, rotations free ✅
- `roller_*`: constrain 1-2 translations ✅
- `spring(node, dof, k)`: spring on any DOF@ dof, k) ✅ (pattern exists)
- `inclined_roller`: 3D version needs a constrained direction vector in 3D space

### 4.7 Load vector assembly

The existing `assemble_global_load_vector` (`beam_fem.rs:2220`) handles:
- Nodal forces (global) — extends directly to 3D (6 components)
- Distributed loads (local) — needs 3D equivalent nodal loads
- Point loads (local) — needs 3D equivalent nodal loads
- Applied moments (global) — extends to 3D (3 moment components)

### 4.8 Result recovery

| Component | 2D | 3D required |
|-----------|----|-------------|
| Displacements | `[ux, uy, rz]` | `[ux, uy, uz, rx, ry, rz]` |
| Reactions | `[Rx, Ry, Mz]` | `[Rx, Ry, Rz, Mx, My, Mz]` |
| Member end forces | `[N_i, V_i, M_i, N_j, V_j, M_j]` | `[N_i, Vy_i, Vz_i, T_i, My_i, Mz_i, ..._j]` (12) |
| Section forces | `{N, V, M}` | `{N, Vy, Vz, T, My, Mz}` (6) |

### 4.9 Mechanism detection

`diagnose_reduced` (`mechanism.rs:253`) is **fully matrix-level and DOF-agnostic**. It takes `k_ff: &SparseMatrix` and `rigid_candidates: &[Vec<f64>]`. For 3D frame, only the candidate generation changes:

- 2D: 3 candidates (Tx, Ty, Rz) — `frame.rs:1179`
- 3D: 6 candidates (Tx, Ty, Tz, Rx, Ry, Rz)

The 3D rigid-body modes at node `(x, y, z)`:
- Tx: `[1, 0, 0, 0, 0, 0]`
- Ty: `[0, 1, 0, 0, 0, 0]`
- Tz: `[0, 0, 1, 0, 0, 0]`
- Rx: `[0, -z, y, 1, 0, 0]`
- Ry: `[z, 0, -x, 0, 1, 0]`
- Rz: `[-y, x, 0, 0, 0, 1]`

### 4.10 Envelope/post-processing

The `Extremum` type (`postprocessing.rs:73`) is a pure scalar min/max tracker — DOF-agnostic. The envelope pattern (per-element samples with source tracking) extends to 3D by adding more component fields. A new `Frame3DEnvelope` type would follow the same pattern as `Envelope` and `Truss3DEnvelope`.

### 4.11 Solver infrastructure

All solver backends (`SparseLU`, `SkylineLDLT`, `DenseGaussian`, `CG`, `ICCG`) are matrix-level and DOF-agnostic. They operate on `SparseMatrix` without knowledge of DOF semantics. **No solver changes needed for 3D frame.** ✅

## 5. Numerical and structural correctness risks

### 5.1 Symmetry and definiteness

3D frame stiffness matrices are **symmetric positive semi-definite** (same as 2D). The existing solvers handle this:
- `SkylineLDLT`: LDLᵀ for symmetric matrices ✅
- `SparseLU`: general, handles symmetric ✅
- `DenseGaussian`: general ✅
- `CG`/`ICCG`: for SPD (after BC application) ✅

### 5.2 Rigid-body modes

An unconstrained 3D frame has **6 rigid-body modes** (vs 3 in 2D). The mechanism detector needs 6 candidates (see §4.9). After proper constraint application, `K_ff` should be SPD if the structure is stable.

### 5.3 Ill-conditioning

3D frames with large stiffness ratios (e.g., very stiff torsion vs flexible bending) may produce ill-conditioned `K_ff`. The existing `diagnose_reduced` with scale-adaptive tolerances handles this. ✅

### 5.4 Degenerate geometry

- **Zero-length member**: rejected in `from_model` (existing pattern) ✅
- **Member parallel to orientation vector**: local y-axis undefined — must reject with `InvalidModel`
- **Coincident nodes**: same as zero-length — rejected ✅

### 5.5 Factorization reuse

The `solve_cases` pattern (Phase 130) is DOF-agnostic — it operates on `k_ff` and `f_reduced` without knowledge of DOF semantics. **Factorization reuse extends directly to 3D frame.** ✅

### 5.6 Result/envelope representation

Current envelope types (`Envelope`, `TrussEnvelope`, `Truss3DEnvelope`) have fixed component counts. A new `Frame3DEnvelope` type with 6 displacement components, 6 reaction components, and 6 section force components is needed. No information loss if properly designed.

## 6. Architecture alternatives

### Option A: Extend `structural-analysis` with new `frame3d` module

**Approach:** Add `src/frame3d.rs` with `FrameModel3D`, `FrameElement3D`, `FrameSection3D`, `FrameSolver3D`, `FrameAnalysisResult3D`, etc. Follow the `truss`/`truss3d` pattern — separate types, no shared element trait.

| Criterion | Assessment |
|-----------|------------|
| API compatibility | ✅ No changes to existing 2D APIs |
| DOF mapping | ✅ New `dof_index` with multiplier 6, no conflict |
| Risk to 2D | ✅ Zero — new module, no shared mutable state |
| Code duplication | Moderate — assembly/condensation pattern repeated, but with different DOF counts |
| Testing burden | New test file, new benchmarks — independent of 2D tests |
| Incremental path | ✅ Can implement MVP (stiffness + solve) first, add loads/releases/envelope later |

### Option B: Shared element/assembly abstraction first

**Approach:** Introduce an internal `Element` trait or shared assembly module before adding 3D frame.

| Criterion | Assessment |
|-----------|------------|
| API compatibility | ⚠️ May require refactoring existing types |
| DOF mapping | ⚠️ Trait would need generic DOF count |
| Risk to 2D | ⚠️ High — refactoring working code |
| Code duplication | Low — shared abstraction |
| Testing burden | High — must re-test all existing elements through new trait |
| Incremental path | ❌ Large upfront investment before any 3D functionality |

**Rejected:** The existing `truss`/`truss3d` pattern demonstrates that separate types work well. A shared trait adds complexity without solving a demonstrated problem. The Phase 94 decision ("Constraint abstraction layer not needed") applies analogously.

### Option C: Separate types + narrowly scoped shared utilities

**Approach:** Same as Option A, but extract shared utilities (e.g., static condensation, small matrix inverse) into internal modules where duplication is clear.

| Criterion | Assessment |
|-----------|------------|
| API compatibility | ✅ No changes to existing APIs |
| DOF mapping | ✅ New types with own DOF mapping |
| Risk to 2D | ✅ Low — shared utilities are pure functions |
| Code duplication | Low — shared extraction where justified |
| Testing burden | Moderate — shared utilities need their own tests |
| Incremental path | ✅ Can extract utilities incrementally |

**Recommended:** Option C is the best balance. Start with Option A (new module), extract shared utilities only when duplication is demonstrated and the extraction is clearly beneficial.

## 7. Findings

| Severity | Finding | Impact |
|----------|---------|--------|
| P0 | None | — |
| P1 | None | — |
| P2 | None | — |
| P3 | `BeamSection` has only 2 properties (A, I); 3D needs ≥4 (A, Iy, Iz, J). Design limitation, not a defect — new `FrameSection3D` type needed. | Future |
| P3 | `invert_small` (`beam_fem.rs:1252`) only supports 1×1/2×2. 3D end releases with >2 released DOFs need a general inverse. | Future (MVP has no releases) |
| P3 | `EquilibriumReport` (`frame.rs:182`) has 3 components (fx, fy, mz). 3D needs 6. | Future |
| P3 | No 0D shear area in `BeamSection` — Timoshenko beam not supported. 3D frame MVP uses Euler-Bernoulli (no shear deformation). | Future enhancement |

**No confirmed defects.** All findings are design limitations relevant to future 3D frame implementation, not bugs in existing code.

## 8. Recommended 3D Frame MVP

The smallest useful 3D frame MVP that can be validated rigorously:

1. **`FrameSection3D`**: `{ area, iy, iz, j }` — 4 properties
2. **`FrameElement3D`**: 12×12 local stiffness (axial + torsion + biaxial bending), 12×12 transformation (direction cosine)
3. **`FrameModel3D`**: nodes with `(x, y, z)`, 6 DOF/node, supports (`fix`, `pin`, `restrain`), nodal loads (6 components)
4. **`FrameSolver3D`**: assembly + static condensation + solve (reusing existing `LinearSolver` infrastructure)
5. **`FrameAnalysisResult3D`**: displacements (6/node), reactions (6/node), member end forces (12/element)
6. **No end releases, no distributed loads, no envelope** — add in later phases

**Validation:** Analytical benchmarks for:
- Axial bar (same as 3D truss)
- Pure torsion shaft (θ = TL/GJ)
- Cantilever bending about each local axis (δ = PL³/3EI)
- 3D frame structure with known solution (e.g., space frame from textbook)

## 9. Dependency-ordered implementation roadmap

| Phase | Scope | Prerequisites | Key risks | Tests | Completion criteria |
|-------|-------|---------------|-----------|-------|---------------------|
| 133 | `FrameSection3D` + `FrameElement3D` local stiffness (12×12) + transformation | None | Sign conventions, orientation degeneracy | Unit tests: symmetry, positive definiteness, known coefficients | Local K is symmetric, matches hand calculation |
| 134 | `FrameModel3D` + `FrameSolver3D` assembly + solve | Phase 133 | Assembly correctness, DOF mapping | Axial bar, torsion shaft, cantilever benchmarks | Displacements match analytical solutions to 1e-9 |
| 135 | Supports, prescribed displacements, reactions | Phase 134 | BC application, reaction recovery | Propped cantilever, fixed-fixed beam | Reactions satisfy equilibrium |
| 136 | Member end forces + section forces | Phase 135 | Force recovery sign conventions | Cantilever with known N, V, M, T | End forces match analytical |
| 137 | Nodal loads (6 components) + LoadCase/LoadCombination | Phase 135 | Load vector assembly | Multi-load-case frame | Results match individual solves |
| 138 | Mechanism diagnostics (6 rigid-body candidates) | Phase 134 | Candidate generation | Unstable frame detection | Correct classification |
| 139 | Distributed loads (3D equivalent nodal loads) | Phase 136 | Consistent load vector | Beam with UDL | Matches analytical deflection |
| 140 | End releases (3D hinges) | Phase 136 | General small matrix inverse | Hinged frame benchmark | Matches analytical |
| 141 | Envelope/post-processing | Phase 137 | Result typeGathering | Multi-case envelope | Extrema correct |
| 142 | Batch solving (`solve_cases`) | Phase 137 | Factorization reuse | Batch vs individual | Results match, speedup confirmed |

Phases 133-135 form the MVP. Phases 136-142 are enhancements that can be reordered or merged.

## 10. Test commands and outcomes

| Command | Result |
|---------|--------|
| `cargo fmt --all -- --check` | ✅ PASS |
| `cargo check -p structural-analysis` | ✅ PASS |
| `cargo test -p structural-analysis --test truss` | ✅ 111 passed |
| `cargo test -p structural-analysis --test truss3d` | ✅ 95 passed |
| `cargo test -p structural-analysis --lib` | ✅ 26 passed |
| `cargo test -p structural-analysis --test frame_api` | ✅ 56 passed |
| `cargo test -p structural-analysis --test frame_correctness` | ✅ 19 passed |
| `cargo test -p structural-analysis --test beam_fem` | ✅ 22 passed |
| `cargo test -p structural-analysis --test end_release` | ✅ 25 passed |
| `cargo doc -p structural-analysis --no-deps` | ✅ 3 pre-existing private-link warnings |
| `cargo clippy -p structural-analysis --all-targets -- -D warnings` | ⚠️ Fails on pre-existing section-properties lint debt (133 errors in dependency, 0 in structural-analysis) |

## 11. Readiness verdict

### **READY WITH LIMITED REFACTOR**

The existing architecture can support 3D frame analysis with the following approach:

- **New module** `frame3d.rs` with separate types (no shared element trait)
- **Reuse** DOF-agnostic infrastructure: `SparseMatrix`, `LinearSolver`, `SolverRegistry`, `diagnose_reduced`, `Extremum`, `LoadSource`
- **No changes** to existing 2D beam/frame or 2D/3D truss code
- **New types**: `FrameSection3D`, `FrameElement3D`, `FrameModel3D`, `FrameSolver3D`, `FrameAnalysisResult3D`, `Dof3D`
- **Section properties** from `section-properties`: all required properties (A, Iy, Iz, J, G) are available

The "limited refactor" is the extraction of shared utilities (static condensation pattern, small matrix inverse) only if duplication is demonstrated during implementation. No upfront abstraction is needed.

## 12. Recommended Phase 133

**Scope:** `FrameSection3D` + `FrameElement3D` local stiffness (12×12) + coordinate transformation (12×12 direction cosine matrix).

**Acceptance criteria:**
1. `FrameSection3D::new(area, iy, iz, j)` constructor
2. `FrameElement3D::local_stiffness()` returns symmetric 12×12 matrix
3. `FrameElement3D::transformation_matrix()` returns 12×12 orthogonal matrix
4. `FrameElement3D::global_stiffness()` = Tᵀ · K_local · T
5. Local stiffness matches hand calculation for: axial only, torsion only, bending about y only, bending about z only
6. Transformation is orthogonal: T · Tᵀ = I
7. Degenerate orientation (member parallel to reference vector) rejected with `InvalidModel`
8. All tests pass; no changes to existing code
