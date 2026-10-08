# Phase 115 — 3D Truss Deep Audit / Numerical & API Review

**Date:** 2026-10-08
**Baseline:** `b6d9236` (Phase 114)
**Production code changes:** 0 (test-only additions)
**Output:** 6 new integration tests + this document

---

## 1. Baseline

```text
HEAD: b6d9236 (Phase 114: 3D Truss MVP)
working tree: clean (only old untracked PHASE docs)
Phase 114 commit: b6d9236, 4 files, +1749 lines
```

---

## 2. Mathematical Audit

### 2.1 Geometry and direction cosines

**Implementation** (`truss3d.rs:195–218`):

```text
L = sqrt(dx² + dy² + dz²)
lx = dx / L,  ly = dy / L,  lz = dz / L
```

**Verification:**

- `lx² + ly² + lz² = (dx² + dy² + dz²) / L² = L² / L² = 1` — mathematically
  guaranteed when `L > 0` and coordinates are finite. ✓
- Zero-length rejection: `if l <= 0.0` catches `L = 0`. ✓
- **NaN coordinates**: If any coordinate is NaN, `L` is NaN, and
  `NaN <= 0.0` is `false`, so the check passes silently. This is a
  **P2 finding** (see §7). The 2D Truss has the same behaviour; this is
  a pre-existing pattern, not a regression.

### 2.2 Stiffness matrix

**Implementation** (`truss3d.rs:229–258`):

```text
K = (EA/L) · [ c·cᵀ   -c·cᵀ
              -c·cᵀ    c·cᵀ ]
```

where `c = (lx, ly, lz)`.

**Verification:**

- **Symmetry**: `K[i][j] = K[j][i]` by construction (outer product
  `c·cᵀ` is symmetric). Confirmed by `test_stiffness_symmetry` for
  arbitrary `(1.5, 2.3, 0.7)` direction. ✓
- **Rank**: Single truss element has rank 1 (one independent stiffness
  `EA/L`), nullity 5 (6 DOF − 1 rank). The 6 rigid-body modes restricted
  to 2 nodes span a 5-dimensional space (rotation about the element's
  own axis gives zero displacement at both end nodes). ✓
- **PSD**: `K = (EA/L) · c · cᵀ` is PSD since `EA/L > 0` and `c·cᵀ` is
  PSD. ✓

### 2.3 Coordinate transformation

The stiffness is built directly in global coordinates via direction
cosines, equivalent to `K_global = Tᵀ · K_local · T` where
`K_local = diag(EA/L, 0, 0, EA/L, 0, 0)`.

**Verified orientations:**

| Direction | Test | Result |
|---|---|---|
| `(1,0,0)` — X axis | `test_axial_bar_along_x` | ux = FL/(EA), N = F ✓ |
| `(0,1,0)` — Y axis | `test_axial_bar_along_y` | uy = FL/(EA), N = F ✓ |
| `(0,0,1)` — Z axis | `test_axial_bar_along_z` | uz = FL/(EA), N = F ✓ |
| `(1,2,3)` — arbitrary | `test_3d_geometry` | N = F·√14 ✓ |
| `(3,0,0), (0,4,0), (0,0,5)` — tripod | `test_tetrahedral_benchmark` | full analytical match ✓ |

### 2.4 Node-order invariance

**Mathematical proof:**

Swapping `node_i ↔ node_j` negates `(dx, dy, dz)` and hence `(lx, ly, lz)`.
Since `K = (EA/L) · c · cᵀ` and `c` changes sign, `c · cᵀ` is unchanged.
The DOF mapping is permuted (first 3 ↔ last 3), but the assembled global
stiffness is identical.

For axial force: `N = (EA/L) · (lx·Δux + ...)` — both `lx` and `Δux`
change sign, so `N` is unchanged.

**Test**: `test_node_order_invariance` — two models with `element(0,1)`
and `element(1,0)` produce identical displacement and axial force. ✓

### 2.5 Axial force

**Implementation** (`truss3d.rs:270–285`):

```text
N = (EA/L) · (lx·(ux_j − ux_i) + ly·(uy_j − uy_i) + lz·(uz_j − uz_i))
```

**Tension positive** — consistent with the stiffness matrix and the 2D
Truss convention.

**Verified against analytical solutions:**

| Case | Expected N | Computed N | Match |
|---|---|---|---|
| X bar, F = 5 | 5.0 | 5.0 | ✓ |
| X bar, F = −5 | −5.0 | −5.0 | ✓ |
| (1,2,3) bar, F = 1 | √14 ≈ 3.7417 | √14 | ✓ |
| Tripod member 0-1 | −1.0 | −1.0 | ✓ |
| Tripod member 0-2 | −2.0 | −2.0 | ✓ |
| Tripod member 0-3 | −3.0 | −3.0 | ✓ |

### 2.6 Global equilibrium

**Implementation** (`truss3d.rs:975–1063`):

```text
ΣFx = Σ(applied_fx) + Σ(reaction_fx) ≈ 0
ΣFy = Σ(applied_fy) + Σ(reaction_fy) ≈ 0
ΣFz = Σ(applied_fz) + Σ(reaction_fz) ≈ 0
ΣMx = Σ(applied_mx) + Σ(reaction_mx) ≈ 0
ΣMy = Σ(applied_my) + Σ(reaction_my) ≈ 0
ΣMz = Σ(applied_mz) + Σ(reaction_mz) ≈ 0
```

Moments about the global origin: `M = r × F`.

**Verification of moment formulas:**

- Fx at (x,y,z): `Mx = 0, My = z·Fx, Mz = −y·Fx` — matches
  `r × F = (x,y,z) × (Fx,0,0) = (0, z·Fx, −y·Fx)`. ✓
- Fy at (x,y,z): `Mx = −z·Fy, My = 0, Mz = x·Fy` — matches
  `r × F = (x,y,z) × (0,Fy,0) = (−z·Fy, 0, x·Fy)`. ✓
- Fz at (x,y,z): `Mx = y·Fz, My = −x·Fz, Mz = 0` — matches
  `r × F = (x,y,z) × (0,0,Fz) = (y·Fz, −x·Fz, 0)`. ✓

**Tests**: `test_3d_geometry`, `test_prescribed_displacement`,
`test_mixed_constraints`, `test_equilibrium`,
`test_tetrahedral_benchmark` — all report `is_balanced() = true`. ✓

---

## 3. Constraint Audit

### 3.1 Fixed DOF

`fix_dof(node_idx, dof, value)` — validates node index and finite value.
Each DOF constrained independently, no hidden X/Y/Z coupling. ✓

### 3.2 Nonzero prescribed displacement

**Implementation** (`truss3d.rs:589–637`):

```text
F_f' = F_f − K_fc · u_c
```

**Test A** (`test_prescribed_displacement`): Two-bar series with
`ux_0 = δ`. Analytical: `ux_1 = δ/2`, `N = −δ/2`, `Rx_0 = δ/2`.
All match to 1e-14. ✓

### 3.3 Mixed constraints

`test_mixed_constraints`: Partial fix (uy/uz only at node 1) + force in
free direction. Produces finite, positive displacement. ✓

### 3.4 Invalid constraints

`test_non_finite_input`: NaN/Inf prescribed values rejected with
`FemError::InvalidInput`. Out-of-bounds node rejected with
`FemError::InvalidNode`. ✓

### 3.5 Repeated constraints

`fix_dof` pushes to a `Vec`; `from_model` processes them sequentially,
later values overwrite earlier ones in the `prescribed_values` array.
Deterministic behaviour, not a bug. ✓

---

## 4. Mechanism Audit

### 4.1 Rigid body modes

**Implementation** (`truss3d.rs:817–851`): 6 candidates (Tx, Ty, Tz,
Rx, Ry, Rz) restricted to free DOFs.

**Verification of rotation modes** (cross product `ω × r`):

| Mode | ux | uy | uz | Correct? |
|---|---|---|---|---|
| Rx (ω = x̂) | 0 | −z | y | ✓ |
| Ry (ω = ŷ) | z | 0 | −x | ✓ |
| Rz (ω = ẑ) | −y | x | 0 | ✓ |

**Test** (`test_mechanism`): Free-free bar along (1,1,1) →
`RigidBodyMode` with `rigid_modes ≥ 5`. ✓

### 4.2 Partial mechanism

Not explicitly tested — the shared `diagnose_reduced` handles this
case (DOF-agnostic, confirmed in Phase 113). The 3D Truss delegates
to it without any 3D-specific mechanism logic. ✓

### 4.3 Collinear members

Not explicitly tested — collinear members in 3D produce the same
rank-deficient `K_ff` as in 2D. The shared diagnostic handles it. ✓

### 4.4 Stable 3D truss

**Test** (`test_stable_diagnostic`): Tripod (3 members, 4 nodes, 3
fixed) → `StructuralDiagnostic::Stable`. ✓

### 4.5 Shared diagnostic reuse

`diagnostic()` calls `diagnose_reduced(&reduced.k_ff, &candidates)` —
no 3D-specific mechanism implementation. ✓

---

## 5. Numerical Benchmark

### Tetrahedral benchmark (`test_tetrahedral_benchmark`)

**Geometry:**

```text
Node 0: (0, 0, 0) — free
Node 1: (3, 0, 0) — fixed
Node 2: (0, 4, 0) — fixed
Node 3: (0, 0, 5) — fixed
Members: 0-1 (L=3), 0-2 (L=4), 0-3 (L=5)
E = A = 1
Force at node 0: (1, 2, 3)
```

**Independent analytical solution:**

```text
K_ff = diag(EA/L₁, EA/L₂, EA/L₃) = diag(1/3, 1/4, 1/5)

ux = Fx / (1/3) = 3
uy = Fy / (1/4) = 8
uz = Fz / (1/5) = 15

N₀₁ = (1/3) · 1 · (0 − 3) = −1  (compression)
N₀₂ = (1/4) · 1 · (0 − 8) = −2  (compression)
N₀₃ = (1/5) · 1 · (0 − 15) = −3 (compression)

R₁ = (−1, 0, 0)
R₂ = (0, −2, 0)
R₃ = (0, 0, −3)
```

**Computed vs analytical:**

| Quantity | Analytical | Computed | Error |
|---|---|---|---|
| ux | 3.0 | 3.0 | < 1e-10 |
| uy | 8.0 | 8.0 | < 1e-10 |
| uz | 15.0 | 15.0 | < 1e-10 |
| N₀₁ | −1.0 | −1.0 | < 1e-10 |
| N₀₂ | −2.0 | −2.0 | < 1e-10 |
| N₀₃ | −3.0 | −3.0 | < 1e-10 |
| R₁x | −1.0 | −1.0 | < 1e-10 |
| R₂y | −2.0 | −2.0 | < 1e-10 |
| R₃z | −3.0 | −3.0 | < 1e-10 |
| R₁y | 0.0 | 0.0 | < 1e-12 |
| R₁z | 0.0 | 0.0 | < 1e-12 |
| Equilibrium | balanced | balanced | ✓ |

The analytical solution was derived independently from the
implementation (direct stiffness method for a tripod with orthogonal
members). ✓

---

## 6. Result Ownership

`TrussAnalysisResult3D` is a **true owned snapshot**:

- Stores `Vec<f64>` for displacements, reactions, axial forces — all
  owned data, no borrows. ✓
- `displacement()`, `reaction()`, `member_axial_force()` are O(1) array
  reads — no recomputation. ✓
- `equilibrium()` computes residuals from stored data — does not rerun
  the solver. ✓
- Independent of mutable solver/model state — cloning the result gives a
  valid standalone snapshot. ✓
- Stores geometry snapshot (`node_coords`, `element_nodes`,
  `nodal_forces`) — sufficient for post-processing without referencing
  the original model. ✓

**Not too much, not too little**: stores exactly what is needed for
post-processing and equilibrium verification. No solver internals
(factors, matrices) are exposed. ✓

---

## 7. API Audit

### Stable API (should remain)

| API | Assessment |
|---|---|
| `TrussDof3D` enum | Clean, simple, documented |
| `TrussNode3D` | Public fields with units, consistent with 2D |
| `TrussElement3D` | Public fields with units, `new()` validates E/A |
| `TrussModel3D` | Builder pattern, validates inputs |
| `TrussSolver3D` | `from_model` / `solve_configured` / queries |
| `TrussAnalysisResult3D` | Owned snapshot, cheap queries |
| `TrussEquilibriumReport3D` | All 6 residuals exposed, `is_balanced()` |

### Premature API

None identified. All public types are necessary for the solver API.

### Incorrect API

None identified. All semantics are unambiguous and documented.

### P2 finding: coordinate validation

`TrussNode3D::new` does not validate coordinates for finiteness. NaN/Inf
coordinates pass through silently, producing NaN results without error.
This is consistent with the 2D `TrussNode` behaviour (pre-existing
pattern). **Not fixed in this phase** — would require either a breaking
change to `new()` or adding a `try_new()` variant, which is a feature
addition beyond audit scope.

---

## 8. 2D/3D Consistency

| Concept | 2D Truss | 3D Truss | Consistent? |
|---|---|---|---|
| node coordinates | `(x, y)` | `(x, y, z)` | ✓ dimensional |
| member definition | `(node_i, node_j, E, A)` | `(node_i, node_j, E, A)` | ✓ identical |
| DOF numbering | `2·node + d` | `3·node + d` | ✓ dimensional |
| constraints | `fix_dof`, `fix_node` | `fix_dof`, `fix_node` | ✓ identical API |
| loads | `add_nodal_force(fx, fy)` | `add_nodal_force(fx, fy, fz)` | ✓ dimensional |
| prescribed displacement | `fix_dof(node, dof, value)` | `fix_dof(node, dof, value)` | ✓ identical |
| displacement result | `displacement(node, dof)` | `displacement(node, dof)` | ✓ identical |
| reaction result | `reaction(node, dof)` | `reaction(node, dof)` | ✓ identical |
| axial force | `axial_force(elem_idx)` | `axial_force(elem_idx)` | ✓ identical |
| sign convention | tension positive | tension positive | ✓ identical |
| reaction formula | `R = K·u − f` | `R = K·u − f` | ✓ identical |
| error type | `FemError` | `FemError` | ✓ shared |
| mechanism diagnosis | `diagnose_reduced` | `diagnose_reduced` | ✓ shared |
| rigid modes | 3 (Tx, Ty, Rz) | 6 (Tx, Ty, Tz, Rx, Ry, Rz) | ✓ dimensional |
| equilibrium | 3 residuals (Fx, Fy, Mz) | 6 residuals (Fx, Fy, Fz, Mx, My, Mz) | ✓ dimensional |
| coordinate validation | none | none | ✓ consistent (both P2) |

**No accidental inconsistencies found.** All differences are
dimensional (2 DOF vs 3 DOF, 3 rigid modes vs 6, 3 equilibrium
residuals vs 6).

---

## 9. Future LoadCase Compatibility

```text
YES
```

The current `add_nodal_force(node, fx, fy, fz)` API is compatible with
eventually introducing `LoadCase` without any rewrite. A `LoadCase`
would simply be a collection of `(node, fx, fy, fz)` tuples. The solver
would solve each case independently using the same factorized `K_ff`.
The `TrussModel3D` struct already stores `nodal_forces` as a `Vec`,
which could be swapped for `Vec<LoadCase>` or extended with a
`add_load_case()` method without breaking existing callers.

---

## 10. Future Envelope Compatibility

```text
YES
```

`TrussAnalysisResult3D` stores all data needed for envelope computation:
displacements, reactions, axial forces, and geometry. An envelope would
collect multiple `TrussAnalysisResult3D` snapshots and compute
max/min/abs-max per DOF/member. No redesign needed — the result type is
already a self-contained owned snapshot.

---

## 11. Code-Size Assessment

```text
truss3d.rs:  1154 lines
truss3d tests: 620 lines (17 tests)
```

**Breakdown of truss3d.rs:**

| Category | Lines | Assessment |
|---|---|---|
| Module docs | ~60 | necessary |
| DOF enum | ~50 | necessary |
| Node struct | ~25 | necessary |
| Element (stiffness, axial force, validation) | ~125 | necessary |
| Model (builder, validation) | ~100 | necessary |
| Solver (assembly, condensation, solve) | ~280 | necessary |
| Solver queries | ~100 | necessary |
| Diagnostic (rigid candidates) | ~50 | necessary |
| AnalysisResult + EquilibriumReport | ~290 | necessary |
| results() method | ~25 | necessary |

**Total: ~1105 lines of necessary code + ~50 lines of docs.**

No duplication beyond the mechanical 2D→3D extension (explicitly allowed
by Phase 113). No accidental complexity. No traits introduced solely to
reduce line count. The size is appropriate for a complete FEM solver
module with owned result snapshot and equilibrium verification.

---

## 12. Changes Made

| File | Status | Lines |
|---|---|---|
| `crates/structural-analysis/tests/truss3d.rs` | modified | +270 (6 new tests) |

**No production code changes.** The implementation (`truss3d.rs`) is
unchanged — the audit confirmed mathematical correctness, numerical
robustness, and API soundness.

### New tests added

| # | Test | Coverage gap filled |
|---|---|---|
| 12 | `test_axial_bar_along_y` | Y-axis direction cosines |
| 13 | `test_axial_bar_along_z` | Z-axis direction cosines |
| 14 | `test_node_order_invariance` | Member reversal invariance |
| 15 | `test_tetrahedral_benchmark` | Independent analytical benchmark |
| 16 | `test_stable_diagnostic` | Stable classification |
| 17 | `test_non_finite_input` | NaN/Inf rejection |

---

## 13. Validation

| Command | Result |
|---|---|
| `cargo fmt --all -- --check` | ✅ pass |
| `cargo check --workspace` | ✅ pass (3 pre-existing pardiso warnings) |
| `cargo test -p structural-analysis --release` | ✅ all suites pass |
| `cargo test --doc --workspace --release` | ✅ 13 doc tests pass |
| `cargo test --examples --workspace --release` | ✅ pass |
| `cargo clippy --workspace --all-targets --all-features` | ✅ no new warnings |

### Test counts

```text
truss3d.rs:      17 passed (was 11, +6 new)
truss.rs:        25 passed (2D regression)
Total structural-analysis: all suites pass
```

---

## 14. Final Classification

```text
P0: (none)
P1: (none)
P2: TrussNode3D::new does not validate coordinate finiteness
    (consistent with 2D TrussNode, not a regression)

Mathematical correctness:
PASS

Numerical robustness:
PASS

API:
STABLE

3D Truss:
READY

2D Truss regression:
PASS

LoadCase readiness:
READY

Envelope readiness:
READY

3D Frame:
DEFERRED

Architecture:
KEEP

Generic abstraction:
NO

Next phase:
3D Truss accepted as stable foundation for future 3D load
cases / post-processing. No implementation changes required.
P2 coordinate validation may be addressed in a future phase
alongside the 2D TrussNode for consistency.
```

---

## 15. Audit Summary

Phase 114's 3D Truss MVP is **mathematically correct, numerically
robust, and architecturally sound**. The deep audit verified:

- **Stiffness matrix**: correct 6×6 formulation, symmetric, correct rank
- **Direction cosines**: verified for X, Y, Z, and arbitrary 3D
  orientations
- **Axial force**: tension-positive convention, consistent with
  stiffness, verified against 6 analytical cases
- **Node-order invariance**: mathematically proven and tested
- **Equilibrium**: all 6 residuals (Fx, Fy, Fz, Mx, My, Mz) verified
- **Prescribed displacement**: RHS correction `F_f' = F_f − K_fc·u_c`
  confirmed
- **Mechanism diagnostics**: correctly delegates to shared
  `diagnose_reduced`, 6 rigid-body modes properly constructed
- **Tetrahedral benchmark**: full analytical match (displacements,
  forces, reactions, equilibrium)
- **API**: stable, no premature/incorrect APIs, future LoadCase/Envelope
  compatible
- **2D/3D consistency**: all differences are dimensional, no accidental
  inconsistencies

**6 new tests were added to fill real coverage gaps** (Y/Z axis members,
node-order invariance, tetrahedral benchmark, stable diagnostic,
non-finite input). No production code changes were required.
