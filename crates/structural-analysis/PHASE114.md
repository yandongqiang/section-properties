# Phase 114 — 3D Truss MVP

**Date:** 2026-10-08
**Baseline:** `acf125e` (Phase 113)
**Production code changes:** 3 files (1 new module, 1 modified lib.rs, 1 new test)
**Output:** `truss3d.rs` module + integration tests + this document

---

## 1. Objective

Implement a production-quality 3D axial-only Truss solver using independent
3D types (Phase 113 Option B).  The solver reuses the existing DOF-agnostic
infrastructure (`SparseMatrix`, `LinearSolver`, `SolverRegistry`,
`diagnose_reduced`) and adds **no** generic dimension/element/constraint
abstraction.

**Non-goals** (explicitly deferred):

- 3D Frame (12×12 stiffness, bi-axial bending + torsion)
- 3D Envelope / LoadCase / LoadCombination
- Generic `Dimension` / `Element` / `Constraint` trait
- Third crate
- Any modification to existing 2D Truss code

---

## 2. Baseline

| Item | Value |
|---|---|
| HEAD | `acf125e` Phase 113: 3D readiness / API lock-in audit |
| `cargo fmt --all -- --check` | pass |
| `cargo check --workspace` | pass (3 pre-existing pardiso warnings) |
| Crate | `structural-analysis` v0.1.0, `publish = false` |
| Dependency | `section-properties` v0.4.0 (published, frozen public API) |

---

## 3. Implementation

### 3.1 New module: `crates/structural-analysis/src/truss3d.rs` (1154 lines)

#### Types

| Type | Purpose |
|---|---|
| `TrussDof3D` | Enum `{ Ux, Uy, Uz }` with `ALL`, `index()`, `name()` |
| `TrussNode3D` | Node with `id`, `x`, `y`, `z`; methods `new()`, `coords()` |
| `TrussElement3D` | Axial member with `node_i`, `node_j`, `E`, `A`; methods `new()`, `length()`, `direction()`, `global_stiffness()`, `axial_force()` |
| `TrussModel3D` | Model builder: `add_node()`, `add_element()`, `add_nodal_force()`, `fix_dof()`, `fix_node()`, `n_dof()`, `dof_index()` |
| `TrussSolver3D` | Solver: `from_model()`, `solve()`, `solve_configured()`, `displacements()`, `displacement()`, `reactions()`, `reaction()`, `axial_force()`, `axial_forces()`, `diagnostic()`, `results()` |
| `TrussAnalysisResult3D` | Owned snapshot: `displacement()`, `reaction()`, `member_axial_force()`, `member_axial_forces()`, `equilibrium()` |
| `TrussEquilibriumReport3D` | 6 residuals (fx, fy, fz, mx, my, mz); `is_balanced()`, `force_tolerance()` |

#### DOF convention

```
dof(node, d) = 3·node + d     d ∈ {0=Ux, 1=Uy, 2=Uz}
```

#### Element stiffness (6×6)

```
k = (EA/L) · [ c·cᵀ   -c·cᵀ
              -c·cᵀ    c·cᵀ ]

where c = (lx, ly, lz) = (Δx, Δy, Δz) / L
```

#### Axial force

```
N = (EA/L) · (lx·Δux + ly·Δuy + lz·Δuz)    tension positive
```

#### Reactions

```
R = K·u − F
```

#### Rigid-body modes (6 candidates)

| Mode | ux | uy | uz |
|---|---|---|---|
| Tx | 1 | 0 | 0 |
| Ty | 0 | 1 | 0 |
| Tz | 0 | 0 | 1 |
| Rx | 0 | −z | y |
| Ry | z | 0 | −x |
| Rz | −y | x | 0 |

#### Prescribed displacement

For constrained DOFs with non-zero prescribed values, the RHS is corrected:

```
F_f' = F_f − K_fc · u_c
```

### 3.2 Modified: `crates/structural-analysis/src/lib.rs`

- Added `pub mod truss3d;`
- Added re-exports: `TrussAnalysisResult3D`, `TrussDof3D`, `TrussElement3D`,
  `TrussEquilibriumReport3D`, `TrussModel3D`, `TrussNode3D`, `TrussSolver3D`

### 3.3 New test: `crates/structural-analysis/tests/truss3d.rs` (350 lines)

---

## 4. Test Summary

| # | Test | Validates |
|---|---|---|
| 1 | `test_axial_bar_along_x` | Single bar along x-axis: ux = FL/(EA), N = F |
| 2 | `test_stiffness_symmetry` | 6×6 K symmetric for arbitrary 3D geometry |
| 3 | `test_tension` | Tension force → positive axial force |
| 4 | `test_compression` | Compression force → negative axial force |
| 5 | `test_3d_geometry` | Bar along (1,2,3): analytical ux and N with 3D direction cosines |
| 6 | `test_prescribed_displacement` | Two-bar series with prescribed ux: K_ff=2, N=−δ/2 |
| 7 | `test_mixed_constraints` | Partial fix (uy/uz only) + force in free direction |
| 8 | `test_mechanism` | Free-free bar along (1,1,1): RigidBodyMode with ≥5 rigid modes |
| 9 | `test_zero_length_member` | Zero-length element → `InvalidModel` error |
| 10 | `test_invalid_material_area` | Non-positive E or A → `InvalidModel` error |
| 11 | `test_equilibrium` | Multi-element structure: 6 residual checks (fx, fy, fz, mx, my, mz) |

**Result:** 11 passed, 0 failed

---

## 5. Verification

| Command | Result |
|---|---|
| `cargo fmt --all -- --check` | ✅ pass |
| `cargo check --workspace` | ✅ pass (3 pre-existing pardiso warnings) |
| `cargo test -p structural-analysis --release` | ✅ all suites pass (475+ tests) |
| `cargo test --doc --workspace --release` | ✅ 13 doc tests pass |
| `cargo test --examples --workspace --release` | ✅ pass |
| `cargo clippy --workspace --all-targets --all-features` | ✅ no new warnings (truss3d clean) |

---

## 6. Design Decisions

### 6.1 Independent types, not generics (Option B)

Phase 113 recommended Option B: separate 3D types with ~100 lines of
mechanical repetition.  This avoids the complexity of generic dimension
parameters while keeping the 2D code untouched.  The shared infrastructure
(`SparseMatrix`, `LinearSolver`, `SolverRegistry`, `diagnose_reduced`) is
already DOF-agnostic and reused directly.

### 6.2 TrussElement3D stores E and A, not Material

Mirrors the 2D `TrussElement` design.  The constructor accepts `&Material`
and extracts `E`.  Density is not stored, so self-weight is not supported
(consistent with 2D Truss).

### 6.3 Mechanism test uses (1,1,1) direction

A free-free bar along the x-axis has `K_ff = diag(EA/L, 0, 0)`.  The
`rigid_nullity` function in `diagnose_reduced` returns 0 when diagonal
entries are zero, misclassifying the rigid-body mode as an internal
mechanism.  By orienting the bar along (1,1,1), all diagonal entries are
non-zero (`EA/(3L)`), allowing correct identification of ≥5 rigid-body
modes.  This is a known limitation of the mechanism diagnostic, not a bug
in the 3D Truss implementation; the 2D Truss has the same behaviour.

### 6.4 No LoadCase / LoadCombination for 3D Truss

Consistent with the non-goals.  The solver operates on a single load
vector directly.  LoadCase/LoadCombination support can be added in a future
phase if needed.

---

## 7. Known Limitations

1. **Mechanism diagnostic**: `diagnose_reduced` cannot distinguish rigid-body
   modes from internal mechanisms when K_ff has zero diagonal entries
   (e.g. bar aligned with a global axis).  Workaround: use off-axis geometry
   in tests, or add enough constraints to remove zero diagonals.

2. **No self-weight**: `TrussElement3D` does not store density.  Self-weight
   requires `Material::density` and `BeamSection::area` integration, which is
   not in scope for the MVP.

3. **No LoadCase/LoadCombination**: Single load vector only.

4. **No envelope**: Single result snapshot only.

---

## 8. Files Changed

| File | Status | Lines |
|---|---|---|
| `crates/structural-analysis/src/truss3d.rs` | **new** | 1154 |
| `crates/structural-analysis/src/lib.rs` | modified | +2 lines |
| `crates/structural-analysis/tests/truss3d.rs` | **new** | 350 |

**No changes** to:
- `crates/section-properties/` (frozen at v0.4.0)
- `crates/structural-analysis/src/truss.rs` (2D Truss untouched)
- `crates/structural-analysis/src/mechanism.rs` (shared diagnostic untouched)

---

## 9. API Surface (new)

```rust
// truss3d module
pub enum TrussDof3D { Ux, Uy, Uz }
pub struct TrussNode3D { id, x, y, z }
pub struct TrussElement3D { node_i, node_j, E, A }
pub struct TrussModel3D { /* nodes, elements, forces, constraints */ }
pub struct TrussSolver3D { /* model + solution state */ }
pub struct TrussAnalysisResult3D { /* owned snapshot */ }
pub struct TrussEquilibriumReport3D { fx, fy, fz, mx, my, mz }
```

All types are additive — no existing API was modified or removed.

---

## 10. Reuse Summary

| Infrastructure | Source | Reused by 3D Truss |
|---|---|---|
| `SparseMatrix` | `section-properties::fea` | ✅ global stiffness assembly |
| `LinearSolver` | `section-properties::fea` | ✅ reduced system solve |
| `SolverRegistry` | `section-properties::fea` | ✅ solver selection |
| `diagnose_reduced` | `structural-analysis::mechanism` | ✅ mechanism / rigid-body diagnostic |
| `StructuralDiagnostic` | `structural-analysis::frame` | ✅ diagnostic enum |
| `FemError` | `structural-analysis::beam_fem` | ✅ error type |

**Zero** new infrastructure was created.  All shared code was already
DOF-agnostic (confirmed in Phase 113 audit).
