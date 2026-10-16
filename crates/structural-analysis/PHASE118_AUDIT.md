# Phase 118 — 3D Truss LoadCase / LoadCombination Deep Audit

**Date:** 2026-10-09
**Base commit:** `71b294f` (Phase 117)
**Scope:** Deep audit of Phase 117's 3D Truss `solve_case` / `solve_combination` implementation
**Principle:** Audit first, fix only with evidence, no new features

---

## 1. Audit Dimensions

| # | Dimension | Status |
|---|-----------|--------|
| 1 | LoadCase shared-storage semantics | ✅ Correct |
| 2 | 3D DOF mapping (`global_dof = node * 3 + local_dof`) | ✅ Correct |
| 3 | LoadCombination linear algebra (`F = Σ factor_i · F_i`) | ✅ Correct |
| 4 | Prescribed displacement rules (solve_case vs solve_combination) | ✅ Correct |
| 5 | LoadSource provenance | ✅ Correct |
| 6 | Reaction / axial force equilibrium | ✅ Correct |
| 7 | 2D Frame / Beam regression | ✅ No regression |
| 8 | Error handling (NaN, ±∞, invalid DOF) | ⚠️ P2 found → fixed |
| 9 | Result ownership / snapshot isolation | ✅ Correct |
| 10 | FEM benchmark timeout classification | ✅ PRE-EXISTING CONFIRMED |
| 11 | Architecture constraint (solver layer agnosticism) | ✅ Maintained |
| 12 | LoadCase reusability across models | ✅ Correct (read-only) |
| 13 | Load addition order independence | ✅ Correct (addition commutative) |

---

## 2. Findings

### P0 — Critical (mathematical correctness): **None**

No mathematical errors found. The core equations are correct:

- `solve_case`: assembles `RHS = model_nodal_forces + case_nodal_forces`, applies prescribed
  displacements as constraints, solves `K·u = f`, computes reactions `R = K·u − f`.
- `solve_combination`: assembles `RHS = Σ factor_i · (model_forces + case_i_forces)`,
  rejects any case with prescribed displacements, solves once.

### P1 — High (semantic / API contract): **None**

- LoadCase is **read-only** during solve — `solve_case` / `solve_combination` take `&LoadCase`,
  not `&mut`. The same `LoadCase` can be safely used by multiple models or solved multiple times.
- Prescribed displacement semantics match Frame: `solve_case` applies them as constraints
  (overriding or adding to model constraints); `solve_combination` rejects them.
- `LoadSource` provenance is correct: `solve_case` → `LoadCase`, `solve_combination` →
  `LoadCombination`, `results()` → `ModelLoads`.
- 2D regression: `frame.rs` changes in Phase 117 were pure visibility (`fn` → `pub(crate) fn`);
  `load.rs` changes were additive only. No 2D behavior affected.

### P2 — Medium (robustness / edge cases): **1 found → fixed**

**`prescribed_displacement_3d` did not validate `dof < 3`.**

- **Root cause:** The method accepted `dof: usize` without bounds checking. A caller passing
  `dof >= 3` would cause an array out-of-bounds panic deep inside `TrussSolver3D::from_model`
  rather than returning a clear `FemError::InvalidInput`.
- **Contrast with 2D:** The 2D `prescribed_displacement` method takes `dof: Dof` (an enum with
  only `Ux`, `Uy` variants), so invalid DOF is impossible at the type level.
- **Fix:** Added `dof >= 3` guard at the top of `prescribed_displacement_3d`
  (`load.rs:370-374`), returning `FemError::InvalidInput` with a descriptive message.
- **Test:** Test 31 verifies the guard rejects `dof = 3` and `dof = 99`.

### P3 — Low (style / documentation): **None**

No documentation or style issues found beyond pre-existing workspace-wide lint warnings.

---

## 3. Detailed Audit Notes

### 3.1 LoadCase Shared-Storage Semantics

`LoadCase` stores loads in two `Vec`s:
- `nodal_forces: Vec<(usize, usize, f64)>` — `(node_idx, dof, value)`
- `prescribed_displacements: Vec<(usize, usize, f64)>` — `(node_idx, dof, value)`

Both are DOF-agnostic (raw `usize` for DOF index). The 3D methods `nodal_load_3d` and
`prescribed_displacement_3d` write into the same vectors as the 2D methods. The solver
interprets the DOF index based on the model dimensionality (2D: 0=ux, 1=uy; 3D: 0=ux, 1=uy, 2=uz).

**Key finding:** `nodal_load_3d` always uses `dof` values 0, 1, or 2 (hardcoded in the method),
so it is safe. `prescribed_displacement_3d` accepted arbitrary `usize` — this was the P2 gap.

### 3.2 LoadCombination Mathematics

```
F_combined = Σ_{i=1}^{n} factor_i × F_i
```

where `F_i` includes both model-level loads and `LoadCase_i` loads. The combined RHS is solved
in a single `K·u = f` call.

Verified:
- **Positive factors:** correct scaling.
- **Zero factors:** contribute nothing; result equals solving without that case.
- **Negative factors:** correct sign reversal (e.g., wind uplift).
- **Empty combination:** zero RHS → zero displacement, zero reactions.
- **Same case added twice:** factors accumulate (`factor_1 + factor_2`).
- **NaN/±∞ factors:** rejected by `add_case` with `FemError::InvalidInput`.

### 3.3 Prescribed Displacement Rules

| Method | Prescribed disp behavior |
|--------|--------------------------|
| `solve_case` | Applies prescribed displacements as constraints. Overrides model-level constraint at same DOF. |
| `solve_combination` | Rejects any case containing prescribed displacements (`FemError::InvalidInput`). |

This matches the Frame implementation exactly.

### 3.4 Reaction / Axial Force Equilibrium

- Reactions: `R = K·u − F` where `F` is the combined RHS (model + case loads).
- Axial forces: computed from combined displacements using element stiffness.
- No stale result caching: each `solve_case` / `solve_combination` call creates a fresh
  `TrussSolver3D` and returns an owned `TrussAnalysisResult3D` snapshot.

### 3.5 2D Regression

Phase 117's changes to existing code:
- `frame.rs`: `load_case_source` and `load_combination_source` changed from `fn` to `pub(crate) fn`
  (pure visibility change, no behavioral impact).
- `load.rs`: only additive (new methods `nodal_load_3d`, `prescribed_displacement_3d`).
- `truss.rs` (2D Truss): no changes. 2D Truss does not have `solve_case` / `solve_combination`.

**Conclusion:** No 2D regression possible.

### 3.6 FEM Benchmark Timeout — PRE-EXISTING CONFIRMED

`cargo test --workspace` times out due to FEM benchmark tests in `section-properties` crate.
Phase 117/118 only modified files in `structural-analysis` crate. The timeout is unrelated to
this audit's changes.

Classification: **PRE-EXISTING CONFIRMED** — not a regression introduced by Phase 117 or 118.

### 3.7 Architecture Constraint

The solver layer (`TrussSolver3D`) does not know about `LoadCase`, `LoadCombination`, or
`LoadSource`. These types live in `load.rs` and `truss3d.rs`. The solver only sees raw
`nodal_forces` and `prescribed_displacements` slices. This boundary is maintained.

---

## 4. Fixes Applied

### Fix 1 (P2): `prescribed_displacement_3d` DOF validation

**File:** `crates/structural-analysis/src/load.rs:370-374`
**Change:** Added bounds check `dof >= 3` returning `FemError::InvalidInput`.

```rust
pub fn prescribed_displacement_3d(
    &mut self,
    node_idx: usize,
    dof: usize,
    value: f64,
) -> Result<(), FemError> {
    if dof >= 3 {
        return Err(FemError::InvalidInput(format!(
            "3D truss DOF index must be 0 (ux), 1 (uy), or 2 (uz), got {dof}"
        )));
    }
    // ... existing finite check and push ...
}
```

---

## 5. Tests Added

6 new tests in `crates/structural-analysis/tests/truss3d.rs` (lines 1013–1219):

| Test | Line | Description |
|------|------|-------------|
| 28 | 1017 | Combination with zero and negative factors |
| 29 | 1048 | Prescribed displacement via LoadCase with concurrent nodal force |
| 30 | 1089 | LoadCombination rejects prescribed displacements |
| 31 | 1111 | Invalid DOF (`≥ 3`) rejected by `prescribed_displacement_3d` |
| 32 | 1136 | Repeated solve isolation — results don't pollute each other |
| 33 | 1176 | Multi-node load accumulation and cancellation |

---

## 6. Verification Results

| Command | Result |
|---------|--------|
| `cargo fmt --all -- --check` | ✅ Pass |
| `cargo check --workspace` | ✅ Pass (3 pre-existing pardiso warnings) |
| `cargo test -p structural-analysis` | ✅ Pass (40 test suites, 0 failures, truss3d: 33 tests) |
| `cargo test --doc --workspace` | ✅ Pass (16 doctests, 0 failures) |
| `cargo test --examples --workspace` | ✅ Pass (0 examples) |
| `cargo clippy --workspace --all-targets --all-features` | ✅ Pass (all warnings pre-existing in `section-properties`) |
| `cargo test --workspace` | ⏱️ Timeout — PRE-EXISTING CONFIRMED (`section-properties` FEM benchmarks) |

---

## 7. Summary

| Category | Count |
|----------|-------|
| P0 (critical) | 0 |
| P1 (high) | 0 |
| P2 (medium) | 1 → fixed |
| P3 (low) | 0 |
| New tests | 6 |
| Production code changes | 1 file (+5 lines) |
| Test code changes | 1 file (+207 lines) |

**Conclusion:** Phase 117's 3D Truss LoadCase/LoadCombination implementation is mathematically
correct and architecturally sound. One P2 robustness gap (missing DOF bounds check in
`prescribed_displacement_3d`) was found and fixed. No regressions introduced.
