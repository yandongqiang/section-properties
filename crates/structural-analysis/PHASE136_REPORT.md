# Phase 136 — 3D Frame API Invariants and Numerical Robustness

## Summary

Hardened the 3D frame (`frame3d`) implementation against invalid-state bypasses,
non-finite inputs, and ambiguous constraint semantics. All structural types now
encapsulate their fields as private, with validated getters and setters. The
solver re-validates all model data at the boundary as defense-in-depth.

## Changes

### Step 2 — Fix invalid-state bypasses

- **`FrameSection3D`**: fields privatized (`E`, `G`, `area`, `iy`, `iz`, `j`).
  Added getters (`E()`, `G()`, `area()`, `iy()`, `iz()`, `j()`) and validated
  setters (`set_E()`, `set_G()`, `set_area()`, `set_iy()`, `set_iz()`, `set_j()`).
  Added `validate()` (pub(crate)) for solver-boundary defense.
- **`FrameElement3D`**: fields privatized (`node_i`, `node_j`, `section`,
  `ref_vec`). Added getters (`node_i()`, `node_j()`, `section()`, `ref_vec()`).
- **`FrameNode3D`**: fields privatized (`id`, `x`, `y`, `z`). Added getters
  (`id()`, `x()`, `y()`, `z()`, `coords()`).
- **`FrameModel3D`**: fields privatized (`nodes`, `members`, `nodal_forces`,
  `fixed_dofs`, `member_loads`). Added accessors (`n_nodes()`, `n_members()`,
  `node()`, `member()`).
- **`FrameAnalysisResult3D`**: kept public (output snapshot, no validation needed).
- **`from_model`**: added solver-boundary re-validation — node coordinates,
  nodal-load indices/values, fixed-DOF indices/values, section parameters,
  member-load equivalent forces.

### Step 3 — Harden numerical validation

- **`local_stiffness`**: validates coordinate finiteness **before** computing
  length (prevents NaN bypassing `L <= 0.0` check). Checks all 144 matrix
  entries for finiteness before returning.
- **`length`**: changed from `sqrt(dx² + dy² + dz²)` to `dx.hypot(dy).hypot(dz)`
  for overflow resistance.
- **`local_axes`**: cross-product and reference-vector norms changed to `hypot`.

### Step 4 — Define constraint semantics

- **`fix_dof`**: detects duplicate constraints on the same DOF.
  - Same value → no-op (idempotent).
  - Different value → `Err(FemError::InvalidInput)`.
  - Non-finite value → `Err(FemError::InvalidInput)`.
  - Out-of-bounds node → `Err(FemError::InvalidNode)`.

### Step 5 — Documentation

- Updated module-level docs with new "API invariants and validation" section
  covering encapsulation, numerical robustness, constraint semantics, and
  solver-boundary re-validation.
- Updated type-level docs for `FrameSection3D`, `FrameElement3D`, `FrameNode3D`,
  and `FrameModel3D` to mention private fields and getter/setter access.

### Step 6 — Tests

Added 35 new tests in `phase136` module:

| Category | Tests | Count |
|----------|-------|-------|
| `local_stiffness` NaN/inf rejection | `rejects_nan_pi`, `rejects_nan_pj`, `rejects_inf_pi`, `rejects_inf_pj`, `rejects_neg_inf` | 5 |
| `local_stiffness` zero/near-zero length | `rejects_zero_length`, `rejects_near_zero_length_overflow` | 2 |
| `local_stiffness` finite entries | `extreme_coordinates_finite`, `all_entries_finite` | 2 |
| `local_axes` NaN/inf rejection | `rejects_nan`, `rejects_inf` | 2 |
| Section setters reject invalid | `rejects_nan`, `rejects_zero`, `rejects_negative`, `rejects_inf`, `rejects_neg_inf` | 5 |
| Section setter updates | `updates_correctly` | 1 |
| Section getters | `match_constructor` | 1 |
| Element getters | `match_constructor` | 1 |
| `add_node` NaN/inf | `rejects_nan`, `rejects_inf` | 2 |
| `add_nodal_load` invalid | `rejects_invalid_node`, `rejects_nan_force` | 2 |
| Member load invalid | `rejects_invalid_member`, `rejects_nan_intensity` | 2 |
| `fix_dof` semantics | `idempotent_same_value`, `conflicting_values`, `rejects_nan_value`, `rejects_inf_value`, `rejects_invalid_node`, `different_dofs_same_node_ok` | 6 |
| Solver | `rejects_empty_model`, `produces_finite_solution` | 2 |
| Model accessors | `model_accessors` | 1 |
| `length` hypot | `hypot_overflow_resistance` | 1 |

### Step 7 — Validation

| Check | Result |
|-------|--------|
| `cargo fmt --all` | ✅ pass |
| `cargo check --workspace` | ✅ pass |
| `cargo test --package structural-analysis --release` | ✅ all pass (0 failed) |
| frame3d tests | ✅ 106 passed (71 existing + 35 new) |
| `cargo doc --package structural-analysis --no-deps` | ✅ 3 pre-existing warnings (none from Phase 136) |
| `cargo clippy` on `frame3d.rs` | ✅ 0 warnings |

## Files modified

- `crates/structural-analysis/src/frame3d.rs` — encapsulation, validation, docs
- `crates/structural-analysis/tests/frame3d.rs` — 35 new Phase 136 tests + 13 field-access updates

## Test count

- Before: 71 frame3d tests
- After: 106 frame3d tests (+35)
- Total structural-analysis tests: all pass, 0 failures
