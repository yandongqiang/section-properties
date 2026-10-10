# Phase 144 — Support and Boundary Condition Editing MVP

**Status:** COMPLETE  
**Date:** 2026-10-10  
**Base:** Phase 143 (commit `0997c58`)  

## Objective

Enable the user to view, create, modify, and delete nodal supports
(boundary conditions) in the egui desktop application, with correct
conversion to the `FrameModel3D` analysis core and viewport visualization.

## Constraint API Audit (Step 1)

### 3D Frame (`FrameModel3D`) — supported constraint APIs

| Method | DOFs constrained | Notes |
|--------|-----------------|-------|
| `fix_node(idx)` | All 6 (Ux, Uy, Uz, Rx, Ry, Rz) = 0 | Fully fixed |
| `pin_node(idx)` | Ux, Uy, Uz = 0; Rx, Ry, Rz free | Pinned |
| `fix_dof(idx, dof, value)` | Single DOF to prescribed value | Can be non-zero |

- Constraint storage: `Vec<(usize, usize, f64)>` = (node_idx, dof_idx, value)
- Solver: static condensation / direct elimination
- Reactions: `R = K_original · u - f_global`
- Conflict detection: same (node, dof) with different value → error

### 3D Frame — NOT supported

- Spring supports
- Inclined/skewed constraints
- Mechanism diagnosis (returns opaque `SolverError` on singular matrix)

### 2D Frame (`FrameModel`) — additional (not used in 3D GUI)

- `roller_x`, `roller_y`, `spring`, `inclined_roller`, `restrain`, `diagnostic()`

## Implementation

### Data Model (`edit_model.rs`)

**New type: `SupportSpec`**

```rust
pub struct SupportSpec {
    dofs: [Option<f64>; 6],  // None = free, Some(v) = constrained to v
}
```

Constructors:
- `SupportSpec::fixed()` — all 6 DOFs = Some(0.0)
- `SupportSpec::pinned()` — Ux/Uy/Uz = Some(0.0), Rx/Ry/Rz = None
- `SupportSpec::free()` — all None
- `SupportSpec::custom(dofs)` — arbitrary

Queries:
- `is_fixed()`, `is_pinned()`, `is_free()`, `n_constrained()`, `label()`
- `dof(Dof3D) -> Option<f64>`, `dofs() -> [Option<f64>; 6]`
- `set_dof(Dof3D, Option<f64>)`

**EditModel changes:**
- Replaced `fixed_nodes: Vec<NodeId>` and `fixed_dofs: Vec<(NodeId, Dof3D)>`
  with `supports: Vec<(NodeId, SupportSpec)>`
- `fix_node(id)` delegates to `set_support(id, SupportSpec::fixed())`
- `fix_dof(id, dof)` merges into existing support (or creates new)
- New methods: `set_support()`, `clear_support()`, `support()`, `has_support()`, `supports()`
- `delete_node()` cleans up supports
- `to_frame_model()` uses `fix_node` for fixed, `pin_node` for pinned, `fix_dof` for custom

### UI (`main.rs`)

- Node inspector: support label, preset buttons (Fixed, Pinned, Custom),
  per-DOF checkboxes, Clear button
- Model contents: support label shown next to each node
- Viewport: support symbols (ring + label: F/P/Cn) at constrained nodes
- Window title updated to "Phase 144"

### Scene (`scene.rs`)

- Added `node_supports: Vec<Option<SupportSpec>>` field
- Populated in `from_edit_model()` from `EditModel::support()`

### Exports (`lib.rs`)

- `SupportSpec` added to public exports

## Verification

### IMPLEMENTED

- [x] `SupportSpec` type with fixed/pinned/free/custom presets
- [x] `set_support`, `clear_support`, `support`, `has_support` methods
- [x] `fix_node` delegates to `set_support(fixed())`
- [x] `fix_dof` merges into existing support
- [x] `delete_node` removes supports
- [x] `to_frame_model` uses `fix_node`/`pin_node`/`fix_dof` appropriately
- [x] UI: support preset buttons, per-DOF checkboxes, clear button
- [x] UI: model contents shows support labels
- [x] Viewport: support symbols at constrained nodes
- [x] Scene: `node_supports` populated from EditModel
- [x] Analysis invalidation on support change (`mark_analysis_stale`)

### TESTED

- **Unit tests (98 passed):**
  - SupportSpec constructors and queries (7 tests)
  - Support operations: set/clear/has/supports (8 tests)
  - fix_node delegation, fix_dof merge (3 tests)
  - delete_node removes support (1 test)
  - Conversion with pinned/custom supports (2 tests)
  - All pre-existing tests unchanged

- **Integration tests (31 passed in edit_model.rs):**
  - set_support_fixed/pinned then solve (2 tests)
  - clear_support, fix_dof merge into pinned (2 tests)
  - Support preserved after coordinate update (1 test)
  - Preset inclined supports correct (1 test)
  - All presets with supports solve (1 test)
  - Scene reflects supports / no supports / after clear (3 tests)
  - All pre-existing integration tests unchanged

- **Other integration tests (43 passed):**
  - app_state (13), deformation (6), geometry (6),
    performance (1), robustness (12), selection (4)

- **structural-analysis regression (230 passed):**
  - No changes to structural-analysis crate
  - All existing tests pass

### NUMERICALLY VERIFIED

- Portal frame with fixed supports solves correctly
- Portal frame with custom support (5/6 DOFs) solves correctly
- All 5 preset models build and solve via EditModel with supports
- Inclined preset: node N4 has 3 translational DOFs constrained (pinned-like)

### MANUALLY VERIFIED

- (Not manually run — GUI application, headless environment)

### UNVERIFIED

- Pinned-base portal frame is a mechanism in 3D (columns twist freely) —
  this is correct structural behavior, not a bug. The GUI will show a
  solver error if the user creates this configuration.

## Scope Restrictions (per spec)

- No load editor (loads still set via presets only)
- No undo/redo
- No persistence (no save/load)
- No new GUI framework (still egui)
- No spring supports (3D Frame API doesn't support them)
- No inclined supports (3D Frame API doesn't support them)
- No mechanism diagnosis (3D Frame API doesn't support it)

## Files Changed

| File | Change |
|------|--------|
| `apps/viewport-poc/src/edit_model.rs` | SupportSpec type, supports field, support methods, to_frame_model update, tests |
| `apps/viewport-poc/src/lib.rs` | Export SupportSpec |
| `apps/viewport-poc/src/scene.rs` | node_supports field, from_edit_model update |
| `apps/viewport-poc/src/main.rs` | Support editing UI, viewport symbols, window title |
| `apps/viewport-poc/tests/edit_model.rs` | 10 new integration tests |
| `apps/viewport-poc/tests/robustness.rs` | Updated ViewportScene constructors |

## Test Summary

| Suite | Tests | Status |
|-------|-------|--------|
| viewport-poc unit | 98 | PASS |
| viewport-poc integration (edit_model) | 31 | PASS |
| viewport-poc integration (app_state) | 13 | PASS |
| viewport-poc integration (deformation) | 6 | PASS |
| viewport-poc integration (geometry) | 6 | PASS |
| viewport-poc integration (performance) | 1 | PASS |
| viewport-poc integration (robustness) | 12 | PASS |
| viewport-poc integration (selection) | 4 | PASS |
| structural-analysis | 230 | PASS |
| **Total** | **401** | **PASS** |

`cargo fmt --all` clean. `cargo check --workspace` clean.
