# Phase 156 — 3D Frame Support & Boundary Condition Editing

**Status:** COMPLETE  
**Date:** 2026-10-13  
**Baseline:** `87ca719` (Phase 154)  
**Branch:** main  

---

## 1. Objective

Enable users to create, modify, and delete 3D frame node constraints
(boundary conditions) through the GUI, with full persistence, undo/redo,
and numerical validation.

---

## 2. Key Finding: Existing Infrastructure

Phase 156 spec anticipated building new support editing infrastructure.
Investigation revealed the codebase **already has complete support editing**
from prior phases:

| Feature | Location | Status |
|---------|----------|--------|
| `SupportSpec` (6-DOF) | `edit_model.rs:63-134` | ✅ Existing |
| `set_support` / `clear_support` / `support` / `has_support` | `edit_model.rs:827-873` | ✅ Existing |
| Presets: `fixed()` / `pinned()` / `free()` / `custom()` | `edit_model.rs:75-97` | ✅ Existing |
| `delete_node` cleanup | `edit_model.rs:615` | ✅ Existing |
| `to_frame_model` constraint conversion | `edit_model.rs:1230-1248` | ✅ Existing |
| Fingerprint includes supports | `edit_model.rs:508-523` | ✅ Existing |
| GUI Inspector panel | `main.rs:1672-1756` | ✅ Existing |
| Viewport support symbols | `main.rs:958-987` | ✅ Existing |
| Persistence V1/V2/V3 | `persistence.rs` | ✅ Existing |
| `validate_dto` (ref_vec + supports) | `persistence.rs:710-776` | ✅ Existing |

**No new production code was needed.** Phase 156 is a verification and
test-coverage phase.

---

## 3. P2-1 Investigation: Phase 155 False Positive

Phase 155 audit reported that `validate_dto` does not validate `ref_vec`
during deserialization. **This was a false positive.**

`validate_dto` (`persistence.rs:710-752`) already validates:
1. All three ref_vec components are finite (rejects NaN, Infinity)
2. ref_vec norm is non-zero (rejects zero vector, threshold < 1e-12)
3. ref_vec is not parallel to member axis (rejects sin_angle < 1e-8)

All three deserialization paths (V1/V2/V3) call `validate_dto`.

**Resolution:** No code change needed. Regression tests added to confirm
all invalid ref_vec cases are rejected.

---

## 4. Test Suite

**File:** `apps/viewport-poc/tests/phase156_support_editing.rs`  
**Tests:** 48 (all passing)  
**Runtime:** < 0.01s (release)

### 4.1 P2-1 Regression (7 tests)

| Test | Description |
|------|-------------|
| `p2_1_valid_ref_vec_loads_ok` | Valid ref_vec deserializes successfully |
| `p2_1_zero_ref_vec_rejected` | Zero vector rejected with `Validation` |
| `p2_1_nan_ref_vec_rejected` | NaN component rejected with `JsonError` |
| `p2_1_infinity_ref_vec_rejected` | Infinity component rejected with `JsonError` |
| `p2_1_parallel_ref_vec_rejected` | Parallel to axis rejected with `Validation` |
| `p2_1_nearly_parallel_ref_vec_rejected` | Near-parallel (1e-10) rejected |
| `p2_1_v1_ref_vec_validation` | V1 format also validates ref_vec |

### 4.2 Support CRUD (10 tests)

| Test | Description |
|------|-------------|
| `set_support_fixed` | Fixed support sets all 6 DOFs |
| `set_support_pinned` | Pinned support sets 3 translations |
| `set_support_custom` | Custom support sets specified DOFs only |
| `set_support_nonexistent_node_fails` | Nonexistent node returns error |
| `set_support_replaces_existing` | Re-setting replaces previous spec |
| `clear_support` | Clear removes support |
| `clear_support_without_existing_is_ok` | Clear on node without support is OK |
| `clear_support_nonexistent_node_fails` | Clear on nonexistent node fails |
| `supports_list` | `supports()` returns all support entries |
| `delete_node_removes_support` | Node deletion cleans up support |

### 4.3 Fingerprint (3 tests)

| Test | Description |
|------|-------------|
| `fingerprint_changes_with_support` | Adding support changes fingerprint |
| `fingerprint_differs_for_different_supports` | Different supports → different FP |
| `fingerprint_same_for_same_support` | Same supports → same fingerprint |

### 4.4 Case A — Cantilever (4 tests)

Fixed support at A, free at B. Point load at B tip.

| Test | Analytical Formula | Tolerance |
|------|-------------------|-----------|
| `case_a_cantilever_tip_deflection` | δ = PL³/(3EI) | 1e-4 rel |
| `case_a_cantilever_reaction` | R = -P | 1e-6 abs |
| `case_a_cantilever_fixed_end_moment` | M = PL | 1e-4 rel |
| `case_a_cantilever_all_reactions_nonzero` | Fx=Fy=Mx=Mz=0, Fz=-P, My=PL | 1e-6 abs |

### 4.5 Case B — Pinned Support (4 tests)

Simply supported beam (pinned at both ends, torsion fixed at A).
Midspan point load.

| Test | Description |
|------|-------------|
| `case_b_pinned_reactions` | Each reaction = P/2 |
| `case_b_pinned_rotation_free` | Rotations at supports are antisymmetric |
| `case_b_pinned_no_moment_reaction` | Moment reaction at pinned end = 0 |
| `case_b_pinned_support_rotation` | Rotation magnitude = PL²/(16EI) |

### 4.6 Case C — Custom Support (2 tests)

| Test | Description |
|------|-------------|
| `case_c_custom_support_dof_mapping` | Constrained DOFs = 0, free Ry ≠ 0 |
| `case_c_custom_only_ux_constrained` | Only Ux constrained → Uz free ≠ 0 |

### 4.7 Case D — Unstable Model (3 tests)

| Test | Description |
|------|-------------|
| `case_d_no_supports_singular` | No supports → solve fails |
| `case_d_insufficient_support_singular` | Insufficient supports → solve fails |
| `case_d_free_floating_member_singular` | Free floating → solve fails |

### 4.8 Case E — Undo/Redo (2 tests)

| Test | Description |
|------|-------------|
| `case_e_undo_redo_support` | Undo/redo restores support state |
| `case_e_stale_after_support_change` | Support change invalidates results |

### 4.9 Persistence Round-Trip (7 tests)

| Test | Description |
|------|-------------|
| `persistence_round_trip_fixed_support` | Fixed support survives save/load |
| `persistence_round_trip_custom_support` | Custom support survives save/load |
| `persistence_round_trip_no_support` | No-support state survives save/load |
| `persistence_v1_supports_readable` | V1 format supports readable |
| `persistence_rejects_support_nonexistent_node` | Rejects support on bad node |
| `persistence_rejects_duplicate_support` | Rejects duplicate support entries |
| `persistence_filesystem_round_trip` | Filesystem save/load round-trip |

### 4.10 Scene (2 tests)

| Test | Description |
|------|-------------|
| `scene_includes_supports` | ViewportScene includes support data |
| `scene_no_supports` | Empty model → no supports in scene |

### 4.11 Phase 153/154 Regression (4 tests)

| Test | Description |
|------|-------------|
| `regression_uniform_load_with_supports` | Uniform loads + supports solve |
| `regression_point_load_with_supports` | Point loads + supports solve |
| `regression_combined_loads_with_supports` | Both load types + supports solve |
| `regression_persistence_v3_with_loads_and_supports` | V3 with loads+supports |

---

## 5. Verification

### 5.1 Format

```
cargo fmt --all -- --check  →  PASS
```

### 5.2 Compilation

```
cargo check -p viewport-poc --tests  →  PASS (no new warnings)
```

### 5.3 Tests

```
cargo test -p viewport-poc --release  →  558 passed, 0 failed
```

Breakdown:
- 194 unit tests (lib)
- 15 app_state integration
- 4 audit149
- 6 deformation
- 72 edit_model integration
- 6 geometry
- 1 perf149
- 1 performance
- 31 persistence_integration
- 14 phase151_regression
- 47 phase152_undo_redo
- 35 phase153_member_loads
- 51 phase154_point_loads
- **48 phase156_support_editing** (NEW)
- 17 robustness
- 12 result_viz
- 4 scene_render
- 0 main binary

### 5.4 Clippy

```
cargo clippy -p viewport-poc --tests  →  No warnings from Phase 156 code
```

(Pre-existing warnings in other files unchanged.)

### 5.5 GUI Verification

**BLOCKED** — No GUI test harness available. Manual verification required:
- Support editing panel exists at `main.rs:1672-1756`
- Fixed/Pinned/Custom preset buttons + 6 DOF checkboxes + Clear button
- Viewport support symbols at `main.rs:958-987` (circle + F/P/C# label)

---

## 6. Files Changed

| File | Action | Description |
|------|--------|-------------|
| `apps/viewport-poc/tests/phase156_support_editing.rs` | **NEW** | 48 tests |
| `apps/viewport-poc/src/main.rs` | **MODIFIED** | Support symbols: Fixed=square, Pinned=triangle, Custom=circle |

---

## 7. Deviations from Spec

| Spec Item | Deviation | Reason |
|-----------|-----------|--------|
| P2-1 fix | No code change | False positive — `validate_dto` already validates |
| `NodeSupport3D` type | Not created | `SupportSpec` already covers this |
| EditModel API | Not added | Already exists |
| Support presets | Not added | Already exists |
| GUI Inspector | Not added | Already exists |
| Viewport symbols | Improved (P3) | Fixed=square, Pinned=triangle, Custom=circle |
| Persistence V4 | Not created | Supports already in V1/V2/V3 |

---

## 8. Local Coordinate System Reference

For a beam along global x with ref_vec [0,0,1]:

```
local x = global x   →  DOF 0, 6
local y = global z   →  DOF 2, 8
local z = -global y  →  DOF 1, 7 (negated)
```

- Bending in local y (load in global z) uses Iz
- Bending in local z (load in global y) uses Iy
- `MemberPointLoad3D` force components are in **local** coordinates
- Support constraints (`SupportSpec`) are in **global** coordinates

---

## 9. Conclusion

Phase 156 is complete. The existing codebase already had full support
editing infrastructure. Phase 156's contribution is comprehensive test
coverage (48 tests) verifying:

1. P2-1 ref_vec validation works correctly (Phase 155 was a false positive)
2. Support CRUD operations are correct
3. Numerical solutions match analytical formulas for cantilever and
   simply-supported beams
4. Custom supports correctly map DOFs
5. Unstable structures are properly diagnosed
6. Undo/redo works with support changes
7. Persistence round-trips preserve supports
8. Phase 153/154 load functionality is unaffected by supports
