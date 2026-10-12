# Phase 147 — Analysis Workflow & Diagnostics

## Objective

Implement a reliable analysis workflow in the viewport-poc egui desktop
application: explicit analysis status tracking, model revision counting,
solve transaction semantics, actionable diagnostics, and result display
Policy A (hide stale/failed results).

## Baseline

- **HEAD**: `57cae0f` (Phase 146 — Member Property Editing)
- **Test count before**: 235
- **Test count after**: 259 (+24)

## Changes

### 1. Model Revision Tracking (`edit_model.rs`)

Added a `revision: u64` counter to `EditModel` that increments on every
**effective** mutation and stays unchanged on no-ops.

| Method | Bumps on | No-op detection |
|---|---|---|
| `add_node` | Always | — |
| `update_node` | Coords changed | Compare (x,y,z) |
| `delete_node` | Always | — |
| `add_member` | Always | — |
| `delete_member` | Always | — |
| `set_member_ref_vec` | Vec changed | Compare [f64;3] |
| `set_member_section` | Params changed | Compare E/G/A/Iy/Iz/J |
| `fix_dof` | DOF not already 0.0 | `spec.dof(dof) == Some(0.0)` |
| `set_support` | Spec changed | `PartialEq` on `SupportSpec` |
| `clear_support` | Support existed | Check presence |
| `set_nodal_load` | Spec changed | `PartialEq` on `NodalLoadSpec` |
| `clear_nodal_load` | Load existed | Check presence |

API: `pub fn revision(&self) -> u64`, `fn bump_revision(&mut self)`.

### 2. AnalysisStatus Enum (`app_state.rs`)

```rust
pub enum AnalysisStatus { NeverSolved, Current, Stale, Failed }
```

**Priority**: `Failed` > `NeverSolved` > `Current`/`Stale` (by revision
comparison).

`Project::analysis_status()` determines the status by:
1. `last_solve_failed == true` → `Failed`
2. `analysis.is_none()` → `NeverSolved`
3. `solved_revision == model.revision()` → `Current`
4. Otherwise → `Stale`

### 3. Solve Transaction (`app_state.rs`)

`Project::solve()` is the single authoritative entry point:
- **Success**: stores `AnalysisResult { result, solved_revision }` and
  clears `last_solve_failed`.
- **Failure**: sets `last_solve_failed = true` but leaves any prior result
  intact (not displayed per Policy A).
- **Atomic**: model is never partially modified.

### 4. Result Display Policy A

`Project::analysis_result()` returns `Some` **only** when
`analysis_status() == Current`. Stale, failed, and absent results return
`None`, so the UI never displays outdated diagnostics.

### 5. Diagnostics (`edit_model.rs`, `app_state.rs`)

Changed `FemError` formatting from `Debug` (`{e:?}`) to `Display`
(`{e}`) in both `to_frame_model()` and `run_analysis()`. This preserves
the actionable thiserror messages (e.g., "Solver error: reduced stiffness
matrix is singular") instead of the raw debug representation.

### 6. GUI Integration (`main.rs`)

- **Status bar**: shows all 4 `AnalysisStatus` variants with
  color-coded labels (green ✓ / orange STALE / red FAILED / gray "not run").
- **Warning banner**: shows "Results are stale" (orange) or "Last solve
  failed" (red) in the toolbar.
- **Removed**: all 10 manual `mark_analysis_stale()` calls — revision
  tracking handles staleness automatically.
- **`analysis_result()`**: delegates to `Project::analysis_result()`
  (Policy A).

### 7. `load_model()` Semantics

`load_model()` now discards prior analysis results (sets `analysis =
None`, `last_solve_failed = false`). The new model has its own revision
counter starting at 0, so any prior result is meaningless. This avoids
false `Current` status from revision counter collision.

## Tests

### 20 Revision Tracking Regression Tests (`tests/edit_model.rs`)

1. `revision_starts_at_zero`
2. `add_node_bumps_revision`
3. `update_node_bumps_revision`
4. `update_node_no_op_preserves_revision`
5. `delete_node_bumps_revision`
6. `add_member_bumps_revision`
7. `delete_member_bumps_revision`
8. `set_member_ref_vec_bumps_revision`
9. `set_member_ref_vec_no_op_preserves_revision`
10. `set_member_section_bumps_revision`
11. `set_member_section_no_op_preserves_revision`
12. `fix_dof_bumps_revision`
13. `fix_dof_no_op_preserves_revision`
14. `set_support_bumps_revision`
15. `set_support_no_op_preserves_revision`
16. `clear_support_bumps_revision`
17. `clear_support_no_op_preserves_revision`
18. `set_nodal_load_bumps_revision`
19. `set_nodal_load_no_op_preserves_revision`
20. `clear_nodal_load_bumps_and_no_op`

### 2 Integration Scenarios (`tests/app_state.rs`)

1. **`workflow_preset_solve_edit_stale_resolve_current`**:
   Preset → Solve → Current → Edit → Stale → Re-solve → Current

2. **`workflow_valid_solve_mechanism_failed_repair_solve_current`**:
   Valid beam → Solve → Current → Clear supports → Stale → Solve →
   Failed → Re-add supports → Failed → Solve → Current

### Updated Existing Tests

- `app_state.rs` unit tests: updated for new `AnalysisStatus` API,
  `load_model` discard semantics, `solve_failed` flag.
- `app_state.rs` integration tests: replaced `mark_analysis_stale` tests
  with revision-based staleness tests.
- `edit_model.rs` integration tests: removed `mark_analysis_stale` calls
  from `section_edit_marks_stale` and `ref_vec_edit_marks_stale`.

## Validation

| Check | Result |
|---|---|
| `cargo fmt --check -p viewport-poc` | Clean |
| `cargo check --workspace` | Clean |
| `cargo test -p viewport-poc` | 259 passed, 0 failed |
| `cargo clippy -p viewport-poc` | No new warnings (pre-existing only) |

## Scope Compliance

- No new element types, distributed loads, load combinations, database,
  undo/redo, file persistence, async, or Tauri changes.
- No changes to `section-properties` or `structural-analysis` crates.
- No changes to `camera.rs`, `display.rs`, `project.rs`, `selection.rs`,
  `scene.rs`, `model.rs`, or `robustness.rs`.

## Files Changed

| File | Lines |
|---|---|
| `apps/viewport-poc/src/app_state.rs` | +196 −... |
| `apps/viewport-poc/src/edit_model.rs` | +71 ... |
| `apps/viewport-poc/src/main.rs` | +60 −... |
| `apps/viewport-poc/tests/app_state.rs` | +136 −... |
| `apps/viewport-poc/tests/edit_model.rs` | +238 ... |
| **Total** | +612 −89 |
