# Phase 148 — Structural Result Visualization

**Status:** Complete  
**Baseline:** `396621f` (Phase 147)  
**Scope:** `apps/viewport-poc` only

## Objective

Implement a coherent result-visualization workflow for the 3D frame GUI.
Users can display undeformed/deformed geometry, adjust amplification, inspect
nodal displacements, reactions, and member end forces, identify the current
mode, and return to the undeformed view — all gated by result validity
(Policy A).

## Design

### ResultViewMode enum

Replaced the redundant `show_deformed` boolean with a single authoritative
enum:

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ResultViewMode {
    #[default]
    Model,              // undeformed geometry only
    Deformed,           // amplified deformed shape
    Displacement,       // color-coded nodal |u|
    Reaction,           // force/moment arrows at supports
    MemberEndForces,    // end-force arrows for selected member
}
```

`requires_result()` returns `false` only for `Model`; all other modes
require a current analysis result.

### ResultDisplayState refactor

| Before | After |
|--------|-------|
| `show_deformed: bool` | `mode: ResultViewMode` |
| `show_undeformed` | kept (overlay) |
| `show_local_axes` | kept (overlay) |
| `show_grid` | kept (overlay) |
| `show_global_axes` | kept (overlay) |
| `show_node_labels` | kept (overlay) |
| `deformation_scale` | kept |

Overlays are independent of mode.  In `Deformed` mode, `show_undeformed`
toggles the reference geometry; in other result modes the base geometry is
always shown.

### Visualization helpers (display.rs)

| Function | Purpose |
|----------|---------|
| `displacement_magnitude(ux, uy, uz)` | \|u\| = √(ux²+uy²+uz²), translational only |
| `displacement_magnitudes(disp, n)` | per-node magnitudes from flat DOF vector |
| `max_abs(values)` | max \|v\|, defensive against non-finite |
| `normalize(value, max)` | maps to [0,1], avoids div-by-zero |
| `magnitude_color(t)` | blue→green→red color ramp for [0,1] |
| `fmt_eng(v)` | engineering notation with adaptive precision |

All helpers defend against NaN/Inf by returning safe defaults (0.0 or "NaN").

### Rendering (main.rs)

| Mode | Rendering |
|------|-----------|
| Model | undeformed members, nodes, supports, loads |
| Deformed | amplified deformed shape + optional undeformed reference |
| Displacement | color-coded nodes by \|u\|, legend with max value |
| Reaction | force arrows (SUPPORT_C) + moment indicators (LOAD_M_C + "R") at supported nodes |
| MemberEndForces | force arrows at both ends of selected member (SELECTED_C) |

`show_base` variable controls base geometry: `mode != Deformed || show_undeformed`.

### GUI layout

- Mode selector: 5 `SelectableLabel` buttons
- "no current result" warning when mode requires result but none available
- Deformation scale slider + reset (Deformed mode only)
- Undeformed reference checkbox (Deformed mode only)
- Local axes / grid / global axes / node labels checkboxes (always)

### Result validity (Policy A)

All result access goes through `self.analysis_result()` →
`Project::analysis_result()`, which returns `None` for stale or failed
analysis.  No direct `project.analysis` field access in the rendering path.

### Stable ID mapping

`ViewportScene::node_index()` / `member_index()` preserve the
`EditModel NodeId/MemberId ↔ solver index ↔ result array` mapping.
Deletion and re-insertion produce new stable IDs; non-contiguous ID
spaces are handled correctly.

## Tests

### Unit tests (display.rs): 28

Covers `ResultViewMode` variants, `requires_result()`, `label()`,
`ResultDisplayState::default()`, `reset()`, `displacement_magnitude`,
`displacement_magnitudes`, `max_abs`, `normalize`, `magnitude_color`,
`fmt_eng` including non-finite inputs and edge cases.

### Integration tests (result_viz.rs): 17

| # | Test | Verifies |
|---|------|----------|
| 1 | `default_mode_is_model` | default state is Model mode |
| 2 | `mode_switching` | all 5 modes, requires_result transitions |
| 3 | `deformation_amplification_scales_linearly` | scale 0/1/2 produces correct positions |
| 4 | `reaction_sign_preserved` | cantilever reaction = -F at fixed end |
| 5 | `member_end_force_ordering` | 12 values, axial equilibrium N_i + N_j = 0 |
| 6 | `local_global_force_distinction` | local ≠ global for inclined member |
| 7 | `stable_id_mapping_after_delete_and_reinsert` | deleted ID gone, reinsert gets new ID |
| 8 | `stable_id_mapping_non_contiguous` | non-contiguous IDs map correctly |
| 9 | `stale_result_hidden_by_policy_a` | edit → stale → analysis_result() = None |
| 10 | `failed_result_hidden_by_policy_a` | mechanism → failed → analysis_result() = None |
| 11 | `re_solve_refreshes_result` | re-solve after edit produces new results |
| 12 | `all_presets_produce_accessible_results` | all presets solve and yield displacements |
| 13 | `uniform_value_normalization` | uniform values normalize to 1.0 |
| 14 | `non_finite_inputs_handled` | NaN/Inf defended everywhere |
| 15 | `analytical_axial_cantilever` | u = FL/EA, R = -F (0.01% tolerance) |
| 16 | `analytical_transverse_cantilever` | δ = FL³/3EI, R = -F, M = -FL (0.01% tolerance) |
| 17 | `solved_displacement_magnitude_consistent` | magnitude helper matches manual calculation |

### Analytical verification

**Axial cantilever** (E=200 GPa, A=1e-3 m², L=1 m, F=1000 N):
- u_b = FL/EA = 5.0e-6 m ✓
- R_a = -F = -1000 N ✓

**Transverse cantilever** (E=200 GPa, Iz=1e-6 m⁴, L=1 m, F=1000 N in Z):
- δ_b = FL³/3EIz = 1.667e-3 m ✓
- R_z,a = -F = -1000 N ✓
- M_x,a = -FL = -1000 N·m ✓

## Validation

| Check | Result |
|-------|--------|
| `cargo fmt -p viewport-poc` | ✓ clean |
| `cargo check --workspace` | ✓ compiles |
| `cargo test -p viewport-poc` | ✓ 304 passed, 0 failed |
| `cargo clippy -p viewport-poc --lib` | 9 warnings (down from 10 baseline; no new warnings) |
| `cargo clippy -p viewport-poc --tests` | result_viz.rs: 0 warnings |

### Test breakdown (304 total)

| Suite | Count |
|-------|-------|
| lib (incl. display.rs unit tests) | 171 |
| app_state | 15 |
| deformation | 6 |
| geometry | 72 |
| edit_model | 6 |
| performance | 1 |
| result_viz | 17 |
| robustness | 12 |
| selection | 4 |

## Files changed

| File | Change |
|------|--------|
| `apps/viewport-poc/src/display.rs` | Rewritten: ResultViewMode enum, refactored ResultDisplayState, 6 visualization helpers, 28 unit tests |
| `apps/viewport-poc/src/main.rs` | Updated imports, 5-mode rendering, display section GUI, window title |
| `apps/viewport-poc/tests/result_viz.rs` | New: 17 integration tests + 2 analytical cases |

## Scope compliance

- ✓ No distributed loads, load combinations, or internal force diagrams
- ✓ No stress contours
- ✓ No database / undo-redo / persistence changes
- ✓ No Tauri app changes
- ✓ No new solvers or solver modifications
- ✓ No section-properties API changes
- ✓ No structural-analysis API changes
