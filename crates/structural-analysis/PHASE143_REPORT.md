# Phase 143 — Structural Model Editing MVP

**Status:** Complete  
**Date:** 2026-10-10  
**Base:** Phase 142 (commit `4bd4c02`)

## Objective

Implement an editable structural model with stable IDs for nodes and members,
enabling create/read/update/delete operations through the egui desktop UI.
The editable model (`EditModel`) converts to `FrameModel3D` for analysis,
preserving full backward compatibility with the 5 preset models.

## Scope

### In scope
- Stable node/member identifiers (`NodeId(u32)`, `MemberId(u32)`)
- Node CRUD: add, update coordinates, delete (with referential integrity)
- Member CRUD: add (with validation), delete
- Single selection state shared across viewport, inspector, and model browser
- Project mutation boundary: edits mark analysis results stale
- Preset model compatibility: all 5 presets build and solve via `EditModel`
- Per-member reference vectors and partial DOF constraints for preset fidelity

### Out of scope (deferred)
- Material/section/load editors
- Undo/redo
- Model serialization (save/load to file)
- Constraint editing UI (fix_node/fix_dof not editable, only preset-set)

## Architecture

### Edit model layer (`edit_model.rs`)

```
NodeId(u32) / MemberId(u32)     — stable, never reused after deletion
EditNode { id, x, y, z }        — editable node
EditMember { id, node_i, node_j, ref_vec }  — editable member
EditModel { nodes, members, fixed_nodes, fixed_dofs, nodal_loads, ... }
EditError (8 variants)          — all error cases
```

Key design decisions:
- **Stable IDs**: `NodeId`/`MemberId` are monotonically increasing `u32` counters.
  Deleted IDs are never reused, so stale references are detectable.
- **Atomic operations**: All mutation methods validate the complete operation
  before applying any change. A failed operation leaves the model untouched.
- **Zero-length tolerance**: 1e-12 (absolute, physical lower bound).
- **Duplicate member policy**: Topological — same node pair in either direction
  is rejected.
- **Node deletion**: Blocked if referenced by any member. The error includes
  the list of referencing member IDs so the user can be told what blocks deletion.
- **Reference vectors**: Stored per-member in `EditMember.ref_vec`, defaulting
  to `[0, 0, 1]`. Settable via `set_member_ref_vec()` for preset compatibility.
- **Partial constraints**: `fix_dof(node, dof)` stored in `fixed_dofs` list,
  applied during `to_frame_model()` conversion.

### Conversion to FrameModel3D

`EditModel::to_frame_model()` builds a `FrameModel3D` by:
1. Adding nodes in storage order (preserving index correspondence)
2. Adding members with their stored `ref_vec` and a default steel section
3. Applying full fixes (`fix_node`) and partial fixes (`fix_dof`)
4. Applying nodal loads

The default section is `FrameSection3D::new(200e9, 80e9, 1e-3, 2e-6, 1e-6, 3e-6)`
(steel I-section, same as preset models).

### Scene from EditModel

`ViewportScene::from_edit_model()` replaces the former `from_result()`.
The scene stores:
- `node_positions` / `node_ids` — parallel arrays
- `member_nodes` / `member_ids` — parallel arrays
- `member_ref_vecs` — per-member reference vectors

ID-to-index lookups via `scene.node_index(id)` / `scene.member_index(id)`.

### Selection state

`SelectionState` now uses stable IDs:
- `SelectionState::Node(NodeId)` (was `Node(usize)`)
- `SelectionState::Member(MemberId)` (was `Member(usize)`)

The viewport `pick()` function maps screen-space hits to stable IDs via
`scene.node_ids[i]` / `scene.member_ids[i]`.

### Project mutation boundary

`Project::mark_analysis_stale()` marks existing results stale after model edits.
The UI calls this after every successful edit, then rebuilds the scene.
Solving produces a fresh (non-stale) result.

### UI changes

**Left panel (model browser):**
- Preset model buttons (existing)
- "New empty model" button — creates an empty `EditModel`
- "Clear project" button

**Right panel (inspector):**
- "Add" section: text inputs for node coordinates and member node IDs
- "Model contents" section: clickable node/member lists
- "Selection" section: coordinate editing via `DragValue`, delete buttons,
  displacement/reaction/force display
- "Display" and "Camera" sections (existing)

**File menu:**
- "New (empty model)" — creates an empty `EditModel`
- "Clear project" — clears to no-model state

## Files changed

### New files
- `apps/viewport-poc/src/edit_model.rs` — Core editing layer (~700 lines)
- `apps/viewport-poc/tests/edit_model.rs` — Integration tests (21 tests)

### Modified files
- `apps/viewport-poc/src/lib.rs` — Added `edit_model` module and re-exports
- `apps/viewport-poc/src/model.rs` — Added `build_edit()` methods for all 5 presets
- `apps/viewport-poc/src/scene.rs` — `from_edit_model()` uses `member.ref_vec`
- `apps/viewport-poc/src/selection.rs` — Stable IDs (was `usize`)
- `apps/viewport-poc/src/app_state.rs` — `Project` uses `EditModel`, `mark_analysis_stale()`
- `apps/viewport-poc/src/main.rs` — Editing UI, stable IDs, scene from EditModel
- `apps/viewport-poc/tests/app_state.rs` — Updated for EditModel
- `apps/viewport-poc/tests/selection.rs` — Updated for stable IDs
- `apps/viewport-poc/tests/geometry.rs` — Updated for EditModel
- `apps/viewport-poc/tests/deformation.rs` — Updated for EditModel
- `apps/viewport-poc/tests/robustness.rs` — Updated for ViewportScene new fields
- `apps/viewport-poc/tests/performance.rs` — Updated for EditModel

## Test results

### viewport-poc (140 tests, all pass)
- Unit tests (lib): 77
- Integration — app_state: 13
- Integration — deformation: 6
- Integration — edit_model: 21
- Integration — geometry: 6
- Integration — performance: 1
- Integration — robustness: 12
- Integration — selection: 4

### Regression (all pass)
- structural-analysis: 26 tests
- tauri-viewport-poc: 14 tests

### Compilation
- `cargo fmt --all`: clean
- `cargo check -p viewport-poc`: clean, no warnings

## Preset compatibility

All 5 preset models build and solve via `EditModel`:

| Preset | Nodes | Members | Solves |
|--------|-------|---------|--------|
| Portal Frame | 4 | 3 | ✓ |
| Inclined Frame | 5 | 4 | ✓ |
| Grid 1×1 | 8 | 5 | ✓ |
| Grid 3×3 | 32 | 29 | ✓ |
| Grid 10×10 | 242 | 341 | ✓ |

The inclined frame preserves its non-trivial reference vector `[0.3, 0.7, 0.5]`
on member 4, and partial DOF constraints (Ux, Uy, Uz) on node N4.

The grid frame preserves per-member reference vectors: `[0, 0, 1]` for columns
and X-direction beams, `[1, 0, 0]` for Z-direction beams.
