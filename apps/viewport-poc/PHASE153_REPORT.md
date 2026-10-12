# Phase 153 Report: 3D Frame Uniform Distributed Loads

## Status: COMPLETE

## Objective

Introduce member uniform distributed-load editing for the 3D frame GUI.
Users can select a member, assign uniform distributed loads (qx, qy, qz in
local coordinates), edit/clear/visualize/solve/undo-redo/save the loads.

## Implementation Path

**Path A (Reuse)** — Phase 135 already implemented complete 3D frame member
load API in `structural-analysis::frame3d`.  This phase reuses the existing
solver support (`add_uniform_axial`, `add_uniform_y`, `add_uniform_z`) and
adds the editing, persistence, visualization, and undo/redo integration
layers in `viewport-poc`.

## Changes

### `edit_model.rs` — MemberUniformLoad3D + API

- **New struct**: `MemberUniformLoad3D { qx, qy, qz }` — uniform distributed
  load in local coordinates (force per unit length, N/m).
  - Methods: `new()`, `zero()`, `is_zero()`, `is_finite()`, `magnitude()`.
- **New error variant**: `EditError::NonFiniteLoad`.
- **New storage field**: `member_uniform_loads: Vec<(MemberId, MemberUniformLoad3D)>`
  on `EditModel`.  One record per member.
- **New API methods**:
  - `set_member_uniform_load(id, load)` — set/replace, validates MemberId + finite.
  - `clear_member_uniform_load(id)` — remove load record.
  - `member_uniform_load(id)` — query.
  - `has_member_uniform_load(id)` — existence check.
  - `member_uniform_loads()` — all loads slice.
- **Updated `delete_member()`** — removes associated load record.
- **Updated `fingerprint()`** — includes member uniform loads (qx, qy, qz bits).
- **Updated `from_parts()`** — accepts `member_uniform_loads` parameter.
- **Updated `to_frame_model()`** — transfers loads to `FrameModel3D` via
  `add_uniform_axial/y/z` using stable MemberId→index mapping.  Zero
  components are skipped to avoid unnecessary solver entries.

### `persistence.rs` — ProjectFileV2

- **`CURRENT_FORMAT_VERSION`** bumped from 1 to 2.
- **New V2 DTO types**: `ProjectFileV2`, `ModelFileV2`, `MemberUniformLoadFileV2`.
- **V1 backward compatibility**: `deserialize_project` peeks at
  `format_version` and dispatches:
  - Version 1 → `dto_to_model` (empty member loads).
  - Version 2 → `dto_v2_to_model` (with member loads).
  - Unknown → `UnsupportedVersion` error.
- **V2 validation**: reuses V1 validation + validates member load references,
  duplicates, and finite values.
- **Serialization** always produces V2 format.
- **Clone** derived on V1 DTO types for V2 conversion.

### `scene.rs` — ViewportScene

- **New field**: `member_loads: Vec<Option<MemberUniformLoad3D>>` — parallel
  to `member_ids`, populated by `from_edit_model()`.

### `main.rs` — GUI

- **MemberEditBuffer** extended with `load_qx`, `load_qy`, `load_qz` fields.
- **`reset_member_edit()`** — loads current member load values into buffer.
- **`apply_member_edit()`** — applies section + ref_vec + load as a single
  Undo/Redo transaction.  Zero loads are cleared rather than stored.
- **Inspector panel**: "Distributed Load (local)" collapsing section with
  qx/qy/qz DragValue editors + "Clear Load" button.
- **Viewport visualization**: draws 4 green arrows along each loaded member,
  direction follows local-to-global transformation of the load vector.
- **Model contents**: members with loads show `[L]` label.
- **New color**: `MEMBER_LOAD_C` = green (100, 255, 100).

### `lib.rs`

- Exports `MemberUniformLoad3D`.

## Numerical Verification

### Case A: Fixed-Fixed Beam (L=4m, q=-1000 N/m in local y)

- Vertical reactions: ±qL/2 = ±2000 N at each end ✓
- Fixed-end moments: qL²/12 = ±1333.33 N·m ✓
- Equilibrium: sum of reactions = -qL ✓

### Case B: Cantilever (L=3m, q=-5000 N/m in local y)

- Tip deflection: qL⁴/(8EI) = -0.253125 m ✓
- Reaction force: -qL = 15000 N ✓
- Reaction moment: qL²/2 = -22500 N·m ✓

### Case C: Uniform Axial Load (L=2m, q=2000 N/m in local x)

- Axial reactions: -qL/2 = -2000 N at each end ✓
- Equilibrium: sum = -qL = -4000 N ✓

### Case D: Rotated 3D Member (L=5m, ref=[1,0,0], q=-1000 N/m in local y)

- Local y axis = (4/5, -3/5, 0) in global coords
- Total reactions = (4000, -3000, 0) ✓
- Equilibrium verified in all 3 global axes ✓

## Persistence Migration

- **V1 → V2**: V1 files load with empty member uniform loads.
- **V2 round-trip**: member loads survive save/load cycle exactly.
- **Golden V1 fixture**: unchanged, loads as V1 with 0 member loads.
- **Unknown versions**: rejected with `UnsupportedVersion` error.
- **Invalid V2**: nonexistent member ID, duplicate loads, non-finite values
  all rejected with `Validation` error.

## Test Summary

- **New tests**: 35 (in `tests/phase153_member_loads.rs`)
  - 4 numerical (Case A/B/C/D)
  - 31 integration (API, persistence, scene, undo/redo, dirty-state)
- **Total tests**: 459 (424 existing + 35 new), all pass.
- **GUI runtime verification**: BLOCKED (no graphical environment).

## Validation

- `cargo fmt -p viewport-poc --check`: PASS
- `cargo check -p viewport-poc`: PASS
- `cargo test -p viewport-poc --release`: 459 passed, 0 failed
- `cargo clippy -p viewport-poc`: no new warnings (pre-existing baseline only)

## Files Modified

- `apps/viewport-poc/src/edit_model.rs` — MemberUniformLoad3D, API, fingerprint, to_frame_model
- `apps/viewport-poc/src/persistence.rs` — V2 DTO, serialization, V1 compat
- `apps/viewport-poc/src/scene.rs` — member_loads field
- `apps/viewport-poc/src/main.rs` — GUI inspector, viewport visualization, edit buffer
- `apps/viewport-poc/src/lib.rs` — export MemberUniformLoad3D

## Files Created

- `apps/viewport-poc/tests/phase153_member_loads.rs` — 35 tests
- `apps/viewport-poc/PHASE153_REPORT.md` — this report

## Scope Restrictions

No trapezoidal/concentrated/thermal/moving loads, no load combinations/cases,
no N/V/M diagrams, no stress contours, no new element types, no Tauri changes.
