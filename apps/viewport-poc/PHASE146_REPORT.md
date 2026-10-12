# Phase 146 — Member Property Editing

**Status:** COMPLETE  
**Date:** 2026-10-12  
**Base:** Phase 145 (commit `7249456`)  

## Objective

Enable the user to inspect and edit 3D frame member section/material
properties and reference vectors through the egui inspector, with correct
conversion to the `FrameModel3D` analysis core and analytical verification.

## Baseline Inventory

### EditMember (before Phase 146)
- Fields: `id`, `node_i`, `node_j`, `ref_vec`
- **No section/material properties stored** — all members used hardcoded `default_section()` in `to_frame_model`

### FrameSection3D (structural-analysis)
- Private fields: `E`, `G`, `area`, `iy`, `iz`, `j`
- Derives: `Debug, Clone, Copy`
- Constructor: `new(E, G, area, iy, iz, j)` — validates all finite and positive
- Accessors: `E()`, `G()`, `area()`, `iy()`, `iz()`, `j()`
- Setters: `set_E()`, `set_G()`, `set_area()`, `set_iy()`, `set_iz()`, `set_j()`

### FrameElement3D
- Stores `section: FrameSection3D` by value
- `local_axes()` validates ref_vec not parallel to member axis (PARALLEL_TOL = 1e-8 on sin(angle))

### to_frame_model (before Phase 146)
- Used `default_section()` (E=200e9, G=80e9, A=1e-3, Iy=2e-6, Iz=1e-6, J=3e-6) for ALL members
- Did NOT use per-member section properties

### set_member_ref_vec (before Phase 146)
- Only validated MemberId exists
- No finite/nonzero/parallel validation

## Implementation

### Data Model (`edit_model.rs`)

**EditMember extended:**

```rust
pub struct EditMember {
    pub id: MemberId,
    pub node_i: NodeId,
    pub node_j: NodeId,
    pub ref_vec: [f64; 3],
    pub section: FrameSection3D,  // NEW
}
```

- `add_member` initializes `section: default_section()` (steel I-section)
- `FrameSection3D` stored directly — no duplicate representation

**New EditError variants:**

```rust
InvalidSection(String),   // non-finite, zero, or negative section param
InvalidRefVec(String),    // non-finite, zero, or parallel ref_vec
```

**New methods:**

| Method | Behavior |
|--------|----------|
| `set_member_section(id, E, G, A, Iy, Iz, J)` | Validates all 6 params finite & positive; rejects nonexistent MemberId; atomic (no partial mutation) |
| `member_section(id)` | `Option<&FrameSection3D>` |
| `member_ref_vec(id)` | `Option<[f64; 3]>` |

**Enhanced `set_member_ref_vec`:**
- Validates: all components finite, vector nonzero, not parallel to member axis
- Uses same `PARALLEL_TOL = 1e-8` as `frame3d.rs`
- Parallel check: `sin(angle) = |x_hat × ref| / |ref| < PARALLEL_TOL`
- Atomic: on failure, member is left unchanged

### Solver Integration (`to_frame_model`)

- Replaced `default_section()` with `member.section` for each member
- Each member's edited section is passed to `FrameModel3D::add_member`
- No hardcoded preset properties override edited properties

### GUI Inspector (`main.rs`)

**Member edit buffer:**

```rust
struct MemberEditBuffer {
    member_id: Option<MemberId>,
    e: f64, g: f64, area: f64, iy: f64, iz: f64, j: f64,
    ref_x: f64, ref_y: f64, ref_z: f64,
}
```

- Synced when selected member changes
- **Section / Material** (collapsing): 6 DragValue fields (E, G, A, Iy, Iz, J)
  - Appropriate speeds and ranges for each parameter
- **Reference Vector** (collapsing): 3 DragValue fields (ref_x, ref_y, ref_z)
- **Apply Changes** button: validates buffer, applies section then ref_vec atomically, marks stale
- **Reset** button: reloads buffer from model
- On Apply failure (e.g. parallel ref_vec): displays error via AppMessage, resets buffer

**Buffer isolation:** Selection changes call `sync_member_edit_buffer` which only resets if the MemberId changed — unsaved edits to one member are not accidentally applied to another.

### Preset Compatibility

All 5 presets continue to work:
- `add_member` uses `default_section()` which matches the previous `steel_section()` values
- Preset ref_vecs are validated by the enhanced `set_member_ref_vec` (all existing ref_vecs are non-parallel)
- Preset solver results remain consistent

## Tests

### Unit Tests (`edit_model.rs` — 22 new tests)

**Section tests (10):**
- `set_member_section_success`
- `set_member_section_rejects_nonexistent`
- `set_member_section_rejects_zero`
- `set_member_section_rejects_negative`
- `set_member_section_rejects_nan`
- `set_member_section_rejects_infinity`
- `set_member_section_atomic_on_failure`
- `set_member_section_does_not_affect_others`
- `member_section_query_none_for_nonexistent`
- `default_section_is_steel`

**Ref_vec tests (7):**
- `set_member_ref_vec_success`
- `set_member_ref_vec_rejects_nonexistent`
- `set_member_ref_vec_rejects_zero`
- `set_member_ref_vec_rejects_nan`
- `set_member_ref_vec_rejects_parallel`
- `set_member_ref_vec_rejects_near_parallel`
- `set_member_ref_vec_atomic_on_failure`
- `member_ref_vec_query_none_for_nonexistent`

**Integration tests (5):**
- `section_edit_preserves_connectivity`
- `section_edit_preserves_supports_and_loads`
- `to_frame_model_uses_edited_section`
- `to_frame_model_uses_edited_ref_vec`

### Integration Tests (`tests/edit_model.rs` — 11 new tests)

- `preset_section_preservation` — all 5 presets: section matches between EditModel and FrameModel3D
- `preset_ref_vec_preservation` — all 5 presets: ref_vec matches
- `member_id_stable_after_unrelated_deletion` — section survives unrelated node deletion
- `section_edit_marks_stale` — Project stale tracking after section edit
- `ref_vec_edit_marks_stale` — Project stale tracking after ref_vec edit
- `edited_section_solve_remains_stable` — portal with edited section solves
- `edited_ref_vec_solve_remains_stable` — portal with edited ref_vec solves

### Numerical Integration Tests (4 tests)

**Case A — Axial deformation (u = FL/EA):**
- `axial_deformation_matches_formula` — fixed-free member, axial force, verifies u = FL/EA to 0.01% relative error
- `axial_deformation_doubles_when_area_halves` — halving A doubles u

**Case B — Cantilever bending (δ = FL³/3EI):**
- `cantilever_bending_matches_formula` — fixed-free member, transverse Z force, verifies δ = FL³/(3·E·Iz) to 0.01% relative error
- `cantilever_bending_doubles_when_i_halves` — halving Iz doubles δ

**Local axis convention verified:** For member along X with ref_vec [0,0,1]:
- Local y = global Z, Local z = global -Y
- Force in global Z → bending about local z → uses Iz
- Formula: δ = FL³/(3·E·Iz) ✓

## Regression Validation

| Check | Result |
|-------|--------|
| `cargo fmt --all -- --check` | ✅ Pass |
| `cargo check --workspace` | ✅ Pass |
| `cargo test -p viewport-poc` | ✅ 235 tests pass (141 lib + 52 edit_model + 12 robustness + 4 selection + 26 other) |
| `cargo test -p structural-analysis --release` | ✅ All pass |
| `cargo clippy -p viewport-poc --all-targets --no-deps -- -D warnings` | Same as baseline (pre-existing lint debt, no new warnings) |

## GUI Smoke-Test Status

**UNVERIFIED** — GUI execution not available in this environment.

Manual verification checklist (for user):
1. Open a preset
2. Select a member
3. Modify its section (e.g. change E from 200e9 to 210e9)
4. Click Apply
5. Confirm stale-result indication
6. Solve
7. Confirm the new result differs from the original
8. Change the reference vector
9. Solve again
10. Confirm no crash or unintended property reset

## Files Changed

| File | Change |
|------|--------|
| `apps/viewport-poc/src/edit_model.rs` | EditMember.section field, set_member_section/member_section/member_ref_vec methods, enhanced set_member_ref_vec validation, PARALLEL_TOL, InvalidSection/InvalidRefVec error variants, to_frame_model uses per-member section, 22 unit tests |
| `apps/viewport-poc/src/main.rs` | MemberEditBuffer, section/ref_vec editing UI, Apply/Reset buttons, sync/reset/apply helper methods, window title |

## New / Modified APIs

### New
- `EditMember.section: FrameSection3D` (public field)
- `EditModel::set_member_section(id, E, G, A, Iy, Iz, J) -> Result<(), EditError>`
- `EditModel::member_section(id) -> Option<&FrameSection3D>`
- `EditModel::member_ref_vec(id) -> Option<[f64; 3]>`
- `EditError::InvalidSection(String)`
- `EditError::InvalidRefVec(String)`

### Modified
- `EditModel::set_member_ref_vec` — now validates finite, nonzero, non-parallel
- `EditModel::to_frame_model` — uses per-member section instead of default_section()

## Known Limitations

- No section database or material library
- No cross-section drawing
- No automatic section-property calculation
- No distributed member loads editing (existing in solver, not exposed in GUI)
- No end-release editing
- GUI buffer not persisted across selection changes (by design — prevents accidental cross-member application)
- Manual GUI smoke-test not verified

## P0/P1/P2 Findings

- **P0:** None
- **P1:** None
- **P2:** None

## Recommended Next Phase

- **Phase 147:** Distributed member load editing — expose the existing `FrameMemberLoad` API (uniform/linear, axial/transverse) in the GUI inspector, with equivalent nodal load visualization.
- Alternative: End-release (hinge) editing for 3D frame members — modify member stiffness via static condensation.
