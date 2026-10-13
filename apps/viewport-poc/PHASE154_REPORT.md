# Phase 154 Report — 3D Frame Member Point Loads

## 1. Baseline

- **HEAD**: `8a864ea` Phase 153: add 3D frame uniform distributed loads
- **Working tree**: clean at start, all changes applied during this phase
- **Baseline tests**: 459 viewport-poc tests (194 lib + 265 integration), all passing

## 2. Objective

Add member concentrated forces (point loads) at arbitrary positions along 3D frame members. Users specify:
- **xi** ∈ [0, 1] — nondimensional position along the member (0 = node i, 1 = node j)
- **fx, fy, fz** — force components in the member's local coordinate system (N)

Multiple point loads per member are supported. Loads are transferred to the solver as equivalent nodal loads via Hermite shape functions.

## 3. Solver Extension (frame3d.rs)

The 3D frame solver had no point load support. Added:

- `FrameMemberLoad::PointLoad { member_idx, xi, f_x, f_y, f_z }` enum variant
- `equivalent_nodal_loads_local(L)` using Hermite interpolation:
  - **Axial (f_x)**: linear shape functions N1 = 1-ξ, N2 = ξ
    - f[0] = f_x·(1-ξ), f[6] = f_x·ξ
  - **Transverse y (f_y)**: cubic Hermite N1 = 1-3ξ²+2ξ³, N2 = L·ξ·(1-ξ)², N3 = ξ²·(3-2ξ), N4 = -L·ξ²·(1-ξ)
    - f[1] = f_y·N1, f[5] = f_y·N2, f[7] = f_y·N3, f[11] = f_y·N4
  - **Transverse z (f_z)**: same shape functions, moment signs flipped
    - f[2] = f_z·N1, f[4] = -f_z·N2, f[8] = f_z·N3, f[10] = -f_z·N4
- `add_point_load(member_idx, xi, f_x, f_y, f_z)` method with validation (xi ∈ [0,1], finite values)

All 122 existing frame3d tests pass.

## 4. EditModel Integration (edit_model.rs)

### New types
- `MemberPointLoad3D { xi: f64, fx: f64, fy: f64, fz: f64 }` — with `new`, `zero`, `is_zero`, `is_finite`, `magnitude`
- `EditError::InvalidLoadPosition` — error for xi outside [0, 1]

### Storage
- `member_point_loads: Vec<(MemberId, MemberPointLoad3D)>` — multiple loads per member

### API
| Method | Description |
|--------|-------------|
| `add_member_point_load(id, load)` | Append a point load to a member |
| `update_member_point_load(id, index, load)` | Replace load at index |
| `remove_member_point_load(id, index)` | Remove load at index |
| `clear_member_point_loads(id)` | Remove all point loads for a member |
| `member_point_loads_for(id) -> Vec` | Query all point loads for a member |
| `has_member_point_loads(id) -> bool` | Check if member has any point loads |
| `member_point_loads() -> &[(MemberId, MemberPointLoad3D)]` | Access all point loads |

### Updates
- `from_parts()` — takes `member_point_loads` as 6th argument
- `fingerprint()` — includes point loads
- `delete_member()` — removes associated point loads
- `to_frame_model()` — calls `add_point_load` for each stored load

## 5. Persistence V3 (persistence.rs)

- `CURRENT_FORMAT_VERSION = 3`
- New DTOs: `ProjectFileV3`, `ModelFileV3`, `MemberPointLoadFileV3`
- `serialize_project()` produces V3 format
- `deserialize_project()` dispatches by version:
  - V1 → empty loads (backward compatible)
  - V2 → empty point loads (backward compatible)
  - V3 → full data
  - Unknown → `UnsupportedVersion` error
- V3 validation: member existence, xi ∈ [0,1], finite values
- Removed dead code `model_to_dto_v2()`

## 6. Scene & GUI

### Scene (scene.rs)
- `member_point_loads: Vec<Vec<MemberPointLoad3D>>` field, populated by `from_edit_model()`

### GUI Inspector (main.rs)
- `MemberEditBuffer.point_loads: Vec<MemberPointLoad3D>` — edit buffer
- "Point Loads (local)" collapsing section with:
  - Per-load editors: xi (DragValue, range 0..1), fx, fy, fz
  - "Delete" button per load
  - "Add Point Load" button (default: xi=0.5, fy=-1000)
  - "Clear All" button
- `reset_member_edit` syncs from model, `apply_member_edit` applies as transaction

### Viewport Visualization (main.rs)
- Green arrows (`MEMBER_LOAD_C`) at xi position along member
- Local-to-global direction transformation
- Arrow length = 0.25 × min(member_length, 1.0)
- Skips zero/non-finite loads

### Model Contents Label
- `[P]` label for members with point loads (alongside `[L]` for uniform loads)

## 7. Local Coordinate System

For a beam along global x with ref_vec [0,0,1]:
- **local x** = global x → DOF 0, 6 (translations), 3, 9 (rotations)
- **local y** = global z → DOF 2, 8 (translations), 5, 11 (rotations)
- **local z** = -global y → DOF 1, 7 (translations, **negated**), 4, 10 (rotations, **negated**)
- Bending: local y uses Iz, local z uses Iy

## 8. Numerical Verification

All analytical comparisons use relative tolerance 1e-4.

| Case | Configuration | Check | Result |
|------|--------------|-------|--------|
| A | Fixed-fixed, midspan fy | Reaction equilibrium (ΣRz = -P) | ✅ |
| A | Fixed-fixed, midspan fy | Moment equilibrium | ✅ |
| B | Cantilever, tip fy | Tip deflection = PL³/(3EIz) | ✅ |
| B | Cantilever, tip fy | Tip rotation = PL²/(2EIz) | ✅ |
| B | Cantilever, tip fy | Fixed-end reaction = -P, moment = PL | ✅ |
| C | Cantilever, midspan fy | Tip deflection = Pa²(3L-a)/(6EIz) | ✅ |
| C | Cantilever, midspan fy | Tip rotation = Pa²/(2EIz) | ✅ |
| D | Simply supported, ξ=0.25 fy | Reactions = Pb/L, Pa/L | ✅ |
| D | Simply supported, ξ=0.25 fy | Moment equilibrium | ✅ |
| E | Cantilever, tip fx (axial) | Tip deflection = FL/(EA) | ✅ |
| F | Cantilever, tip fz | Tip deflection (local z → global y) | ✅ |
| G | Cantilever, 2×P/2 superposition | Same as single P | ✅ |
| H | Cantilever, ξ=1 fy vs nodal load | Identical results | ✅ |
| I | Fixed-fixed, 2 loads 3 directions | Total force equilibrium (fx, fy, fz) | ✅ |

## 9. Test Summary

- **Phase 154 tests**: 51 tests in `tests/phase154_point_loads.rs`
  - Data model: 5
  - API (add/query/update/remove/clear): 10
  - Fingerprint: 3
  - to_frame_model: 2
  - delete_member cleanup: 2
  - Scene: 2
  - Persistence V3: 10
  - Numerical correctness: 11
  - Phase 153 regression: 3
- **Total viewport-poc tests**: 510 (194 lib + 316 integration), all passing
- **structural-analysis tests**: all passing (including 122 frame3d tests)

## 10. Verification

| Check | Result |
|-------|--------|
| `cargo fmt --all -- --check` | ✅ Clean |
| `cargo check -p viewport-poc` | ✅ No errors |
| `cargo test -p viewport-poc --release` | ✅ 510 passed, 0 failed |
| `cargo test -p structural-analysis --release` | ✅ All passed |
| `cargo clippy -p viewport-poc` | 32 warnings (all pre-existing) |
| `cargo clippy -p structural-analysis` | 15 warnings (all pre-existing) |
| GUI runtime test | ⛔ BLOCKED (no display) |

## 11. Files Modified

| File | Changes |
|------|---------|
| `crates/structural-analysis/src/frame3d.rs` | +85 lines: PointLoad variant, Hermite shape functions, add_point_load |
| `apps/viewport-poc/src/edit_model.rs` | +200 lines: MemberPointLoad3D, storage, API, fingerprint, to_frame_model, delete_member |
| `apps/viewport-poc/src/persistence.rs` | +221 lines: V3 DTOs, serialization, validation, V1/V2 backward compat |
| `apps/viewport-poc/src/scene.rs` | +14 lines: member_point_loads field |
| `apps/viewport-poc/src/main.rs` | +199 lines: Inspector panel, viewport arrows, [P] label |
| `apps/viewport-poc/src/lib.rs` | +4 lines: export MemberPointLoad3D |
| `apps/viewport-poc/tests/phase154_point_loads.rs` | New: 51 tests |
| `apps/viewport-poc/tests/phase153_member_loads.rs` | Updated: version 2→3 |
| `apps/viewport-poc/tests/robustness.rs` | +6 lines: scene field |
| `apps/viewport-poc/tests/result_viz.rs` | +1 line: scene field |

## 12. Known Limitations

1. **No concentrated moments** — only forces (fx, fy, fz), no moment loads
2. **No trapezoidal/linear loads** — only uniform (Phase 153) and point (Phase 154)
3. **3D simply supported beams** require torsional restraint at one end to avoid singular matrix (standard FEA practice)
4. **GUI runtime not verified** — no display available; inspector/viewport changes are code-complete but untested at runtime
5. **Persistence version bumped to V3** — V1 and V2 files remain readable; V3 is the new default

## 13. Commit Recommendation

Changes are complete and verified. Recommended commit message:

```
Phase 154: add 3D frame member point loads (concentrated forces)
```

Not committed per spec — awaiting user confirmation.
