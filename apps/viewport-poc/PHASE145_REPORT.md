# Phase 145 — Nodal Load Editing MVP

**Status:** COMPLETE  
**Date:** 2026-10-12  
**Base:** Phase 144 (commit `e629f9a`)  

## Objective

Enable the user to view, create, modify, and delete nodal loads
(forces and moments) in the egui desktop application, with correct
conversion to the `FrameModel3D` analysis core and viewport visualization.

## Constraint API Audit (Step 1)

### 3D Frame (`FrameModel3D`) — nodal load API

| Method | Signature | Behavior |
|--------|-----------|----------|
| `add_nodal_load` | `(node_idx, fx, fy, fz, mx, my, mz) -> Result<(), FemError>` | **Accumulates** (pushes to `nodal_forces` Vec) |
| `nodal_loads()` | `-> &[(usize, [f64; 6])]` | Read-only access |

- Validation: node must exist, all components must be finite
- Multiple calls to same node **accumulate** (sum up)
- Therefore `to_frame_model` calls `add_nodal_load` once per node,
  relying on `set_nodal_load`'s replace semantics to ensure one record

## Implementation

### Data Model (`edit_model.rs`)

**New type: `NodalLoadSpec`**

```rust
pub struct NodalLoadSpec {
    pub fx: f64,
    pub fy: f64,
    pub fz: f64,
    pub mx: f64,
    pub my: f64,
    pub mz: f64,
}
```

Constructors & queries:
- `NodalLoadSpec::new(fx, fy, fz, mx, my, mz)` — explicit
- `NodalLoadSpec::zero()` / `Default` — all zeros
- `is_zero()` — true if all components are 0.0
- `is_finite()` — true if all components are finite (not NaN/inf)
- `force_magnitude()` — Euclidean norm of (fx, fy, fz)
- `moment_magnitude()` — Euclidean norm of (mx, my, mz)
- `as_array()` — `[fx, fy, fz, mx, my, mz]`

**EditModel load storage:**

```rust
nodal_loads: Vec<(NodeId, NodalLoadSpec)>
```

- One record per node (replace semantics, not accumulate)
- Sorted/iterated by insertion order; lookup is linear (small N)

**Methods:**

| Method | Behavior |
|--------|----------|
| `set_nodal_load(id, spec)` | Replace existing or insert new; rejects nonexistent NodeId (`NodeNotFound`) and NaN/inf (`NonFiniteCoordinate`) |
| `clear_nodal_load(id)` | Remove load record; OK if node has no load; errors if node doesn't exist |
| `nodal_load(id)` | `Option<&NodalLoadSpec>` |
| `has_nodal_load(id)` | `bool` |
| `nodal_loads()` | `&[(NodeId, NodalLoadSpec)]` |

**Node deletion:** `delete_node` calls `self.nodal_loads.retain()` to clean up.

### Solver Integration (`to_frame_model`)

For each `(node_id, spec)` in `nodal_loads`:
1. Resolve `node_id` to index via stable mapping
2. Call `fm.add_nodal_load(idx, spec.fx, spec.fy, spec.fz, spec.mx, spec.my, spec.mz)`
3. Since `set_nodal_load` ensures one record per node, no double-counting

### Scene (`scene.rs`)

```rust
pub node_loads: Vec<Option<NodalLoadSpec>>,
```

- Populated by `from_edit_model`: one entry per node, `Some(spec)` if load exists, `None` otherwise
- Index-aligned with `node_ids` and `node_positions`

### GUI Inspector (`main.rs`)

Node inspector panel gains a **Nodal Load** section:
- 6 `DragValue` fields in a 2-column grid: Fx, Fy, Fz, Mx, My, Mz
- Each `DragValue` has appropriate speed and range
- **Clear Load** button sets all components to zero and removes the record
- Current load indicator shows force/moment magnitude
- Any edit marks analysis as stale (triggers re-solve on next frame)

### Viewport Visualization (`main.rs`)

**Force indicator:**
- Drawn as a line from the node position in the force direction
- Fixed 0.5 m world length (not scaled by magnitude) — direction only
- Colored circle at the tip
- Color: `LOAD_F_C` (orange `(1.0, 0.6, 0.2)`)

**Moment indicator:**
- Drawn as a circle outline offset from the node
- "M" label at the center of the circle
- Color: `LOAD_M_C` (purple `(0.7, 0.4, 1.0)`)

**Zero loads are not rendered.**

### Model Browser

Nodes with nonzero loads show `[L]` label in the model contents tree.

### Preset Compatibility

All 5 preset models (`portal_frame`, `portal_edit`, `inclined_frame`, `inclined_edit`, `grid_10x10`) updated to use `set_nodal_load` with `NodalLoadSpec`. All presets build and solve correctly.

## Tests

### Unit Tests (`edit_model.rs` — 21 new tests)

**NodalLoadSpec type tests (8):**
- `nodal_load_default_is_zero`
- `nodal_load_zero`
- `nodal_load_new`
- `nodal_load_is_finite_rejects_nan`
- `nodal_load_is_finite_rejects_infinity`
- `nodal_load_force_magnitude`
- `nodal_load_moment_magnitude`
- `nodal_load_as_array`

**Load operation tests (13):**
- `set_nodal_load_success`
- `set_nodal_load_replaces_existing`
- `set_nodal_load_rejects_nonexistent_node`
- `set_nodal_load_rejects_nan`
- `set_nodal_load_rejects_infinity`
- `clear_nodal_load`
- `clear_nodal_load_nonexistent_is_ok`
- `clear_nodal_load_nonexistent_node_fails`
- `delete_node_removes_load`
- `nodal_loads_list`
- `zero_load_is_valid`
- `support_and_load_coexist`
- `to_frame_model_with_edited_load_solves`

### Integration Tests (`tests/edit_model.rs` — 11 new tests)

- `scene_reflects_loads` — scene correctly shows loads from EditModel
- `scene_no_loads_when_none_set` — all None when no loads
- `scene_loads_match_model_after_clear` — clearing updates scene
- `solver_load_transfer` — load transfers to solver, produces correct displacement direction
- `preset_load_preservation` — all 5 presets preserve loads through to_frame_model
- `edited_load_produces_expected_response` — numerical: downward force → negative Uy at apex, ~zero Ux at fixed supports
- `load_preserved_after_node_coordinate_update` — load survives node move
- `delete_node_removes_load_from_scene` — deleting node removes load
- `replace_load_updates_scene` — replacing load updates scene and keeps one record
- `support_and_load_coexist_in_scene` — support and load on different nodes coexist

## Regression Validation

| Check | Result |
|-------|--------|
| `cargo fmt --all -- --check` | ✅ Pass |
| `cargo check --workspace` | ✅ Pass |
| `cargo test -p viewport-poc` | ✅ 202 tests pass (119 lib + 41 edit_model + 12 robustness + 4 selection + 26 other) |
| `cargo test -p structural-analysis --release` | ✅ All pass |
| `cargo clippy -p viewport-poc --all-targets --no-deps -- -D warnings` | Same as baseline (pre-existing lint debt, no new warnings) |

## Files Changed

| File | Change |
|------|--------|
| `apps/viewport-poc/src/edit_model.rs` | NodalLoadSpec type, load methods, to_frame_model update, 21 unit tests |
| `apps/viewport-poc/src/lib.rs` | Export NodalLoadSpec |
| `apps/viewport-poc/src/scene.rs` | node_loads field, from_edit_model update, test constructors |
| `apps/viewport-poc/src/main.rs` | Load editing UI, viewport indicators, model browser [L] label, window title, LOAD_F_C/LOAD_M_C colors |
| `apps/viewport-poc/src/model.rs` | Preset builders updated to set_nodal_load + NodalLoadSpec |
| `apps/viewport-poc/tests/edit_model.rs` | 11 new integration tests, import NodalLoadSpec |
| `apps/viewport-poc/tests/robustness.rs` | ViewportScene constructors updated with node_loads field |

## Constraints Honored

- ✅ Kept egui + eframe (no Tauri migration)
- ✅ No new Element trait
- ✅ No third numerical crate
- ✅ No unnecessary structural-analysis API changes
- ✅ No architecture refactor
- ✅ Phase 144 support functionality preserved
- ✅ Warping FEM untouched
- ✅ section-properties public API untouched
