# Phase 108 — Self-Weight / Body-Force Loads

## Baseline

- HEAD: b67c1d8
- Phase 107: completed (multi-load-case factorization reuse)

## Existing load architecture

### Load types

| Type | Coordinates | Stored by |
|------|-------------|-----------|
| `DistributedLoad` | LOCAL (qx axial, qy transverse) | element index |
| `PointLoad` | LOCAL (fx, fy, mz) | element index + position ξ |
| `AppliedMoment` | GLOBAL (CCW positive) | node index |
| Nodal forces | GLOBAL (fx, fy) | node index + DOF |

### LoadCase

A named collection of loads. Stores:
- `nodal_forces: Vec<(usize, usize, f64)>` — (node, DOF, value) in global coords
- `distributed_loads: Vec<DistributedLoad>` — in local coords
- `point_loads: Vec<PointLoad>` — in local coords
- `applied_moments: Vec<AppliedMoment>` — in global coords

### Load assembly

`assemble_global_load_vector` converts local distributed/point loads to global
via `f_global = T^T · f_local`, where T is the element transformation matrix.

### Coordinate transformation

`BeamElement::to_local_force(ni, nj, gx, gy)` converts a global force/load
intensity to local coordinates:
```text
qx_local =  c·gx + s·gy
qy_local = -s·gx + c·gy
```
where c = dx/L, s = dy/L.

## Self-weight mathematical definition

For each member:
```text
w_global = (ρ · A · gx, ρ · A · gy)   [N/m]
(qx_local, qy_local) = to_local_force(w_global)   [N/m]
```

Added as `DistributedLoad::new(member_idx, qx_local, qy_local)`.

## API

### `FrameModel::self_weight(gx, gy) -> Result<LoadCase, FemError>`

Creates a `LoadCase` named `"self_weight"` containing distributed loads for
all members.

### `LoadCase::add_self_weight(model, gx, gy) -> Result<(), FemError>`

Adds self-weight loads to an existing `LoadCase`. Repeated calls accumulate.

### `FrameModel::self_weight_distributed_loads(gx, gy)` (pub(crate))

Shared backend computing the distributed loads.

## Coordinate convention

- `gx`, `gy` are in **global** coordinates [m/s²]
- For standard downward gravity: `gx = 0.0, gy = -9.81`
- The resulting line load is converted to each member's **local** axes

## Density / unit convention

- `Material::density` [kg/m³] — already exists in section-properties
- `BeamSection::area` [m²] — already exists
- `w = ρ · A · g` [N/m]
- The library does not perform unit conversion; user must ensure consistency

## Beam / Frame support

Both `FrameModel` and `LoadCase` support self-weight. `FrameModel` wraps
`BeamModel`; all mechanics are delegated. No separate Beam implementation.

## Truss decision

**Not supported.** `TrussElement` stores only `E` and `A`, not `Material`
(and therefore not `density`). Implementing truss self-weight would require
either modifying `TrussElement` to store density or passing it separately,
both of which are out of scope for this phase.

## LoadCase integration

Self-weight is a **load source** on `LoadCase`, not on the model. This
avoids cross-case contamination:
```text
LoadCase A: add_self_weight(model) + nodal_load(...)
LoadCase B: nodal_load(...)  // no self-weight
```

## LoadCombination integration

Self-weight in a `LoadCase` is scaled by the combination factor naturally:
```text
1.2D + 1.6L  where D includes self_weight
```
No special `SelfWeightCombination` type needed.

## Factorization reuse

Self-weight modifies **RHS only** — the stiffness matrix K is unchanged.
Phase 107 factorization reuse (`PreparedFrameAnalysis`) works without
modification.

## Tests

12 tests in `tests/self_weight.rs`:
- A: fixed-fixed beam reactions (R = wL/2, M = wL²/12)
- B: simply supported beam (symmetric reactions, equilibrium)
- C: 45° inclined beam (global gravity direction)
- D: multi-member frame (total weight = Σ ρ·A·L·g)
- E: LoadCase isolation (no cross-case contamination)
- F: LoadCombination (1.2D + 1.6L superposition)
- Validation: NaN/infinity rejected, zero density → zero load
- API consistency: `self_weight()` vs `add_self_weight()`
- Factorization reuse: `PreparedFrameAnalysis` with self-weight
- Vertical member: axial-only self-weight
- Horizontal gravity: transverse-only on horizontal beam

## Limitations

1. **Truss**: not supported (`TrussElement` lacks density)
2. **Composite sections**: density semantics for composite sections are not
   explicitly tested; the `Material::density` of the member's material is
   used directly
3. **3D**: not applicable (2D only)

## Final architecture decision

Self-weight is implemented as a **load source** that converts to existing
`DistributedLoad` entries, reusing the entire load assembly pipeline. No
new load types, traits, or abstractions are introduced. The data flow is:

```text
density × area × gravity
          ↓
     self-weight (global line load)
          ↓
 to_local_force → equivalent distributed load (local)
          ↓
      LoadCase RHS
          ↓
 existing factorization (unchanged K)
          ↓
       Result
```
