# Phase 93 Report — Support & Load Refinement

## Summary

Phase 93 adds three features to the 2D Beam/Frame MVP:

1. **Spring Supports** — finite-stiffness translational/rotational springs at node DOFs
2. **Inclined Rollers** — roller supports constraining displacement along an arbitrary direction
3. **Trapezoidal Distributed Loads** — linearly varying distributed loads on members

All features are implemented in the existing `beam_fem.rs` / `frame.rs` / `load.rs`
architecture. No changes to `section-properties` (frozen v0.4.0) or `warping_fem.rs`
(Phase 24 frozen). All 491 tests pass (490 baseline + 1 lib test, plus 27 new tests
across 3 new test files).

## Implementation

### 1. Spring Supports

**Model**: `BeamModel.spring_supports: Vec<(node_idx, dof, stiffness)>`

**Mechanism**: Spring stiffness is added to the global stiffness matrix diagonal
**after** `k_original` is cloned. This ensures:
- The spring DOF remains free (finite restraint, not a constraint)
- The reaction at the spring DOF is `R = K_original * u - f = -k * u` (restoring force)
- No penalty method, no Lagrange multiplier — exact direct stiffness

**API**:
- `BeamModel::spring(node_idx, dof, stiffness) -> Result<(), FemError>`
- `FrameModel::spring(node, dof, stiffness) -> Result<(), FemError>`

**Validation**: stiffness must be finite and positive (zero, negative, NaN, infinity rejected).

### 2. Inclined Rollers

**Model**: `BeamModel.inclined_rollers: Vec<(node_idx, nx, ny, prescribed_value)>`

**Mechanism**: Post-assembly coordinate transformation. After assembling K_global and
f_global, a 2×2 rotation `R = [[c, s], [-s, c]]` is applied to each roller node's
(Ux, Uy) DOFs, transforming them to (U_n, U_t) where U_n is the constrained direction.
The transformation uses a dense matrix intermediate (SparseMatrix has no get/set, and
section-properties is frozen). After transformation, U_n is marked as a standard
single-DOF constraint (static condensation). The solver architecture is unchanged.

**Result recovery**: After solving, `transform_back_to_global()` applies the inverse
rotation to `u_global`, `f_global`, and `k_original`, so all public accessors
(`displacements()`, `reactions()`, `element_end_forces()`, `element_section_forces()`)
return results in the global frame without API changes.

**API**:
- `BeamModel::inclined_roller(node_idx, nx, ny, value) -> Result<(), FemError>`
- `FrameModel::inclined_roller(node, nx, ny, value) -> Result<(), FemError>`

**Validation**: direction (nx, ny) must be non-zero and finite (auto-normalized).
Maximum one inclined roller per node.

### 3. Trapezoidal Distributed Loads

**Model**: `DistributedLoad` extended with `qx_end` and `qy_end` fields.
`DistributedLoad::new()` defaults to uniform (qx_end=qx, qy_end=qy) — backward compatible.

**Equivalent nodal forces** (Hermite shape function exact integration):
```
f_u_i = L*(2*qx + qx_end)/6          f_u_j = L*(qx + 2*qx_end)/6
f_v_i = L*(7*qy + 3*qy_end)/20       f_θ_i = L²*(3*qy + 2*qy_end)/60
f_v_j = L*(3*qy + 7*qy_end)/20       f_θ_j = -L²*(2*qy + 3*qy_end)/60
```
Verified: q_start = q_end = q degenerates to the uniform formulas.

**Section force recovery** (trapezoidal equilibrium):
```
N(x) = N_i - qx*x - (qx_end - qx)*x²/(2L)
V(x) = -V_i + qy*x + (qy_end - qy)*x²/(2L)
M(x) = M_i - x*V_i + qy*x²/2 + (qy_end - qy)*x³/(6L)
```

**API**:
- `DistributedLoad::trapezoidal(element_idx, qx, qy, qx_end, qy_end)`
- `BeamModel::add_trapezoidal_load(element_idx, qx, qy, qx_end, qy_end)`
- `FrameModel::member_trapezoidal(member, qx, qy, qx_end, qy_end)`
- `LoadCase::member_trapezoidal(member, qx, qy, qx_end, qy_end)`

**Single implementation**: `consistent_nodal_load_trapezoidal()` is the sole equivalent
nodal force computation. The uniform `consistent_nodal_load()` delegates to it.
`assemble_global_load_vector` and `element_equivalent_nodal_forces` both call the
trapezoidal version. No duplicate math.

## Files Modified

| File | Changes |
|------|---------|
| `crates/structural-analysis/src/beam_fem.rs` | Spring/inclined roller fields, `from_model` transformation, `transform_back_to_global`, trapezoidal load formulas, new API methods |
| `crates/structural-analysis/src/frame.rs` | `spring()`, `inclined_roller()`, `member_trapezoidal()` API methods |
| `crates/structural-analysis/src/load.rs` | `LoadCase::member_trapezoidal()` |

## Files Created

| File | Description |
|------|-------------|
| `crates/structural-analysis/PHASE93_DESIGN.md` | Design document |
| `crates/structural-analysis/tests/spring_support.rs` | 9 spring support tests |
| `crates/structural-analysis/tests/inclined_roller.rs` | 9 inclined roller tests |
| `crates/structural-analysis/tests/trapezoidal_load.rs` | 9 trapezoidal load tests |

## Breaking Changes

None. All existing APIs are unchanged:
- `DistributedLoad::new()` still creates uniform loads (qx_end=qx, qy_end=qy)
- `consistent_nodal_load()` signature unchanged (delegates to trapezoidal)
- `displacements()` still returns `&[f64]` (inclined roller results transformed internally)
- `reactions()` still returns `Vec<f64>` (works naturally after transform-back)

## Architecture Audit

- ✅ `section-properties` v0.4.0 not modified
- ✅ `warping_fem.rs` not modified (Phase 24 frozen)
- ✅ Single load assembly path (`assemble_global_load_vector`)
- ✅ Single equivalent nodal force implementation (`consistent_nodal_load_trapezoidal`)
- ✅ Solver layer does not know about LoadCase/LoadCombination/Spring/InclinedRoller
- ✅ All new APIs return `Result<(), FemError>`
- ✅ All construction through constructors (no struct literals)
- ✅ No penalty method, no Lagrange multiplier
- ✅ Trapezoidal load uses exact Hermite integration, not approximation

## Test Results

```
cargo test -p structural-analysis --release
Total: 491 passed, 0 failed
```

New test files:
- `spring_support.rs`: 9 tests (axial, transverse, rotational springs, validation, equilibrium)
- `inclined_roller.rs`: 9 tests (direction matching, 45° roller, reactions, validation, equilibrium)
- `trapezoidal_load.rs`: 9 tests (uniform match, triangular reactions, section forces, superposition, validation)

## Deferred

None. All three features are fully implemented and tested.
