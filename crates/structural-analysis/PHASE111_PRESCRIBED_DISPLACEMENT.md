# Phase 111 — Prescribed Displacement / Support Settlement

## Summary

Added per-`LoadCase` prescribed displacement (support settlement) support.
Non-zero prescribed displacements are now expressible inside a `LoadCase` and
correctly enter the reduced RHS via `f_f -= K_fc · u_c`.

## Baseline

```
HEAD: 2e36717 (Phase 110: audit structural analysis MVP architecture)
```

## Implementation

### Mathematical Formulation

For `K u = F` with prescribed DOFs `u_c` and free DOFs `u_f`:

```
K_ff u_f = F_f - K_fc u_c
```

The RHS correction `- K_fc · u_c` was **already implemented** in
`BeamSolver::apply_boundary_conditions` and `TrussSolver::apply_boundary_conditions`.
This phase exposes it through `LoadCase` and adds per-case isolation.

### API

**New `LoadCase` methods:**

```rust
pub fn prescribed_displacement(&mut self, node: NodeHandle, dof: Dof, value: f64)
    -> Result<(), FemError>

pub fn has_prescribed_displacements(&self) -> bool
```

**New `LoadCase` field (pub(crate)):**

```rust
prescribed_displacements: Vec<(usize, usize, f64)>  // (node_idx, dof_idx, value)
```

### Frame

`FrameModel::solve_case` applies the load case's prescribed displacements via
`try_override_dof` after `inner_with_loads`. This overrides existing support
values (e.g. support settlement) or adds new constraints.

`FrameModel::solve_combination` **rejects** combinations where any case has
prescribed displacements (`FemError::InvalidInput`). Rationale: linearly
combining prescribed displacements `u = Σ α_i u_i` has no reliable engineering
semantics without a specific code requirement.

### Truss

`TrussModel::fix_dof(node, dof, value)` already supported non-zero prescribed
values. No code changes needed — tests added for verification.

### PreparedFrameAnalysis

`PreparedFrameAnalysis::solve_case` applies prescribed displacements but
**rejects** cases that prescribe displacements at DOFs not already constrained
in the model (`FemError::InvalidInput`). Rationale: adding new constraints
changes `K_ff`, invalidating the cached factorisation. Overriding existing
constrained DOFs (same `K_ff`, different `u_c`) is safe — the RHS correction
is recomputed by `apply_boundary_conditions` during `solve_pre_factored`.

`PreparedFrameAnalysis::solve_combination` also rejects prescribed displacements.

### Reaction Recovery

Unchanged: `R = K_original · u - f_global`. The prescribed DOFs are included
in `u_global` (set to their prescribed values), so reactions at supports are
correctly recovered. Free DOF residuals remain ~0.

### Factorization Reuse

- Same constrained DOF set, different `u_c` values: **safe** — `K_ff` unchanged,
  only RHS correction changes. `PreparedFrameAnalysis` reuses the factorisation.
- Different constrained DOF set: **rejected** by `PreparedFrameAnalysis` with
  `FemError::InvalidInput`. User should use `FrameModel::solve_case` instead.

### Envelope

No regression. `Envelope::from_frame_results` reads `FrameAnalysisResult`
which contains the correct displacements and reactions. Phase 110's fix
(free DOF residuals excluded from reaction envelope) is preserved — the
`Envelope` only tracks support reactions at constrained nodes.

## Tests

10 tests in `tests/prescribed_displacement.rs`:

| Test | Verifies |
|------|----------|
| `a_axial_prescribed_displacement` | Axial settlement: displacement, reaction, analytical force `EA/L·δ` |
| `b_zero_prescribed_equivalent_to_no_prescription` | Zero prescribed ≡ no prescription (displacement, reaction, end forces) |
| `c_transverse_prescribed_displacement` | Transverse settlement: displacement, reaction, equilibrium |
| `d_rotational_prescribed_displacement` | Rotational settlement: rotation, moment reaction, equilibrium |
| `e_truss_prescribed_displacement` | Truss support settlement (statically indeterminate) |
| `f_load_case_isolation` | Different cases with different settlements don't contaminate |
| `g_load_combination_rejects_prescribed_displacement` | Combination with prescribed disp → error; without → ok |
| `h_equilibrium_with_prescribed_displacement` | Mixed loads + settlements: force + moment equilibrium |
| `i_prepared_analysis_prescribed_override` | Prepared analysis: override existing constrained DOF (same K_ff) |
| `j_prepared_analysis_rejects_new_constraint` | Prepared analysis: new constraint at free DOF → error; direct solve ok |

## Verification

```
cargo fmt --all -- --check                    → OK
cargo check --workspace --all-targets         → OK (no new warnings)
cargo test -p structural-analysis --release   → 40 suites, all passed (0 failed)
cargo doc -p structural-analysis --no-deps    → OK (0 warnings)
```

## P0 / P1 / P2

```
P0: 0
P1: 0
P2: 0 (no new technical debt introduced)
```

## Changed Files

| File | Change |
|------|--------|
| `src/load.rs` | `prescribed_displacements` field, `prescribed_displacement()` / `has_prescribed_displacements()` methods, `Dof` import |
| `src/frame.rs` | `apply_prescribed_displacements` / `check_prescribed_compatible` helpers; `solve_case` applies prescribed; `solve_combination` rejects; `PreparedFrameAnalysis::solve_case` checks compatibility; `PreparedFrameAnalysis::solve_combination` rejects |
| `tests/prescribed_displacement.rs` | 10 new tests (new file) |
| `PHASE111_PRESCRIBED_DISPLACEMENT.md` | This document |

## Deferred Items

- Inclined roller with non-zero prescribed displacement: the `inclined_roller`
  API already accepts a `value` parameter, but per-LoadCase inclined roller
  settlement is not tested. Deferred to future phase.
- Spring + prescribed displacement combination: springs add stiffness to K
  (DOF remains free), prescribed displacement constrains the DOF. These are
  independent mechanisms and can coexist, but dedicated tests are deferred.
- Typed `LoadSource` enum (P2-4 from Phase 110): not addressed in this phase.
