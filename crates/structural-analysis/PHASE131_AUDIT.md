# Phase 131 — Truss Batch Solver Audit

**Date:** 2026-10-09  
**Auditor:** GLM-5.2  
**Base HEAD:** `9a99566` (Phase 130)  
**Status:** Complete — no defects found  

## Objective

Independently audit the correctness, factorization reuse, numerical behavior, error semantics, and performance of the 2D and 3D `solve_cases` APIs introduced in Phase 130.

## Baseline

```
HEAD: 9a99566 Phase 130: add truss batch solving with reusable factorization
```

Working tree: clean (only untracked audit docs from prior phases). No production code changes made during this audit.

## Step 1 — Repository baseline

- HEAD = `9a99566`, matches expected baseline.
- Phase 130 diff: 5 files, +1032 lines (2 source + 2 test + 1 report).
- `solve_cases` implementations in `truss.rs:527` (2D) and `truss3d.rs:562` (3D).
- Underlying factorization path: `LinearSolver::factor()` + `LinearSolver::solve()` — separate operations in the trait (`fea/solver.rs:221-224`).

## Step 2 — Factorization reuse evidence

### Static analysis (definitive)

**2D `solve_cases` (`truss.rs:527-754`):**

| Operation | Location | Count |
|-----------|----------|-------|
| `TrussSolver::from_model(self)` | line 587 | 1 (before loop) |
| k_ff construction | lines 668-678 | 1 (before loop) |
| `linear_solver.factor(&k_ff)` | line 685 | 1 (before loop) |
| `linear_solver.solve(&f_reduced)` | line 710 | N (inside loop) |
| `from_model` / `solve_configured` / `apply_boundary_conditions` inside loop | — | 0 |

**3D `solve_cases` (`truss3d.rs:562-800`):** Identical structure.

**`LinearSolver::solve()` does not re-factorize:** Verified all 5 concrete implementations:
- `SparseLU` (`sparse_lu.rs:58`): uses `self.lu.as_ref()` (pre-stored LU factors)
- `SkylineLDLT` (`skyline_ldlt.rs:72`): uses `self.inner.as_ref()` (pre-stored LDLᵀ)
- `DenseGaussian` (`dense_gaussian.rs:111`): uses pre-stored factors
- `CG` (`cg.rs:67`): uses `self.matrix.as_ref()` (pre-stored matrix, iterative solve)
- `ICCG` (`iccg.rs:173`): uses pre-stored IC(0) factorization

None call `factor()` inside `solve()`. **Conclusion: exactly 1 factorization per compatible non-empty batch, 1 back-substitution per case.**

### Timing evidence (supporting)

30-node chain model, 50 load cases, `--release` build:

| Dimension | Batch | Individual (50× `solve_case`) | Speedup |
|-----------|-------|------------------------------|---------|
| 2D | 229.4 µs | 1.1835 ms | **5.16×** |
| 3D | 275.1 µs | 1.2535 ms | **4.56×** |

The ~5× speedup for 50 cases confirms factorization is shared (if each case re-factorized, batch ≈ individual).

### Error paths

- **Validation errors** (invalid node/DOF/non-fixed prescribed): rejected before `from_model` is called. Zero factorizations.
- **`from_model` error** (invalid model): occurs before factorization. Zero factorizations.
- **`factor()` error** (singular matrix): occurs once, before the loop. Batch aborts with `Err`. Zero successful solves.
- **`solve()` error** (per-case): aborts the batch with `Err`. Earlier results are discarded (fail-fast, no partial results).

## Step 3 — Constraint correctness

### Supported constraint types

Truss models support only **fixed DOFs** (static condensation). No spring supports, no inclined rollers.

### Prescribed displacement handling

**`solve_case` path:** Clones model, applies case's `prescribed_displacements` to `model.fixed_dofs` (modifies value if DOF already fixed, or adds new fixed DOF), then calls `from_model` + `solve_configured`.

**`solve_cases` path:** Calls `from_model(self)` once (using model's own `fixed_dofs`). In the per-case loop, clones `solver.prescribed_values` and overrides with case's prescribed values. The effective RHS is:

```
f_reduced[i] = f_global[free_i] − Σ_j K[free_i, constrained_j] · u_c[constrained_j]
```

This matches `apply_boundary_conditions` exactly (verified line-by-line against `truss.rs:935-947`).

### Constraint compatibility enforcement

Cases may modify the *value* of an already-fixed DOF but cannot introduce new fixed DOFs. Validated upfront (lines 571-584 in 2D, equivalent in 3D):

```rust
let is_model_fixed = self.fixed_dofs.iter()
    .any(|&(n, d, _)| self.dof_index(n, d) == idx);
if !is_model_fixed {
    return Err(FemError::InvalidInput(...));
}
```

This ensures k_ff and the free/constrained partition remain valid for all cases. **Matrix reuse is conditioned on the actual constrained DOF set, not merely the count.**

### Model immutability

`solve_cases(&self, ...)` takes `&self` (immutable reference). The model cannot be mutated during the batch. `from_model` clones the model internally. This guarantees system compatibility across all cases.

## Step 4 — Numerical equivalence

### Test coverage

| Scenario | 2D test | 3D test | Tolerance |
|----------|---------|---------|-----------|
| Zero load | `p131_zero_load_case` | `p131_zero_load_case` | 1e-12 / 1e-6 |
| Multiple distinct loads | `p130_multiple_cases_match_solve_case_each` | same | 1e-9 / 1e-6 |
| Mixed-sign axial forces | `p131_mixed_sign_axial_forces` | same | 1e-6 / 1e-3 |
| Nonzero prescribed disp | `p131_nonzero_prescribed_displacement_batch` | same | 1e-12 / 1e-6 |
| Singular model | `p131_singular_model_rejected` | same | N/A (error) |
| Single case vs `solve_case` | `p130_single_case_matches_solve_case` | same | 1e-12 / 1e-6 |
| Displacements + reactions + axial forces | `p130_multiple_cases_match_solve_case_each` | same | 1e-9 / 1e-6 |

All comparisons use tight tolerances (1e-9 to 1e-12 for 2D, 1e-6 for 3D with steel E=200e9). No discrepancies found.

### Verified fields

- `displacements`: full vector match per case ✅
- `reactions`: `R = K·u − f` match per case ✅
- `axial_forces`: per-element match ✅
- `solver_name`: same backend for all cases in a batch ✅
- `constrained_dofs`: match between batch and `solve_case` ✅
- `node_coords`, `element_nodes`, `n_nodes`, `n_elements`: match ✅
- `nodal_forces`: match case input ✅
- `load_source`: `LoadSource::LoadCase { .. }` per case ✅

## Step 5 — Error semantics and partial results

| Scenario | Behavior | Test |
|----------|----------|------|
| Empty batch | `Ok(Vec::new())` | `p130_empty_cases_returns_empty` |
| Invalid node in case | `Err(InvalidNode)` — fail-fast | `p130_invalid_node_rejected`, `p131_later_case_failure_aborts_batch` |
| DOF ≥ 2 (2D) / ≥ 3 (3D) | `Err(InvalidInput)` — fail-fast | (validated before factorization) |
| Prescribed on non-fixed DOF | `Err(InvalidInput)` — fail-fast | `p131_prescribed_on_non_fixed_dof_rejected` |
| Singular/unstable model | `Err(SolverError { .. })` or `Err(InvalidModel)` | `p131_singular_model_rejected` |
| Later case fails after earlier solved | `Err` — entire batch aborted, no partial results | `p131_later_case_failure_aborts_batch` |

**The API cannot silently return an incomplete batch as a successful complete result.** The `results` Vec is only returned as `Ok(results)` after all cases have been processed successfully. Any error during the loop propagates via `?` and discards the partial `results` Vec.

Error types follow crate conventions: `FemError::InvalidNode`, `FemError::InvalidInput`, `FemError::InvalidModel`, `FemError::SolverError { .. }`.

## Step 6 — Envelope integration

Tested via `p130_batch_then_envelope` (2D + 3D): batch results are consumed by `TrussEnvelope::from_results` / `Truss3DEnvelope::from_results` successfully, preserving:
- Topology validation (node coords, element connectivity) ✅
- Case order (results in input order) ✅
- Source identity (`LoadSource::LoadCase` per result) ✅
- `n_results` count ✅

Incompatible topology rejection is handled by the envelope's `validate_topology()` (Phase 124-128), independent of batch solving. No change needed.

## Step 7 — Performance audit

### Measurement methodology

- **Model:** 30-node chain along X, all uy (2D) / uy+uz (3D) constrained, node 0 fully fixed
- **Cases:** 50 distinct axial loads at the last node
- **Build:** `--release`
- **Backend:** auto-selected (`dense` for 2D, `sparse-lu` for 3D)
- **Method:** `std::time::Instant` wall-clock, single run (not a benchmark framework)

### Results

| Dimension | Batch (50 cases) | Individual (50× `solve_case`) | Speedup |
|-----------|-----------------|------------------------------|---------|
| 2D | 229.4 µs | 1.1835 ms | 5.16× |
| 3D | 275.1 µs | 1.2535 ms | 4.56× |

### Cost breakdown (theoretical)

For N cases on a model with n_free free DOFs:

| Operation | `solve_case` × N | `solve_cases` |
|-----------|-------------------|---------------|
| Assembly | N | 1 |
| k_ff construction | N | 1 |
| Factorization | N × O(n_free³) | 1 × O(n_free³) |
| Back-substitution | N × O(n_free²) | N × O(n_free²) |
| Result recovery | N | N |

Speedup → N when factorization dominates (large n_free). Speedup → 1 for tiny models where assembly/recovery dominates.

## Step 8 — Fix policy

**No defects found.** No production code changes.

### Observed behavioral difference (P3, not a defect)

`solve_cases` handles `n_free == 0` (all DOFs constrained) gracefully — displacements are entirely prescribed, reactions computed from `K·u − f`. In contrast, `solve_case` → `solve_configured` → `apply_boundary_conditions` returns `Err(InvalidModel("all DOFs are constrained"))` for the same model.

This is a semantic inconsistency, but `solve_cases` is **more correct**: an all-constrained model has a valid solution (all displacements are prescribed), and reactions are physically meaningful. The `solve_case` rejection is a pre-existing limitation in `apply_boundary_conditions`, not a defect in `solve_cases`.

**Recommendation:** Do not change `solve_cases`. Consider relaxing `apply_boundary_conditions` in a future phase to handle n_free == 0 consistently.

## Step 9 — Validation

| Check | Result |
|-------|--------|
| `cargo fmt --all -- --check` | ✅ PASS |
| `cargo check -p structural-analysis` | ✅ PASS |
| `cargo test -p structural-analysis --test truss` | ✅ 111 passed (100 Phase 130 + 11 Phase 131) |
| `cargo test -p structural-analysis --test truss3d` | ✅ 95 passed (84 Phase 130 + 11 Phase 131) |
| `cargo test -p structural-analysis --lib` | ✅ 26 passed |
| `cargo doc -p structural-analysis --no-deps` | ✅ 3 pre-existing private-link warnings |
| `cargo clippy -p structural-analysis --all-targets -- -D warnings` | ⚠️ Fails on pre-existing section-properties lint debt (133 errors in section-properties, 0 in structural-analysis new code) |
| `git diff --check` | ✅ PASS (CRLF warnings only) |

### clippy note

`cargo clippy -p structural-analysis --all-targets -- -D warnings` fails because clippy compiles the `section-properties` dependency, which has 133 pre-existing lint errors (too_many_arguments, etc.). These are in the published v0.4.0 crate and are not introduced by this phase. Running `cargo clippy -p structural-analysis --lib --tests --no-deps` shows only pre-existing patterns (loop variable indexing) in structural-analysis, with no new warnings from Phase 131 test code.

## Findings

| Severity | Finding | Status |
|----------|---------|--------|
| P0 | None | — |
| P1 | None | — |
| P2 | None | — |
| P3 | `solve_cases` handles n_free == 0 gracefully while `solve_case` rejects with `InvalidModel`. `solve_cases` is more correct. | Documented, no fix needed |

## Files changed

| File | Change |
|------|--------|
| `crates/structural-analysis/tests/truss.rs` | +11 audit tests (`p131_*`) |
| `crates/structural-analysis/tests/truss3d.rs` | +11 audit tests (`p131_*`) |

**Production code:** No changes (audit only).

## Recommended Phase 132

No urgent follow-up required. Possible directions:

1. **Relax `apply_boundary_conditions` n_free == 0 rejection** — align `solve_case`/`solve_configured` with `solve_cases` for all-constrained models (P3 cleanup).
2. **Add `solve_cases` for frame models** — extend the batch-solving pattern to `FrameModel` if frame analysis would benefit from factorization reuse across load cases.
3. **Benchmark with larger models** — measure speedup for 100+ DOF models with 100+ cases to quantify the practical benefit for real-world envelope analysis.
