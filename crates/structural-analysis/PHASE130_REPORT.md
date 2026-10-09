# Phase 130 — Truss Batch Solving with Reusable Factorization

**Date:** 2026-10-09  
**Base:** `e4df851` (Phase 129)  
**Status:** Complete  

## Objective

Add a `solve_cases` method to both 2D (`TrussModel`) and 3D (`TrussModel3D`) truss models that solves multiple `LoadCase`s sharing a single stiffness matrix factorization. This avoids re-assembling and re-factorizing `K_ff` for each case when the constraint structure is identical across all cases.

## Implementation

### API

```rust
pub fn solve_cases(&self, cases: &[&LoadCase]) -> Result<Vec<TrussAnalysisResult>, FemError>      // 2D
pub fn solve_cases(&self, cases: &[&LoadCase]) -> Result<Vec<TrussAnalysisResult3D>, FemError>    // 3D
```

### Algorithm

1. **Validate all cases** (fail-fast): node bounds, DOF bounds (≥ 2 for 2D, ≥ 3 for 3D), and prescribed displacement constraints — a case may modify the *value* of an already-fixed DOF but must not introduce a new fixed DOF.
2. **Create solver** from model (`from_model`): assembles global `K`, sets up `fixed_dofs` and `prescribed_values`.
3. **Build free-DOF mapping** (`free_to_global`, `global_to_free`) once.
4. **Extract CSR** from `k_global` for efficient row iteration.
5. **Assemble `k_ff`** (reduced stiffness) once and **factorize** once via `LinearSolver::factor()`.
6. **For each case:**
   - Assemble `f_global` from nodal forces.
   - Apply prescribed displacements (override values on already-fixed DOFs).
   - Compute `f_reduced` = `f_global[free]` − `K_fc · u_c` (contribution from prescribed displacements).
   - Back-substitute: `u_free = solver.solve(f_reduced)`.
   - Recover `u_global`, compute reactions `R = K·u − f`, compute axial forces.
   - Build `TrussAnalysisResult` with `LoadSource::LoadCase` provenance.

### Edge case: all DOFs constrained (`n_free == 0`)

When every DOF is fixed, `k_ff` is 0×0 and no linear solve is needed. Displacements are entirely prescribed; reactions and axial forces are computed directly from `K·u`.

### Constraint compatibility

Each case's `prescribed_displacements` are checked against the model's `fixed_dofs`. A case may override the *value* of a fixed DOF but cannot add a new fixed DOF. This ensures `k_ff` and the free-DOF mapping remain valid for all cases. Violations return `FemError::InvalidInput`.

### Dimension differences

| Aspect | 2D | 3D |
|--------|----|----|
| DOF per node | 2 | 3 |
| `dof_index` | `node * 2 + dof` | `node * 3 + dof` |
| DOF bound check | `dof >= 2` | `dof >= 3` |
| `constrained_dofs` map | `(i / 2, i % 2)` | `(i / 3, i % 3)` |
| Node coords | `(f64, f64)` via `point()` | `(f64, f64, f64)` via `coords()` |
| `axial_force` input | `[f64; 4]` | `[f64; 6]` |

## Files modified

- `crates/structural-analysis/src/truss.rs` — Added `TrussModel::solve_cases` (+253 lines)
- `crates/structural-analysis/src/truss3d.rs` — Added `TrussModel3D::solve_cases` (+246 lines)
- `crates/structural-analysis/tests/truss.rs` — Added 11 `p130_*` tests
- `crates/structural-analysis/tests/truss3d.rs` — Added 11 `p130_*` tests

## Tests

### 2D tests (11)

| Test | Description |
|------|-------------|
| `p130_empty_cases_returns_empty` | Empty input → empty Vec |
| `p130_single_case_matches_solve_case` | Single case equivalence with `solve_case` |
| `p130_multiple_cases_match_solve_case_each` | Multi-case equivalence (3 cases on triangular truss) |
| `p130_results_preserve_input_order` | Results returned in input order |
| `p130_load_source_is_load_case` | `LoadSource::LoadCase` provenance preserved |
| `p130_prescribed_displacement_modifies_fixed_dof` | Prescribed displacement on already-fixed DOF |
| `p130_new_fixed_dof_rejected` | Prescribing at non-fixed DOF → `InvalidInput` |
| `p130_invalid_node_rejected` | Out-of-bounds node → `InvalidNode` |
| `p130_all_constrained_n_free_zero` | All-DOF-constrained path (`n_free == 0`) |
| `p130_batch_then_envelope` | Batch results compatible with `TrussEnvelope` |
| `p130_solver_name_populated_when_free_dofs_exist` | `solver_name` set when free DOFs exist |

### 3D tests (11)

Mirror of the 2D tests using `axial_bar()` and `tripod()` helpers.

### Test results

```
2D: 100 passed; 0 failed  (89 existing + 11 new)
3D:  84 passed; 0 failed  (73 existing + 11 new)
```

No regressions in existing tests.

## Performance analysis

### Theoretical speedup

For `n` cases on a model with `n_free` free DOFs:

- **`solve_case` × n**: `n` × (assembly + factorization + solve) = `n × O(n_free³)` for factorization
- **`solve_cases`**: 1 × (assembly + factorization) + `n` × solve = `O(n_free³) + n × O(n_free²)`

Speedup ≈ `n` when factorization dominates (large `n_free`), approaching 1 for tiny models.

### Practical benefit

Most beneficial when:
- Number of cases is large (e.g., 50+ load combinations in envelope analysis)
- Model has many free DOFs (factorization cost is significant)
- All cases share the same constraint structure (typical in code-based load combination analysis)

## Validation

| Check | Result |
|-------|--------|
| `cargo fmt --all -- --check` | Clean |
| `cargo check --workspace` | Clean |
| `cargo test -p structural-analysis --release --test truss` | 100 passed |
| `cargo test -p structural-analysis --release --test truss3d` | 84 passed |
| `cargo doc -p structural-analysis --no-deps` | Clean (3 pre-existing private-link warnings) |
| `cargo clippy -p structural-analysis --lib --no-deps` | 12 warnings (all pre-existing patterns) |

## Design decisions

1. **Fail-fast error semantics**: First invalid case returns `Err`; no partial results.
2. **Empty input**: Returns `Ok(Vec::new())`.
3. **Prescribed displacements**: Cases may override values of already-fixed DOFs; new fixed DOFs are rejected.
4. **No new dependencies**: Uses existing `LinearSolver`, `SolverRegistry`, `SparseMatrix`.
5. **No API changes**: Existing `solve_case` and `solve_combination` signatures unchanged.
6. **`SolverSelection::Auto`**: Same solver selection as `solve_case`.
7. **Result provenance**: Each result carries `LoadSource::LoadCase` with the case name.
