# Phase 129 — Truss Postprocessing API & Performance Audit

**Date:** 2026-10-09  
**Baseline HEAD:** `c2fd46a` (Phase 128)  
**Scope:** Audit 2D and 3D Truss postprocessing APIs for correctness, complexity, memory, consistency, and batch-solving readiness.

---

## 1. Actual Baseline and HEAD

| | |
|---|---|
| Expected HEAD | `c2fd46a` |
| Actual HEAD | `c2fd46a` ✅ |
| Working tree | Clean (only untracked historical docs) |
| Production changes | None (audit-only phase) |

---

## 2. Current Public API Map

### 2D Truss

| Type | Source | Role |
|------|--------|------|
| `TrussModel` | `truss.rs` | Model: nodes, elements, loads, BCs |
| `TrussModel::solve_case(&LoadCase)` | `truss.rs:377` | Solve single load case → `TrussAnalysisResult` |
| `TrussModel::solve_combination(&LoadCombination)` | `truss.rs:457` | Solve combination (merged RHS, single factorization) → `TrussAnalysisResult` |
| `TrussAnalysisResult` | `truss.rs:914` | Owned snapshot: displacements, reactions, axial forces, topology, provenance |
| `TrussEnvelope` | `postprocessing.rs:820` | Multi-case envelope over `TrussAnalysisResult` |
| `TrussEnvelope::from_results(&[&TrussAnalysisResult])` | `postprocessing.rs:853` | Build envelope from results |
| `TrussNodeDisplacementSample` | `postprocessing.rs:749` | Per-node displacement extrema (ux, uy) |
| `TrussNodeReactionSample` | `postprocessing.rs:764` | Per-node reaction extrema (rx, ry) |
| `TrussAxialForceSample` | `postprocessing.rs:778` | Per-element axial-force extrema |

### 3D Truss

| Type | Source | Role |
|------|--------|------|
| `TrussModel3D` | `truss3d.rs` | Model: nodes, elements, loads, BCs |
| `TrussModel3D::solve_case(&LoadCase)` | `truss3d.rs:445` | Solve single load case → `TrussAnalysisResult3D` |
| `TrussModel3D::solve_combination(&LoadCombination)` | `truss3d.rs:509` | Solve combination → `TrussAnalysisResult3D` |
| `TrussAnalysisResult3D` | `truss3d.rs:994` | Owned snapshot: same fields as 2D, 3 DOF/node |
| `Truss3DEnvelope` | `postprocessing.rs:493` | Multi-case envelope over `TrussAnalysisResult3D` |
| `Truss3DEnvelope::from_results(&[&TrussAnalysisResult3D])` | `postprocessing.rs:526` | Build envelope from results |
| `Truss3DNodeDisplacementSample` | `postprocessing.rs:422` | Per-node displacement extrema (ux, uy, uz) |
| `Truss3DNodeReactionSample` | `postprocessing.rs:439` | Per-node reaction extrema (rx, ry, rz) |
| `Truss3DAxialForceSample` | `postprocessing.rs:455` | Per-element axial-force extrema |

### Shared types

| Type | Source | Role |
|------|--------|------|
| `Extremum` | `postprocessing.rs:73` | Signed min/max + governing source index |
| `LoadSource` | `load.rs` | Provenance enum (ModelLoads/LoadCase/LoadCombination) |
| `FemError` | `lib.rs` | Error type |

### Data retention

Both `TrussAnalysisResult` and `TrussAnalysisResult3D` are **owned snapshots** — they do not borrow the solver or model. They retain:
- Full displacement and reaction vectors
- Axial forces per element
- Node coordinates and element connectivity (for topology validation)
- Nodal forces that were applied
- Constrained DOFs (for reaction sampling)
- Load-source provenance

### Operation requirements

| Operation | Requires |
|-----------|----------|
| `solve_case` / `solve_combination` | `&TrussModel` / `&TrussModel3D` + `&LoadCase` / `&LoadCombination` |
| `TrussEnvelope::from_results` | `&[&TrussAnalysisResult]` — no model needed |
| `Truss3DEnvelope::from_results` | `&[&TrussAnalysisResult3D]` — no model needed |
| Topology validation | Only result data (node_coords, element_nodes) — no model needed |

---

## 3. Complexity Analysis

Let R = number of input results, N = nodes, E = elements, D = DOF per node (2 for 2D, 3 for 3D).

### `from_results()` pipeline

```
from_results()
  ├── count validation         O(R)
  ├── validate_topology()      O(R × (N×D + E))
  ├── compute_displacements()  O(R × N × D)
  ├── compute_reactions()      O(R × C)    [C = constrained DOFs ≤ N×D]
  ├── compute_axial_forces()   O(R × E)
  └── sources collect          O(R)
```

**Total time:** O(R × (N×D + E))  
**Total space (output):** O(N×D + E + R)  
**Temporary space:** O(N×D + E) — intermediate Extremum arrays, freed after conversion

### Pass count

Each input result is traversed **4 times**:
1. Topology validation (coordinates + connectivity)
2. Displacement extrema
3. Reaction extrema
4. Axial-force extrema

This could be reduced to 1–2 passes by fusing the loops, but:
- The current separation is clean and testable
- The constant factor is small (each pass is a simple linear scan)
- Fusing would complicate the code without changing the asymptotic complexity

### Allocations

| Allocation | Count | Size |
|------------|-------|------|
| `node_displacements` output Vec | 1 | N |
| `support_reactions` output Vec | 1 | N |
| `axial_forces` output Vec | 1 | E |
| `sources` output Vec | 1 | R |
| Intermediate Extremum arrays | 3 (freed after conversion) | N×D or E |

**No unnecessary copies.** All input data is accessed via references (`&r.displacements`, `&r.reactions`, etc.). No intermediate collections are allocated repeatedly.

### Quadratic behavior

**None.** All operations are O(R × (N + E)). There is no pairwise comparison of results, no nested loop over results × results.

### Topology validation dominance

Topology validation is O(R × (N×D + E)), which is the **same order** as the envelope computation. For large models, validation does not dominate — it adds at most a constant factor of ~1× to the total time. The `is_finite()` check adds 6 comparisons per node per result (2D) or 6 per node per result (3D), which is negligible.

---

## 4. Memory-Allocation Observations

| Aspect | 2D | 3D |
|--------|----|----|
| Result size | 2N + 2N + E + N + E + ... ≈ O(N + E) | 3N + 3N + E + N + E + ... ≈ O(N + E) |
| Envelope size | 2N + 2N + E + R ≈ O(N + E + R) | 3N + 3N + E + R ≈ O(N + E + R) |
| Intermediate | O(N + E) | O(N + E) |
| Copies | None (all by reference) | None (all by reference) |

Both implementations are memory-efficient. The envelope is roughly the same size as a single result, plus R source entries.

---

## 5. Correctness and Consistency Findings

| Check | 2D | 3D | Evidence |
|-------|----|----|---------|
| Empty input rejected | ✅ | ✅ | `p124_empty_input_rejected`, `p120_empty_input_rejected` |
| Singleton input | ✅ | ✅ | `p124_single_result_envelope_matches_original`, `p120_single_result_envelope_matches_original` |
| Deterministic tie-breaking (strict <, >) | ✅ | ✅ | `p124_ties_keep_first_source`, `p120_ties_keep_first_source` |
| Correct governing-source indices | ✅ | ✅ | `p124_governing_source_correct`, `p120_governing_load_source_correct` |
| Independent governing sources per component | ✅ | ✅ | `p125_independent_displacement_governing_sources`, `p126_independent_displacement_governing_sources` |
| Topology validation (coordinates + connectivity) | ✅ | ✅ | Phase 124 (2D), Phase 127 (3D) |
| Non-finite coordinate rejection | ✅ | ✅ | Phase 128 (2D), Phase 127 (3D) |
| Near-zero coordinate tolerance | ✅ | ✅ | `p125_near_zero_coordinate_*`, `p127_near_zero_coordinate_*` |
| Large coordinate tolerance | ✅ | ✅ | `p125_large_coordinate_*`, `p127_large_coordinate_*` |
| Tension/compression signs | ✅ | ✅ | `p124_axial_force_tension_compression`, `p121_axial_force_tension_compression_signs` |
| No partial envelope after failure | ✅ | ✅ | `p128_non_finite_later_candidate_rejected`, `p127_later_mismatch_no_partial_contamination` |
| Reversed endpoints rejected | ✅ | ✅ | `p124_different_element_connectivity_rejected`, `p127_reversed_endpoints_rejected` |
| Phase 127/128 protections intact | ✅ | ✅ | All 89 + 73 tests pass |

**No inconsistencies found.** The 2D and 3D implementations have identical behavior, differing only in DOF count and coordinate dimensionality.

---

## 6. Benchmark Results

**No benchmark performed.** Rationale:

1. No existing benchmark infrastructure (no `benches/` directory, no criterion dependency).
2. The complexity is O(R × (N + E)), which is **optimal** — every result's every node and element must be touched at least once.
3. A benchmark would only measure the constant factor, which is not actionable without a specific performance complaint.
4. Adding a criterion dependency for a one-time audit is not justified.

The code-derived complexity is sufficient evidence that the implementation is asymptotically optimal.

---

## 7. Batch-Solving Readiness Assessment

### Current behavior

Each `solve_case` call:
1. Clones the model — O(N + E)
2. Creates a new `TrussSolver::from_model(&model)` — re-assembles K — O(N + E)
3. Calls `solve_configured()` — applies BCs, creates solver, **factorizes K**, solves — O(factorize(N))
4. Extracts results — O(N + E)

For R load cases with the same model (same K, same BCs, no prescribed displacements), the current API performs **R factorizations**.

### `solve_combination` optimization

`solve_combination` merges multiple load cases into a single RHS and solves once. This is efficient for combinations but produces only **one result** — it doesn't help with envelope construction over independent load cases.

### Batch API feasibility

A batch API could:
1. Factorize K once
2. Solve R right-hand sides using the retained factorization
3. Produce R results

**Benefit:** Reduces time from O(R × factorize(N)) to O(factorize(N) + R × solve(N)).

For dense solvers: factorize = O(N³), solve = O(N²) → speedup ≈ R× for large N.
For sparse solvers: factorize = O(N²) or better, solve = O(N) → speedup ≈ R× for large N.

**Complication:** `solve_case` supports prescribed displacements, which modify boundary conditions. If different load cases have different prescribed displacements, the reduced K changes and the factorization cannot be reused. A batch API would need to:
- Group load cases by their prescribed displacement set
- Factorize once per group
- Solve all cases within a group using the same factorization

**Assessment:** A batch API is feasible and would provide meaningful speedup for users with many load cases on the same model. However:
- It's not a correctness issue — current results are correct
- It requires careful handling of prescribed displacement groups
- It would change the solver internals (retain factorization)
- It should be a separate phase with its own spec

**Recommendation:** Defer to a future phase. The current API is correct and the `solve_combination` method already provides optimization for load combinations.

---

## 8. API Boundary Assessment

### Should 2D and 3D envelopes be merged?

| Factor | Separate (current) | Merged |
|--------|-------------------|--------|
| API clarity | ✅ Explicit types | ❌ Generic or trait-based |
| Code duplication | ❌ ~100 lines × 2 | ✅ Single implementation |
| Runtime overhead | ✅ Zero | ❌ Possible vtable/monomorphization |
| Maintenance | ✅ Stable, rarely changes | ❌ More complex abstractions |
| User experience | ✅ Clear 2D vs 3D | ❌ Template parameters |

**Recommendation: Keep separate.** The duplication is ~100 lines per dimension and is stable (no changes since Phase 124/127). A generic framework would add complexity without clear user benefit. The shared types (`Extremum`, `LoadSource`) already eliminate the meaningful duplication.

---

## 9. Findings

| Severity | Count | Description |
|----------|-------|-------------|
| P0 (crash) | 0 | No panics or crashes |
| P1 (silent wrong) | 0 | All results correct |
| P2 (design gap) | 0 | API is clean, consistent, and well-documented |
| P3 (edge case) | 0 | All edge cases handled (Phases 125–128) |

**No production code changes.** The audit found no defects.

---

## 10. Validation Results

| Command | Result |
|---------|--------|
| `cargo fmt --all -- --check` | ✅ Clean |
| `cargo check -p structural-analysis` | ✅ Clean (3 pre-existing pardiso warnings) |
| `cargo test -p structural-analysis --test truss` | ✅ 89 passed, 0 failed |
| `cargo test -p structural-analysis --test truss3d` | ✅ 73 passed, 0 failed |
| `cargo test -p structural-analysis` | ✅ All 42 test binaries passed, 0 failed |
| `cargo doc -p structural-analysis --no-deps` | ✅ 1 pre-existing warning (`truss3d.rs:9`) |
| `cargo clippy -p structural-analysis --all-targets -- -D warnings` | ⚠️ 412 pre-existing lints; 0 new |
| `cargo test --workspace` | ⏱️ Timed out (pre-existing: section-properties FEM benchmarks) |
| `git diff --check` | ✅ No changes (audit-only phase) |

---

## 11. Evidence-Based Recommendations for Phase 130

### Option A: Batch-solving API (recommended)

Add a batch-solving method to `TrussModel` and `TrussModel3D` that:
1. Factorizes the stiffness matrix once
2. Solves multiple load cases with the same BCs using the retained factorization
3. Returns a `Vec<TrussAnalysisResult>` for direct use in `TrussEnvelope::from_results()`

**Motivation:** The current API performs R factorizations for R load cases. A batch API would reduce this to 1 factorization + R back-substitutions, providing a R× speedup for large models. This directly benefits envelope construction, which is the primary use case for multiple load cases.

**Scope:** Medium — requires changes to solver internals (retain factorization), new public API method, and tests. No changes to envelope or result types.

### Option B: Frame Envelope topology validation

Audit whether `FrameAnalysisResult` exposes topology data and whether the Frame `Envelope` should validate topology like 2D/3D Truss envelopes.

**Motivation:** The Frame envelope only validates node/element counts, not topology. This is the same gap that 3D Truss had before Phase 127.

**Scope:** Small — audit + possible validation addition.

### Option C: Fuse envelope computation passes

Reduce the 4 passes over input results to 1–2 by fusing the displacement/reaction/axial-force loops.

**Motivation:** Minor constant-factor improvement.

**Scope:** Small — but complicates the code for negligible benefit. **Not recommended** unless profiling shows the constant factor is a bottleneck.

**Recommendation:** Option A (batch-solving API) provides the most user value and is directly motivated by the envelope use case.
