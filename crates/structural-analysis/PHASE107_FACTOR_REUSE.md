# Phase 107 — Multi-Load-Case Factorization Reuse

**Date**: 2026-09-24  
**Baseline**: commit `57afbf2` (Phase 106)  
**Status**: Complete  
**Production changes**: Yes — new API for factorization reuse  

---

## 1. Baseline

```text
HEAD: 57afbf2 Phase 106: audit structural analysis capabilities and solver architecture
tests: 563 pass
doctests: 10 pass
rustdoc: 0 warnings
fmt: PASS
```

---

## 2. Current Solve Pipeline (Before Phase 107)

```text
FrameModel::solve_case(&LoadCase)
    ↓
validate()
    ↓
inner_with_loads(case) → new BeamModel with case loads
    ↓
BeamSolver::from_model(&model)
    ├── assemble K_global from elements (incl. releases, rollers, springs)
    ├── assemble f_global from loads
    └── store fixed_dofs, prescribed_values, node_rotations
    ↓
beam.solve_configured()
    ├── apply_boundary_conditions()
    │     ├── identify free/constrained DOFs
    │     ├── extract K_ff from K_global
    │     └── reduce f: f_reduced = f_f - K_fc · U_c
    ├── create solver via SolverRegistry
    ├── solver.factor(K_ff)          ← EXPENSIVE: O(n³)
    └── solver.solve(f_reduced)      ← back-substitution: O(n²)
```

**Root cause**: Each `solve_case()` call creates a new `BeamSolver`, which
re-assembles K, re-condenses, and **re-factorizes** — even though K is identical
across load cases (only f differs).

---

## 3. Root Cause

The `LinearSolver` trait already supports factorization reuse:

```rust
solver.factor(&matrix)?;       // factor once
let u1 = solver.solve(&rhs1)?; // back-substitute
let u2 = solver.solve(&rhs2)?; // reuse factorization
```

But the Frame/Beam API does not expose this. Each `solve_case()` / `solve_combination()`
creates a fresh `BeamSolver` and calls `solve_configured()`, which factors from scratch.

---

## 4. Design

### 4.1 Key Insight

The stiffness matrix K depends only on:
- Geometry (node positions)
- Element properties (material, section)
- Boundary conditions (supports, springs, inclined rollers)
- End releases

It does **not** depend on loads. Only the force vector f changes per LoadCase.

### 4.2 Approach

1. **Prepare**: Assemble K, condense BCs, factor K_ff — **once**.
2. **Per case**: Assemble f, reduce f, back-substitute — **N times**.

The per-case work still re-assembles K (O(n_elements × 36), cheap) and re-condenses
(O(n²), moderate) because `BeamSolver::from_model` bundles K and f. The key saving
is skipping the O(n³) factorization.

### 4.3 Why not avoid K re-assembly too?

`BeamSolver::from_model` assembles K and f together. Separating them would require
restructuring BeamSolver's constructor — a larger refactor with no clear benefit
(K assembly is O(n²), factorization is O(n³)). The current approach is the minimal
change that captures the dominant cost saving.

---

## 5. API

### 5.1 New Types

```rust
pub struct PreparedFrameAnalysis<'a> {
    model: &'a FrameModel,
    factored: Option<Box<dyn LinearSolver>>,
    solver_name: Option<String>,
    solver_selection: SolverSelection,
}
```

### 5.2 New Methods

```rust
// FrameModel
impl FrameModel {
    pub fn prepare(&self) -> Result<PreparedFrameAnalysis<'_>, FemError>;
    pub fn prepare_with(&self, selection: SolverSelection) -> Result<PreparedFrameAnalysis<'_>, FemError>;
    pub fn solve_cases(&self, cases: &[LoadCase]) -> Result<Vec<FrameAnalysisResult>, FemError>;
}

// PreparedFrameAnalysis
impl<'a> PreparedFrameAnalysis<'a> {
    pub fn solver_name(&self) -> Option<&str>;
    pub fn solve_case(&self, case: &LoadCase) -> Result<FrameAnalysisResult, FemError>;
    pub fn solve_cases(&self, cases: &[LoadCase]) -> Result<Vec<FrameAnalysisResult>, FemError>;
    pub fn solve_combination(&self, combo: &LoadCombination) -> Result<FrameAnalysisResult, FemError>;
}
```

### 5.3 Usage

```rust
// Option A: prepare once, solve many
let prepared = frame.prepare()?;
let result_a = prepared.solve_case(&case_a)?;
let result_b = prepared.solve_case(&case_b)?;
let result_c = prepared.solve_case(&case_c)?;

// Option B: one-shot convenience
let results = frame.solve_cases(&[case_a, case_b, case_c])?;

// Option C: combination with pre-factored matrix
let prepared = frame.prepare()?;
let result = prepared.solve_combination(&combo)?;
```

### 5.4 No solver internals exposed

`PreparedFrameAnalysis` stores `Box<dyn LinearSolver>` as a private field. The
public API exposes only `solver_name()` for observability. Users cannot access
the factorization, the matrix, or the solver backend.

---

## 6. Factorization Lifecycle

```text
FrameModel::prepare()
    ↓
validate()
    ↓
BeamSolver::from_model(&self.inner)     ← assemble K (no loads)
    ↓
beam.condense()                         ← apply BCs, extract K_ff
    ↓
SolverRegistry::create_selected(K_ff)   ← select backend
    ↓
solver.factor(K_ff)                     ← FACTOR ONCE (O(n³))
    ↓
PreparedFrameAnalysis { factored: solver }
```

```text
PreparedFrameAnalysis::solve_case(case)
    ↓
model.inner_with_loads(case)            ← create model with case loads
    ↓
BeamSolver::from_model(&model)          ← re-assemble K (cheap) + assemble f
    ↓
beam.solve_pre_factored(&factored)      ← NO factorization, only:
    ├── apply_boundary_conditions()     ←   reduce f (O(n²))
    ├── factored.solve(f_reduced)       ←   back-substitute (O(n²))
    └── expand + transform              ←   O(n)
    ↓
FrameAnalysisResult { beam, model, load_source }
```

---

## 7. Spring / Roller / Release Handling

### 7.1 Springs

Spring stiffness is added to K_global's diagonal in `from_model`. Since the
model's spring supports are the same across cases (only loads differ), K is
identical. ✅

### 7.2 Inclined Rollers

The coordinate rotation for inclined rollers is applied to K_global in
`from_model`. Since the roller configuration is the same across cases, the
rotation is identical. `transform_back_to_global()` is called per case in
`solve_pre_factored` to transform displacements back. ✅

### 7.3 End Releases

End release condensation is part of element stiffness assembly
(`released_global_stiffness`), which happens in `from_model`. Since releases
are a model property (not a load property), K is identical across cases. ✅

---

## 8. LoadCase

`PreparedFrameAnalysis::solve_case(case)`:
1. Creates a BeamModel with the case's loads via `inner_with_loads`.
2. Creates a BeamSolver from that model (re-assembles K + assembles f).
3. Calls `solve_pre_factored` with the pre-factored solver.
4. Returns `FrameAnalysisResult` with `load_source = "case:{name}"`.

**Order preservation**: `solve_cases` iterates cases in order, pushing results
to a Vec. Order is guaranteed. ✅

**State isolation**: Each case creates a fresh `BeamSolver` with its own
`u_global`, `f_global`, and `model`. No state is shared between cases. ✅

---

## 9. LoadCombination

`PreparedFrameAnalysis::solve_combination(combo)`:
1. Merges RHS: `f = Σ factor_i × f_case_i` (same as existing `solve_combination`).
2. Creates a BeamModel with the merged loads.
3. Creates a BeamSolver from that model.
4. Calls `solve_pre_factored` with the pre-factored solver.
5. Returns `FrameAnalysisResult` with `load_source = "combination:{name}"`.

This is correct: the combination is a single RHS, so one back-substitution
suffices. ✅

---

## 10. `solve_pre_factored` Implementation

```rust
pub(crate) fn solve_pre_factored(
    &mut self,
    factored: &dyn LinearSolver,
) -> Result<(), FemError> {
    // 1. Apply BCs (reduces f, re-extracts K_ff — same result)
    let (f_reduced, constrained_dofs, constrained_values) =
        self.apply_boundary_conditions();

    // 2. Back-substitute using pre-factored solver (NO factor() call)
    let u_free = factored.solve(&f_reduced)?;

    // 3. Expand to full DOF space
    self.u_global = ...;
    self.transform_back_to_global();
    Ok(())
}
```

The key difference from `solve_configured`: step 2 calls `solve()` without
a preceding `factor()`. The factorization was done once in `prepare()`.

---

## 11. Tests

11 new tests in `tests/multi_load_case.rs`:

| Test | Verifies |
|------|----------|
| `numerical_equivalence_individual_vs_prepared` | `solve_case()` results match `prepared.solve_case()` (displacements, reactions, forces, section forces) |
| `solve_cases_preserves_order` | Results in same order as input cases |
| `solve_cases_convenience_method` | `frame.solve_cases()` matches `frame.prepare()?.solve_cases()` |
| `load_combination_equivalence` | `solve_combination()` matches `prepared.solve_combination()` |
| `load_combination_vs_superposition` | Combination result = Σ factor × individual results |
| `spring_support_multi_case` | Spring supports work with prepared analysis |
| `inclined_roller_multi_case` | Inclined rollers work with prepared analysis |
| `end_release_multi_case` | End releases work with prepared analysis |
| `prepared_solver_name` | Solver name is correctly reported |
| `prepared_with_explicit_solver` | `prepare_with(SolverSelection::dense())` works |
| `equilibrium_all_cases` | All cases pass equilibrium check |

---

## 12. Performance Implication

For N load cases on a structure with n free DOFs:

| Approach | Factorization | Back-substitution | Total |
|----------|--------------|-------------------|-------|
| N × `solve_case()` | N × O(n³) | N × O(n²) | O(N · n³) |
| `prepare()` + N × `solve_case()` | 1 × O(n³) | N × O(n²) | O(n³ + N · n²) |

For N = 10, n = 100: **10× speedup** on the factorization step.

The per-case overhead (K re-assembly + condensation) is O(n²), which is
negligible compared to the O(n³) saving.

---

## 13. API Compatibility

- **No breaking changes**: All existing methods (`solve`, `solve_with`,
  `solve_case`, `solve_combination`) are unchanged.
- **Additive only**: New types (`PreparedFrameAnalysis`) and new methods
  (`prepare`, `prepare_with`, `solve_cases`) are added.
- **Re-exports**: `PreparedFrameAnalysis` is re-exported at crate root.
- **No new dependencies**: Uses existing `LinearSolver` and `SolverRegistry`
  from `section-properties`.

---

## 14. Changed Files

| File | Change |
|------|--------|
|0 `src/beam_fem.rs` | +62 lines: `solve_pre_factored` method |
| `src/frame.rs` | +170 lines: `PreparedFrameAnalysis` struct + impl, `prepare`, `prepare_with`, `solve_cases` on FrameModel |
| `src/lib.rs` | +1 line: re-export `PreparedFrameAnalysis` |
| `tests/multi_load_case.rs` | +370 lines: 11 new tests |

---

## 15. Verification

```text
HEAD:     (after commit)
baseline: 57afbf2
fmt:      PASS
check:    PASS
tests:    574 tests PASS (563 existing + 11 new)
doctests: 11 PASS (10 existing + 1 new)
doc:      0 warnings
factorization reuse: YES — 1 factorization + N back-substitutions
P0: 0
P1: 0
P2: 0
changed files: beam_fem.rs, frame.rs, lib.rs, tests/multi_load_case.rs
```
