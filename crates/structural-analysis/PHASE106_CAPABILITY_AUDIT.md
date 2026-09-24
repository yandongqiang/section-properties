# Phase 106 — Structural Analysis Capability & Solver Architecture Audit

**Date**: 2026-09-24  
**Baseline**: commit `57b1983` (Phase 105)  
**Status**: Complete  
**Production changes**: None  

---

## 1. Executive Summary

The `structural-analysis` crate provides a **correct and comprehensive** 2D
linear static analysis platform for beam, frame, and truss structures. The
Euler-Bernoulli beam formulation, coordinate transformation, static condensation
(end releases, boundary conditions), load vector assembly, and force recovery
are all mathematically correct. The solver infrastructure (5 backends via
`section-properties`) is sound and supports factorization reuse at the trait
level.

**0 P0 / 0 P1 issues found.** The architecture is clean: no abstraction
refactoring is needed (no `Element` trait, no `Constraint` trait, no third
crate — consistent with Phase 94/95/99/101 decisions).

**The single most impactful capability gap** is that the Frame/Beam API does
not expose factorization reuse for multi-load-case analysis. Each
`solve_case()` / `solve_combination()` creates a new `BeamSolver`, re-assembles
the stiffness matrix, re-condenses boundary conditions, and re-factorizes —
even though the stiffness matrix is identical across load cases. For N load
cases, this is O(N × factorize) instead of O(1 × factorize + N × solve).

---

## 2. Current Architecture

```text
section-properties (v0.4.0, published)
    ├── fea::SparseMatrix (CSR)
    ├── fea::solver::LinearSolver trait
    │     ├── DenseGaussianSolver    (O(n³), n ≤ 500)
    │     ├── SkylineLdltSolver      (LDLᵀ, n ≤ 10000)
    │     ├── SparseLuSolver         (dense-backed, n ≤ 3000)
    │     ├── CgSolver               (iterative, SPD)
    │     └── IccgSolver             (IC(0)-PCG, SPD)
    ├── fea::solver::SolverRegistry  (factory + auto-select)
    └── io::{json, csv, dxf, svg, mesh_export}

structural-analysis (v0.1.0, workspace-only)
    ├── beam_fem.rs   (~4413 lines) — BeamSolver, BeamModel, BeamAnalysisResult
    ├── frame.rs      (~1950 lines) — FrameModel, FrameSolver, FrameAnalysisResult
    ├── truss.rs      (~946 lines)  — TrussSolver, TrussModel, TrussAnalysisResult
    ├── load.rs       (~307 lines)  — LoadCase, LoadCombination
    ├── postprocessing.rs (~207 lines) — Envelope, EnvelopeSample
    ├── mechanism.rs  (~578 lines)  — StructuralDiagnostic
    └── lib.rs        (~141 lines)  — re-exports
```

### Layer separation (confirmed stable)

```text
Model → Solver → AnalysisResult → Post-processing → (future Export/Visualization)
```

---

## 3. Capability Matrix

| Capability | Beam | Frame | Truss | Status | Notes |
|-----------|------|-------|-------|--------|-------|
| Nodal load | ✓ | ✓ | ✓ | implemented | global coords |
| Member point load | ✓ | ✓ | — | implemented | local coords, xi ∈ [0,1] |
| Distributed load (UDL) | ✓ | ✓ | — | implemented | local coords |
| Trapezoidal load | ✓ | ✓ | — | implemented | linear variation |
| Applied moment | ✓ | ✓ | — | implemented | global, CCW positive |
| End release (hinge) | ✓ | ✓ | — | implemented | static condensation |
| Spring support | — | ✓ | — | implemented | diagonal stiffness |
| Inclined roller | — | ✓ | — | implemented | constraint direction |
| Prescribed displacement | ✓ | ✓ | ✓ | implemented | via condensation |
| Fix / pin / roller | ✓ | ✓ | ✓ | implemented | support vocabulary |
| Load case | — | ✓ | — | implemented | named load collection |
| Load combination | — | ✓ | — | implemented | linear superposition |
| Section forces N(x), V(x), M(x) | ✓ | ✓ | — | implemented | O(1) per query |
| Member end forces (local) | ✓ | ✓ | — | implemented | [N,V,M] × 2 |
| Member end forces (global) | ✓ | ✓ | — | implemented | Tᵀ f_local |
| Axial force | — | — | ✓ | implemented | tension positive |
| Force diagram | ✓ | ✓ | — | implemented | sampled, no interpolation |
| Envelope (multi-case) | — | ✓ | — | implemented | min/max per component |
| Equilibrium check | ✓ | ✓ | ✓ | implemented | Fx, Fy, Mz about origin |
| Geometry API | ✓ | ✓ | ✓ | implemented | node pos, connectivity, length |
| Mechanism diagnostics | ✓ | ✓ | ✓ | implemented | rigid-body, rank-deficient |
| Solver selection | ✓ | ✓ | ✓ | implemented | Auto / Named |
| **Factorization reuse** | — | — | — | **NOT available** | re-factorizes per case |
| **Multi-RHS solve** | — | — | — | **NOT available** | trait supports it, API doesn't |
| **Self-weight / body force** | — | — | — | **NOT available** | manual UDL workaround |
| **Temperature load** | — | — | — | **NOT available** | — |
| **3D analysis** | — | — | — | **NOT available** | — |
| **Nonlinear** | — | — | — | **NOT available** | — |
| **Dynamic / modal** | — | — | — | **NOT available** | — |
| **Buckling** | — | — | — | **NOT available** | — |
| Export / Serialization | — | — | — | deferred | Phase 105 |
| Visualization | — | — | — | external | consumes API directly |

---

## 4. Beam / Frame Audit

### 4.1 Stiffness Matrix

Standard 2D Euler-Bernoulli beam element stiffness (`beam_fem.rs:636`):

```text
k_local = [ EA/L        0           0          -EA/L        0           0        ]
          [ 0           12EI/L³     6EI/L²      0           -12EI/L³    6EI/L²   ]
          [ 0           6EI/L²      4EI/L       0           -6EI/L²     2EI/L    ]
          [ -EA/L       0           0           EA/L        0           0        ]
          [ 0          -12EI/L³    -6EI/L²      0           12EI/L³    -6EI/L²   ]
          [ 0           6EI/L²      2EI/L       0           -6EI/L²     4EI/L    ]
```

**Verdict**: ✅ Correct. Standard textbook formulation.

### 4.2 Coordinate Transformation

`T` is the standard 6×6 rotation matrix (`beam_fem.rs:694`): translational
DOFs rotated by `(c, s) = (dx/L, dy/L)`, rotational DOFs unchanged. Global
stiffness: `k_global = Tᵀ · k_local · T`.

**Verdict**: ✅ Correct.

### 4.3 End Releases

Implemented via static condensation of the element local stiffness matrix
(`beam_fem.rs:795` `condense_matrix`). Released rotational DOFs are condensed
out before transformation to global coordinates. The node's rotational DOF
remains in the global system.

**Verdict**: ✅ Correct. Standard FEM approach.

### 4.4 Load Vector Assembly

- Nodal forces: direct addition to global force vector. ✅
- Distributed loads: equivalent nodal forces via `element_equivalent_nodal_forces` (`beam_fem.rs:3586`). ✅
- Trapezoidal loads: linear variation from `(qx_i, qy_i)` to `(qx_j, qy_j)`. ✅
- Point loads: at `xi ∈ [0, 1]`, with `xi = 0` or `xi = 1` acting at a node (not double-counted). ✅
- Applied moments: direct addition to rotational DOF. ✅

**Verdict**: ✅ Correct.

### 4.5 Boundary Conditions

Static condensation: `K_ff u_f = f_f - K_fc u_c`. No penalty methods.
Spring supports add stiffness to the diagonal; the DOF remains free.
Inclined rollers constrain a specified direction `(nx, ny)`.

**Verdict**: ✅ Correct.

### 4.6 Reactions

`R = K_original · u - f_global` (global). Computed on demand.

**Verdict**: ✅ Correct.

### 4.7 Section Forces

`N(x)`, `V(x)`, `M(x)` recovered from element end forces and load integration.
O(1) per query (Phase 102 fix).

**Verdict**: ✅ Correct.

### 4.8 Equilibrium

`compute_equilibrium` (`frame.rs:1402`): sums applied forces (nodal, distributed,
point, moments) and support reactions about the global origin. Conditioning-aware
tolerance with `l_char` scaling.

**Verdict**: ✅ Correct.

---

## 5. Truss Audit

### 5.1 Stiffness Matrix

Standard 2D truss element stiffness (`truss.rs:191`): 4×4 matrix with
`EA/L · [c², cs, -c², -cs; ...]`. Axial only, 2 DOF/node.

**Verdict**: ✅ Correct.

### 5.2 Axial Force

`N = EA/L · (-c·u_i - s·v_i + c·u_j + s·v_j)`, tension positive.

**Verdict**: ✅ Correct.

### 5.3 Mechanism Diagnostics

Reuses `diagnose_reduced` from `mechanism.rs` with 2-DOF rigid-body candidates
(Tx, Ty, Rz about origin).

**Verdict**: ✅ Correct.

---

## 6. Solver Audit

### 6.1 Available Backends

| Backend | Algorithm | Size limit | Multi-RHS | SPD required |
|---------|-----------|-----------|-----------|-------------|
| `dense` | Gaussian elimination + partial pivoting | 500 | ✓ | no |
| `skyline_ldlt` | LDLᵀ with RCM ordering | 10000 | ✓ | yes |
| `sparse_lu` | Dense-backed LU (misnamed) | 3000 | ✓ | no |
| `cg` | Conjugate gradient | unlimited | per-RHS | yes |
| `iccg` | IC(0)-preconditioned CG | unlimited | per-RHS | yes |

### 6.2 Auto-Selection

`SolverSelection::Auto`:
1. `n ≤ 500` → `dense`
2. Symmetric + positive diagonal + within size limit → `skyline_ldlt`
3. Otherwise → `sparse_lu`
4. **Never selects `cg`/`iccg`** (SPD cannot be reliably established from matrix alone)

**Verdict**: ✅ Reasonable. The auto-selection is conservative and correct.

### 6.3 Factorization Reuse

The `LinearSolver` trait supports factorization reuse:
```rust
solver.factor(&matrix)?;     // factor once
let u1 = solver.solve(&rhs1)?;  // solve RHS 1
let u2 = solver.solve(&rhs2)?;  // solve RHS 2 (reuses factor)
```

**However**, the Frame/Beam/Truss API does NOT expose this capability:
- `FrameModel::solve_case(&LoadCase)` creates a new `BeamSolver`, re-assembles,
  re-condenses, and re-factorizes for each call.
- `FrameModel::solve_combination(&LoadCombination)` merges RHS and solves once
  (correct, but still re-factorizes).
- For N load cases, the stiffness matrix (which is identical) is factorized N times.

**Verdict**: ⚠️ P2 performance gap. The infrastructure supports reuse; the API doesn't expose it.

### 6.4 `sparse_lu` Misnomer

The `sparse_lu` backend is actually dense-backed (O(n³), n ≤ 3000). The name
is misleading but documented in the source. This is a `section-properties`
issue, not a `structural-analysis` issue, and is out of scope.

---

## 7. Numerical Stability

### 7.1 Equilibrium Tolerance

`EquilibriumReport` uses a conditioning-aware relative tolerance:
```text
effective_rel_tol = max(1e-6, C · λ²_max · ε)
```
where `λ²_max = max(L²·A/I)` is a dimensionless slenderness proxy. This
prevents false "unbalanced" verdicts for slender frames where solver round-off
is legitimately larger than 1e-6.

**Verdict**: ✅ Sound. Unit-invariant.

### 7.2 `TrussEquilibriumReport` Tolerance

Uses a fixed `1e-6 · Σ|F|` without `l_char` scaling for the moment check.
For structures with large `l_char` and small forces, the moment check may
be too loose.

**Verdict**: ⚠️ P2 (pre-existing, deferred from Phase 104).

### 7.3 Singular Matrix Handling

Failed solves attach a `StructuralDiagnostic` to the error message, classifying
the failure as under-restrained, internally rank-deficient, or ill-conditioned.
The diagnosis reads the solver's own condensed system — no second assembly.

**Verdict**: ✅ Correct. Single-source guarantee.

---

## 8. Result / Post-processing Boundary

Confirmed stable (Phases 102–105):

```text
Model → Solver → AnalysisResult → Post-processing → (future Export)
```

- `FrameAnalysisResult`: owned, not Clone (SparseMatrix). ✅
- `BeamAnalysisResult<'a>`: borrows solver. ✅
- `TrussAnalysisResult`: owned snapshot. ✅
- `Envelope`: pure data, owned, Clone. ✅
- Export/Serialization: DEFER (Phase 105). ✅

**No changes needed.**

---

## 9. Test Coverage

**563 tests** across 34 test files + 10 doctests + 7 examples.

| Category | Test files | Key tests |
|----------|-----------|-----------|
| Analytical benchmarks | `beam_analytical_benchmarks.rs` | cantilever tip load, UDL, propped cantilever |
| Stiffness symmetry | `beam_fem_contract.rs` | k = kᵀ for various elements |
| Rigid-body / mechanism | `mechanism_diagnostics.rs` | free beam, under-restrained, rank-deficient |
| Equilibrium | `frame_correctness.rs`, `postprocessing.rs` | Fx, Fy, Mz residual ≈ 0 |
| End releases | `end_release.rs` | hinge at i, j, both; released moment = 0 |
| Springs | `spring_support.rs` | spring reaction = -k·disp |
| Inclined rollers | `inclined_roller.rs` | constraint direction, orthogonal free |
| Trapezoidal loads | `trapezoidal_load.rs` | linear variation, equivalent nodal forces |
| Load cases/combinations | `load_case.rs` | case solve, combination superposition |
| Truss | `truss.rs` | stiffness, axial force, equilibrium, mechanism |
| Degenerate geometry | `api_misuse.rs` | zero-length, duplicate member, orphan node |
| Invalid input | `api_robustness_audit.rs` | NaN, infinity, negative E/A/I |
| Numerical edge cases | `beam_fem_numerical_audit.rs` | ill-conditioning, extreme slenderness |
| Convergence | `beam_fem_convergence.rs`, `frame_beam_p2_convergence.rs` | mesh refinement, P-Δ |
| API semantics | `beam_fem_api_semantics.rs`, `beam_fem_api_ergonomics.rs` | sign conventions, DOF mapping |
| Post-processing | `postprocessing.rs` | envelope min/max, force diagrams, geometry |
| Solver backends | `solver_selection.rs`, `beam_solver_backends.rs` | auto-select, explicit, cross-validation |

**Verdict**: ✅ Comprehensive. Tests verify mathematical correctness, not just
"doesn't crash". Analytical benchmarks confirm FEM results against closed-form
solutions.

---

## 10. Performance Risks

### 10.1 Factorization Reuse (P2)

```text
Current: solve_case(case_i) → new BeamSolver → assemble + condense + factor + solve
Problem: For N load cases, factorization is repeated N times (matrix is identical)
Potential solution: Expose solve_cases(&[LoadCase]) → factor once + solve_many
Priority: P2
```

This is the **most significant performance gap**. For a 100-DOF frame with 10
load cases, the current approach does 10 factorizations (~10 × O(n³)) instead
of 1 factorization + 10 back-substitutions (~O(n³) + 10 × O(n²)).

The `LinearSolver` trait already supports this via `factor()` + `solve()` /
`solve_many()`. The gap is purely in the Frame/Beam API layer.

### 10.2 Reaction Recomputation (P2)

`FrameAnalysisResult::reactions()` recomputes `K·u - f` on every call.
`BeamAnalysisResult` snapshots reactions at construction. `TrussAnalysisResult`
stores them as an owned field.

```text
Current: FrameAnalysisResult::reactions() → O(n²) per call (matvec)
Problem: Repeated calls recompute the same matrix-vector product
Potential solution: Cache on first call (interior mutability) or snapshot at construction
Priority: P2
```

### 10.3 No Other Architectural Performance Issues

- No O(n²) assembly (element-by-element, O(n_elements × 36)). ✅
- No dense intermediate matrix (uses SparseMatrix). ✅
- No unnecessary cloning (FrameAnalysisResult not Clone by design). ✅
- Envelope sampling: O(cases × members × samples), each O(1). ✅

---

## 11. P0 / P1 / P2 Findings

### P0: None

### P1: None

### P2 (deferred):

| # | Finding | Category | Priority |
|---|---------|----------|----------|
| P2-1 | Factorization not reused across load cases | Performance | High impact |
| P2-2 | `FrameAnalysisResult::reactions()` recomputes per call | Performance | Low impact |
| P2-3 | `TrussEquilibriumReport` moment tolerance lacks `l_char` scaling | Numerical | Pre-existing |
| P2-4 | `sparse_lu` is actually dense-backed | Naming | In section-properties |
| P2-5 | Auto-select never picks CG/ICCG | Feature gap | By design (conservative) |

---

## 12. Candidate Roadmap

### Option A — 3D Structural Analysis

**Verdict: NOT RECOMMENDED next.**

- Complexity: **XL**. Requires 6 DOF/node (3 translational + 3 rotational),
  3D rotation matrices (12×12 element stiffness), 3D geometric nonlinearity
  for frame elements (P-Δ in 3D), 3D support vocabulary.
- Architecture impact: Major. Current 2D formulation is deeply embedded
  (DOF mapping `3*node + dof`, rigid-body candidates, mechanism diagnostics).
- Breaking API change: Yes (new types, new module).
- User value: High, but premature — 2D platform should be feature-complete first.

### Option B — 2D Structural Completeness

**Verdict: PARTIAL.** Self-weight and temperature loads are the most valuable
missing linear static capabilities.

- Self-weight: **S** complexity. `FrameModel::add_self_weight(g)` → applies
  `ρ·g·A` as member UDL. No new math, just convenience.
- Temperature load: **M** complexity. Thermal strain `ε = α·ΔT`, equivalent
  nodal forces. New load type + element vector.
- Tension/compression-only: **L** complexity. Iterative solver, nonlinear.
  Not recommended for linear static platform.

### Option C — Advanced Post-processing

**Verdict: PARTIAL.** Reaction/displacement envelopes are natural follow-ups.

- Reaction envelope: **S** complexity. Extend `Envelope` to track min/max
  reactions across load cases.
- Displacement envelope: **S** complexity. Same pattern.
- Governing case tracking: **M** complexity. Track which load case produced
  each envelope extreme.

### Option D — Solver / Numerical Infrastructure

**Verdict: HIGH PRIORITY.** Factorization reuse is the biggest gap.

- Multi-RHS solve: **M** complexity. `FrameModel::solve_cases(&[LoadCase])`
  that factorizes once and solves all cases. Requires exposing the factorized
  solver or a new `MultiCaseSolver` type.
- No new dependencies, no new crate, no breaking API change (additive methods).

---

## 13. Final Architecture Decision

### No abstraction refactoring needed

- **No `Element` trait**: Beam (3 DOF/node, bending) and Truss (2 DOF/node,
  axial only) have fundamentally different formulations. Unifying would add
  complexity without benefit (confirmed Phase 101).
- **No `Constraint` trait**: Fix/pin/roller/spring/inclined-roller have
  independent mathematical models, no shared code (confirmed Phase 94).
- **No `Result` trait**: Three result types with different ownership models
  (owned/borrowed/snapshot) serve different use cases (confirmed Phase 104).
- **No `Solver` trait in structural-analysis**: `LinearSolver` trait already
  exists in `section-properties::fea::solver` and is sufficient.
- **No third crate**: Current two-crate structure is correct (confirmed Phase 95).

### Recommended Roadmap (3 items)

```text
Phase 107: Multi-load-case solver with factorization reuse
  Why:   Biggest performance gap. Real engineering analysis uses 5-20 load cases.
         Current API re-factorizes for each case (O(N × n³) vs O(n³ + N × n²)).
  What:  FrameModel::solve_cases(&[LoadCase]) → Vec<FrameAnalysisResult>
         Factorize once, solve multiple RHS via LinearSolver::solve_many().
  Impact: Additive API (new methods, no breaking changes). No new deps.
  Complexity: M

Phase 108: Self-weight / body force loading
  Why:   Every structural analysis needs self-weight. Currently manual.
  What:  FrameModel::add_self_weight(g: f64) → applies ρ·g·A as member UDL.
         TrussModel::add_self_weight(g: f64) → applies as nodal loads.
  Impact: Additive API. No new math (uses existing distributed load path).
  Complexity: S

Phase 109: Reaction and displacement envelopes
  Why:   Natural follow-up to Phase 103 section-force envelopes.
         Design checks need worst-case reactions and displacements.
  What:  Envelope::from_frame_results_reactions() / _displacements()
         or extend Envelope with reaction/displacement tracking.
  Impact: Additive API in postprocessing.rs. No breaking changes.
  Complexity: S
```

---

## 14. Verification

```text
HEAD:     57b1983 Phase 105: define structural export boundary
working tree: clean (only untracked PHASE*.md docs)
fmt:      PASS
check:    PASS (pre-existing pardiso warnings)
tests:    563 tests PASS (structural-analysis)
doctests: 10 PASS
doc:      0 warnings
P0:       0
P1:       0
P2:       5 (deferred)
roadmap:  Phase 107 (M) → Phase 108 (S) → Phase 109 (S)
commit:   audit-only, no production code changes
```

---

## 15. Summary

The `structural-analysis` crate is a **correct, well-tested, architecturally
clean** 2D linear static analysis platform. 563 tests verify mathematical
correctness against analytical benchmarks. The four-layer architecture
(Model → Solver → Result → Post-processing) is stable. No abstraction
refactoring is needed.

The most impactful next step is **Phase 107: multi-load-case solver with
factorization reuse** — it addresses the single biggest performance gap
(re-factorization per load case) using infrastructure that already exists
(`LinearSolver::factor()` + `solve()`), with an additive API and no breaking
changes.
