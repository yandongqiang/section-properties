# Phase 94 — Constraint / Analysis Architecture Audit

## 1. Architecture Map

### 1.1 Layer Structure

```
┌─────────────────────────────────────────────────────────────┐
│  Layer 4: Frame Façade                                      │
│  FrameModel ──► FrameSolver ──► FrameAnalysisResult         │
│       │              │                      │               │
│       │         validate()             typed accessors      │
│       │         diagnose()            (displacement,        │
│       │                              reaction, forces,      │
│       │                              equilibrium)          │
├───────┼──────────────────────────────────────────────────────┤
│  Layer 3: Load                                              │
│  LoadCase ──► LoadCombination                               │
│    nodal_forces    terms: Vec<(LoadCase, f64)>              │
│    distributed_loads                                        │
│    point_loads                                              │
│    applied_moments                                          │
├─────────────────────────────────────────────────────────────┤
│  Layer 2: Solver                                            │
│  BeamSolver                                                 │
│    from_model()     : BeamModel → assemble K, f, constraints │
│    apply_boundary_conditions() : static condensation         │
│    factor_and_expand()        : solve K_ff·u_f = f_reduced   │
│    transform_back_to_global() : inverse rotation for rollers │
│    reactions()                : R = K_original·u - f         │
│    element_end_forces()       : f_equiv - K_local·u_local    │
│    element_section_forces()   : N(x), V(x), M(x)             │
├─────────────────────────────────────────────────────────────┤
│  Layer 1: Model                                             │
│  BeamModel                                                  │
│    nodes, elements                                          │
│    nodal_forces, distributed_loads, point_loads,             │
│    applied_moments                                          │
│    fixed_dofs, spring_supports, inclined_rollers             │
├─────────────────────────────────────────────────────────────┤
│  Layer 5: Diagnostics (mechanism.rs)                        │
│  diagnose_reduced(K_ff) → Stable | RigidBodyMode |          │
│                           Mechanism | IllConditioned |       │
│                           Indeterminate                      │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 Call Graph (solve path)

```
FrameModel::solve()
  └─► FrameSolver::new(self).solve()
        ├─► FrameModel::validate()           [orphan nodes, connectivity]
        ├─► BeamSolver::from_model(&inner)   [assemble K, f, constraints]
        │     ├─► assemble element stiffness (released_global_stiffness)
        │     ├─► assemble_global_load_vector (trapezoidal + point + moment)
        │     ├─► process fixed_dofs
        │     ├─► process inclined_rollers (coordinate rotation)
        │     ├─► k_original = K_global.clone()
        │     └─► add spring stiffness to K_global diagonal
        ├─► BeamSolver::solve_configured()
        │     ├─► apply_boundary_conditions()  [static condensation]
        │     │     └─► retain ReducedSystem
        │     ├─► SolverRegistry::create_selected()
        │     └─► factor_and_expand()
        │           ├─► solver.factor(&K_ff)
        │           ├─► solver.solve(&f_reduced)
        │           └─► transform_back_to_global()
        └─► FrameAnalysisResult { beam, model, load_source }

FrameModel::solve_case(case)
  └─► inner_with_loads(case.loads)  [clone inner, replace loads]
  └─► BeamSolver::from_model(merged)
  └─► solve_configured()

FrameModel::solve_combination(combo)
  └─► merge: f = Σ factor_i × f_case_i  [single RHS, single factorize]
  └─► BeamSolver::from_model(merged)
  └─► solve_configured()
```

### 1.3 Source File Map

| File | Lines | Responsibility |
|------|-------|----------------|
| `beam_fem.rs` | 4249 | Core FEM: BeamModel, BeamSolver, BeamElement, EndRelease, loads, assembly, solve, recovery |
| `frame.rs` | 2010 | Frame façade: FrameModel, FrameSolver, FrameAnalysisResult, EquilibriumReport |
| `load.rs` | 307 | LoadCase, LoadCombination |
| `mechanism.rs` | 578 | Structural diagnostics: diagnose_reduced, Cholesky probe |
| `lib.rs` | 45 | Module declarations and re-exports |

### 1.4 Test File Map (32 files)

| Category | Files |
|----------|-------|
| Core FEM | beam_fem, beam_fem_api_*, beam_fem_contract, beam_fem_convergence, beam_fem_numerical_audit, beam_fem_reference, beam_fem_result_ergonomics, beam_fem_robustness, beam_fem_public_api_smoke |
| Analytical | beam_analytical_benchmarks, beam_section_forces, beam_force_diagrams |
| Solver | beam_solver_backends, beam_multisolver_cross_validation, solver_robustness, solver_selection |
| Frame | frame_api, frame_beam_integration, frame_beam_p2_convergence, frame_correctness, frame_solver_selection, frame_transformation_contract |
| Constraints | end_release, spring_support, inclined_roller |
| Loads | load_case, trapezoidal_load |
| Diagnostics | mechanism_diagnostics |
| API | api_misuse, api_robustness_audit, beam_analysis_result, beam_fem_api_ergonomics |

## 2. Constraint Taxonomy

| Constraint | Math Model | Implementation | Applied At | Layer |
|------------|-----------|----------------|------------|-------|
| Fixed DOF | u_i = v_prescribed | Direct elimination via static condensation | `from_model` → `apply_boundary_conditions` | Solver |
| Roller X | u_i = 0 | Same as Fixed (DOF 0) | Same | Solver |
| Roller Y | v_i = 0 | Same as Fixed (DOF 1) | Same | Solver |
| Inclined Roller | n·u = v_prescribed | Coordinate rotation → single DOF elimination | `from_model` (pre-assembly) | Solver |
| Spring | F_i = -k·u_i | Add k to K_global diagonal (after k_original clone) | `from_model` (post-rotation) | Solver |
| End Release | M_i = 0 | Element-level static condensation of K_local | `released_global_stiffness` | Element |

### 2.1 Constraint Semantic Audit

**Fixed/Roller (A)**: Clean. `fixed_dofs: Vec<(node_idx, dof, value)>` in BeamModel. `from_model` marks `fixed_dofs[idx] = true` and sets `prescribed_values[idx]`. `apply_boundary_conditions` partitions free/constrained and condenses. No ambiguity.

**Inclined Roller (B)**: Clean. `inclined_rollers: Vec<(node_idx, nx, ny, pres_val)>` in BeamModel. `from_model` applies 2×2 rotation R = [[c,s],[-s,c]] to (Ux,Uy) → (U_n,U_t), marks U_n as fixed. `transform_back_to_global` applies R^T after solve. One roller per node enforced. Direction auto-normalized. Zero/NaN direction rejected.

**Spring (C)**: Clean. `spring_supports: Vec<(node_idx, dof, stiffness)>` in BeamModel. `from_model` adds stiffness to K_global diagonal AFTER k_original clone, so spring reactions computed as residual: R_spring = K_original·u - f (which includes -k·u from the spring). Stiffness must be finite and positive (zero rejected — spring with k=0 is no spring, not a free DOF).

**End Release (D)**: Clean. `EndRelease { start_rotation, end_rotation }` on BeamElement. Applied via `condense_matrix` (element-level static condensation) in `released_global_stiffness`. Released DOF displacements recovered in `released_local_displacement` via u_r = K_rr⁻¹(f_r - K_rc·u_c). End release is a MEMBER property, not a NODE constraint — correctly separated.

### 2.2 Key Distinction: Node Constraint vs Member Release

- **Node constraints** (Fixed/Roller/Inclined/Spring) act on NODE DOFs in the GLOBAL system.
- **Member end releases** act on MEMBER-END DOFs in the LOCAL element system.
- These are correctly separated: node constraints in BeamModel, end releases in BeamElement.
- A node can have a fixed DOF AND be an end of a released member — no conflict.

## 3. Constraint Layer Abstraction Decision

**Decision: No Constraint abstraction needed.**

**Evidence**:
1. Each constraint type has a distinct mathematical model (direct elimination, coordinate rotation, diagonal addition, element condensation). A generic `Constraint` enum would add indirection without reducing code.
2. The current implementation has no duplicated constraint logic — each type is handled exactly once in `from_model`.
3. The constraint types are fixed and small (4 types). No extensibility requirement.
4. A generic `C·u = 0` linear constraint is not needed — current types cover all practical 2D beam/frame support conditions.

**Consequence**: No refactoring. The current direct handling in `from_model` is the simplest correct design.

## 4. Solver / Assembly / Recovery Boundary Audit

### 4.1 Solver Boundary
- `BeamSolver` does NOT know about `LoadCase` or `LoadCombination` — those are handled by `FrameModel`.
- `BeamSolver` only sees `BeamModel` with raw load vectors (nodal_forces, distributed_loads, etc.).
- Solver selection (`SolverSelection`) is the only configurable parameter.
- **Verdict: Clean.**

### 4.2 Assembly Boundary
- Single assembly path: `assemble_global_load_vector` → `released_consistent_nodal_load_trapezoidal` → `consistent_nodal_load_trapezoidal`.
- No duplicate equivalent nodal force implementation.
- Trapezoidal is the general case; uniform delegates to trapezoidal with qx_end=qx, qy_end=qy.
- **Verdict: Clean.**

### 4.3 Recovery Boundary
- `reactions()`: R = K_original · u - f_global. Standard, correct.
- `element_end_forces()`: f_end = f_equiv - K_local · u_local (with end release recovery). Correct.
- `element_section_forces()`: N(x), V(x), M(x) from equilibrium with trapezoidal formulas. Correct.
- **Verdict: Clean.**

## 5. Reaction / AnalysisResult / Load Architecture Audit

### 5.1 Reaction Architecture
- k_original is cloned AFTER inclined roller rotation but BEFORE spring addition.
- This ensures: (a) reactions in rotated frame are correct, (b) spring reactions = -k·u appear as residual.
- `transform_back_to_global` transforms k_original back to global frame for correct reaction extraction.
- **Verdict: Clean.**

### 5.2 AnalysisResult Architecture
- `FrameAnalysisResult` wraps `BeamSolver` + `FrameModel` without recomputation.
- All accessors delegate to BeamSolver methods.
- `load_source` tracks provenance (None / "case:{name}" / "combination:{name}").
- **Verdict: Clean.**

### 5.3 Load Architecture
- `LoadCase`: owns loads, provides `pub(crate)` read access for assembly.
- `LoadCombination`: owns `(LoadCase, factor)` pairs, provides `terms()` for merging.
- `solve_case`: clones inner model, replaces loads from case. Preserves geometry/constraints/springs/rollers.
- `solve_combination`: merges RHS (single factorize). **BUG FOUND: trapezoidal variation was lost** (P0, fixed).
- **Verdict: Clean after P0 fix.**

## 6. API / Test / Numerical Stability Audit

### 6.1 API Audit
- All new APIs (spring, inclined_roller, trapezoidal) return `Result<(), FemError>`.
- Public path does not use `unwrap()`/`expect()` (panicking variants exist but delegate to fallible).
- `Dof` enum provides compile-time safety for DOF access.
- `NodeHandle`/`MemberHandle` provide typed indices.
- **Verdict: Clean.**

### 6.2 Test Audit
- 493 tests (491 existing + 2 new), 0 failures.
- Constraint tests: end_release (19), spring_support (9), inclined_roller (9).
- Load tests: load_case (25), trapezoidal_load (11).
- No `#[ignore]` tests. No CI timeout规避.
- **Verdict: Clean.**

### 6.3 Numerical Stability
- Spring stiffness: finite and positive (zero rejected, infinity rejected, NaN rejected).
- Inclined roller direction: non-zero and finite (auto-normalized).
- Trapezoidal load: all four components validated finite in `from_model` (P1 fix).
- Static condensation: exact (no penalty method).
- **Verdict: Clean after P1 fix.**

## 7. Bugs Found and Fixed

### P0: `solve_combination` drops trapezoidal load variation
- **Location**: `frame.rs:830-835` (pre-fix)
- **Problem**: `DistributedLoad::new()` was used to merge loads, which defaults to uniform (qx_end=qx, qy_end=qy). Trapezoidal variation (qx_end, qy_end) was silently lost.
- **Impact**: `solve_combination` produced incorrect results for any LoadCase containing trapezoidal loads.
- **Fix**: Changed to `DistributedLoad::trapezoidal()` with scaled qx_end and qy_end.
- **Test**: `trapezoidal_load_in_combination_preserves_variation`

### P1: `equilibrium()` ignores trapezoidal variation
- **Location**: `frame.rs:1355-1362` (pre-fix)
- **Problem**: Computed distributed load resultant as `qx * len` (uniform formula) and centroid at midpoint. For trapezoidal loads, resultant = (q_start + q_end)/2 * L and centroid = L*(q_start + 2*q_end)/(3*(q_start + q_end)).
- **Impact**: `EquilibriumReport` was incorrect for trapezoidal loads — false imbalance reported.
- **Fix**: Use trapezoidal resultant and centroid formula. Axial force moment is centroid-independent (acts along member); transverse force moment uses correct centroid.
- **Test**: `trapezoidal_load_equilibrium_balanced`

### P1: `from_model` doesn't validate `qx_end` / `qy_end`
- **Location**: `beam_fem.rs:2445-2452` (pre-fix)
- **Problem**: Only validated `dl.qx` and `dl.qy` finite, not `dl.qx_end` and `dl.qy_end`. Since `DistributedLoad` fields are public, a struct literal with `qx_end = NaN` would pass validation.
- **Impact**: NaN propagation through the solve.
- **Fix**: Added `!dl.qx_end.is_finite() || !dl.qy_end.is_finite()` to validation.

## 8. Architecture Decisions

| # | Question | Decision | Evidence | Consequence |
|---|----------|----------|----------|-------------|
| 1 | Constraint Layer abstraction needed? | **No** | 4 distinct math models, no duplication, fixed small set | No refactoring |
| 2 | General Linear Constraint (C·u=0) needed? | **No** | Current types cover all practical 2D supports | No over-engineering |
| 3 | Solver boundary clean? | **Yes** | BeamSolver doesn't know LoadCase/LoadCombination | No change |
| 4 | Assembly boundary clean? | **Yes** | Single path, no duplicate implementation | No change |
| 5 | Reaction architecture correct? | **Yes** | k_original cloned at right point, transform_back correct | No change |
| 6 | AnalysisResult clean? | **Yes** | Wraps BeamSolver without recomputation | No change |
| 7 | Load architecture clean? | **Yes after P0 fix** | solve_combination now preserves trapezoidal | P0 fix applied |
| 8 | EndRelease regression? | **Yes** | All 493 tests pass | No change |
| 9 | API clean? | **Yes** | All new APIs return Result, no unwrap in public path | No change |
| 10 | Ready for Truss? | **Not this phase** | P0 bug fixed; Truss is a future structure-type decision | Out of scope |

## 9. Files Modified

| File | Change |
|------|--------|
| `src/frame.rs` | P0: `DistributedLoad::trapezoidal` in `solve_combination`; P1: trapezoidal resultant/centroid in `equilibrium` |
| `src/beam_fem.rs` | P1: validate `qx_end`/`qy_end` finite in `from_model` |
| `tests/trapezoidal_load.rs` | +2 tests: combination preserves variation, equilibrium balanced |

## 10. Conclusion

The architecture is clean and well-separated. The 4 constraint types (Fixed, Inclined Roller, Spring, End Release) have distinct mathematical models and are correctly handled at different layers (solver for node constraints, element for member releases). No generic Constraint abstraction is needed.

Three bugs were found and fixed: one P0 (solve_combination dropping trapezoidal variation) and two P1 (equilibrium ignoring trapezoidal, missing qx_end/qy_end validation). All 493 tests pass.
