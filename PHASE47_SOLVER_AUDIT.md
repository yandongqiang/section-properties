# Phase 47 — Solver Capability & Availability Contract Audit

**Date**: 2026-09-20
**Baseline**: `e429fbb` ("Fix IPE300 J verification reference to filleted geometry")
**Verdict**: **PASS** — P0 = 0, P1 = 0. All solver capability and availability contracts are satisfied with code and test evidence. Three P2 observations (documentation / dead-code) recorded; no production code changed.

---

## 1. Solver Inventory & Capability Matrix

Every entry below is backed by code evidence (capability constructor + factor/solve implementation).

| Solver | SPD req | Symmetric req | General | Sparse | Multi-RHS | max_size | Availability |
|--------|---------|---------------|---------|--------|-----------|----------|--------------|
| `dense` | no | no | yes | no (converts to dense) | yes | 500 | built-in, default-registered |
| `skyline_ldlt` | yes (verified at factor) | yes (checked) | no | yes (skyline) | yes | 10000 | built-in, default-registered |
| `sparse_lu` | no | no | yes | yes (converts to dense) | yes | 3000 | built-in, default-registered |
| `cg` | yes (verified at solve) | yes (checked) | no | yes | no | none | built-in, default-registered |
| `iccg` | yes (verified at solve) | yes (checked) | no | yes | no | none | built-in, default-registered |
| `pardiso` | no | no | yes | yes | yes | none | **not registered** (feature-gated stub; requires explicit `register`) |

**Evidence**:
- Capability constructors: `src/fea/solver.rs:20-115` (`SolverCapabilities` per solver).
- Default registry: `SolverRegistry::default()` registers dense, skyline_ldlt, sparse_lu, cg, iccg. PARDISO `PardisoSolverFactory` is `#[cfg(feature = "pardiso")]` but **not** registered by default.
- Test: `tests/solver_selection.rs:test_pardiso_not_available_by_default` asserts `pardiso` is absent from `registry.list()` and that `create_selected` returns `SolverError::Unsupported`.

---

## 2. Per-Solver Implementation Audit

### 2.1 Dense (`src/fea/solver/impls/dense_gaussian.rs`, 166 lines)
- True partial pivoting with scale-aware pivot tolerance (`PIVOT_TOL_BASE * scale`).
- Singular → `SolverError::SingularMatrix`; on failure, old factorization is cleared (no stale state).
- No `unwrap`/`expect` swallowing errors.
- **Regression tests**: `tests/solver_architecture_correctness.rs:test_dense_gaussian_forces_pivot_2x2` and `test_dense_gaussian_forces_multiple_pivots_3x3` (regression for commit `a8b535e`).

### 2.2 SkylineLdlt (`src/fea/solver/impls/skyline_ldlt.rs` 90-line adapter + `src/fea.rs:1895-2053` impl)
- Symmetry check (`is_symmetric(1e-12)`) → `NotSymmetric`.
- In-place Crout LDL^T; `d > 0.0` check → non-SPD returns `FemError::SingularMatrix`.
- **Never returns a seemingly-successful but wrong result on non-SPD input.**
- Scale-aware pivot tolerance.
- **Test**: `tests/solver_architecture_correctness.rs:test_symmetric_indefinite_factorization_fails_on_skyline`.

### 2.3 SparseLu (`src/fea/solver/impls/sparse_lu.rs` 78-line wrapper + `src/fea/solvers.rs:256-527` impl)
- True row partial pivoting (PA = LU).
- Per-column scale-aware pivot tolerance (`EPS * n * col_scale * 100`).
- Zero-column / singular detection is explicit.
- General matrix support.
- **Tests**: `tests/sparse_lu.rs` (17 tests: zero-diagonal, must-pivot, multiple-pivots, PA=LU verification, singular, near-singular, scale-invariance, RHS scaling, etc.).

### 2.4 CG (`src/fea/solver/impls/cg.rs`, 149 lines)
- Symmetry check → `NotSymmetric`.
- `p^T A p <= 0` → `SingularMatrix` (detects non-SPD at solve time).
- Not converged → `NotConverged`.
- **P2-DOC-02**: The `factor()` method (L56-72) has a comment claiming "positive definiteness (diagonal dominance heuristic)" check, but the loop body is empty. The comment is misleading; however, the `solve()` method correctly catches non-SPD via `p^T A p <= 0`, so runtime behavior is correct.
- **Test**: `tests/solver_architecture_correctness.rs:test_cg_factor_fails_on_indefinite`.

### 2.5 ICCG (`src/fea/solver/impls/iccg.rs`, 275 lines)
- Symmetry check → `NotSymmetric`.
- IC(0) factorization; `sum <= 0.0` triggers modified IC (`l_diag = 1e-12`) rather than rejection.
- `p^T A p <= 0` → `SingularMatrix`; not converged → `NotConverged`.
- **P2 observation**: Modified IC for severely indefinite matrices may produce a numerically ill-conditioned preconditioner, but Auto never selects ICCG; only explicit requests use it.
- **Test**: `tests/solver_robustness.rs:test_cg_iccg_on_spd_system` and `test_iterative_report_failure_on_singular`.

### 2.6 PARDISO (`src/fea/solver/impls/pardiso_wrapper.rs`, 64 lines)
- Feature enabled → `SolverError::unsupported("PARDISO general solver not yet implemented...")`.
- Feature disabled → `SolverError::backend_not_available("PARDISO feature not enabled")`.
- **Never silently falls back to SparseLu.**
- `PardisoSolverFactory` only compiles under `feature = "pardiso"`; `SolverRegistry::default()` does not register it.
- **Test**: `tests/solver_selection.rs:test_pardiso_not_available_by_default` and `tests/solver_architecture_correctness.rs:test_pardiso_stub_not_auto_selected`.

---

## 3. Auto Selection Audit (`src/fea/solver.rs:550-601`)

`auto_select_info` policy (in order):
1. `n <= 500` and dense registered → **dense** (`SmallSystem`).
2. `is_symmetric(1e-12)` + `has_positive_diagonal` and skyline fits → **skyline_ldlt** (`SymmetricPositiveDiagonal`).
3. (`feature = "pardiso"` and pardiso registered) → **pardiso** — never fires for default registry.
4. sparse_lu registered → **sparse_lu** (`GeneralSystem`).
5. Otherwise → error.

| Question | Answer | Evidence |
|----------|--------|----------|
| Q1: Does Auto ever select cg/iccg? | **No.** No cg/iccg branch in `auto_select_info`. Docs (L544, L548-549) explicitly state this. | `tests/solver_selection.rs:test_auto_selection_decisions` §6; `tests/solver_architecture_correctness.rs:test_auto_select_*_not_cg` |
| Q2: How is SPD detected? | `is_symmetric(1e-12)` + `has_positive_diagonal` (necessary, not sufficient — conservative screen). | `solver.rs:568` |
| Q3: What if SPD detection fails? | Falls through to `sparse_lu` (general, safe). | `tests/solver_selection.rs:test_auto_selection_decisions` §3 |
| Q4: Does Auto fallback retry on failure? | **No.** `beam_fem.rs:solve_configured` calls `create_selected` once; `factor_and_expand` has no retry. Error propagates directly. | See P2-DOC-01 below. |
| Q5: Does Named selection fallback? | **No.** `create_selected` → `validate_selection` → error returned directly. | `tests/solver_selection.rs:test_manual_selection_rejects_incompatible_matrix`; `tests/frame_solver_selection.rs:unavailable_solver_returns_error` |

---

## 4. Error Propagation Contract

### 4.1 `SolverError` (15+ variants)
`src/fea/solver.rs`: `InvalidInput`, `NonFiniteInput`, `DimensionMismatch`, `SingularMatrix`, `NearSingularMatrix`, `FactorizationFailed`, `SolveFailed`, `BackendUnavailable`, `BackendError`, `Unsupported`, `ConvergenceFailed`, `BackendNotAvailable`, `NotImplemented`, `NotSymmetric`, `NotFactorized`, `NotConverged`.

### 4.2 `FemError::SolverError` (`crates/structural-analysis/src/beam_fem.rs:2988-3036`)
```rust
pub enum FemError {
    SolverError { source: SolverError, message: String },
    // ... structural variants
}
```
- `From<SolverError> for FemError` preserves the structured `source` and creates a human-readable `message`.
- `FemError::solver_error()` accessor returns `Option<&SolverError>`.
- `FrameSolver::solve` failure path (`with_structural_diagnosis`) retains the `SolverError` variant; it only appends diagnosis to `message`, never flattens to a non-solver variant.
- `BeamSolver::solve_configured`: `solver_name = None` is set before solving; on success, the name is set. On failure, no stale name remains.
- **Test**: `tests/solver_selection.rs:test_solver_name_invalidated_on_failed_solve`; `tests/frame_solver_selection.rs:unavailable_solver_returns_error` (asserts `err.solver_error().is_some()`).

### 4.3 No silent fallback
- Unknown solver name → `SolverError::Unsupported` (never substitutes another backend).
- Capability mismatch → `SolverError::Unsupported`.
- `max_size` violation → `SolverError::Unsupported` with descriptive message.
- **Tests**: `tests/solver_selection.rs:test_manual_selection_rejects_incompatible_matrix`, `test_max_size_validation`, `test_pardiso_not_available_by_default`.

---

## 5. Test Coverage Audit

| Test file | Tests | Coverage |
|-----------|-------|----------|
| `tests/solver_architecture_correctness.rs` | 13 | Auto not CG/ICCG, small→dense, PARDISO not auto-selected, solve-before-factor, solve_many consistency, skyline rejects indefinite, CG fails on indefinite, dense pivot regressions, failed-refactor invalidation |
| `tests/solver_selection.rs` | 10 | Manual selection per backend, reject incompatible, auto decisions, singular refactor invalidation, beam cross-backend consistency, CG on beam, solver_name observability, max_size validation, PARDISO not default, solver_name invalidated on failure |
| `tests/solver_robustness.rs` | 10 | Cross-solver consistency (SPD/indefinite/non-sym), uniform scaling, singular/rank-deficient/near-singular, iterative failure on singular, lifecycle, beam backends consistency, CG/ICCG on SPD |
| `tests/frame_solver_selection.rs` | 7 | Explicit solver respected, auto reports name, numerical equivalence, convenience constructors, unavailable→error, builder pattern, default is auto |
| `tests/sparse_lu.rs` | 17 | SparseLu pivoting, PA=LU, singular, near-singular, scale-invariance, RHS scaling |

**Total solver-contract tests**: 57 — all pass in debug and release.

### Contract → Test mapping

| Contract | Test evidence |
|----------|---------------|
| No silent fallback (Named) | `test_manual_selection_rejects_incompatible_matrix`, `unavailable_solver_returns_error` |
| Auto never picks CG/ICCG | `test_auto_select_*_not_cg`, `test_auto_selection_decisions` §6 |
| PARDISO not in default registry | `test_pardiso_not_available_by_default`, `test_pardiso_stub_not_auto_selected` |
| max_size enforced | `test_max_size_validation` |
| Stale factorization invalidated | `test_failed_refactor_invalidates_previous_factorization`, `test_singular_refactor_invalidates_previous_factorization`, `test_lifecycle_invalid_factor_clears_state` |
| solve() before factor() → NotFactorized | `test_solve_before_factor_returns_not_factorized` |
| Cross-solver numerical equivalence | `test_cross_solver_consistency_*`, `test_beam_backends_consistency`, `test_beam_manual_solver_consistency`, `all_solvers_numerically_equivalent` |
| Singular matrix → error (not garbage) | `test_singular_and_rank_deficient_matrices`, `test_iterative_report_failure_on_singular` |
| solver_name observability | `test_beam_reports_selected_solver`, `explicit_solver_is_respected`, `auto_solver_reports_name` |
| solver_name cleared on failure | `test_solver_name_invalidated_on_failed_solve` |

---

## 6. Numerical Consistency

Cross-solver agreement verified at two levels:

1. **Matrix level** (`tests/solver_robustness.rs`): Dense, SkylineLdlt, SparseLu produce identical solutions (rel err < 1e-9) on SPD, symmetric-indefinite, and non-symmetric systems. Uniform scaling `A → αA` correctly yields `x → x/α` for α ∈ {1e-12, 1e-6, 1, 1e6, 1e12}.

2. **Beam FEM level** (`tests/solver_selection.rs`, `tests/solver_robustness.rs`): Cantilever tip-force and UDL cases solved with dense/skyline_ldlt/sparse_lu agree to rel err < 1e-9 on displacement, rotation, reactions, section forces, and strain energy. Analytical checks (uy = PL³/3EI, rz = PL²/2EI, U = P²L³/6EI) pass at rel err < 1e-9.

3. **Frame level** (`tests/frame_solver_selection.rs`): Portal frame solved with dense/skyline_ldlt/sparse_lu agrees with Auto at rel err < 1e-10 on displacements and < 1e-8 on reactions. Equilibrium balanced for all backends.

---

## 7. P2 Observations (non-blocking)

### P2-DOC-01: Auto fallback documentation misleading
**Location**: `crates/structural-analysis/src/frame.rs:534-535`, `crates/structural-analysis/src/frame.rs:547-549`
**Claim**: "Auto may fall back to a different solver if the first choice fails" / "internal selection/fallback policy; the registry may substitute a different backend if the initial choice fails."
**Reality**: `beam_fem.rs:solve_configured` calls `create_selected` exactly once; `factor_and_expand` has no fallback retry logic. Auto selection picks one solver; if it fails, the error propagates directly.
**Impact**: The actual behavior is *safer* than documented (errors are never masked by retrying), but the documentation may mislead callers into expecting retry behavior that does not exist.
**Note**: `src/fea/solver.rs:318-319` correctly describes Auto as "the registry analyses the matrix and picks a backend" without claiming fallback. The inconsistency is isolated to `frame.rs`.

### P2-DEAD-01: `src/fea/solvers_impl.rs` is dead code
**Location**: `src/fea/solvers_impl.rs` (518 lines)
**Issue**: Defines duplicate `DenseGaussianSolver`, `SkylineLdltSolver`, `SparseLuSolver`, `CgSolver`, `IccgSolver`, `PardisoSolverWrapper` implementations, but `src/fea.rs` only declares `pub mod solver; pub mod solvers;` — there is no `mod solvers_impl;`. The file is completely uncompiled.
**Impact**: No runtime impact (dead code). May cause confusion for future maintainers.

### P2-DOC-02: CG factor SPD-check comment is misleading
**Location**: `src/fea/solver/impls/cg.rs:56-72`
**Issue**: The `factor()` method contains a comment claiming "positive definiteness (diagonal dominance heuristic)" check, but the loop body is empty — no check is performed.
**Impact**: None at runtime. The `solve()` method correctly detects non-SPD via `p^T A p <= 0` → `SingularMatrix`. The comment is documentation-only noise.

---

## 8. Regression Test Results

All tests run with `CARGO_INCREMENTAL=0 RUST_MIN_STACK=2147483648`.

| Check | Result |
|-------|--------|
| `cargo fmt --check` | PASS |
| `cargo check --all-features` | PASS (3 existing pardiso warnings, non-regression) |
| `cargo test --doc` | PASS (8/8) |
| `cargo test --lib beam_fem` | PASS (7/7) |
| `cargo test --test solver_architecture_correctness` | PASS (13/13) |
| `cargo test --test solver_selection` | PASS (10/10) |
| `cargo test --test solver_robustness` | PASS (10/10) |
| `cargo test --test frame_solver_selection` | PASS (7/7) |
| `cargo test --test frame_api` | PASS (25/25) |
| `cargo test --test frame_correctness` | PASS (see commit) |
| `cargo test --test frame_transformation_contract` | PASS (5/5) |
| `cargo test --test frame_beam_integration` | PASS (see commit) |
| `cargo test --test frame_beam_p2_convergence` | PASS (18/18) |
| `cargo test --test mechanism_diagnostics` | PASS (11/11) |
| `cargo test --test sparse_lu` | PASS (17/17) |
| `cargo test --release` (solver + frame_solver_selection suites) | PASS (all) |

---

## 9. Scope Compliance

- **No production code changed.** Zero modifications to `src/`.
- **No solver rewritten, no new solver added, no tolerance changed.**
- **No `#[ignore]` added, no assertion weakened, no CI timeout changed.**
- **No new dependencies introduced.**
- **Warping FEM untouched** (frozen at Phase 24, commit `2c4c8fa`).
- **PARDISO ABI/linkage untouched.**
- **`global_matrix_exports/` JSON untouched** (no test generated new exports).

---

## 10. Conclusion

The solver capability and availability contracts are **fully satisfied**:

1. Every solver declares accurate `SolverCapabilities` (SPD/symmetric/general/sparse/multi-RHS/max_size).
2. `SolverRegistry::default()` registers exactly {dense, skyline_ldlt, sparse_lu, cg, iccg}; PARDISO is intentionally absent.
3. Auto selection never picks CG/ICCG (SPD cannot be established from matrix alone) and never claims PARDISO from the default registry.
4. Named selection is authoritative — no silent fallback, no substitution. Capability violations return structured `SolverError::Unsupported`.
5. Error propagation preserves the structured `SolverError` through `FemError::SolverError { source, message }`; `solver_name` is cleared on failure (no stale claims).
6. Cross-solver numerical equivalence is verified at matrix, beam, and frame levels.
7. Factorization lifecycle is correct: failed `factor()` invalidates stale state; `solve()` before `factor()` returns `NotFactorized`.

**P0 = 0, P1 = 0 → PASS.**
