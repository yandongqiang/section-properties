# Changelog

## Unreleased

## 0.4.0 - 2026-09-22

### Breaking changes

- **Workspace split.** `beam_fem`, `frame`, and `fea::mechanism` modules have
  been moved to the new `structural-analysis` crate. `section-properties` now
  focuses on cross-section properties, geometry, materials, and numerical
  infrastructure (FEA solvers, mesh, warping FEM). Structural analysis (beam
  FEM, frame FEM, mechanism diagnostics) is in `structural-analysis` 0.1.0,
  which depends on `section-properties`.
  - `section_properties::beam_fem` → `structural_analysis::beam_fem`
  - `section_properties::frame` → `structural_analysis::frame`
  - `section_properties::fea::mechanism` → `structural_analysis::mechanism`
  - Top-level re-exports `BeamModel`, `BeamSolver`, `FrameModel`, `Dof`,
    `FemError`, etc. have moved to `structural_analysis`.
  - Solver infrastructure (`SolverSelection`, `SolverError`, `SolverRegistry`,
    `fea::solver`, `fea::solvers`) remains in `section-properties`.

## 0.3.0 - 2026-09-22

### Breaking changes

- **Removed `SolverError::BackendNotAvailable` variant.** This was a
  near-duplicate of `BackendUnavailable` with no semantic distinction.
  All call sites now use `BackendUnavailable`.
- **Privatized `EquilibriumReport::tolerance` field.** The field name
  suggested a general tolerance but it only holds the **force** tolerance.
  Use `force_tolerance()` or `moment_tolerance()` instead.

### Non-breaking changes

- Added `Default` impl for `BeamModel` (equivalent to `new()`).
- Fixed tautological tests: `rust_on_python_mesh` now asserts FEM success
  and finite results; `boolean_regression` tautological assertion removed;
  `global_matrix_comparison` now asserts output file is non-empty.
- Fixed NaN-accepting assertions in `global_matrix_cross_validation`:
  NaN/Infinity in relative error now fails the assertion.
- Tightened `warping_fem_scale_invariance` tolerance from 100% to 15%
  (observed max error ~6.6%); FEM failures now panic instead of silently
  skipping.
- Documentation: added `# Errors` sections to `BeamSolver::solve`,
  `FrameModel::solve`, `FrameModel::solve_with`; fixed README PCG→CG
  terminology; added Rust 1.85+ requirement to README.
- Fixed stale documentation: `BeamElement::transformation_matrix` doc
  direction (global→local, not local→global); README quick start missing
  `ParametricSection` trait import; `ElasticComposite::new` doc claimed
  validation that was not implemented.

## 0.2.0

### Breaking changes

- **Removed `FemError::SingularMatrix(String)` variant.** This variant was never
  constructed (0 call sites). Singular-matrix errors now arrive as
  `FemError::SolverError { source, message }` where `source` is a
  `SolverError::SingularMatrix` or `SolverError::NearSingularMatrix`.

- **Unified `SolverError`.** The duplicate `fea::solvers::SolverError` enum has
  been removed; `fea::solvers` now uses the canonical
  `fea::solver::SolverError` internally. If you imported `fea::solvers::SolverError`,
  switch to `fea::solver::SolverError` (or the top-level re-export
  `section_properties::SolverError`).

- **Privatized `CompositeComponent` fields.** The `section` and `material`
  fields are now private. Use `CompositeComponent::new(section, material)`
  (returns `Result<Self, CompositeError>`) and the accessors `.section()`,
  `.material()`, `.youngs_modulus()`, `.area()` instead of struct-literal
  construction.

- **Restricted `dof_index` to `pub(crate)`.** The low-level `dof_index`
  helper is no longer part of the public API. Use the `Dof` enum with
  `BeamSolver::displacement_dof(node, Dof::Ux)` / `reaction_dof(node, Dof::Ux)`
  or `FrameAnalysisResult::displacement(handle, Dof::Ux)` instead.

### Non-breaking changes

- Deleted orphaned dead code `src/fea/matrix.rs` (328 lines, never compiled).
- Added `# Panics` / `# Errors` documentation sections to public API functions.
- Documented `CompositeComponent` invariant encapsulation and unchecked
  constructor notes.

## 0.1.1

- Hardened composite transformed-section calculations against non-finite modular ratios (overflow to Inf/NaN now returns a structured `CompositeError::NonFiniteModularRatio`).
- Detect conflicting prescribed displacements in `try_fix_dof` instead of silently using last-write-wins (`FemError::ConflictingPrescribedDisplacement`).
- Added explicit prescribed-displacement override APIs (`try_override_dof`, `try_override`) for the settlement-after-fix workflow.
- Clarified `Section` boundary-touching hole validation behavior in documentation.

## 0.1.0

- Initial release.
