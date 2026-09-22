# Changelog

## 0.2.0

### Breaking changes

- **Removed `FemError::SingularMatrix(String)` variant.** This variant was never
  constructed (0 call sites). Singular-matrix errors now arrive as
  `FemError::SolverError { source, message }` where `source` is a
  `SolverError::SingularMatrix` or `SolverError::NearSingularMatrix`.

- **Unified `SolverError`.** The duplicate `fea::solvers::SolverError` enum has
  been removed; `fea::solvers` now re-exports the canonical
  `fea::solver::SolverError`. If you imported `fea::solvers::SolverError`,
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
