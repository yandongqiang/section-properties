# Changelog

## 0.1.1

- Hardened composite transformed-section calculations against non-finite modular ratios (overflow to Inf/NaN now returns a structured `CompositeError::NonFiniteModularRatio`).
- Detect conflicting prescribed displacements in `try_fix_dof` instead of silently using last-write-wins (`FemError::ConflictingPrescribedDisplacement`).
- Added explicit prescribed-displacement override APIs (`try_override_dof`, `try_override`) for the settlement-after-fix workflow.
- Clarified `Section` boundary-touching hole validation behavior in documentation.

## 0.1.0

- Initial release.
