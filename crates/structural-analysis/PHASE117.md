# Phase 117 — 3D Truss LoadCase / LoadCombination Support

## Summary

Implemented `LoadCase` and `LoadCombination` support for the 3D truss solver,
reusing the existing load infrastructure from `load.rs` with minimal changes.
No new types (`LoadCase3D`, `LoadCombination3D`, `LoadSource3D`) were created.

## Changes

### `src/load.rs` (+61 lines)

- **`LoadCase::nodal_load_3d(node_idx, fx, fy, fz)`** — applies a 3D global
  nodal force. Stores three `(node_idx, dof, value)` triples with `dof = 0, 1, 2`
  for `ux, uy, uz`. Validates finiteness. Does not require `NodeHandle`.

- **`LoadCase::prescribed_displacement_3d(node_idx, dof, value)`** — prescribes
  a displacement at a 3D truss DOF. `dof` is the raw index (`0 = ux, 1 = uy,
  2 = uz`). Validates finiteness. Documents the `LoadCombination` exclusion.

### `src/frame.rs` (2 lines changed)

- `load_case_source` and `load_combination_source` changed from `fn` to
  `pub(crate) fn` so `truss3d.rs` can reuse them. No logic change.

### `src/truss3d.rs` (+134 lines)

- **Imports**: added `LoadCase`, `LoadCombination`, `LoadSource` from
  `crate::load`, and `load_case_source`, `load_combination_source` from
  `crate::frame`.

- **`TrussModel3D::solve_case(&self, case: &LoadCase)`** — solves the 3D truss
  under a single load case. Clones the model, replaces `nodal_forces` with the
  case's forces, applies prescribed displacements (overriding existing fixed
  DOFs or adding new constraints), builds a solver, solves, and returns a
  `TrussAnalysisResult3D` with `load_source = LoadSource::LoadCase`. Validates
  node indices.

- **`TrussModel3D::solve_combination(&self, combo: &LoadCombination)`** — solves
  the 3D truss under a load combination. Rejects prescribed displacements in
  any case (consistent with Frame behavior). Merges RHS:
  `f = Σ factor_i × f_case_i`, solves `K u = f` once, returns result with
  `load_source = LoadSource::LoadCombination`. Validates node indices.

- **`TrussAnalysisResult3D::load_source`** — new private field of type
  `LoadSource`, set to `LoadSource::ModelLoads` in `results()` and overridden
  in `solve_case` / `solve_combination`.

- **`TrussAnalysisResult3D::load_source()`** — public accessor method.

- Updated module doc comment: removed "No `LoadCase` / `LoadCombination`
  support" line.

### `tests/truss3d.rs` (+390 lines)

10 new tests (Test 18–27):

| # | Test | Description |
|---|------|-------------|
| 18 | `test_loadcase_x_direction` | LoadCase with X-force on tripod; independent analytical solution `ux = 3·Fx` |
| 19 | `test_loadcase_y_direction` | LoadCase with Y-force; `uy = 4·Fy` |
| 20 | `test_loadcase_z_direction` | LoadCase with Z-force; `uz = 5·Fz` |
| 21 | `test_loadcase_arbitrary_3d` | LoadCase with all three force components; full analytical check |
| 22 | `test_loadcase_isolation` | Two cases solved separately don't pollute each other |
| 23 | `test_loadcombination` | `1.4D + 1.6L` combination; verifies scaled displacements |
| 24 | `test_combination_equivalence` | `solve_combination(A,B)` == `solve_case(A+B)` for all DOFs and axial forces |
| 25 | `test_loadsource_provenance` | `load_source()` returns correct `LoadSource` for case, combination, and model loads |
| 26 | `test_reaction_equilibrium` | Reactions match analytical `R_1x = -Fx`, `R_2y = -Fy`, `R_3z = -Fz`; off-axis reactions zero |
| 27 | `test_2d_regression` | Planar problem (z=0, uz fixed) in 3D solver matches 2D analytical solution |

**Independent analytical solution**: Tests 18–22 and 26–27 use the tripod
benchmark with `K_ff = diag(1/3, 1/4, 1/5)`, giving `ux = 3·Fx`, `uy = 4·Fy`,
`uz = 5·Fz`, `N_0k = -F_k`. This is derived from first principles, not by
calling the FEM implementation.

## Design decisions

1. **Reuse `LoadCase` / `LoadCombination` / `LoadSource`** — these types are
   DOF-agnostic (`nodal_forces: Vec<(usize, usize, f64)>` stores raw node and
   DOF indices). No `LoadCase3D` or `LoadCombination3D` was needed.

2. **`solve_case` / `solve_combination` on `TrussModel3D`** — consistent with
   `FrameModel::solve_case` / `FrameModel::solve_combination`. The solver
   (`TrussSolver3D`) remains an implementation detail.

3. **`load_case_source` / `load_combination_source` shared** — made
   `pub(crate)` in `frame.rs` rather than duplicating. These are pure
   conversion functions with no Frame-specific logic.

4. **Prescribed displacement override** — `solve_case` overrides existing
   fixed DOFs with the case's prescribed values, or adds new constraints if
   the DOF is not already fixed. This matches Frame behavior.

5. **`LoadCombination` rejects prescribed displacements** — consistent with
   Frame: combining prescribed displacements is physically ambiguous and
   rejected with `FemError::InvalidInput`.

6. **3D methods take raw `usize` node indices** — `nodal_load_3d` and
   `prescribed_displacement_3d` take `node_idx: usize` (not `NodeHandle`),
   matching `TrussModel3D` conventions.

## Verification

| Command | Result |
|---------|--------|
| `cargo fmt --all -- --check` | ✅ clean |
| `cargo check --workspace` | ✅ no errors |
| `cargo test -p structural-analysis` | ✅ 39 test suites, 0 failures |
| `cargo test --doc --workspace` | ✅ 16 doctests, 0 failures |
| `cargo test --examples --workspace` | ✅ 0 tests, 0 failures |
| `cargo clippy --workspace --all-targets --all-features` | ✅ no errors |
| `cargo test --workspace` | ⏱️ timeout (pre-existing FEM benchmarks in section-properties) |

## Line counts

| File | Changed | Total new |
|------|---------|-----------|
| `src/load.rs` | +61 | 61 |
| `src/frame.rs` | +2 −2 | 0 |
| `src/truss3d.rs` | +134 −1 | 133 |
| `tests/truss3d.rs` | +390 | 390 |
| **Total** | +586 −3 | **584** |

Production code: ~194 lines (load.rs 61 + frame.rs 0 + truss3d.rs 133).
Test code: 390 lines.
This matches the Phase 116 estimate of ~190 lines production + ~100 lines tests
(tests exceeded estimate due to comprehensive analytical assertions).
