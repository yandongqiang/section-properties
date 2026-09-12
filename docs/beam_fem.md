# Beam FEM contract

This document is the authoritative description of the **current** 2D
Euler–Bernoulli beam finite-element implementation in `src/beam_fem.rs`
(`BeamModel`, `BeamElement`, `BeamSolver`, `BeamAnalysisResult`). It records
behaviour that is already implemented and covered by the test suite; it does
not describe future work. If the documentation and the implementation ever
disagree, the implementation is the source of truth — report the discrepancy
rather than changing behaviour to match prose.

The semantics below are exercised by:

| Area | Tests |
| --- | --- |
| Reference solutions | `tests/beam_fem_reference.rs` |
| Convergence / coverage | `tests/beam_fem_convergence.rs` |
| Multi-solver cross-validation | `tests/beam_fem_rotated_beam` in `tests/beam_multisolver_cross_validation.rs` |
| API / load / BC semantics | `tests/beam_fem_api_semantics.rs` |
| Robustness / validation | `tests/beam_fem_robustness.rs` |
| Core element/assembly | `tests/beam_fem.rs` |
| Contract pinning | `tests/beam_fem_contract.rs` |

---

## 1. Degrees of freedom

Each node has three degrees of freedom in the **global** system:

```text
node DOFs = [ux, uy, rz]
```

with the global assembly mapping (confirmed by `BeamModel::dof_index`):

```text
dof(node, 0) = 3*node      // ux
dof(node, 1) = 3*node + 1  // uy
dof(node, 2) = 3*node + 2  // rz
```

- `ux`, `uy` are translations along the global X/Y axes.
- `rz` is the rotation about the global Z axis, **counter-clockwise positive**.
- The global displacement vector is `[ux0, uy0, rz0, ux1, uy1, rz1, ...]`.

## 2. Coordinate systems

- Nodes carry global coordinates `(x, y)` (`BeamNode`).
- Each element defines a **local** system: local x runs from `node_i` to
  `node_j`; local y is the transverse axis, taken positive so that a positive
  local moment/rotation is counter-clockwise (the standard
  `(local y) = z × (local x)` orientation).
- The transformation matrix `T` maps global → local
  (`u_local = T · u_global`); a global quantity is recovered as
  `Tᵀ · (local quantity)`.

Local and global coincide only for a horizontal element (`node_i` to `node_j`
along +X).

## 3. Load coordinate systems — read this carefully

Different load types are expressed in **different** coordinate systems:

| Load | API | Coordinates |
| --- | --- | --- |
| Nodal force | `add_nodal_force(node, dof, value)` | **global** |
| Distributed load | `add_distributed_load(elem, qx, qy)` | **local** (element) |
| Point load | `add_point_load(elem, xi, fx, fy, mz)` | **local** (element) |
| Applied moment | `add_applied_moment(node, value)` | **global** (`θ` DOF) |

For local loads, `qy > 0` / `fy > 0` is upward in local +y and `qx > 0` /
`fx > 0` is tensile (towards `node_j`); `mz > 0` is counter-clockwise in local
coordinates. For a horizontal element these coincide with the global axes.

## 4. Sign conventions

```text
rz > 0            counter-clockwise rotation (global)
M > 0             sagging bending moment (tension on the local +y...-y bound; M = E·I·v'')
V = dM/dx         shear is defined by dM/dx = V
N > 0             axial tension
```

For a horizontal cantilever with a downward tip load `P`, the internal moment
is negative (hogging): `M(x) = -P(L - x)`. For a uniform downward load `q`,
`M(x) = -q(L - x)²/2`. These are asserted in
`tests/beam_fem_reference.rs`.

## 5. Element end-force convention

`BeamSolver::element_end_forces()` (and `BeamAnalysisResult::element_end_forces`)
returns **element-on-node** forces in **local** coordinates, ordered
`[N_i, V_i, M_i, N_j, V_j, M_j]`, defined by

```text
f_end = f_equiv - K_e · u_local
```

where `f_equiv` is the element's equivalent nodal load vector from distributed
and point loads only, `K_e` is the element stiffness matrix and `u_local = T·u_global`.

- `element_end_forces_global()` returns the same forces transformed to global
  axes: `f_global = Tᵀ · f_end`.
- `element_section_forces(elem, xi)` returns the internal section resultants
  `N`, `V`, `M` at a normalised position `xi ∈ [0,1]` (see
  `beam_fem` rustdoc for the recovery and jumps).

**These are internal element forces, not external loads.** They must not be
added to the applied loads when checking equilibrium. For a free-end element
carrying an applied tip moment `+M`, the element-on-node free-end moment is
`M_j = -M` while the internal section moment is `+M`; this distinction is
intentional and is asserted in `tests/beam_fem_api_semantics.rs`.

## 6. Loads, equivalent nodal loads and reactions

The assembled right-hand side is

```text
f_global = nodal forces + Σ(e element equivalent loads) + applied moments
```

Equivalent nodal loads for distributed/point loads are an **assembly
representation** of the physical loads, not additional physical loads. When
checking global equilibrium, count each physical load exactly once:

```text
Σ reactions + Σ applied loads = 0        (ΣFx, ΣFy, ΣMz)
```

`BeamSolver::reactions()` returns the **global** external support reactions

```text
R = K_original · u - f_global
```

- At a constrained DOF, `R` is the support reaction.
- At a free DOF, `R` is numerically zero (round-off level).

### Point loads at element boundaries

A point load may be placed anywhere with `xi ∈ [0, 1]`:

- `0 < xi < 1`: inside the element; produces a shear jump `ΔV = fy` and a
  moment jump `ΔM = -mz` across the load.
- `xi = 0` or `xi = 1`: at an element end, i.e. **at a node**. This is
  equivalent to a direct nodal force and is **not** double-counted — a `xi = 1`
  load on element `e` and a `xi = 0` load on element `e+1` both act at the
  shared node once. Verified in `tests/beam_fem_api_semantics.rs`.

An applied moment at an internal node produces the section-moment relation
`M_right - M_left + M_applied = 0`, equivalent to the element-on-node relation
`M_j(left) + M_i(right) + M_applied = 0`.

## 7. Boundary conditions

| API | Fallibility | Notes |
| --- | --- | --- |
| `fix_node(node)` | panics on programmer error | fixes `ux = uy = rz = 0` |
| `try_fix_node(node)` | returns `Result` | checked counterpart |
| `fix_dof(node, dof, value)` | panics on programmer error | prescribed value (default `0.0`) |
| `try_fix_dof(node, dof, value)` | returns `Result` | checked counterpart |

- `uz`... `dof` must be `0`, `1` or `2`.
- A non-zero `value` prescribes a displacement/rotation (e.g. a support
  settlement), which is enforced exactly and induces the corresponding
  reactions (`tests/beam_fem_api_semantics.rs`).
- Constraints are applied by **static condensation**: the constrained DOFs are
  eliminated, the reduced free-free system `K_ff u_f = f_f - K_fc u_c` is
  solved, and the full displacement vector is reconstructed. `K_original` and
  `f_global` are retained for reaction recovery.
- The legacy `fix_*` APIs intentionally **panic** on invalid node/DOF indices
  (programmer error); use the `try_*` variants to get a structured `FemError`.

## 8. Model snapshot semantics

`BeamSolver::from_model(&model)` **snapshots** the model: it clones the model
and assembles the global stiffness matrix and force vector once. Consequently:

- Mutating the original `BeamModel` after `from_model` does **not** affect the
  existing solver.
- Re-solving the same `BeamSolver` is deterministic and does not accumulate
  loads or boundary conditions.
- To pick up model changes, build a new `BeamSolver::from_model`.

This is asserted by
`tests/beam_fem_api_semantics.rs::test_mutation_after_solve_snapshot_semantics`.
There is no public API to mutate a `BeamSolver`'s model in place.

## 9. Solving and solver selection

`BeamSolver` uses the crate's unified `LinearSolver` abstraction.

- `BeamSolver::solve(&mut dyn LinearSolver)` — caller supplies the solver.
- `BeamSolver::solve_configured()` — uses the configured `SolverSelection`
  (default `Auto`), choosing the backend against the **condensed** matrix.
- `BeamSolver::set_solver(SolverSelection)` / `solver_selection()`.
- `BeamSolver::solver_name()` / `BeamAnalysisResult::solver_name()` report the
  backend used by the last **successful** solve (cleared before each attempt).

`SolverSelection`:

- `Auto` — the registry selects a backend from the matrix size/class. For the
  small systems typical of beam models this is `dense`.
- `Named("dense" | "skyline_ldlt" | "sparse_lu" | "cg" | "iccg")` — an explicit,
  authoritative request. An explicit request is validated against the backend's
  capabilities (symmetry, `max_size`) and **never silently falls back** to
  another backend.

Solver choice must not change the physical results within numerical tolerance;
this is verified across `dense`, `skyline_ldlt` and `sparse_lu` in
`tests/beam_multisolver_cross_validation.rs` and
`tests/beam_fem_api_semantics.rs`. No performance claims are made here.

## 10. Error behaviour

`FemError` is the error type returned by the fallible Beam FEM APIs:

```text
FemError::InvalidModel(String)   // structural problems with the model
FemError::InvalidInput(String)   // invalid arguments (index, position, non-finite value)
FemError::SolverError(String)    // propagated linear-solver failure (e.g. singular system)
FemError::SingularMatrix(String)
```

Examples (all covered by `tests/beam_fem_robustness.rs`):

- zero-length / malformed element, out-of-range connectivity → `InvalidModel`;
- non-finite or non-positive `E`, `A`, `I` → `InvalidModel`;
- non-finite loads, out-of-range element/node/DOF/`xi` → `InvalidInput`;
- under-constrained (singular) system or solver failure → `SolverError`.

The legacy `fix_node` / `fix_dof` / `add_nodal_force` APIs panic on programmer
error and keep that compatibility contract; their `try_*` counterparts return
`FemError`.

---

## 11. Typed ergonomic helpers (additive)

These helpers are naming/convenience layers only; they do **not** change any
convention above.

- `Dof::Ux | Dof::Uy | Dof::Rz` maps to `0 | 1 | 2`
  (`Dof::index()`, `Dof::ALL`, `Dof::name()`, `TryFrom<usize>`,
  `From<Dof> for usize`).
- `BeamModel::try_fix(node, Dof, value)` — typed form of `try_fix_dof`.
- `BeamModel::try_fix_node_with_values(node, ux, uy, rz)` — prescribed
  displacement/rotation for all three DOFs of a node (e.g. a support
  settlement). It delegates to `try_fix_dof`, shares the same boundary-condition
  storage and static-condensation path, and validates all inputs before
  recording anything, so a rejected call leaves the model unchanged.
- `BeamElement::to_local_force(node_i, node_j, gx, gy) -> (fx_local, fy_local)`
  — rotate a **global** force (or load-intensity) vector into the element's
  **local** axes for use with `add_distributed_load` / `add_point_load`.

`Dof` is always a **global** DOF. Node and element indices remain plain
`usize`; strongly typed `NodeId` / `ElementId` / `DofId` wrappers are
intentionally **not** introduced in this phase — raw indices are adequate and a
typed-ID redesign would be a breaking change with no numerical benefit.

### Result access

- `BeamSolver::displacement_dof(node, Dof)` and
  `BeamSolver::reaction_dof(node, Dof)` are typed accessors over the global
  displacement vector and the reaction vector; they return exactly what
  `displacement(node, 0/1/2)` and `reaction(node, 0/1/2)` return, and
  `Err(FemError::InvalidInput)` for an out-of-bounds node.
- `BeamAnalysisResult::displacement(node)` / `reaction(node)` remain the
  struct-based accessors (`BeamNodalDisplacement`, `BeamReaction`). One
  documented difference: `BeamAnalysisResult::reaction` reports **exactly 0.0**
  at free DOFs (the raw residual there is round-off, not a physical support
  reaction), whereas `BeamSolver::reactions()` returns the raw `K·u - f`
  residual (~1e-13). No new result structs were added; the existing ones are
  generated from the same vectors.
- Reading results before a successful solve: the displacement vector is
  initialised to zero, so displacement access returns `0.0`; reactions evaluated
  before a solve are `K·0 - f` and are **not** physical support reactions.
  `BeamSolver::solver_name()` is `Some(..)` only after a successful solve.
- The legacy raw-index `displacement(node, dof)` / `reaction(node, dof)` are
  unchanged and kept for compatibility. Beware that a raw `dof >= 3` in those
  APIs aliases into the following node (`dof_index = 3*node + dof`); the typed
  `Dof` API makes that impossible.

## Notes

- Only 2D Euler–Bernoulli beams are implemented; there is no shear deformation,
  geometric nonlinearity or dynamics in this module.
- The Python `sectionproperties` package is a cross-section/warping library and
  provides **no** beam finite element; it is not a reference for this module.
