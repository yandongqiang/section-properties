# 2D frame analysis - architecture audit and design contract

**Status:** the façade described here is **implemented** (Phase 13) in
`src/frame.rs`; see [Implemented API](#implemented-api-phase-13) for the exact
surface, and the sections below for the rationale (Phase 12) and the remaining
design/future items. Code references were verified against commit `3c445ab`
(the audit) and updated for the implementation.

Everything in this document that is **not** described as implemented is design
intent or future work - specifically: connectivity diagnostics beyond what is
listed, and Timoshenko/3D/nonlinear/dynamic extensions. Structural mechanism
diagnostics are implemented (Phase 15, see
[below](#structural-mechanism-diagnostics-phase-15)).

Related: [`docs/beam_fem.md`](beam_fem.md) is the frozen Beam FEM contract and
remains authoritative for every convention reused here.

---


## Implemented API (Phase 13)

Implemented in `src/frame.rs` and re-exported from the crate root. The façade
delegates **all** mechanics to the existing Beam FEM core - it contains no
element stiffness, transformation, assembly, condensation or recovery code.

| Type | Purpose |
| --- | --- |
| `NodeHandle` / `MemberHandle` | typed, `Copy` handles (`index()`, `from_index()`); cannot be confused with a DOF index |
| `FrameModel` | node/member construction, supports, loads, validation, `solve` / `solve_with` / `solver()` |
| `FrameSolver<'a>` | builder for an explicit backend (`with_selection` / `set_solver`), then `solve()` |
| `FrameAnalysisResult` | typed displacement/reaction access, member end forces (local + global), `equilibrium()` |
| `EquilibriumReport` | ΣFx/ΣFy/ΣMz residuals about the global origin, plus the applied and reaction totals and `is_balanced()` |

`FrameModel` methods: `new`, `add_node`, `add_member`, `fix`, `pin`,
`roller_y`, `roller_x`, `restrain`, `nodal_load`, `nodal_moment`, `member_udl`,
`member_point_load`, `solve`, `solve_with`, `solver`, `validate`, `diagnostic`,
`node_handle`, `member_handle`, `n_nodes`, `n_members`.

### Validation actually implemented

| Rule | When | Error |
| --- | --- | --- |
| non-finite node coordinates | `add_node` | `InvalidInput` |
| invalid node handle | every node-taking method | `InvalidNode` |
| invalid member handle | every member-taking method | `InvalidMember` |
| member connecting a node to itself, or coincident nodes | `add_member` | `ZeroLengthMember` |
| duplicate connectivity `(i,j)` / `(j,i)` | `add_member` | `DuplicateMember` |
| non-finite / non-positive `E`, `A` or `I` | `add_member` | `InvalidInput` |
| non-finite nodal force component | `nodal_load` | `InvalidInput` - **both** components are validated before either is recorded, so a rejected call leaves the model unchanged (no half-applied load) |
| empty model, orphan node, disconnected components | `solve` (via `validate`) | `InvalidModel` / `OrphanNode` / `DisconnectedStructure` |
| insufficient restraint (mechanism) | `solve` | **`SolverError`** (singular system), with the structural diagnosis appended to the message; the variant is kept so no caller changes meaning. The verdict itself is available on demand from `FrameModel::diagnostic` (Phase 15) |

### Lifecycle contract (Phase 14)

```text
FrameModel  --(solve / solve_with / solver().solve())-->  FrameAnalysisResult
   ^                                                             |
   |  mutators (add_node/add_member/fix/pin/roller_*/restrain/    |  read-only
   |            nodal_load/nodal_moment/member_udl/              |  accessors
   |            member_point_load)                                |
   +--------------------------------------------------------------+
             a result is never modified by a later mutation
```

`FrameModel` **owns no solution**. It is a pure model description: nodes,
members, supports and loads. `FrameModel::solve` / `solve_with` take `&self`,
validate, assemble a **fresh** `BeamSolver` from the *current* model state and
return an owned `FrameAnalysisResult` that holds its own solved state plus a
private snapshot of the model.

| Situation | Behaviour |
| --- | --- |
| read a result before a solve | not expressible: `FrameModel` has no result accessor; the only way to obtain one is `solve()`, which returns `Err` for an incomplete model (`InvalidModel` / `OrphanNode` / `DisconnectedStructure` / `SolverError`). No placeholder, no zeros, no panic |
| modify the model after a solve | the already-returned `FrameAnalysisResult` is a detached snapshot and is **not** touched or invalidated in place; it keeps reporting the solve it came from (displacements, reactions, end forces, and the applied loads in `equilibrium()`). There is therefore no "stale result" state and no invalidation machinery is needed |
| solve twice with no modification | deterministic; the second solve re-assembles the same K and f and returns a bitwise-identical solution |
| solve → change load → solve | each solve assembles from the current model, so the previous RHS is **not** carried over; the new result reflects the model's accumulated loads exactly |
| solve → change BC → solve | identical: constraints are re-read from the model on every solve; the old result keeps the BC set it was solved with |

Load and support mutators **accumulate** on the model (core semantics, no
replace/clear API): repeated `nodal_load` / `nodal_moment` / `member_udl` /
`member_point_load` calls add to the previous ones, and repeated `restrain` /
support calls add constraints. This is a property of the model, not of the
solver: accumulation happens only when the caller calls the mutator, never
inside `solve()`.

There is deliberately **no** auto-resolve: after a mutation the caller must
call `solve()` again to get an updated result. Frame is append-only - no
remove/reorder APIs.

### Evidence

* `tests/frame_api.rs` - 20 tests: the seven Phase 12 reference cases
  (axial, cantilever tip force/moment, portal, apex with base thrust,
  prescribed settlement, member subdivision), solver cross-validation, the
  support vocabulary (pin + roller simply supported beam, `5qL^4/384EI`), and
  every validation rule above. Six further tests pin down the lifecycle
  contract: a result is an independent snapshot, a repeated solve is bitwise
  identical, a member-load change is re-solved against the analytically expected
  total, a BC change takes effect on the next solve, invalid handles (including
  `from_index(usize::MAX)`) always return typed errors, and empty / member-less /
  unrestrained models fail cleanly and then recover.
* `tests/frame_correctness.rs` - 19 tests: closed-form single-member cases for
  every load type (tip transverse/axial force, tip applied moment, uniform
  transverse/axial distributed load, combined loading), the tip-moment sign
  convention (`M_j = -M`, derived from statics), distributed-load end-force
  recovery (`f_end = f_equiv - K_e u_e`, with the applied moment never
  re-deducted), two-member and portal internal-force transfer (`Σ f_end = 0` at
  an unloaded joint, `f_end = -R` at a fixed base), API robustness, and
  load / stiffness / coordinate scale sweeps. The coordinate sweep documents the
  Euler-Bernoulli conditioning boundary (`κ = max(12/L², 4L²/3)` in absolute
  units) and requires a clean `SolverError` - never a silently wrong answer -
  outside f64's resolution.
* `tests/frame_transformation_contract.rs` - transformation invariants.
* `tests/mechanism_diagnostics.rs` - 11 Phase 15 tests: a completely free
  structure, a single-DOF-restrained structure, a stable cantilever (with
  bit-exact displacement/reaction reference values), a simply supported beam, a
  multi-member portal mechanism (and its stable controls), geometric-similarity
  scale invariance at `1e-6 ... 1e6`, load independence, ill-conditioning at an
  extreme coordinate scale, the probe's size/symmetry limits, and a synthetic
  reduced system for the internal-mechanism and shallow-deficiency verdicts.
* `examples/frame_portal.rs` - end-to-end public-API usage; equilibrium
  residual ~1e-9 on a 20 kN load (balanced).

## Structural mechanism diagnostics (Phase 15)

Implemented in `src/fea/mechanism.rs` (the linear algebra) plus
`FrameModel::diagnostic` / `FrameSolver::solve` in `src/frame.rs` (the
structure). An under-constrained or rank-deficient frame stops being an opaque
`solver error: singular matrix` and is classified as follows.

| Verdict | Meaning |
| --- | --- |
| `StructuralDiagnostic::Stable` | full structural rank and not numerically singular: a direct solver can resolve the reduced system |
| `StructuralDiagnostic::RigidBodyMode { n_free, rank, rigid_modes }` | rank deficient, and **every** null direction is a global rigid-body motion (Tx, Ty and/or Rz) that the supports do not remove - i.e. under-restraint |
| `StructuralDiagnostic::Mechanism { n_free, rank }` | rank deficient with at least one null direction that is *not* a rigid-body motion - an internal mechanism |
| `StructuralDiagnostic::IllConditioned { n_free, rank }` | full structural rank, but the **raw** reduced system is singular to the probe's scale-relative tolerance: a limit of the numbers, **not** a mechanism |
| `StructuralDiagnostic::Indeterminate { n_free, reason }` | not classified (`SystemTooLarge` / `NotSymmetric` / `NotPositiveSemidefinite`); deliberately not a mechanism claim |

The classification is produced by `FrameModel::diagnostic`, which validates the
model and classifies the **reduced** (boundary-conditioned) stiffness block
`K_ff` - without solving, assembling with penalties or perturbing anything.

**Architecture guarantee (single source of truth).** The diagnostic does not
build its own copy of `K_ff`: it condenses through the *actual* Beam/Framework
solve path (`BeamSolver`) and reads the retained reduced system, so the
classified matrix is bit-identical to the one a solve would factorise. Two
independent assemblies would be mathematically equal but not guaranteed
bit-identical, and a rank probe at a relative tolerance floor is exactly the
kind of consumer a ULP difference can flip. `K_ff` is never cached on
`FrameModel`; every `diagnostic()` call is a fresh analysis.

A successful `solve` runs **no** diagnostic and its numerical path is
unchanged; only a *failed* solve classifies the reduced system and appends the
verdict to the error message:

```text
Solver error: singular matrix: Singular or near-singular matrix at column 3: ...
  ; structural diagnosis: rigid-body mechanism: 2 unrestrained rigid-body
    mode(s) (5 free DOFs, system rank 3); restrain the remaining translations/rotation
```

The error **variant** is deliberately unchanged (`SolverError` stays
`SolverError`), so no caller or exhaustive match changes meaning. The verdict is
a property of the *structure*: the load vector is never consulted, so scaling,
reversing or removing loads cannot change it.

### Criterion (scale-aware - no absolute epsilon)

Two probes, both relative to the analysed matrix's own scale, never to an
absolute epsilon:

1. **Structural probe** on the equilibrated system `A = D K D`,
   `D_ii = 1/sqrt(K_ii)`. Equilibration is a congruence, so it preserves the
   rank of `K` exactly while removing the conditioning caused purely by
   unit/stiffness/coordinate scaling. A **pivoted** Cholesky of `A` (pivot on the
   largest remaining diagonal, so the trailing block is never divided by a
   value at the noise level) gives the numerical rank; a remaining diagonal at
   or below `n * eps` (the standard backward-error-bound threshold, relative to
   `A`'s unit scale) is numerically zero.
2. **Rigid-body test** (only for a deficiency). The supplied rigid motions are
   mapped into the equilibrated metric, orthonormalised, and the restriction of
   `K` to that space is ranked: `dim(span R) - rank(R^T K R)` is the number of
   independent rigid-body motions in the null space. If it accounts for the
   whole nullity the verdict is `RigidBodyMode` (under-restraint); otherwise
   `Mechanism`. This is what makes a rigid rotation about a point other than the
   origin - a *combination* of the supplied candidates - classify correctly.
3. **Numerical probe** (only when the structure has full rank). The same
   pivoted factorisation of the **raw** `K`: a deficiency there is
   `IllConditioned`, never a mechanism.

A deficiency counts as structural only when the residual stiffness after
equilibration is at or below `PIVOT_TOL_BASE = 1e-15` (relative) - the project's
own scale-invariant pivot criterion (`src/fea.rs`). Between that floor and the
`n * eps` rank threshold the probe refuses the mechanism verdict and reports
`IllConditioned`: that is the band in which a rank deficiency cannot be
separated from extreme conditioning in double precision.

### What is and is not claimed

* For this element type a frame-level mechanism is **always** an
  under-restraint. The beam element's only zero-energy deformations are its own
  rigid-body motions and a rigid joint shares all three DOFs, so
  `v^T K_ff v = 0` implies the zero-padded vector lies in `null(K)`, i.e. it is a
  global rigid-body motion. `Mechanism` is therefore the mathematically general
  verdict, reachable for a reduced system supplied with a non-rigid null
  direction (tested directly on the probe), rather than the one a rigid-jointed
  frame produces.
* `Stable` means "this structure is not rank deficient and is not numerically
  singular". It is **not** a promise that the solve succeeds (a bad backend
  selection still fails) and it says nothing about load equilibrium.

### Limitations (documented, not papered over)

* The probe is **bounded and dense**: `O(n^3)` time / `O(n^2)` memory, and it
  refuses a reduced system larger than `MAX_DENSE_PROBE_DOF = 500` - the size at
  which this project already accepts a dense factorisation
  (`SolverCapabilities::dense().max_size`) - with
  `DiagnosticLimit::SystemTooLarge` instead of allocating. Mechanisms in larger
  frames are therefore **not** classified: only the solver error remains.
* `IllConditioned` versus a mechanism is decided by tolerance-based probes, not
  exact arithmetic. A deficiency shallower than `1e-15` relative (a scaled
  condition number beyond ~`1e15`) is not claimed as a mechanism.
* A non-symmetric or non-positive-semi-definite reduced matrix is reported as
  `Indeterminate`: the probe is only rank-revealing for symmetric PSD matrices,
  so no mechanism claim is made at all.
* Per-DOF attribution (which node/DOF is unrestrained) is not reported; the
  verdict carries the rank, the nullity via `n_free - rank`, and the number of
  rigid-body modes.

## 1. What the existing core already provides

Audited from `src/beam_fem.rs` (each item verified in the current source, not
assumed):

| Contract | Current state |
| --- | --- |
| Node DOFs | `[ux, uy, rz]`, `dof(node, d) = 3*node + d`, typed `Dof::{Ux, Uy, Rz}` |
| Local axes | local x = `node_i -> node_j`, local y transverse (+up), `rz` CCW positive |
| Section | axial `EA`, bending `EI` (`BeamSection::new(area, second_moment)`) |
| Element stiffness | `k_global = Tᵀ k_local T`, 6x6, symmetric |
| Transformation | `T = [[c,s,0],[-s,c,0],[0,0,1]]` per node block; `u_local = T u_global`, `f_global = Tᵀ f_local` |
| End forces | `f_end = f_equiv - K_e u_local`, element-on-node, local axes |
| Reactions | `R = K_original·u - f_global` (global, all DOFs) |
| Loads | nodal forces **global**; distributed/point loads **local**; applied moments **global** `theta` |
| Constraints | per-DOF `fixed_dofs: Vec<(node, dof, value)>`, static condensation |
| Condensation | `K_ff u_f = f_f - K_fc u_c`, full `u` reconstructed; **no penalty constraints** |
| Solvers | `LinearSolver` trait + `SolverRegistry` (`dense`, `skyline_ldlt`, `sparse_lu`, `cg`, `iccg`), symmetric capability check is scale-aware |
| Validation | bounds/finite/zero-length checks; singular systems propagate `FemError::SolverError` |

**Key audit finding: connectivity is already arbitrary.** `BeamModel` stores
`nodes: Vec<BeamNode>` and `elements: Vec<BeamElement>` with free
`node_i`/`node_j` indices; assembly, condensation, reaction recovery and
end-force recovery make **no** assumption of a straight chain or of element
ordering. Constraints are per-DOF, so pinned / roller / directional supports and
prescribed displacements are already expressible.

This was verified end-to-end in `tests/frame_transformation_contract.rs`
(Phase 12):

```text
two-member apex frame   ux(apex) = 0 (symmetry), Ry(base) = P/2 each,
                        equal-and-opposite base thrust, ΣFx=ΣFy=ΣMz=0,
                        dense == skyline_ldlt == sparse_lu
portal frame            sway = 4.03e-3 m, ΣRx = -F, equilibrium + 3 backends agree
support settlement      prescribed uy enforced exactly, self-equilibrated reactions
transformation          TᵀT = I, k_global(θ) = R k_global(0) Rᵀ, symmetric
```

So the core is **already a 2D frame solver**; what is missing is a
frame-shaped public façade, connectivity validation, and node addressing.

## 2. Reusable infrastructure

| Component | Classification |
| --- | --- |
| `BeamElement::local_stiffness` / `global_stiffness` / `transformation_matrix` | **REUSE AS-IS** |
| `BeamSection` (`area`, `second_moment`) | **REUSE AS-IS** |
| `Material` | **REUSE AS-IS** |
| `Dof` | **REUSE AS-IS** |
| `LinearSolver` / `SolverRegistry` / `SolverSelection` | **REUSE AS-IS** |
| Static condensation (`apply_boundary_conditions`) | **REUSE AS-IS** |
| Reaction / end-force / section-force recovery | **REUSE AS-IS** |
| Input validation (finite, bounds, zero-length, singular) | **REUSE AS-IS**, plus new connectivity checks |
| `BeamModel` (nodes + elements + loads + BCs) | **REUSE WITH EXTENSION** - add connectivity validation, node addressing, and a support vocabulary |
| `BeamSolver` | **REUSE WITH EXTENSION** - same assembly/solve; add frame-level result views |
| `BeamAnalysis` / `BeamAnalysisResult` | **REUSE WITH EXTENSION** for single-member post-processing; unchanged |
| `needs_new` typed node/element handles | **REQUIRES NEW ABSTRACTION** (thin, optional) |
| `sample_forces` / `beam_force_diagram` arclength helpers | **NOT SUITABLE** for branched frames (they assume element order defines a single beam axis); keep beam-only |

## 3. Beam vs Frame: options

| Criterion | A: extend `BeamModel` | B: separate `Frame*` types | C: shared core + two façades |
| --- | --- | --- | --- |
| Backward compatibility | good (additive) | good | good |
| API clarity | **poor** - "Beam" naming for a frame confuses users | good | good |
| Code reuse | total | none (duplicate assembly) | total (single core) |
| Solver reuse | total | total | total |
| 3D extension | **difficult** - 3-DOF layout is baked into the name and docs | needs a second duplicate | clean (new core DOF layout + new façade) |
| Nonlinear / dynamics | new algorithm anyway | new algorithm anyway | cleanest (core owns assembly) |
| Distributed/point loads | reuse | duplicate | reuse |
| Result recovery | reuse | duplicate | reuse |
| Risk now | low | low | **lowest if the "core" is the existing types** |

**Recommendation: Option C, implemented incrementally and non-destructively.**

There is no need to move code: `BeamModel`/`BeamSolver` *are* the shared core.
The frame layer should therefore be:

1. **`FrameModel`** - a thin façade that wraps the existing core types and adds
   frame-oriented construction (node addressing, connectivity checks, typed
   supports). It must not re-implement assembly, condensation or recovery.
2. **`FrameSolver` / `FrameResult`** - thin wrappers over `BeamSolver`
   /`BeamAnalysisResult` exposing frame-level queries (member end forces in
   global axes, node displacement/reaction lookup by node handle, equilibrium
   report).
3. Keep `BeamModel`/`BeamSolver` public and unchanged; `docs/beam_fem.md`
   remains authoritative for the shared conventions.

This preserves every frozen contract, reuses the verified numerics, avoids a
risky refactor, and leaves 3D/nonlinear free to introduce a *new* core DOF
layout behind the same façade pattern.

**Explicitly rejected:** renaming `BeamModel` to `FrameModel` (breaking, churn
without numerical benefit) and duplicating the element/solver code in a parallel
frame module (two divergent implementations of the same mechanics).

## 4. Proposed public API (design record - implemented in Phase 13)

```rust
// Addressing: node handles are returned by construction.
pub struct FrameModel { /* wraps BeamModel */ }
impl FrameModel {
    pub fn new() -> Self;
    pub fn add_node(&mut self, x: f64, y: f64) -> NodeHandle;      // returns a handle
    pub fn add_member(&mut self, a: NodeHandle, b: NodeHandle,
                      material: Material, section: BeamSection) -> Result<MemberHandle, FemError>;

    // Supports (typed, per-DOF; mirrors try_fix_dof semantics)
    pub fn fix(&mut self, n: NodeHandle) -> Result<(), FemError>;                 // fixed
    pub fn pin(&mut self, n: NodeHandle) -> Result<(), FemError>;                 // ux, uy
    pub fn roller_y(&mut self, n: NodeHandle) -> Result<(), FemError>;            // uy
    pub fn restrain(&mut self, n: NodeHandle, dof: Dof, value: f64) -> Result<(), FemError>;

    // Loads (coordinate system stated per method, matching the core contract)
    pub fn nodal_load(&mut self, n: NodeHandle, fx: f64, fy: f64, mz: f64) -> Result<(), FemError>; // global
    pub fn nodal_moment(&mut self, n: NodeHandle, mz: f64) -> Result<(), FemError>;                 // global
    pub fn member_udl(&mut self, m: MemberHandle, qx: f64, qy: f64) -> Result<(), FemError>;         // local
    pub fn member_point_load(&mut self, m: MemberHandle, xi: f64,
                             fx: f64, fy: f64, mz: f64) -> Result<(), FemError>;                     // local

    pub fn solve(&mut self) -> Result<FrameResult, FemError>;                    // Auto selection
    pub fn solve_with(&mut self, sel: SolverSelection) -> Result<FrameResult, FemError>;
}

pub struct FrameResult { /* borrows the solved core */ }
impl FrameResult {
    pub fn displacement(&self, n: NodeHandle, dof: Dof) -> Result<f64, FemError>;
    pub fn reaction(&self, n: NodeHandle, dof: Dof) -> Result<f64, FemError>;
    pub fn member_end_forces(&self, m: MemberHandle) -> Result<[f64; 6], FemError>;        // local
    pub fn member_end_forces_global(&self, m: MemberHandle) -> Result<[f64; 6], FemError>;
    pub fn equilibrium(&self) -> EquilibriumReport;   // ΣFx, ΣFy, ΣMz(residual), about origin
}
```

Design rules kept deliberately small:

* **No generic type system.** `NodeHandle`/`MemberHandle` are thin
  newtypes over `usize` (index + generation is *not* needed yet); raw indices
  stay available through the existing `BeamModel`.
* **One load convention**: local for member loads, global for nodal loads and
  moments - exactly the frozen Beam FEM contract. No second convention.
* **Fallible construction**: handles are validated on use; `add_member` rejects
  out-of-range handles and zero-length members; `solve` rejects disconnected or
  mechanism-ridden structures via the existing singular-system error.

## 5. Supports and load contracts

| Concept | Contract |
| --- | --- |
| fixed | `ux = uy = rz = 0` |
| pinned | `ux = uy = 0`, rotation free |
| roller / directional | one translation restrained (`uy = 0` typical; `ux = 0` allowed) |
| prescribed displacement | `restrain(n, dof, value)` with a finite `value`; enforced exactly |
| nodal load | global `(Fx, Fy)` at a node |
| nodal moment | global `Mz` on the `theta` DOF |
| member UDL | local `(qx, qy)`; `qy > 0` up in local +y, `qx > 0` tensile |
| member point load | local `(fx, fy, mz)` at `xi in [0, 1]`; `xi = 0/1` is a node and must not double count |
| applied member moment | **not** offered separately - use a nodal moment (global) or a local point moment; do not add a third convention |

## 6. Transformation contract

For a member with `dx = xj - xi`, `dy = yj - yi`, `L = hypot(dx, dy)`,
`c = dx/L`, `s = dy/L`:

```text
T  = blockdiag([ [c, s, 0], [-s, c, 0], [0, 0, 1] ], same block for node j)
u_local  = T  · u_global
f_global = Tᵀ · f_local
k_global = Tᵀ · k_local · T
```

Verified properties (`tests/frame_transformation_contract.rs`):

```text
Tᵀ T = I                     (orthogonal; no stored inverse needed)
T[2][2] = T[5][5] = 1        (the rotational DOF is not transformed)
k_global(θ) = R k_global(0) Rᵀ with R = Tᵀ   (transformation applied exactly once)
k_global symmetric after rotation
```

Angles audited: 0°, 45°, 90°, −45°, 135°.

## 7. Global equilibrium contract

For every solved frame:

```text
ΣFx = 0,  ΣFy = 0,  ΣMz = 0
```

with the moment summed **about the global origin (0, 0)**, combining:

```text
Σ over nodes of (support reactions: x·Ry − y·Rx + Rz)
+ Σ applied nodal loads (x·Fy − y·Fx) + Σ applied nodal moments Mz
+ Σ member distributed/point load resultants applied at their true locations
```

Practical requirement: member load resultants are `q·L` (UDL) acting at mid-span
and `f` (point load) acting at `xi·L`, with the local→global rotation applied
before the moment sum. Equilibrium must hold independently of the displacement
solution; it is a property of the assembled system and the recovered reactions.

## 8. Analytical reference cases (cases 1-7 implemented in `tests/frame_api.rs`)

| # | Case | Expected quantities | Sign convention |
| --- | --- | --- | --- |
| 1 | axial bar, tip load `P` | `u = PL/EA`, `Rx = -P`, `N(x) = +P` | tension positive |
| 2 | cantilever, tip transverse `P` | `v = PL³/3EI`, `θ = PL²/2EI`, `Ry = +P`, `Rz = +PL` | sagging positive; `v` negative for `-P` |
| 3 | cantilever, tip moment `M` | `θ = ML/EI`, `v = ML²/2EI`, `Rz = -M`, `M_j = -M` | see `docs/beam_fem.md` §5 |
| 4 | portal frame, lateral `F` | `ΣRx = -F`, sway positive in the load direction, `ΣM = 0` | verified numerically in Phase 12 |
| 5 | two-member apex frame, apex load `P` | `Ry = P/2` per base, equal-and-opposite base thrust, `ux(apex) = 0` | verified numerically in Phase 12 |
| 6 | multi-element straight beam | member subdivision must not change nodal results for consistent loads (nodal exactness) | already covered by Beam FEM tests |
| 7 | prescribed displacement | prescribed value exact; reactions self-equilibrated when there is no external load | `K_ff u_f = f_f - K_fc u_c` |

All seven cases now have executable coverage: cases 4, 5 and 7 through
`tests/frame_api.rs` and `tests/frame_transformation_contract.rs`; cases 1, 2, 3
and 6 through `tests/frame_api.rs`.

## 9. Future extensibility

| Extension | Clean path? | Architectural trap to avoid |
| --- | --- | --- |
| Timoshenko beam | yes - new element stiffness behind the same 6-DOF layout | do **not** bake Euler-Bernoulli shape functions into `FrameModel` |
| 3D frame (6 DOF/node) | yes, as a *new* core DOF layout + façade | do **not** let `3*node + dof` leak into the frame public API (use `Dof`/handles) |
| Geometric nonlinearity | yes - iterate on the existing assembly | do **not** assume linear assembly in `FrameSolver`'s public contract |
| Material nonlinearity | yes - element-level state | do **not** make `FrameModel` store a pre-assembled `K` as the public source of truth |
| Dynamics | yes - mass matrix alongside stiffness | do **not** hard-code "solve = factor once" into the façade |
| Buckling / eigenvalues | yes - same assembly | do **not** expose only displacement-based results |

The single most damaging shortcut would be to make the frame layer a
copy of the beam assembly with frame naming; that doubles the mechanics code and
makes every extension twice as expensive.

## 10. Issues discovered in this audit

| # | Finding | Classification |
| --- | --- | --- |
| 1 | `BeamNode::id` is stored but **never used**; node references are positional vector indices | API DESIGN ISSUE - **resolved in the frame façade** by `NodeHandle`; `BeamModel` unchanged |
| 2 | `add_node` / `add_element` return `()`; callers must track indices themselves | API DESIGN ISSUE - **resolved**: `FrameModel::add_node` returns a handle |
| 3 | No connectivity validation (duplicate members, disconnected components, orphan nodes) - only per-element checks | TEST GAP / API DESIGN ISSUE - **resolved** for the frame layer (`merge/duplicate/orphan/disconnected` diagnostics) |
| 4 | No support vocabulary (`pin`, `roller`); users must call `try_fix_dof` per DOF | API DESIGN ISSUE - **resolved**: `fix` / `pin` / `roller_x` / `roller_y` / `restrain` |
| 5 | Branched models work but are undocumented as such; `docs/beam_fem.md` describes a beam, and arclength helpers (`sample_forces`, `beam_force_diagram`) assume a single beam axis | DOCUMENTATION GAP |
| 6 | A disconnected component with no support produces a singular system error, not a targeted diagnostic | NUMERICAL CONTRACT ISSUE - **resolved**: disconnected/orphan are targeted, and an under-restrained frame is classified as a rigid-body mechanism with the verdict carried in the `SolverError` message (Phase 15) |
| 7 | Frame-level equilibrium reporting (ΣFx/ΣFy/ΣMz about origin) does not exist as an API; tests must recompute it | TEST GAP / FUTURE DESIGN ITEM - **resolved**: `FrameAnalysisResult::equilibrium` |
| 8 | The `!ear_found` fan fallback in the triangulation (Phase 11) remains unsafe-but-unreachable; unrelated to frames | FUTURE DESIGN ITEM (pre-existing) |
| - | Beam FEM formulation, solver implementations, geometry, triangulation, warping | NO ISSUE |

**No production defect was found that blocks a frame layer.** Items 1-7 are
additive design work for the next phase, not bugs.

## 11. Implementation plan (Phase 13 completed the first four steps)

1. `FrameModel` façade over `BeamModel`: `NodeHandle`/`MemberHandle`, `add_node`
   returning a handle, `add_member` with validation, support helpers
   (`fix`, `pin`, `roller_y`, `restrain`), load helpers delegating to the
   existing (already validated) core methods.
2. Connectivity validation: handle bounds, zero-length members, duplicate
   members, orphan nodes, and a *clear* error for disconnected/mechanism
   structures (replacing the generic singular-system message where possible).
3. `FrameSolver` / `FrameResult` façade: typed displacement/reaction lookup,
   member end forces (local and global), and an `equilibrium()` report
   implementing §7.
4. Reference suite from §8 (cases 1-3 and 6 extend existing Beam FEM references;
   4, 5, 7 already exist in `tests/frame_transformation_contract.rs`).
5. Only then consider 3D / Timoshenko / nonlinear work, each as a separate
   initiative with its own contract.

No Beam FEM, solver, geometry, triangulation or warping change is required or
permitted for steps 1-4; the frame layer is additive.
