# 2D frame analysis - architecture audit and design contract

**Status:** the façade described here is **implemented** (Phase 13) in
`src/frame.rs`; see [Implemented API](#implemented-api-phase-13) for the exact
surface, and the sections below for the rationale (Phase 12) and the remaining
design/future items. Code references were verified against commit `3c445ab`
(the audit) and updated for the implementation.

Everything in this document that is **not** described as implemented is design
intent or future work - specifically: connectivity diagnostics beyond what is
listed, Timoshenko/3D/nonlinear/dynamic extensions, and mechanism detection.

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
`member_point_load`, `solve`, `solve_with`, `solver`, `validate`,
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
| insufficient restraint (mechanism) | `solve` | **`SolverError`** (singular system) - deliberately not a targeted diagnostic; distinguishing a mechanism from a very soft structure is not reliable with the current infrastructure |

### Evidence

* `tests/frame_api.rs` - 14 tests: the seven Phase 12 reference cases
  (axial, cantilever tip force/moment, portal, apex with base thrust,
  prescribed settlement, member subdivision), solver cross-validation, the
  support vocabulary (pin + roller simply supported beam, `5qL^4/384EI`), and
  every validation rule above.
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
* `examples/frame_portal.rs` - end-to-end public-API usage; equilibrium
  residual ~1e-9 on a 20 kN load (balanced).

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
| 6 | A disconnected component with no support produces a singular system error, not a targeted diagnostic | NUMERICAL CONTRACT ISSUE - **partially resolved**: disconnected/orphan are now targeted; *mechanism* still reports `SolverError` (documented future item) |
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
