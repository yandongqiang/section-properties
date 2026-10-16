# Phase 101 — 2D Truss Element

## 1. Baseline

| Item | Value |
|------|-------|
| HEAD (start) | `c144df7` Phase 100: document structural-analysis API and examples |
| Tests | 493 (structural-analysis), 9 doctests |
| Examples | 6 |

## 2. Truss mathematical model

2D linear elastic pin-jointed truss element. Each node has 2 DOF: `[ux, uy]`.
Each element has 4 DOF: `[u_i, v_i, u_j, v_j]`.

Stiffness matrix (4×4, global coordinates):

```text
EA/L * [
  c²   cs  -c²  -cs
  cs   s²  -cs  -s²
 -c²  -cs   c²   cs
 -cs  -s²   cs   s²
]
```

where `c = dx/L`, `s = dy/L`.

Axial force: `N = EA/L · (-c·u_i - s·v_i + c·u_j + s·v_j)`, tension positive.

## 3. API added

### New module: `src/truss.rs`

| Type | Description |
|------|-------------|
| `TrussDof` | Enum: `Ux`, `Uy` (2 variants) |
| `TrussNode` | Node with `id`, `x`, `y` |
| `TrussElement` | Axial-only element: `node_i`, `node_j`, `E`, `A` |
| `TrussModel` | Model: nodes, elements, nodal forces, fixed DOFs |
| `TrussSolver` | Solver: assembly, condensation, solve, results |
| `TrussAnalysisResult` | Result bundle: displacements, reactions, axial forces |

### Key methods

- `TrussElement::new(node_i, node_j, &Material, area)` — validates E > 0, A > 0
- `TrussElement::global_stiffness(pi, pj)` — 4×4 stiffness matrix
- `TrussElement::axial_force(pi, pj, u)` — tension positive
- `TrussModel::add_node/add_element/add_nodal_force/fix_node/fix_dof`
- `TrussSolver::from_model/solve/solve_configured/displacement/reaction/axial_force/axial_forces/diagnostic/results`
- `TrussModel::n_dof = nodes * 2`, `dof_index = node * 2 + dof`

### Re-exports in `lib.rs`

```rust
pub use crate::truss::{
    TrussAnalysisResult, TrussDof, TrussElement, TrussModel, TrussNode, TrussSolver,
};
```

## 4. DOF architecture

### Audit findings

The entire solve pipeline is **DOF-agnostic**:

| Component | DOF-agnostic? | Reused by Truss? |
|-----------|--------------|-----------------|
| `SparseMatrix` | Yes | Yes, zero changes |
| `LinearSolver` trait + all backends | Yes | Yes, zero changes |
| `SolverRegistry` / auto-select | Yes | Yes, zero changes |
| Static condensation (`apply_boundary_conditions`) | Yes | Yes, reimplemented with CSR access (same algorithm) |
| Reaction computation (`R = Ku - f`) | Yes | Yes, same formula |
| `diagnose_reduced` (mechanism diagnostics) | Yes | Yes, zero changes |
| `rigid_nullity` | Yes | Yes, zero changes |

### Beam-specific parts (NOT reused)

| Component | Beam (3 DOF) | Truss (2 DOF) |
|-----------|-------------|--------------|
| `n_dof` | `nodes * 3` | `nodes * 2` |
| `dof_index` | `node * 3 + d` | `node * 2 + d` |
| `Dof` enum | `Ux, Uy, Rz` | `TrussDof: Ux, Uy` |
| Element stiffness | `[[f64; 6]; 6]` | `[[f64; 4]; 4]` |
| `rigid_candidates` | `global / 3`, `global % 3` | `global / 2`, `global % 2` |

### Key insight

The "3" in Beam code has two meanings:
1. **DOF per node** (3 for beam, 2 for truss) — different
2. **2D rigid-body modes** (always 3: Tx, Ty, Rz) — same for both

Truss has 3 rigid-body modes (Tx, Ty, Rz) but only 2 DOF per node. The Rz mode
is expressed through node translations only (`ux = -θ·y`, `uy = +θ·x`), without
a rotational DOF.

## 5. Model architecture

`TrussModel` is **independent** from `BeamModel` and `FrameModel`. No shared
model code. The truss module reimplements the assembly/condensation/solve
pipeline with 2-DOF specifics, reusing only the DOF-agnostic infrastructure
from `section-properties::fea`.

## 6. Load/support handling

- **Loads**: nodal forces `(fx, fy)` in global coordinates. No distributed
  loads, no moments (truss elements carry only axial force).
- **Supports**: `fix_node` (both ux, uy), `fix_dof` (single DOF). Same
  static-condensation approach as Beam.

## 7. Axial-force convention

**Tension positive**: `N = EA/L · (-c·u_i - s·v_i + c·u_j + s·v_j)`.

A positive value means the element is being pulled apart along the
`node_i → node_j` direction. Verified against analytical solutions:
- Horizontal bar with tensile load: N = +F
- Horizontal bar with compressive load: N = -F
- Triangular truss: inclined members in compression (N < 0), bottom in tension (N > 0)

## 8. Mechanism handling

`diagnose_reduced` from `mechanism.rs` is reused directly — it is completely
DOF-agnostic (operates on `SparseMatrix` and candidate vectors only).

The truss-specific `rigid_candidates` method builds 3 candidates (Tx, Ty, Rz)
using 2-DOF/node indexing (`global / 2`, `global % 2`). The Rz mode uses
`ux = -y`, `uy = +x` (rotation about origin) without a rotational DOF.

Tested:
- Stable truss → `Stable`
- Insufficient supports → solver error (singular matrix)
- Diagnostic on stable truss → `is_stable() == true`, `is_mechanism() == false`

## 9. Tests

25 new tests in `tests/truss.rs`:

| Category | Count | Tests |
|----------|-------|-------|
| Element mathematics | 5 | horizontal, vertical, inclined 45°, symmetry, rigid-body null space |
| Input validation | 6 | zero length, zero/negative area, zero modulus, NaN, infinity, invalid node |
| Analytical: horizontal bar | 2 | tip load (tension), compression |
| Analytical: inclined member | 1 | 45° member axial load |
| Analytical: triangular truss | 1 | symmetric truss with vertical load |
| Two-bar system | 1 | series bars, displacement and force |
| Mechanism | 3 | insufficient supports, stable, diagnostic |
| Results API | 1 | results collection |
| DOF enum | 1 | indexing and naming |
| Equilibrium | 2 | horizontal bar, triangular truss |
| Solver selection | 1 | explicit sparse_lu |

**Total: 493 (original) + 25 (truss) + 1 (truss doctest) = 519 tests, 0 failed.**

## 10. Analytical validation

### Horizontal bar

`delta = FL/EA`, `N = F` (tension). Verified: `ux = 10.0` for F=10, E=A=L=1.

### Inclined member (45°)

With `uy=0` at tip and `Fx=5`: `N = 5·√2`, `ux = 10·√2`. Verified to 1e-9.

### Triangular truss

Symmetric triangle, P=10 downward at apex:
- Inclined members: `N = -5·√2` (compression) ✓
- Bottom member: `N = +5` (tension) ✓
- Reactions: `Ry = 5` at each support ✓
- Equilibrium: `ΣFx = 0`, `ΣFy = 0`, `ΣM = 0` ✓

## 11. Documentation/examples

- `truss.rs` module docs: mathematical model, sign convention, reused infrastructure
- All public types/methods have rustdoc with `# Errors` sections
- `TrussSolver` has workflow doctest
- `lib.rs` crate docs updated: truss added to scope, removed from "not supported"
- `README.md` updated: truss in scope, `truss_basic` in examples list
- New example: `examples/truss_basic.rs` — triangular truss with full workflow

## 12. Architecture findings

### What was naturally reused (zero changes)

- `SparseMatrix` — pure matrix container, DOF-agnostic
- `LinearSolver` trait + all 5 backends — pure linear algebra
- `SolverRegistry` / auto-select — pure linear algebra
- `diagnose_reduced` — pure matrix diagnostics
- `StructuralDiagnostic` enum — generic diagnostic result

### What was reimplemented (with same algorithm, different constants)

- `apply_boundary_conditions` — same static condensation, but 2-DOF indexing
- `reactions()` — same `R = Ku - f` formula
- `rigid_candidates` — same 3 modes (Tx, Ty, Rz), but `global/2` instead of `global/3`

### What is Beam-specific (not shared)

- `Dof` enum (3 variants) vs `TrussDof` (2 variants)
- Element stiffness: 6×6 (beam) vs 4×4 (truss)
- Load types: beam has distributed/point/moment; truss has only nodal force
- `EndRelease` — beam-only concept (truss joints are always pinned)

### What problems are worth solving now

None. The duplication is minimal (assembly loop, condensation, reaction
formula — each ~20 lines) and the two element types have fundamentally
different DOF counts, stiffness dimensions, and load vocabularies. Forcing
a shared abstraction would add complexity without eliminating meaningful
duplication.

### What problems should stay simple

- No `Element` trait — the element types have different DOF counts (6 vs 4),
  different stiffness return types (`[[f64; 6]; 6]` vs `[[f64; 4]; 4]`), and
  different load semantics. A trait would require associated types or generics
  that add complexity without benefit.
- No shared `Model` — the models have different fields (beam has
  distributed loads, point loads, applied moments, springs, inclined rollers;
  truss has only nodal forces and fixed DOFs).
- No third crate — the truss module is 800 lines in a single file. Not
  enough to justify a separate crate.

## 13. Element abstraction decision

```text
Element abstraction: NOT NEEDED
```

**Code basis**: `TrussElement` and `BeamElement` have:
- Different DOF counts (4 vs 6)
- Different stiffness return types (`[[f64; 4]; 4]` vs `[[f64; 6]; 6]`)
- Different load types (nodal force only vs distributed/point/moment)
- Different physical behavior (axial-only vs axial+bending+shear)

A trait would need associated types for stiffness dimension and load
collection, adding generic complexity for no shared implementation.

## 14. Shared infrastructure decision

The DOF-agnostic infrastructure (`SparseMatrix`, `LinearSolver`,
`diagnose_reduced`) is already shared via `section-properties::fea` and
`crate::mechanism`. No additional sharing is needed.

The DOF-specific code (assembly, condensation, reactions) is duplicated in
miniature (~20 lines each), but the duplication is structural (different DOF
counts) not algorithmic. Extracting a shared "DOF-aware assembler" would
require parameterizing over DOF-per-node, element stiffness dimension, and
DOF mapping — more complexity than the ~60 lines of duplication.

## 15. Third-crate decision

```text
Third crate: NOT NEEDED
```

The truss module is a single 800-line file. The beam/frame code is ~7000
lines across 4 files. Neither is large enough to justify crate splitting,
and they share only the DOF-agnostic infrastructure already in
`section-properties::fea`.

## 16. Verification

```
HEAD:               c144df7 (before commit)
fmt:                PASS
workspace check:    PASS
structural-analysis tests: 519 passed, 0 failed (493 original + 25 truss + 1 doctest)
structural-analysis doctests: 10 passed, 0 failed
examples:           7/7 PASS (6 existing + truss_basic)
rustdoc:            0 warnings
section-properties: no changes (no regression)
```

## 17. Remaining issues

None. All tests pass, all examples run, documentation is complete.

## 18. Final conclusion

The 2D truss element was implemented as an independent module (`truss.rs`)
with its own `TrussModel`, `TrussSolver`, and `TrussAnalysisResult`. It
reuses the DOF-agnostic infrastructure (`SparseMatrix`, `LinearSolver`,
`diagnose_reduced`) from `section-properties::fea` and `crate::mechanism`
without modification.

The architecture audit confirmed that the current structural-analysis design
has a clean separation between:
- **DOF-agnostic infrastructure** (reusable across element types)
- **Element-specific code** (different DOF counts, stiffness dimensions, load vocabularies)

No `Element` trait, no shared `Model`, no third crate is needed. The
duplication between `BeamSolver` and `TrussSolver` is minimal (~60 lines of
assembly/condensation/reaction code) and structural (different DOF counts),
not algorithmic.
