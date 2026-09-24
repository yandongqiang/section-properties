# Phase 102 — Result & Post-processing Architecture

## 1. Baseline

```
HEAD:     ef3e1a3 (Phase 101: add 2D truss element)
working:  only untracked PHASE*.md files (no source changes)
fmt:      PASS
check:    PASS (only pre-existing warnings)
tests:    519 structural-analysis tests + 10 doctests — all pass
```

Phase 100 (`c144df7`) and Phase 101 (`ef3e1a3`) confirmed present.

---

## 2. Current Result Architecture

Three result types exist, each with a fundamentally different ownership model:

### FrameAnalysisResult (frame.rs:1187)

```rust
pub struct FrameAnalysisResult {
    beam: BeamSolver,       // owned solver (owns K, u, model, ...)
    model: FrameModel,      // owned model (wraps BeamModel, private inner)
    load_source: Option<String>,
}
```

- **Ownership**: fully owned (no lifetimes)
- **Clone**: NOT Clone — `BeamSolver` owns `SparseMatrix` which is not `Clone`
- **Independent存活**: YES — solver can be dropped, result survives
- **Geometry access**: NO public accessor for node coordinates or member connectivity
- **Solver exposure**: `beam` and `model` are private fields; no `solver()` or `model()` getter
- **`member_end_forces`**: **BEFORE FIX** — O(n) per call (called bulk `element_end_forces()` then picked one). **AFTER FIX** — O(1) per call (calls `element_on_node_end_forces_local(i)` directly)
- **`section_forces`**: O(1) — delegates to `beam.element_section_forces(i, xi)`
- **`equilibrium()`**: YES — only result type with this
- **`sample_forces` / `diagram`**: NO — not available
- **`load_source`**: `Option<String>` with format `"case:{name}"` or `"combination:{name}"`

### BeamAnalysisResult<'a> (beam_fem.rs:309)

```rust
pub struct BeamAnalysisResult<'a> {
    solver: &'a BeamSolver,  // borrowed solver
    reactions: Vec<f64>,     // owned snapshot
}
```

- **Ownership**: borrows solver, owns reaction snapshot
- **Clone**: YES (`#[derive(Clone)]`)
- **Independent存活**: NO — tied to solver lifetime; cannot outlive it
- **Geometry access**: NO public accessor (can reach `self.solver.model.nodes` internally but does not expose)
- **`element_end_forces(idx)`**: O(1) — delegates to `solver.element_on_node_end_forces_local(idx)`
- **`section_forces`**: O(1)
- **`sample_forces` / `diagram`**: YES — delegates to solver
- **`equilibrium()`**: NO
- **`load_source`**: NO

### TrussAnalysisResult (truss.rs:762)

```rust
pub struct TrussAnalysisResult {
    pub displacements: Vec<f64>,
    pub reactions: Vec<f64>,
    pub axial_forces: Vec<f64>,
    pub solver_name: Option<String>,
    n_nodes: usize,
    n_elements: usize,
}
```

- **Ownership**: fully owned pure data snapshot
- **Clone**: YES (`#[derive(Debug, Clone)]`)
- **Independent存活**: YES
- **Geometry access**: NO — no node coordinates or element connectivity stored
- **`axial_forces`**: O(1) lookup (pre-computed `Vec<f64>`)
- **`equilibrium()`**: NO
- **`sample_forces` / `diagram`**: N/A (truss has only axial force)
- **`load_source`**: NO

### Summary Table

| Feature              | FrameAnalysisResult | BeamAnalysisResult<'a> | TrussAnalysisResult |
|----------------------|---------------------|----------------------|---------------------|
| Ownership            | owned (solver+model) | borrowed (`'a`)      | owned (pure data)   |
| Clone                | NO                  | YES                  | YES                 |
| Independent存活      | YES                 | NO (lifetime)        | YES                 |
| Displacement         | borrow `&[f64]`     | borrow `&[f64]`      | owned `Vec<f64>`    |
| Reactions            | recompute `K·u - f` | owned snapshot       | owned `Vec<f64>`    |
| Element end forces   | O(1) per member*    | O(1) per element     | N/A                 |
| Section forces       | O(1)                | O(1)                 | N/A                 |
| Axial forces         | N/A                 | N/A                  | O(1) lookup         |
| sample_forces        | NO                  | YES                  | N/A                 |
| diagram              | NO                  | YES                  | N/A                 |
| equilibrium()        | YES                 | NO                   | NO                  |
| Geometry accessors   | NO                  | NO                   | NO                  |
| load_source          | `Option<String>`    | NO                   | NO                  |

\* After Phase 102 fix. Before fix: O(n) per member.

---

## 3. Three-Layer Boundary

### Layer A — Analysis

**Responsibilities**: model definition, element formulation, load assembly,
boundary conditions, stiffness assembly, linear solve.

**Types**: `BeamModel`, `FrameModel`, `TrussModel`, `BeamElement`,
`TrussElement`, `BeamSolver`, `TrussSolver`, `LoadCase`, `LoadCombination`,
`DistributedLoad`, `PointLoad`, `AppliedMoment`, `EndRelease`.

**Rule**: Analysis does not depend on Visualization. Analysis does not depend
on Post-processing.

### Layer B — Analysis Result

**Responsibilities**: displacement, reaction, element end force, section
force, equilibrium verification, solver metadata, load source provenance.

**Types**: `FrameAnalysisResult`, `BeamAnalysisResult<'a>`,
`TrussAnalysisResult`, `SectionForces`, `BeamNodalDisplacement`,
`BeamReaction`, `EquilibriumReport`.

**Rule**: Result is produced by Analysis. Result does not own solver
internals (stiffness matrix, factorization). Result may borrow solver
(`BeamAnalysisResult`) or own a data snapshot (`TrussAnalysisResult`).

### Layer C — Post-processing

**Responsibilities**: sampling, diagram data, envelope, combination
aggregation, extrema, derived results.

**Types**: `BeamForceSample`, `BeamForceDiagram` (currently in Layer B —
see §5.1 for discussion).

**Rule**: Post-processing consumes Result. Post-processing does not call
solver methods. Post-processing does not re-assemble stiffness matrices.

### Layer D — Visualization

**Responsibilities**: SVG, PNG, Canvas, Plotly, GUI, Web.

**Types**: none currently (and none planned in structural-analysis).

**Rule**: Visualization consumes Post-processing data. Visualization is
completely external to structural-analysis. No visualization dependencies
in `Cargo.toml`.

### Dependency Direction

```
A → B → C → D
```

Each layer depends only on the layer below it (data flow) and never
reaches up to the layer above.

---

## 4. Result Ownership Analysis

### Q: Solver-backed Result (A) vs Owned Result Snapshot (B)?

**Current state**: all three models are already in use:
- `FrameAnalysisResult` = A (solver-backed, owned)
- `BeamAnalysisResult<'a>` = A (solver-backed, borrowed)
- `TrussAnalysisResult` = B (owned snapshot)

**Analysis**:

| Criterion                | Solver-backed (A)        | Owned Snapshot (B)    |
|--------------------------|--------------------------|-----------------------|
| Clone / aggregate        | NO (SparseMatrix)        | YES                   |
| Independent存活          | YES if owned, NO if &    | YES                   |
| Solver coupling          | YES                      | NO                    |
| Send / Sync              | depends on solver        | YES (plain data)      |
| Serialization            | hard (solver internals)  | easy (plain data)     |
| Memory cost              | low (no copy)            | moderate (copy)       |
| Recomputation needed     | no (delegate to solver)  | must snapshot all     |

**Decision**: Do NOT unify. Do NOT change any existing result type.

**Rationale**:
1. Each model serves its current use case correctly.
2. `FrameAnalysisResult` works for single-case analysis — `equilibrium()`
   needs model loads + solver reactions, both available.
3. `BeamAnalysisResult<'a>` is cheap and correct for single-case beam
   analysis — the lifetime is not a problem when the solver is in scope.
4. `TrussAnalysisResult` is the ideal shape for aggregation — it's already
   a pure data snapshot.
5. Changing `FrameAnalysisResult` to a snapshot would require copying all
   element end forces, section forces, and reaction data at construction
   time — premature until envelope actually needs it.

**When to revisit**: When envelope (Phase 103+) needs to aggregate multiple
`FrameAnalysisResult`s. At that point, introduce a **new** snapshot type
(e.g. `FrameResultSnapshot`) that extracts the needed data from
`FrameAnalysisResult`. Do NOT modify `FrameAnalysisResult` itself.

---

## 5. Geometry / Topology Data Boundary

### Q: Does post-processing need the full FrameModel?

**No.** Post-processing needs:

```
node coordinates:    (x, y) per node
member connectivity: (node_i, node_j) per member
member length:       derivable from coordinates
```

It does NOT need:

```
LoadCase / loads
solver / stiffness matrix
boundary conditions
spring supports
inclined rollers
```

### Q: Should we create a GeometrySnapshot now?

**No.** There is no post-processing code that consumes it yet. Creating a
type with no consumer is premature abstraction (Phase 101 principle:
"真实需求出现后再抽象").

**When to revisit**: Phase 103, when envelope or visualization needs geometry.
At that point, add minimal accessors to the result types:

```rust
// On FrameAnalysisResult:
pub fn node_position(&self, node: NodeHandle) -> Option<Point>
pub fn member_nodes(&self, member: MemberHandle) -> Option<(NodeHandle, NodeHandle)>

// On TrussAnalysisResult:
// Store node coordinates and element connectivity in the snapshot
```

### 5.1 BeamForceSample / BeamForceDiagram placement

Currently `BeamForceSample` and `BeamForceDiagram` are defined in
`beam_fem.rs` (Layer A/B) and produced by `BeamSolver` (Layer A).

**Assessment**: These are **post-processing data** (Layer C), not analysis
results. They are sampled diagrams, not raw solver output.

**Decision**: Do NOT move them now. They are tightly coupled to
`BeamSolver::sample_beam_forces` and `BeamSolver::element_section_forces`.
Moving them to a separate module/crate would require either:
- Duplicating the sampling logic, or
- Exposing solver internals to the new location.

When a post-processing crate is created (§14), these types should move
there, and the sampling logic should be extracted from `BeamSolver` into a
post-processing function that takes `&BeamAnalysisResult` as input.

---

## 6. Beam Result Strategy

**Current**: `BeamAnalysisResult<'a>` borrows `&'a BeamSolver`.

**Assessment**: Correct for single-case analysis. The lifetime constraint
is not a problem in practice — the solver is always in scope when the
result is used.

**Changes needed**: NONE.

**Deferred to Phase 103+**:
- `equilibrium()` — not currently available on Beam. Should be added when
  needed. The implementation would be similar to Frame's but simpler
  (straight beam, no member rotation).
- Geometry accessors — not needed until visualization.

---

## 7. Frame Result Strategy

**Current**: `FrameAnalysisResult` owns `BeamSolver` + `FrameModel`.

**Changes made in Phase 102**:
1. `member_end_forces(member)` — fixed from O(n) to O(1) per call by
   delegating to `BeamSolver::element_on_node_end_forces_local(i)` instead
   of `BeamSolver::element_end_forces()` (bulk) + index.
2. `member_end_forces_global(member)` — fixed from O(n) to O(1) per call
   by delegating to the new `BeamSolver::element_on_node_end_forces_global(i)`.
3. Made `BeamSolver::element_on_node_end_forces_local` public (was private).
4. Added `BeamSolver::element_on_node_end_forces_global` public method
   (single-element version of the existing bulk `element_end_forces_global`).

**Rationale**: The O(n) per call was a real performance bug. If envelope
calls `member_end_forces` for each member in each load case, the total
cost is O(n_cases × n_members²). After the fix, it is O(n_cases × n_members).

**Deferred to Phase 103+**:
- `sample_forces` / `diagram` — not available on Frame. Should be added
  when needed. Can delegate to `BeamSolver::sample_beam_forces` (already
  exists) or provide a frame-specific version that iterates members.
- Geometry accessors — not needed until visualization.
- `Clone` — not possible without changing ownership model. Defer to when
  envelope needs it; introduce a snapshot type at that point.

---

## 8. Truss Result Strategy

**Current**: `TrussAnalysisResult` is a pure owned data snapshot.

**Assessment**: Already the ideal shape for post-processing and aggregation.
It is `Clone`, has no lifetime constraints, and stores all needed results
(displacements, reactions, axial forces).

**Changes needed**: NONE.

**Deferred to Phase 103+**:
- Geometry snapshot — `TrussAnalysisResult` does not store node coordinates
  or element connectivity. For envelope/visualization, add:
  ```rust
  pub node_coords: Vec<(f64, f64)>,
  pub element_nodes: Vec<(usize, usize)>,
  ```
  to the result, populated in `TrussSolver::results()`.
- `equilibrium()` — not available. Should be added when needed.

---

## 9. LoadSource Strategy

**Current**: `FrameAnalysisResult::load_source() -> Option<&str>` returns
`None` for direct solve, `Some("case:{name}")` for `solve_case`, or
`Some("combination:{name}")` for `solve_combination`.

**Assessment**:

| Criterion                    | `Option<String>` | `enum LoadSource` |
|------------------------------|------------------|-------------------|
| Type safety                  | weak             | strong            |
| Pattern matching             | string parsing   | match arms        |
| API breakage                 | none             | yes (return type) |
| Envelope need                | not proven       | not proven        |
| Real programmatic use        | no               | no                |

**Decision**: Do NOT change. No code currently pattern-matches on the
`load_source` string. Envelope can use the string as an opaque label.
The enum would be a cosmetic improvement with an API-breaking cost.

**When to revisit**: If envelope needs to distinguish case vs combination
programmatically (e.g. to apply different safety factors). At that point,
add the enum and provide `From<String>` for backward compatibility.

---

## 10. Envelope Semantics

### What envelope needs

For each member and each section position, across all load cases:
```
max N, min N
max V, min V
max M, min M
```

### Sampling semantics

Three candidate approaches:

#### Element-end envelope
Compare only i-end and j-end forces.

**Problem**: For members with distributed loads, the maximum moment may
occur at an interior point, not at the ends. This would miss the critical
section.

#### Fixed sampling envelope
Sample at fixed `xi` points (e.g. 0, 0.25, 0.5, 0.75, 1.0).

**Problem**: May miss the exact extremum between sample points. Accuracy
depends on sampling density.

#### Event-aware envelope
Sample at:
- Element ends (`xi = 0`, `xi = 1`)
- Point load locations (discontinuity in V)
- Distributed load transitions (discontinuity in load intensity)
- Analytical extrema of M(x) within each segment (where dM/dx = V = 0)

**Assessment**: Most accurate but most complex. Requires knowledge of load
positions, which is available from the model.

### Decision

**Do NOT implement envelope in Phase 102.** The sampling semantics decision
depends on the use cases that Phase 103 will address.

**Recommendation for Phase 103**: Start with **fixed sampling envelope**
(simplest, correct for the common case of no interior point loads). Add
event-aware sampling as a refinement if needed.

**Key constraint**: Envelope must consume already-computed results, not
call solver methods. The API should be:

```rust
// Phase 103 (not implemented now):
// fn envelope(results: &[&FrameAnalysisResult], n_samples: usize) -> Envelope
```

This keeps the post-processing layer from reaching into the analysis layer.

---

## 11. Diagram Data Semantics

### Current state

`BeamForceDiagram` is a plain container of `BeamForceSample` points:
```rust
pub struct BeamForceDiagram {
    pub samples: Vec<BeamForceSample>,
}
```

It provides `axial()`, `shear()`, `moment()`, `x()` convenience accessors
that extract columns from the samples. It performs **no interpolation**
and **no plotting**.

### Assessment

This is correct post-processing data. It is:
- `Clone` (derivable)
- `Debug` (derivable)
- Pure data (no solver reference)
- Consumable by any visualization layer

### Decision

`BeamForceDiagram` and `BeamForceSample` are correctly designed as
post-processing data. They should eventually move to a post-processing
module/crate (§14), but not in Phase 102.

**Frame gap**: `FrameAnalysisResult` has no `sample_forces` or `diagram`
method. This is a capability gap but not a blocking issue for Phase 102.
Phase 103 should add `FrameAnalysisResult::sample_forces` that delegates
to `BeamSolver::sample_beam_forces` (already exists).

---

## 12. Equilibrium Boundary

### Q: Is `equilibrium()` Analysis Result or Post-processing?

**Analysis Result.** Equilibrium verification is a fundamental correctness
check on the solver output, not a derived post-processing quantity. It
answers "did the solver produce a physically valid result?" not "what is
the maximum moment?"

### Current state

- `FrameAnalysisResult::equilibrium()` — YES, comprehensive (reactions +
  nodal loads + member loads + applied moments, with conditioning-aware
  tolerance).
- `BeamAnalysisResult` — NO.
- `TrussAnalysisResult` — NO.

### Decision

**Do NOT add `equilibrium()` to Beam or Truss in Phase 102.**

**Rationale**:
1. Frame's `equilibrium()` is the most important because frames have the
   most complex load paths (distributed, point, moment, inclined members).
2. Beam's equilibrium is a special case of Frame's (straight beam, no
   member rotation). If needed, it can be added by delegating to the same
   logic.
3. Truss equilibrium is simpler (only axial forces + nodal loads) but
   still useful. Not blocking for Phase 102.
4. Adding equilibrium to Beam/Truss is a feature addition, not an
   architecture decision. It belongs in Phase 103.

---

## 13. Multi-case Aggregation Boundary

### Q: Should envelope solve multiple cases, or only aggregate results?

**Only aggregate.** The separation is:

```
Analysis:     model.solve_case(&case_a)  →  FrameAnalysisResult
              model.solve_case(&case_b)  →  FrameAnalysisResult
              model.solve_combination(&combo)  →  FrameAnalysisResult

Aggregation:  Envelope::from_results(&[result_a, result_b])  →  Envelope
```

**Rule**: Envelope must NOT call `model.solve_case()`. If it did, the
post-processing layer would control the analysis process, violating the
layer separation.

### Q: Multi-case result container?

**Not needed now.** A `Vec<FrameAnalysisResult>` or `Vec<&FrameAnalysisResult>`
is sufficient for envelope input. A dedicated container type would be
premature.

---

## 14. Solver Reuse Assessment

### Q: Can multiple load cases share a stiffness matrix factorization?

**Current architecture**: No. Each `solve_case` / `solve_combination` call
creates a new `BeamSolver`, assembles a new stiffness matrix, and
factorizes it. The `BeamSolver` does not retain a factorization between
calls.

**Assessment**:
- For small models (< 1000 DOF), the factorization cost is negligible
  compared to assembly + I/O.
- For large models (> 10k DOF), factorization reuse would be significant.
- The current `LinearSolver` trait does not support re-factorization with
  a new RHS. Adding this would require extending the trait and all 5
  backends.

**Decision**: Do NOT implement factorization reuse in Phase 102. This is
a numerical optimization, not an architecture decision. It belongs in a
future numerical performance phase.

**When to revisit**: If profiling shows factorization is the bottleneck
for multi-case analysis. At that point, extend `LinearSolver` with a
`solve_with_existing_factorization(&self, rhs: &[f64]) -> Result<Vec<f64>>`
method.

---

## 15. Visualization Boundary

### Q: Should structural-analysis include any visualization?

**No.** The visualization layer is completely external:

```
structural-analysis
    ↓ (produces data)
post-processing (BeamForceDiagram, Envelope)
    ↓ (produces render-ready data)
visualization (SVG, PNG, Plotly, GUI) — external crate
```

### Prohibited dependencies

The following must NOT appear in `structural-analysis/Cargo.toml`:
- `plotters`
- `svg`
- `egui`
- `bevy`
- `plotly`
- `canvas`
- any rendering/graphics crate

### Q: Can section-properties::io::svg be reused?

**No.** `section-properties::io::svg` renders **cross-section** shapes
(2D polygons). Structural visualization needs **structure** shapes
(1D members in 2D/3D space). These are fundamentally different rendering
tasks.

---

## 16. section-properties Boundary

### Confirmed: section-properties post-processing is section-level

| Module                          | Scope          | Structural? |
|---------------------------------|----------------|-------------|
| `post/fibre.rs`                 | fibre stress   | NO          |
| `mesh/fem_analysis.rs`          | section FEM    | NO          |
| `io/svg.rs`                     | section SVG    | NO          |
| `StressPlotData`                | section stress | NO          |

**Rule**: `structural-analysis` must NOT depend on
`section-properties::io::svg` or `section-properties::post` for structure-
level visualization. The dependency direction is:

```
structural-analysis → section-properties (for Material, SparseMatrix, solver)
structural-analysis ↛ section-properties::io::svg (not for structure rendering)
```

---

## 17. Post-processing Crate Decision

### Option A: Module in structural-analysis

```text
structural-analysis
    └── src/postprocessing.rs  (or mod postprocessing)
```

### Option B: Separate crate

```text
structural-analysis
        ↑
structural-postprocessing  (new crate)
```

### Option C: No post-processing code yet

```text
structural-analysis  (current state, no post-processing module)
```

### Analysis

| Criterion                | A (module)     | B (crate)      | C (nothing)    |
|--------------------------|----------------|----------------|----------------|
| Current code to extract  | ~50 lines      | ~50 lines      | 0              |
| API stability            | can change     | must stabilize | N/A            |
| Dependency direction     | clean          | clean          | clean          |
| Compilation cost         | same           | separate       | same           |
| Reuse                    | limited        | external       | N/A            |
| Premature?               | slightly       | yes            | no             |

### Decision: Option C — do NOT create a post-processing crate or module now.

**Rationale**:
1. The only post-processing types are `BeamForceSample` and
   `BeamForceDiagram` (~50 lines). These are tightly coupled to
   `BeamSolver` sampling methods.
2. There is no envelope code, no aggregation code, no visualization code.
3. Creating a crate/module with nothing to put in it is premature.
4. Phase 101 principle: "真实需求出现后再抽象".

**When to revisit**: After Phase 103 implements envelope. At that point,
if the post-processing code exceeds ~500 lines and has a stable API,
extract it to `structural-postprocessing`. Until then, keep it in
`structural-analysis`.

---

## 18. Required Minimal API Changes

### Changes made in Phase 102

1. **`BeamSolver::element_on_node_end_forces_local`** — made `pub`
   (was private). This is the single-element local end-force recovery.
   Already used internally by `BeamAnalysisResult::element_end_forces`.

2. **`BeamSolver::element_on_node_end_forces_global`** — new `pub` method.
   Single-element global end-force recovery. Computes local forces for one
   element, then transforms to global. O(1) in number of elements.

3. **`FrameAnalysisResult::member_end_forces`** — changed implementation
   from `beam.element_end_forces()` (bulk, O(n)) + index to
   `beam.element_on_node_end_forces_local(i)` (single, O(1)).
   **No API change** — same signature, same return value, same error type.

4. **`FrameAnalysisResult::member_end_forces_global`** — changed
   implementation from `beam.element_end_forces_global()` (bulk, O(n)) +
   index to `beam.element_on_node_end_forces_global(i)` (single, O(1)).
   **No API change** — same signature, same return value, same error type.

### Changes NOT made (deferred)

| Change                          | Reason              | Phase  |
|---------------------------------|---------------------|--------|
| Geometry accessors              | no consumer yet     | 103    |
| Frame sample_forces/diagram     | not blocking        | 103    |
| Beam/Truss equilibrium()        | feature, not arch   | 103    |
| LoadSource enum                 | no programmatic need| 103+   |
| FrameResultSnapshot             | no envelope yet     | 103+   |
| Post-processing crate           | no code to extract  | 103+   |
| Solver factorization reuse      | numerical opt       | future |
| Truss geometry in result        | no consumer yet     | 103    |

---

## 19. Deferred Work

1. **Envelope** — multi-case max/min aggregation with fixed sampling
   semantics. Consumes ` &[&FrameAnalysisResult]` or equivalent.
   Phase 103.

2. **Frame sample_forces / diagram** — delegate to
   `BeamSolver::sample_beam_forces`. Phase 103.

3. **Beam / Truss equilibrium()** — add equilibrium verification.
   Phase 103.

4. **Geometry accessors** — `node_position`, `member_nodes` on result
   types. Phase 103, when visualization needs them.

5. **Truss geometry snapshot** — store node coords + element connectivity
   in `TrussAnalysisResult`. Phase 103.

6. **FrameResultSnapshot** — owned snapshot of frame results for
   aggregation. Phase 103+, when envelope needs Clone results.

7. **Post-processing crate** — extract when code exceeds ~500 lines.
   Phase 103+.

8. **Solver factorization reuse** — extend `LinearSolver` trait.
   Future numerical performance phase.

---

## 20. Proposed Phase 103

### Phase 103: Envelope and Frame Post-processing

**Goal**: Add multi-case envelope aggregation and fill Frame post-processing
gaps.

**Depends on Phase 102 conclusions**:
- §7: `member_end_forces` is now O(1) — envelope scan is O(n_cases × n_members).
- §10: Envelope consumes results, does not call solver.
- §13: Envelope uses fixed sampling semantics (start simple).
- §17: Post-processing stays in structural-analysis (no new crate).

**Scope**:
1. `FrameAnalysisResult::sample_forces(n_per_element)` — delegate to
   `BeamSolver::sample_beam_forces`.
2. `FrameAnalysisResult::diagram(n_per_element)` — delegate to
   `BeamSolver::beam_force_diagram`.
3. `Envelope` type — max/min N/V/M per member across multiple results.
4. `Envelope::from_results(results: &[&FrameAnalysisResult], n_samples: usize)`.
5. Geometry accessors on `FrameAnalysisResult`:
   `node_position(node)`, `member_nodes(member)`.
6. `BeamAnalysisResult::equilibrium()` — add equilibrium check.
7. `TrussAnalysisResult::equilibrium()` — add equilibrium check.

**Out of scope**:
- Visualization (SVG, PNG, GUI)
- Post-processing crate
- Solver factorization reuse
- LoadSource enum
- Event-aware envelope sampling

---

## 21. Final Architecture

```
                 section-properties
                         ↑
                         │ dependency (Material, SparseMatrix, solver)
                         │
                 structural-analysis
                         │
          ┌──────────────┴──────────────┐
          │                             │
     Layer A: Analysis             Layer B: Analysis Result
          │                             │
 Beam / Frame / Truss          FrameAnalysisResult
    Model / Solver              BeamAnalysisResult<'a>
          │                     TrussAnalysisResult
          │                         │
          │                     SectionForces
          │                     BeamForceSample  ← (Layer C, currently in B)
          │                     BeamForceDiagram ← (Layer C, currently in B)
          │                     EquilibriumReport
          │                         │
          └──────────────┬──────────────┘
                         │
                    data flow
                         ↓
                 Layer C: Post-processing (Phase 103+)
                         │
                ┌────────┴────────┐
                │                 │
             Envelope          Sampling
                │                 │
                └────────┬────────┘
                         │
                    data flow
                         ↓
                 Layer D: Visualization (external)
                         │
                ┌────────┴────────┐
                │                 │
              SVG / PNG         GUI / Web
```

**Arrows**:
- `↑` = dependency (structural-analysis depends on section-properties)
- `│` = containment / type definition
- `↓` = data flow (not dependency)

**Key principle**: Each layer produces data for the next. No layer reaches
up. Visualization is completely external.

---

## Verification

```
HEAD:     ef3e1a3 (unchanged — no commit yet)
working:  beam_fem.rs, frame.rs modified; PHASE102 doc added
fmt:      PASS
check:    PASS (only pre-existing warnings)
tests:    519 structural-analysis tests + 10 doctests — all pass
examples: 7 examples — all compile and run

Production files changed:
  crates/structural-analysis/src/beam_fem.rs  (made method pub + added method)
  crates/structural-analysis/src/frame.rs     (fixed O(n) → O(1) per member)

API changes:
  + BeamSolver::element_on_node_end_forces_local (pub, was private)
  + BeamSolver::element_on_node_end_forces_global (new pub method)
  ~ FrameAnalysisResult::member_end_forces (implementation only, no signature change)
  ~ FrameAnalysisResult::member_end_forces_global (implementation only, no signature change)
```

---

## Final Judgement

### 1. Should structural-analysis own SectionForces / BeamForceSample / BeamForceDiagram?

**YES, for now.** They are tightly coupled to solver methods. Move to
post-processing module/crate when it is created (Phase 103+).

### 2. Should FrameAnalysisResult move from solver+model to owned snapshot?

**NOT YET.** The current model works for single-case analysis. Introduce
a separate snapshot type when envelope needs Clone results (Phase 103+).

### 3. Do Truss / Beam / Frame need a unified Result trait?

**NO.** They have different DOF counts, different force types (axial vs
N/V/M), different ownership models, and different capabilities. A trait
would be premature abstraction with no real consumer.

### 4. Should Envelope consume AnalysisResult or PostProcessingResult?

**AnalysisResult.** Envelope needs displacements, reactions, and section
forces — all are Layer B data. It does NOT need diagram samples (those are
a derivative of section forces). The envelope can internally sample
section forces as needed.

### 5. Do diagram data types belong to structural-analysis or post-processing?

**Post-processing (Layer C).** But they are currently in structural-analysis
for practical reasons (coupling to solver). Move when post-processing
crate is created.

### 6. Is visualization completely independent?

**YES.** No visualization dependencies in Cargo.toml. No rendering code
in structural-analysis. Visualization consumes post-processing data
through a clean data boundary.

### 7. Should we create a third crate now?

**NO.** There is no post-processing code to extract (~50 lines of
diagram types, tightly coupled to solver). Create when code exceeds
~500 lines and has a stable API (Phase 103+).

### 8. What should Phase 103 implement?

**Envelope and Frame post-processing gaps.** See §20 for detailed scope.
Depends on Phase 102's conclusion that `member_end_forces` is now O(1)
(§7), envelope consumes results not solver (§10, §13), and post-processing
stays in structural-analysis (§17).
