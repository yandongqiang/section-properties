# Phase 116 — 3D Truss LoadCase / LoadCombination Architecture Audit

**Date:** 2026-10-08
**Baseline:** `37551a6` (Phase 115)
**Production code changes:** 0
**Output:** this document only

---

## 1. Baseline

```text
HEAD: 37551a6 (Phase 115: 3D Truss deep audit)
working tree: clean (only old untracked PHASE docs)
```

---

## 2. Current Architecture Inventory

### 2.1 LoadCase (`load.rs:49–347`)

```rust
pub struct LoadCase {
    name: String,
    nodal_forces: Vec<(usize, usize, f64)>,      // (node_idx, dof_idx, value)
    distributed_loads: Vec<DistributedLoad>,      // Frame/Beam only
    point_loads: Vec<PointLoad>,                  // Frame/Beam only
    applied_moments: Vec<AppliedMoment>,          // Frame/Beam only
    prescribed_displacements: Vec<(usize, usize, f64)>,  // DOF-agnostic
}
```

**Key observation:** `nodal_forces` and `prescribed_displacements` are already
DOF-agnostic — they store `(node_idx, dof_idx, value)` triples that work for
any DOF count. The Frame-specific parts are the builder methods
(`nodal_load(node, fx, fy)`, `member_udl`, etc.) and the Frame-specific load
types (`DistributedLoad`, `PointLoad`, `AppliedMoment`).

### 2.2 LoadCombination (`load.rs:458–510`)

```rust
pub struct LoadCombination {
    name: String,
    terms: Vec<(LoadCase, f64)>,  // (case, factor)
}
```

The combination logic is **purely DOF-agnostic**: it clones `LoadCase`
objects and scales by factors. The actual load vector assembly happens in
the solver path (`FrameModel::solve_combination`), not in `LoadCombination`
itself.

### 2.3 LoadSource (`load.rs:362–417`)

```rust
pub enum LoadSource {
    ModelLoads,
    LoadCase { name: String, has_prescribed_displacements: bool },
    LoadCombination { name: String, terms: Vec<LoadCombinationTerm> },
}
```

This enum is **fully DOF-agnostic**. It describes result provenance, not
model dimensionality. `LoadCombinationTerm` is also DOF-agnostic.

### 2.4 Envelope (`postprocessing.rs:1–410`)

```rust
pub struct Envelope {
    samples: Vec<EnvelopeSample>,           // Frame: SectionForces (N, V, M)
    node_displacements: Vec<NodeDisplacementSample>,  // (ux, uy, rz) — 3 DOF
    support_reactions: Vec<NodeReactionSample>,       // (rx, ry, mz) — 3 DOF
    sources: Vec<LoadSource>,              // DOF-agnostic
    ...
}
```

`Envelope::from_frame_results` takes `&[&FrameAnalysisResult]`.
`EnvelopeSample` contains `SectionForces` (N, V, M) — a Frame/Beam concept.
`NodeDisplacementSample` and `NodeReactionSample` have 3 components
(`ux, uy, rz` / `rx, ry, mz`) — 2D Frame DOFs.

### 2.5 Extremum (`postprocessing.rs:71–108`)

```rust
pub struct Extremum {
    min: f64, min_source: Option<usize>,
    max: f64, max_source: Option<usize>,
}
```

**Fully DOF-agnostic** — a generic signed min/max tracker with source
provenance. Already shared infrastructure.

### 2.6 Solver integration

| Solver | LoadCase support | LoadSource in result | Envelope support |
|---|---|---|---|
| FrameModel | ✅ `solve_case`, `solve_combination` | ✅ `FrameAnalysisResult.load_source` | ✅ `Envelope::from_frame_results` |
| PreparedFrameAnalysis | ✅ `solve_cases` (batch) | ✅ | ✅ |
| TrussModel (2D) | ❌ direct `add_nodal_force` | ❌ | ❌ |
| TrussModel3D | ❌ direct `add_nodal_force` | ❌ | ❌ |

---

## 3. Question 1: Can LoadCase/LoadCombination extend to 3D Truss without copying?

### Answer: **YES WITH LIMITED CHANGE**

### Analysis

The `LoadCase` struct has two DOF-agnostic fields and three Frame-specific
fields:

| Field | DOF-agnostic? | Applies to 3D Truss? |
|---|---|---|
| `nodal_forces: Vec<(usize, usize, f64)>` | ✅ | ✅ (push 3 entries per node) |
| `prescribed_displacements: Vec<(usize, usize, f64)>` | ✅ | ✅ |
| `distributed_loads: Vec<DistributedLoad>` | ❌ Frame | ❌ Truss has no member loads |
| `point_loads: Vec<PointLoad>` | ❌ Frame | ❌ Truss has no member loads |
| `applied_moments: Vec<AppliedMoment>` | ❌ Frame | ❌ Truss has no moments |

The `LoadCombination` scaling logic is **purely DOF-agnostic** — it operates
on `LoadCase` objects by cloning and scaling factors, with no knowledge of
DOF count or load types.

### Two viable approaches

**Approach A: Share `LoadCase` with Frame (minimal duplication)**

Add a `nodal_load_3d(node, fx, fy, fz)` method to the existing `LoadCase`.
The `nodal_forces` vec already stores `(node, dof, value)` triples, so 3D
forces just push 3 entries. The Frame-specific fields
(`distributed_loads`, etc.) remain empty for Truss use.

- **Pro:** Zero duplication of combination logic. `LoadCombination` works
  unchanged.
- **Con:** `LoadCase` carries Frame-specific fields that are meaningless for
  Truss. A 3D Truss user could accidentally call `member_udl` which would
  silently add a load that the Truss solver ignores.

**Approach B: Separate `TrussLoadCase3D` (clean separation)**

Create a minimal struct with only `nodal_forces` and
`prescribed_displacements`. Create a separate or generic
`LoadCombination` variant.

- **Pro:** Semantically pure — a Truss LoadCase cannot contain Frame loads.
- **Con:** ~50 lines of combination logic duplicated, unless
  `LoadCombination` is made generic.

**Recommended: Approach A with a guard.**

The `nodal_forces` storage is already DOF-agnostic and `LoadCombination`
works unchanged. The only risk is accidental cross-contamination (calling
`member_udl` on a Truss LoadCase), which can be prevented by:
1. The 3D Truss solver path only reads `nodal_forces` and
   `prescribed_displacements` — it silently ignores Frame-specific fields.
2. A documentation note on `LoadCase` clarifying which fields apply to
   which solver types.

This avoids duplicating the combination logic while keeping the door open
for a future `TrussLoadCase3D` if the semantic impurity becomes a real
problem.

### What would need to change

1. **`LoadCase`**: Add `nodal_load_3d(node: usize, fx, fy, fz)` method
   (or accept `TrussNode3D` — but `usize` is simpler and
   DOF-agnostic). ~10 lines.
2. **`TrussSolver3D`**: Add `solve_case(&LoadCase)` and
   `solve_combination(&LoadCombination)` methods that read
   `nodal_forces` and `prescribed_displacements` from the case. ~60 lines
   (analogous to `FrameModel::solve_case`).
3. **`TrussAnalysisResult3D`**: Add `load_source: LoadSource` field.
   ~5 lines.
4. **`LoadCombination`**: No change needed — already DOF-agnostic.

**Total: ~75 lines of new code, 0 lines of duplicated framework.**

---

## 4. Question 2: Should LoadSource/provenance be unified with Frame?

### Answer: **YES — already DOF-agnostic, just needs to be adopted**

### Analysis

`LoadSource` is defined in `load.rs` and re-exported from `lib.rs`. It
carries **no Frame-specific information**:

```rust
pub enum LoadSource {
    ModelLoads,                                    // "loads on the model"
    LoadCase { name, has_prescribed_displacements }, // "a named case"
    LoadCombination { name, terms },               // "a named combination"
}
```

A 3D Truss result solved from a `LoadCase` would have exactly the same
`LoadSource::LoadCase { name, has_prescribed }` as a Frame result. The
provenance is about *how the result was produced*, not *what dimension the
model is*.

### Current gap

`TrussAnalysisResult3D` does **not** have a `load_source` field. This is
acceptable for the MVP (direct `add_nodal_force` → `ModelLoads`), but when
LoadCase support is added, the field should be introduced.

### Recommendation

When 3D Truss LoadCase support is implemented:
1. Add `load_source: LoadSource` to `TrussAnalysisResult3D`.
2. Set it to `LoadSource::ModelLoads` for direct `add_nodal_force` solves.
3. Set it to `LoadSource::LoadCase { ... }` for `solve_case` solves.
4. Set it to `LoadSource::LoadCombination { ... }` for `solve_combination`.

**No change to `LoadSource` itself is needed.** The type is already shared
and DOF-agnostic. This is not a "should it be unified" question — it's
already unified by design, just not yet adopted by 3D Truss.

---

## 5. Question 3: Should Envelope extract shared post-processing infrastructure?

### Answer: **NOT YET — continue independent Result APIs**

### Analysis

#### What's already shared

| Component | Location | DOF-agnostic? | Shared? |
|---|---|---|---|
| `Extremum` | `postprocessing.rs` | ✅ | ✅ already shared |
| `LoadSource` | `load.rs` | ✅ | ✅ already shared |
| `LoadCombinationTerm` | `load.rs` | ✅ | ✅ already shared |

#### What's different

| Component | Frame | 3D Truss (future) | 2D Truss (future) |
|---|---|---|---|
| Member forces | `SectionForces` (N, V, M) at sampled xi | axial force (constant per member) | axial force (constant per member) |
| Node DOFs | 3 (ux, uy, rz) | 3 (ux, uy, uz) | 2 (ux, uy) |
| Sampling | n_per_member points along each member | none (force is constant) | none |
| Result type | `FrameAnalysisResult` | `TrussAnalysisResult3D` | `TrussAnalysisResult` |

#### Why not extract now

1. **Only one concrete implementation exists.** Only Frame has Envelope
   support. 2D Truss and 3D Truss have no Envelope. Extracting a shared
   abstraction over one concrete implementation is premature — there's no
   second implementation to validate the abstraction against.

2. **The differences are physical, not accidental.** Frame envelopes
   `SectionForces` (N, V, M) at sampled points; Truss envelopes a single
   axial force per member (no sampling needed). These are fundamentally
   different physical quantities, not just "the same thing with different
   DOF counts."

3. **A trait-based abstraction would add complexity.** An `EnvelopeResult`
   trait would need methods for:
   - `n_nodes()`, `n_members()` — easy to share
   - `displacement(node, dof) -> f64` — different DOF counts
   - `reaction(node, dof) -> f64` — different DOF counts
   - `member_force(member, xi) -> ???` — different return types
     (`SectionForces` vs. `f64`)
   
   The member force return type difference is the killer: a trait method
   can't return `SectionForces` for Frame and `f64` for Truss without
   either an associated type (which makes the trait complex) or a
   generic enum wrapper (which is exactly the kind of generic abstraction
   Phase 113 said NO to).

4. **The shared parts are already shared.** `Extremum` and `LoadSource`
   are already DOF-agnostic and in shared locations. The node
   displacement/reaction envelope code (~40 lines) could be extracted
   into a helper function parameterized by `n_dof_per_node`, but this is
   a minor refactoring, not an architecture decision.

5. **Phase 113 decision stands.** "NO generic Element/Result/Constraint
   trait." An `EnvelopeResult` trait is a generic Result trait. The
   decision was made deliberately and has not been overturned by any
   new evidence.

#### When to reconsider

If and when 3D Truss gets LoadCase + Envelope support, there will be two
concrete envelope implementations. At that point:
- The node displacement/reaction envelope code (~40 lines) could be
  extracted into a `compute_node_envelopes(results, n_nodes, n_dof_per_node)`
  helper function. This is a **function extraction**, not a trait
  abstraction — simpler and sufficient.
- The member force envelope should remain separate: Frame samples
  `SectionForces` at xi points; Truss reads constant axial force. These
  are different enough to warrant separate code.

**This is a "wait for the second implementation" decision, not a "never"
decision.** The right time to extract shared code is when you have two
concrete implementations to abstract over, not before.

---

## 6. Summary

| Question | Answer | Rationale |
|---|---|---|
| LoadCase/LoadCombination extend to 3D Truss? | **YES WITH LIMITED CHANGE** | Core storage is DOF-agnostic; ~75 lines of new code, 0 duplicated framework |
| LoadSource/provenance unified? | **YES (already DOF-agnostic)** | Just needs adoption: add `load_source` field to `TrussAnalysisResult3D` |
| Envelope shared infrastructure? | **NOT YET** | Only 1 concrete impl; differences are physical; shared parts already shared |

---

## 7. Implementation Path (when LoadCase for 3D Truss is prioritized)

```text
1. Add LoadCase::nodal_load_3d(node, fx, fy, fz)           ~10 lines
2. Add TrussSolver3D::solve_case(&LoadCase)                 ~40 lines
3. Add TrussSolver3D::solve_combination(&LoadCombination)   ~20 lines
4. Add load_source: LoadSource to TrussAnalysisResult3D      ~5 lines
5. Add TrussSolver3D::solve_cases(&[LoadCase]) (batch)      ~15 lines
6. Tests                                                    ~100 lines
                                                     Total: ~190 lines
```

No changes to `LoadCase`, `LoadCombination`, `LoadSource`, or
`LoadCombinationTerm` structs. No new generic traits. No duplicated
combination framework.

---

## 8. Changes Made

```text
Audit-only; no implementation changes.
```

---

## 9. Validation

Not applicable — no code changes.

---

## 10. Final Classification

```text
P0: (none)
P1: (none)
P2: (none)

LoadCase readiness for 3D Truss:
YES WITH LIMITED CHANGE (~75 lines, 0 duplicated framework)

LoadSource unification:
YES (already DOF-agnostic, needs adoption only)

Envelope shared infrastructure:
NOT YET (wait for second concrete implementation)

Architecture:
KEEP (independent Result APIs, shared primitives already extracted)

Generic abstraction:
NO (Phase 113 decision stands)

Next phase:
3D Truss LoadCase support is the lowest-cost highest-value next step.
Envelope shared infrastructure should be reconsidered after 3D Truss
has a concrete Envelope implementation.
```
