# Phase 119 — Cross-Model Envelope Architecture Audit

**Date:** 2026-10-09
**Base commit:** `2ff6ac9` (Phase 118)
**Scope:** Audit existing post-processing / Envelope design to determine the lowest-cost,
highest-value architecture for cross-model envelope support.
**Principle:** Audit first; zero production change is a valid result.

---

## 1. Baseline

```
HEAD = 2ff6ac9 (Phase 118: 3D Truss LoadCase/LoadCombination deep audit)
Working tree: clean (no uncommitted code changes)
Untracked: PHASE118_AUDIT.md and prior phase docs (not part of this audit)
```

---

## 2. Existing Envelope Implementation Inventory

### 2.1 Types

| Type | Location | Model scope | Physics dependency |
|------|----------|-------------|-------------------|
| `Extremum` | `postprocessing.rs:72` | **Model-agnostic** | None — pure `f64` min/max |
| `LoadSource` | `load.rs:435` | **Model-agnostic** | None — DOF-agnostic enum |
| `LoadCombinationTerm` | `load.rs:421` | **Model-agnostic** | None |
| `SectionForceSources` | `postprocessing.rs:20` | Frame only | References `SectionForces` components (N, V, M) |
| `EnvelopeSample` | `postprocessing.rs:51` | Frame only | Uses `SectionForces` (axial, shear, moment) |
| `NodeDisplacementSample` | `postprocessing.rs:112` | Frame only | Hardcoded `ux, uy, rz` (3 DOF) |
| `NodeReactionSample` | `postprocessing.rs:130` | Frame only | Hardcoded `rx, ry, mz` (3 DOF) |
| `Envelope` | `postprocessing.rs:152` | Frame only | `from_frame_results` only |

### 2.2 Methods

| Method | Scope | Notes |
|--------|-------|-------|
| `Extremum::new()` | Shared | Sentinel `min=+∞, max=-∞` |
| `Extremum::update(value, source)` | Shared | Strict `<`/`>` — ties keep first |
| `Extremum::is_populated()` | Shared | Checks `min_source.is_some()` |
| `Envelope::from_frame_results()` | Frame only | Only constructor |
| `Envelope::source(index)` | Frame only | Returns `&LoadSource` |
| `Envelope::member_samples(idx)` | Frame only | Section force samples |
| `Envelope::node_displacement(idx)` | Frame only | Per-node displacement envelope |
| `Envelope::support_reaction(idx)` | Frame only | Per-node reaction envelope |
| `Envelope::max_abs_moment/shear/axial()` | Frame only | Summary queries |

### 2.3 AnalysisResult comparison

| Feature | `FrameAnalysisResult` | `TrussAnalysisResult` | `TrussAnalysisResult3D` |
|---------|----------------------|----------------------|------------------------|
| DOF per node | 3 (ux, uy, rz) | 2 (ux, uy) | 3 (ux, uy, uz) |
| `displacements()` | `&[f64]` | `Vec<f64>` (owned) | `&[f64]` |
| `reactions()` | `&[f64]` | `Vec<f64>` (computed) | `Vec<f64>` (computed) |
| `section_forces()` | `SectionForces {N, V, M}` | N/A | N/A |
| `axial_force(idx)` | N/A | `Result<f64>` | `Result<f64>` |
| `load_source()` | `&LoadSource` | **N/A** | `&LoadSource` |
| `support_dofs()` | `Vec<(usize, Dof)>` (pub(crate)) | **N/A** | **N/A** |
| `n_nodes()` / `n_members()` | Both | `n_nodes` / `n_elements` | `n_nodes` / `n_elements` |
| Envelope support | ✅ | ❌ | ❌ |

---

## 3. Physical Quantity Differences

| Quantity | Frame / Beam | 2D Truss | 3D Truss |
|----------|-------------|----------|----------|
| Node displacement | ux, uy, rz (translation + rotation) | ux, uy (translation only) | ux, uy, uz (3 translations) |
| Node reaction | Rx, Ry, Mz (force + moment) | Rx, Ry (force only) | Rx, Ry, Rz (3 forces) |
| Member internal force | N, V, M (axial, shear, moment) | N (axial only) | N (axial only) |
| Member end forces | [N_i, V_i, M_i, N_j, V_j, M_j] | N/A | N/A |
| LoadCase / LoadCombination | ✅ | ❌ | ✅ |
| LoadSource provenance | ✅ | ❌ | ✅ |
| Envelope | ✅ | ❌ | ❌ |

**Key observation:** 2D Truss has no LoadCase support at all — it uses direct
`add_nodal_force` on the model. Therefore 2D Truss Envelope is not applicable until
LoadCase support is added (out of scope for this audit).

3D Truss has LoadCase/LoadCombination support (Phase 117) and `load_source()`, but no
Envelope and no `support_dofs()`.

---

## 4. Envelope Semantics Audit

| Check | Status | Evidence |
|-------|--------|----------|
| Min and max tracked independently | PASS | `Extremum::update` uses separate `<` and `>` comparisons |
| Signed (not absolute-value) envelope | PASS | Doc: "not absolute-value envelopes"; code confirms |
| Zero extrema handled correctly | PASS | Zero is a valid `f64`; no special-casing |
| Support reactions vs free-DOF residuals distinguished | PASS | `compute_support_reactions` iterates only `r.support_dofs()`; test `reaction_envelope_excludes_free_dof_residuals` |
| Controlling load case tracked | PASS | Every `Extremum` records `min_source`/`max_source` index into `sources` |
| Duplicate case names unambiguous | PASS | Source index disambiguates; test `duplicate_case_names_remain_unambiguous_in_envelope` |
| Empty result set rejected | PASS | `from_frame_results` returns `InvalidInput`; test verifies |
| NaN / infinity rejected | PASS | `require_finite` and `require_node_finite` check all sampled values |
| Tie-breaking: first input wins | PASS | Strict `<`/`>` (not `<=`/`>=`); test `envelope_ties_keep_the_first_source` |
| Input order affects source indices (predictable) | PASS | Source index = position in input slice; deterministic |
| Cross-model compatibility checked | PASS | `n_members` and `n_nodes` validated; tests for mismatch |
| Prescribed displacement flagged in source | PASS | `LoadSource::LoadCase { has_prescribed_displacements }`; test `prescribed_displacement_envelope_is_flagged_and_isolated` |
| Combination terms preserved | PASS | `LoadSource::LoadCombination { terms: Vec<LoadCombinationTerm> }`; test `combination_source_is_preserved_in_envelope` |

**All 13 checks: PASS.** No gaps found in existing Frame Envelope semantics.

---

## 5. Shared Algorithm vs Physical Type Boundary

### Option A — Fully Independent Envelopes

Each model maintains its own envelope types and aggregation logic.

- **Pros:** No abstraction overhead; each type exactly matches its model's physics.
- **Cons:** If 3D Truss Envelope is added, `Extremum` logic is duplicated (or imported
  as a public type from `postprocessing`).
- **Reality:** `Extremum` is already `pub` and model-agnostic. Option A is already
  partially in effect — the shared numeric core exists but physical types are separate.

### Option B — Shared Numeric Aggregation Infrastructure

Retain model-specific physical API but reuse physics-independent capabilities.

- **Pros:** Eliminates duplication of min/max/source tracking logic.
- **Cons:** Requires discipline to keep the shared layer physics-free.
- **Reality:** **Already implemented.** `Extremum` and `LoadSource` are the shared layer.
  They have zero physics dependencies. Any future Truss/3D Truss Envelope can use them
  directly without new abstractions.

### Assessment

The shared numeric core (`Extremum`, `LoadSource`) already exists and is sufficient.
There is **no duplicated aggregation logic** to extract — `Extremum::update` is the single
implementation, used by Frame Envelope and available to any future consumer.

**No action needed.** The architecture is already at Option B for the parts that matter.

---

## 6. Shared Type Minimum Scope

| Candidate | Share? | Rationale | Migration cost | API risk |
|-----------|--------|-----------|---------------|----------|
| `Extremum` | **Already shared** | Pure `f64` min/max + source index; no physics | 0 | None |
| `LoadSource` | **Already shared** | DOF-agnostic enum; used by Frame and 3D Truss | 0 | None |
| `LoadCombinationTerm` | **Already shared** | Name + factor; no physics | 0 | None |
| Aggregation algorithm (`update`) | **Already shared** | Single impl in `Extremum::update` | 0 | None |
| Source index (`Option<usize>`) | **Already shared** | Positional index into `sources` | 0 | None |
| `NodeDisplacementSample` | **No** | DOF count differs: Frame=3(ux,uy,rz), 2D Truss=2(ux,uy), 3D Truss=3(ux,uy,uz) | High | Medium |
| `NodeReactionSample` | **No** | DOF types differ: Frame has moment, Truss does not | High | Medium |
| `EnvelopeSample` (member forces) | **No** | Frame has N/V/M; Truss has only N | High | Medium |
| `Envelope` (top-level) | **No** | Constructor tied to specific Result type | High | Medium |

**Conclusion:** The shareable types are already shared. The physical types cannot be
shared without either (a) generic parameterization that would complicate the API, or
(b) a `Vec<Extremum>` approach that loses field-level readability (e.g., `sample.ux`
vs `sample.displacements[0]`).

---

## 7. Controlling Load Case and Provenance

### 7.1 Questions and Answers

1. **Is `LoadSource` sufficient to represent controlling source?**
   — **YES.** Three variants cover all solve paths:
   - `ModelLoads` — `frame.solve()` / `truss3d.solve()`
   - `LoadCase { name, has_prescribed_displacements }` — `solve_case()`
   - `LoadCombination { name, terms }` — `solve_combination()`

2. **Is an independent input sample index needed?**
   — **NO.** The `source: usize` index into `Envelope::sources` is sufficient.
   It uniquely identifies which input result governed each extremum.

3. **How are equal-value extrema resolved?**
   — **First input wins.** `Extremum::update` uses strict `<` and `>`, so the first
   result achieving a value retains governance. Test `envelope_ties_keep_the_first_source`
   verifies this.

4. **Is behavior predictable after case list reordering?**
   — **YES.** Extremum *values* are unchanged; *source indices* change to reflect the
   new input order. This is correct and expected — the source index is a positional
   index, not a stable ID.

5. **Is combination factor information lost?**
   — **NO.** `LoadSource::LoadCombination` carries `terms: Vec<LoadCombinationTerm>`,
   preserving each case name and factor. Test `combination_source_is_preserved_in_envelope`
   verifies this.

### 7.2 Provenance format

Current format is adequate. No changes recommended.

---

## 8. Model Compatibility and Topology Identity

### 8.1 Current Validation

`Envelope::from_frame_results` checks:
- All results have the same `n_members`
- All results have the same `n_nodes`

### 8.2 Scenario Analysis

| Scenario | Current detection | Risk | Assessment |
|----------|------------------|------|------------|
| Same node count, different coordinates | **Not detected** | Envelope aggregates per-node values; mismatched coordinates produce meaningless results | User error — caller must ensure same model |
| Same member count, different connectivity | **Not detected** | Member force samples would be from different elements | User error — same as above |
| Different materials/sections | **Not detected** | Results are from different stiffness matrices | Not applicable — Envelope only aggregates already-computed values |
| Different node numbering, same structure | **Not detected** | Per-node arrays would be misaligned | User error |
| Same topology, different load cases | **Correct** | This is the intended use case | PASS |

### 8.3 Compatibility Approaches

| Approach | Cost | Benefit | Recommendation |
|----------|------|---------|----------------|
| A. Current count validation | 0 | Catches size mismatches | **KEEP** |
| B. Explicit model identity ID | Low | Could detect topology mismatch | Adds API surface for marginal benefit |
| C. Topology signature (hash) | Medium | Detects coordinate/connectivity mismatch | Over-engineering for a workspace-only crate |
| D. Compatibility object | Medium | Explicit user attestation | Unnecessary complexity |

**Recommendation: CURRENT CHECK SUFFICIENT.**

The Envelope constructor takes `&[&FrameAnalysisResult]` — these are already solved
results from the same model. The typical workflow is:

```rust
let r1 = frame.solve_case(&dead)?;
let r2 = frame.solve_case(&live)?;
let envelope = Envelope::from_frame_results(&[&r1, &r2], 5)?;
```

Mixing results from different models is a user error that no amount of validation can
fully prevent (e.g., same counts but different coordinates). The current count check
catches the most common mistake. Adding topology fingerprints would increase complexity
without meaningful safety gain.

---

## 9. Memory, Performance, and Result Lifetime

### Strategies compared

| Strategy | Memory | Recomputation | Traceability | Complexity |
|----------|--------|---------------|-------------|------------|
| 1. Store all results, then aggregate (current) | O(N × R) | None | Full | Low |
| 2. Stream: keep only extrema | O(E) per DOF | None | Source index only | Medium |
| 3. Store extrema + recompute on demand | O(E) | Per-query | Full (via recompute) | High |

Where N = number of load cases, R = result size, E = envelope size.

### Assessment

- **Strategy 1 (current):** Best for typical engineering workflows (N < 100 cases).
  The user owns the results and passes references. The Envelope itself stores only
  aggregated extrema + sources, not full results.
- **Strategy 2:** Would require an online API (`EnvelopeBuilder::add_result(&r)`).
  Saves memory but loses the ability to query individual results. Not needed at
  current scale.
- **Strategy 3:** Over-engineered. No user has requested lazy recomputation.

**No change recommended.** The current strategy is appropriate for the crate's scope.

---

## 10. 3D Frame Future Compatibility

3D Frame is not implemented. When it arrives, it will need:

| Quantity | Components | Existing shared layer sufficient? |
|----------|-----------|--------------------------------|
| Displacement | ux, uy, uz, θx, θy, θz (6 DOF) | `Extremum` ✅; `NodeDisplacementSample` ❌ (needs 6-DOF variant) |
| Reaction | Rx, Ry, Rz, Mx, My, Mz (6 DOF) | `Extremum` ✅; `NodeReactionSample` ❌ (needs 6-DOF variant) |
| Member forces | N, Vy, Vz, My, Mz, T (6 components) | `Extremum` ✅; `EnvelopeSample` ❌ (needs 6-component variant) |

### Key findings

- **`Extremum` and `LoadSource` are sufficient** for any future model. They are
  pure numeric / provenance types with no physics dependency.
- **Physical sample types must be model-specific.** A 6-DOF displacement sample cannot
  reuse the 3-DOF `NodeDisplacementSample`. Forcing it (e.g., via `Vec<Extremum>`)
  would lose field-level readability (`sample.ux` vs `sample.dofs[0]`).
- **No fixed-length array risk:** Each model defines its own sample types with exactly
  the fields it needs. No `Vec<f64>` position conventions are introduced.
- **No generic trait needed:** `from_frame_results` / `from_truss3d_results` /
  `from_frame3d_results` are separate constructors. No `Result trait` or
  `Envelopeable` trait is needed.

**No preparatory work recommended.** 3D Frame should define its own sample types
when it is implemented.

---

## 11. Disallowed Refactors — Compliance

| Constraint | Status |
|-----------|--------|
| No `Dimension` trait | ✅ Not introduced |
| No `Element` trait | ✅ Not introduced |
| No `Result` trait | ✅ Not introduced |
| No `Constraint` trait | ✅ Not introduced |
| No generic `StructuralResult` | ✅ Not introduced |
| No generic `Envelope3D` | ✅ Not introduced |
| No third crate | ✅ Not introduced |
| `section-properties` unmodified | ✅ Not touched |

---

## 12. Test and Verification Results

| Command | Result |
|---------|--------|
| `cargo fmt --all -- --check` | ✅ PASS |
| `cargo check --workspace` | ✅ PASS (3 pre-existing pardiso warnings) |
| `cargo test -p structural-analysis` | ✅ PASS (all suites, 0 failures) |
| `cargo test --doc --workspace` | ✅ PASS (13 doctests, 0 failures) |
| `cargo test --examples --workspace` | ✅ PASS (0 examples) |
| `cargo clippy --workspace --all-targets --all-features` | ✅ PASS (all warnings pre-existing in `section-properties`) |
| `cargo test --workspace` | ⏱️ TIMEOUT — PRE-EXISTING CONFIRMED (`section-properties` FEM benchmarks) |

---

## 13. Problem List

| Category | Count | Details |
|----------|-------|---------|
| P0 (critical) | 0 | — |
| P1 (high) | 0 | — |
| P2 (medium) | 0 | — |
| P3 (low) | 0 | — |

**No problems found.** The existing Envelope architecture is sound.

---

## 14. Final Conclusions

```
Shared aggregation core:
YES (already exists — Extremum + LoadSource are pub, model-agnostic, zero physics deps)

Shared physical Result API:
NO (FrameAnalysisResult, TrussAnalysisResult, TrussAnalysisResult3D are distinct;
    DOF counts and physical quantities differ fundamentally)

Shared Envelope API:
NOT YET (only Frame has Envelope; 3D Truss has LoadCase but no Envelope;
         2D Truss has neither LoadCase nor Envelope;
         shared numeric core is ready for any future consumer)

Topology compatibility:
CURRENT CHECK SUFFICIENT (n_members + n_nodes validation;
                           deeper checks would add complexity without meaningful safety gain)

3D Frame:
DEFERRED (no preparatory work needed; Extremum + LoadSource are sufficient;
          physical sample types should be defined when 3D Frame is implemented)

Generic traits:
NO (no Dimension/Element/Result/Envelopeable trait needed)

Third crate:
NO

Production changes:
NONE
```

---

## 15. Recommended Phase 120

**Implement 3D Truss Envelope** — the lowest-cost, highest-value next step.

### Rationale

- 3D Truss already has `LoadCase`/`LoadCombination` support (Phase 117) and `load_source()`.
- The shared numeric core (`Extremum`, `LoadSource`) is ready — no new abstractions needed.
- 3D Truss Envelope would provide:
  - Per-node displacement envelope: `ux`, `uy`, `uz` (3 `Extremum` per node)
  - Per-element axial force envelope: 1 `Extremum` per element
  - Per-node reaction envelope: `rx`, `ry`, `rz` (3 `Extremum` per node)
  - Source provenance via existing `LoadSource`

### Estimated scope

- New types: `TrussEnvelope3D`, `TrussNodeDisplacementSample3D`, `TrussNodeReactionSample3D`,
  `TrussAxialForceSample3D` (or similar)
- New constructor: `TrussEnvelope3D::from_results(&[&TrussAnalysisResult3D])`
- No changes to existing types (`Extremum`, `LoadSource` already shared)
- No changes to `section-properties`
- Tests: ~10–15 new tests covering the same semantics audited in Section 4

### What NOT to do in Phase 120

- Do not add 2D Truss Envelope (2D Truss lacks LoadCase support — prerequisite gap)
- Do not add generic `Envelopeable` trait
- Do not refactor existing `Envelope` (Frame) — it is correct and stable
- Do not modify `section-properties`
