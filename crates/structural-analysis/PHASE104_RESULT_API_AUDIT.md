# Phase 104 — Result API & Serialization Architecture Audit

**Date**: 2026-09-24  
**Baseline**: commit `23f527a` (Phase 103)  
**Status**: Complete  
**Production changes**: 7 rustdoc link fixes (P2)  
**New features**: None  

---

## 1. Scope

Audit the Result / Envelope / Geometry / Equilibrium data model established in
Phases 102–103 to determine whether it is stable enough to serve as the public
data foundation for future Visualization / Export / GUI layers.

**Audit-only phase**: no FEM math changes, no new features, no serialization
framework introduced. The only production change is fixing 7 rustdoc broken /
private intra-doc links to achieve `cargo doc` 0 warnings.

---

## 2. Result Type Consistency — Full API Comparison

### 2.1 Ownership & Trait Matrix

| Feature | `FrameAnalysisResult` | `BeamAnalysisResult<'a>` | `TrussAnalysisResult` |
|---------|-----------------------|-------------------------|----------------------|
| **Ownership** | owned (`BeamSolver` + `FrameModel`) | borrows `&'a BeamSolver` | owned pure snapshot |
| **Clone** | NO (`SparseMatrix` not `Clone`) | YES | YES |
| **Debug** | custom (non-exhaustive) | custom (non-exhaustive) | derived |
| **Independent lifetime** | YES | NO (tied to solver) | YES |
| **load_source** | `Option<String>` | N/A | N/A |

### 2.2 Displacement & Reaction API

| Method | `FrameAnalysisResult` | `BeamAnalysisResult<'a>` | `TrussAnalysisResult` |
|--------|-----------------------|-------------------------|----------------------|
| `displacements()` | `&[f64]` (borrowed) | `&[f64]` (borrowed) | `Vec<f64>` (owned field) |
| `displacement(node, dof)` | `Result<f64>` (handle + `Dof`) | `Result<BeamNodalDisplacement>` (index) | `Result<f64>` (index + `TrussDof`) |
| `reactions()` | `Vec<f64>` (recomputed `K·u − f`) | `Vec<f64>` (snapshot at construction) | `Vec<f64>` (owned field) |
| `reaction(node, dof)` | `Result<f64>` (handle + `Dof`) | `Result<BeamReaction>` (index) | `Result<f64>` (index + `TrussDof`) |

### 2.3 Element Force API

| Method | `FrameAnalysisResult` | `BeamAnalysisResult<'a>` | `TrussAnalysisResult` |
|--------|-----------------------|-------------------------|----------------------|
| End forces (local) | `member_end_forces(MemberHandle) → [f64; 6]` | `element_end_forces(usize) → [f64; 6]` | N/A |
| End forces (global) | `member_end_forces_global(MemberHandle) → [f64; 6]` | N/A | N/A |
| Section forces | `section_forces(MemberHandle, xi) → SectionForces` | `section_forces(usize, xi) → SectionForces` | N/A |
| Axial force | N/A | N/A | `axial_force(usize) → f64` (owned field lookup) |
| `sample_forces(n)` | `Vec<BeamForceSample>` | `Vec<BeamForceSample>` | N/A |
| `diagram(n)` | `BeamForceDiagram` | `BeamForceDiagram` | N/A |

### 2.4 Geometry API

| Method | `FrameAnalysisResult` | `BeamAnalysisResult<'a>` | `TrussAnalysisResult` |
|--------|-----------------------|-------------------------|----------------------|
| Node position | `node_position(NodeHandle) → Option<Point>` | `node_position(usize) → Option<Point>` | `node_position(usize) → Option<(f64, f64)>` |
| Element nodes | `member_nodes(MemberHandle) → Option<(NodeHandle, NodeHandle)>` | `element_nodes(usize) → Option<(usize, usize)>` | `element_endpoints(usize) → Option<(usize, usize)>` |
| Element length | `member_length(MemberHandle) → Option<f64>` | N/A | N/A |

### 2.5 Equilibrium API

| Method | `FrameAnalysisResult` | `BeamAnalysisResult<'a>` | `TrussAnalysisResult` |
|--------|-----------------------|-------------------------|----------------------|
| `equilibrium()` | `EquilibriumReport` | `EquilibriumReport` | `TrussEquilibriumReport` |
| Shared computation | `compute_equilibrium(&BeamModel, &[f64])` | same | independent (inline) |
| Conditioning-aware tol | YES (`EQUILIBRIUM_COND_FACTOR`) | YES (shared) | NO (fixed `1e-6`) |
| Moment scale | `Σ|M| + Σ|F|·l_char` | same | `tolerance` only |

---

## 3. Findings

### 3.1 P0 / P1 Issues

**None.** No correctness defects, no API-breaking inconsistencies, no
performance regressions.

### 3.2 P2 Issues — Fixed in This Phase

| # | File | Line | Issue | Fix |
|---|------|------|-------|-----|
| 1 | `beam_fem.rs` | 462 | `FrameAnalysisResult::equilibrium` unresolved link | → `crate::frame::FrameAnalysisResult::equilibrium` |
| 2 | `beam_fem.rs` | 3512 | `FrameAnalysisResult::member_end_forces` unresolved link | → `crate::frame::FrameAnalysisResult::member_end_forces` |
| 3 | `beam_fem.rs` | 3518 | `Self::element_local_displacement` private link | → plain code span |
| 4 | `beam_fem.rs` | 3519 | `Self::element_equivalent_nodal_forces` private link | → plain code span |
| 5 | `beam_fem.rs` | 3664 | `FrameAnalysisResult::member_end_forces_global` unresolved link | → `crate::frame::FrameAnalysisResult::member_end_forces_global` |
| 6 | `frame.rs` | 1310 | `BeamForceSample` unresolved link | → `crate::beam_fem::BeamForceSample` |
| 7 | `frame.rs` | 1328 | `BeamForceDiagram` unresolved link | → `crate::beam_fem::BeamForceDiagram` |

### 3.3 P2 Issues — Deferred

| # | Issue | Decision | Rationale |
|---|-------|----------|-----------|
| D1 | `TrussEquilibriumReport` moment tolerance lacks `l_char` scaling | DEFER | Pre-existing from Phase 103. The moment check uses `self.tolerance` (force-based) without a proper moment scale `Σ|F|·l_char`. For structures with large `l_char` and small forces, the moment check may be too loose. Fix requires careful design to match the Frame/Beam conditioning-aware pattern. Not a regression; deferred to a future phase. |
| D2 | `member_length` not available on `BeamAnalysisResult` / `TrussAnalysisResult` | DEFER | Can be trivially added if a consumer needs it. Geometry is already accessible via `node_position`. Not a defect. |
| D3 | No `serde` derives on result types | DEFER | Intentional — no serde dependency. All result types are plain data (`Vec<f64>`, `Option<String>`, `usize`, `f64`, `SectionForces`) and would be serde-compatible with minimal effort (add `#[derive(serde::Serialize)]` + a serde feature gate). No structural changes needed now. |

---

## 4. API Stability Decisions

### 4.1 KEEP — No Change Needed

| Item | Rationale |
|------|-----------|
| `node_position` return type: `Point` (Frame/Beam) vs `(f64, f64)` (Truss) | Intentional. Truss has 2 DOF/node, no bending; `(f64, f64)` is a simpler, appropriate type. Changing it would add a `Point` import for no engineering value. |
| `member_nodes` (Frame) vs `element_nodes` (Beam/Truss) naming | Reflects the handle-vs-index API level distinction. Frame uses `NodeHandle`/`MemberHandle` throughout; Beam/Truss use raw `usize`. Consistent within each level. |
| `element_endpoints` (Truss) vs `element_nodes` (Beam) naming | Minor. "Endpoints" is arguably more natural for a truss element (which has endpoints, not nodes with rotational DOF). Not worth a breaking change. |
| `EquilibriumReport` vs `TrussEquilibriumReport` | Different physics. Truss has no `Rz` DOF, no reaction moments. `TrussEquilibriumReport` correctly omits `moment_tolerance` / conditioning-aware tolerance. Unifying would require either a less precise shared type or a trait object — both worse. |
| `load_source: Option<String>` | Simple provenance tag. Only `FrameAnalysisResult` has `solve_case` / `solve_combination`. The `String` encoding (`"case:{name}"`, `"combination:{name}"`) is adequate. An enum (`LoadSource::Direct / Case(String) / Combination(String)`) would be over-engineering for a tag that is only set by the solver and read for display. |
| `Envelope::member_samples` returns `&[]` on out-of-range | Graceful degradation. Consistent with Rust slice patterns. An `Option<&[EnvelopeSample]>` would force every caller to unwrap. |
| `FrameAnalysisResult` not `Clone` | Inherent constraint: `BeamSolver` contains `SparseMatrix` which is not `Clone` (it owns CSR storage). `Envelope` correctly takes `&[&FrameAnalysisResult]` to work around this. Making `SparseMatrix` `Clone` would be a `section-properties` API change — out of scope. |
| `BeamAnalysisResult<'a>` borrows solver | Correct for the beam-level API. The solver is the single source of truth; borrowing avoids cloning the stiffness matrix. The lifetime constraint is documented and matches the usage pattern (inspect results while solver is alive). |
| `TrussAnalysisResult` owned snapshot | Correct for truss. Truss results are small (displacements, reactions, axial forces, geometry) and cloning is cheap. The owned snapshot allows the result to outlive the solver, which is useful for multi-case envelopes. |

### 4.2 CHANGE — Fixed in This Phase

| Item | Action |
|------|--------|
| 7 rustdoc broken/private intra-doc links | Fixed (see §3.2). `cargo doc` now produces 0 warnings. |

### 4.3 DEFER — Future Phase

| Item | Target Phase | Rationale |
|------|-------------|-----------|
| `TrussEquilibriumReport` moment tolerance with `l_char` scaling | Future | Needs careful design to match Frame/Beam conditioning-aware pattern without over-complicating the truss API. |
| `member_length` on `BeamAnalysisResult` / `TrussAnalysisResult` | On demand | Trivial to add; not needed by any current consumer. |
| `serde` derives on result types | Phase 105+ | Add as a feature-gated opt-in. Types are already serde-ready (plain data). |

---

## 5. Envelope Audit

### 5.1 Data Model

```rust
pub struct EnvelopeSample { pub member_index: usize, pub xi: f64, pub min: SectionForces, pub max: SectionForces }
pub struct Envelope { pub samples: Vec<EnvelopeSample>, pub n_results: usize, pub n_members: usize, pub n_per_member: usize }
```

- **Ownership**: owned, `Clone`. No solver/model references. ✅
- **`EnvelopeSample`**: `Debug, Clone, Copy, PartialEq`. Pure data. ✅
- **`Envelope`**: `Debug, Clone`. Pure data container. ✅

### 5.2 Construction

`Envelope::from_frame_results(&[&FrameAnalysisResult], n_per_member)`

- Takes a slice of references — necessary because `FrameAnalysisResult` is not `Clone`. ✅
- Validates: non-empty results, `n_per_member >= 2`, consistent member count. ✅
- Does **not** call solver methods or re-assemble stiffness matrices. Pure consumer. ✅

### 5.3 Sampling

- Fixed sampling: `xi = 0, 1/(n-1), ..., 1`. Includes both endpoints. ✅
- **Not** event-aware: no extra points at load discontinuities. Documented. ✅
- Interior extrema between sample points may be missed. Documented. ✅

### 5.4 Min/Max Semantics

- `min` and `max` track each component (`N`, `V`, `M`) **independently**. ✅
- **Not** absolute-value envelopes: `min` may be negative, `max` may be positive. ✅
- `max_abs_moment()`, `max_abs_shear()`, `max_abs_axial()` provided for design checks. ✅

### 5.5 Complexity

- `O(cases × members × samples)` per construction. ✅
- Each `section_forces` call is `O(1)` (Phase 102 fix ensures `member_end_forces` is `O(1)`). ✅
- `member_samples(idx)`: `O(1)` slice view. ✅
- `max_abs_*`: `O(samples)`. ✅

**Verdict**: Envelope data model is sound and ready for visualization consumption.

---

## 6. Geometry API Audit

### 6.1 Frame

| Method | Signature | Returns |
|--------|-----------|---------|
| `node_position` | `(NodeHandle) → Option<Point>` | Typed handle → geometric point |
| `member_nodes` | `(MemberHandle) → Option<(NodeHandle, NodeHandle)>` | Typed handle → handle pair |
| `member_length` | `(MemberHandle) → Option<f64>` | Typed handle → scalar |

Consistent with Frame's handle-based API. Does not expose internal `BeamModel` or `BeamSolver`. ✅

### 6.2 Beam

| Method | Signature | Returns |
|--------|-----------|---------|
| `node_position` | `(usize) → Option<Point>` | Index → geometric point |
| `element_nodes` | `(usize) → Option<(usize, usize)>` | Index → index pair |

Consistent with Beam's index-based API. Delegates to `BeamSolver` methods. ✅

### 6.3 Truss

| Method | Signature | Returns |
|--------|-----------|---------|
| `node_position` | `(usize) → Option<(f64, f64)>` | Index → coordinate pair |
| `element_endpoints` | `(usize) → Option<(usize, usize)>` | Index → index pair |

Consistent with Truss's index-based API. Returns `(f64, f64)` instead of `Point` — intentional (see §4.1). ✅

### 6.4 Cross-Type Consistency

The three APIs use different naming (`member_nodes` vs `element_nodes` vs `element_endpoints`) and different return types (`Point` vs `(f64, f64)`, `NodeHandle` vs `usize`). These differences are **intentional** and reflect the three API levels:

- **Frame**: typed handles, `Point` return — highest-level API
- **Beam**: raw indices, `Point` return — mid-level API
- **Truss**: raw indices, `(f64, f64)` return — simplest API for 2-DOF elements

**Verdict**: No unification needed. The differences are deliberate and documented.

---

## 7. Equilibrium API Audit

### 7.1 Frame / Beam (shared)

Both use `compute_equilibrium(&BeamModel, &[f64]) → EquilibriumReport`:

- Sums applied forces (nodal, distributed, point, moments) and support reactions. ✅
- Conditioning-aware tolerance: `effective_rel_tol = max(1e-6, C · λ²_max · ε)`. ✅
- Force and moment scales with `l_char` (characteristic length). ✅
- Unit-invariant verdict. ✅
- `is_balanced()`, `force_tolerance()`, `moment_tolerance()` exposed. ✅

### 7.2 Truss (independent)

`TrussAnalysisResult::equilibrium() → TrussEquilibriumReport`:

- Sums applied nodal forces and support reactions. ✅
- No distributed/point loads (truss elements carry only axial force). ✅
- Fixed tolerance: `1e-6 · Σ|F|`. No conditioning-aware floor. ✅ (deferred, see D1)
- Moment check: `mz_residual.abs() <= tolerance.max(rel * tolerance)`. The `rel * tolerance` term is negligible and does not provide a proper moment scale. ⚠️ (deferred, see D1)

### 7.3 Type Comparison

| Field | `EquilibriumReport` | `TrussEquilibriumReport` |
|-------|---------------------|--------------------------|
| `fx_residual`, `fy_residual`, `mz_residual` | ✅ | ✅ |
| `applied_fx/fy/mz` | ✅ | ✅ |
| `reaction_fx/fy/mz` | ✅ | ✅ |
| `tolerance` (private in Frame) | private | **public** |
| `force_tolerance()` | ✅ | ✅ (returns `tolerance`) |
| `moment_tolerance()` | ✅ | N/A |
| Conditioning-aware `cond_rel_floor` | ✅ | N/A |
| `is_balanced()` | conditioning-aware | fixed `1e-6` |

**Note**: `TrussEquilibriumReport.tolerance` is public while `EquilibriumReport.tolerance` is private (with `force_tolerance()` / `moment_tolerance()` accessors). This is a minor inconsistency but not worth a breaking change — the Truss report is a simpler type and the public field is adequate.

**Verdict**: The two report types serve different physics. Unification is not recommended.

---

## 8. LoadSource Audit

`FrameAnalysisResult::load_source() → Option<&str>`

- `None` for `solve()` / `solve_with()` — direct solve with model loads. ✅
- `Some("case:{name}")` for `solve_case(&LoadCase)`. ✅
- `Some("combination:{name}")` for `solve_combination(&LoadCombination)`. ✅

**Enum vs String**: An enum (`LoadSource::Direct | Case(String) | Combination(String)`) would be more type-safe but:
1. The value is only set by the solver, never constructed by the user.
2. The only consumer use case is display/logging — a string is adequate.
3. Adding an enum would expand the public API surface for no engineering value.

**Verdict**: KEEP `Option<String>`.

---

## 9. Serialization Readiness

### 9.1 Current State

No `serde` dependency. No `#[derive(serde::Serialize)]` on any type. This is
intentional — serialization is a future concern (Phase 105+).

### 9.2 Type Inventory

| Type | Fields | serde-ready? |
|------|--------|-------------|
| `SectionForces` | `axial: f64, shear: f64, moment: f64` | ✅ trivial |
| `BeamForceSample` | `element_index: usize, xi: f64, x: f64, section_forces: SectionForces` | ✅ trivial |
| `BeamForceDiagram` | `samples: Vec<BeamForceSample>` | ✅ trivial |
| `EnvelopeSample` | `member_index: usize, xi: f64, min: SectionForces, max: SectionForces` | ✅ trivial |
| `Envelope` | `samples: Vec<EnvelopeSample>, n_results: usize, n_members: usize, n_per_member: usize` | ✅ trivial |
| `EquilibriumReport` | 12 fields (all `f64`, one private) | ✅ needs field visibility decision |
| `TrussEquilibriumReport` | 10 public `f64` fields | ✅ trivial |
| `TrussAnalysisResult` | all public fields (`Vec<f64>`, `Option<String>`, `Vec<(f64,f64)>`, `Vec<(usize,usize)>`, `Vec<(usize,usize,f64)>`) | ✅ trivial |
| `FrameAnalysisResult` | private fields (`BeamSolver`, `FrameModel`) | ❌ not directly serializable |
| `BeamAnalysisResult<'a>` | borrows solver | ❌ not directly serializable |

### 9.3 Serialization Strategy (Future)

For `FrameAnalysisResult` and `BeamAnalysisResult`, serialization would go
through a **snapshot** type (similar to `TrussAnalysisResult`):

```text
FrameAnalysisResult → FrameResultSnapshot (owned, serializable)
BeamAnalysisResult  → BeamResultSnapshot  (owned, serializable)
```

This is the same pattern `TrussSolver::results()` already uses. The snapshot
would copy displacements, reactions, and geometry into plain owned data.

**No structural changes needed now.** The existing types are the right shape;
serialization is an additive feature, not a refactor.

**Verdict**: DEFER to Phase 105+. Types are serde-ready.

---

## 10. Public API Audit

### 10.1 `cargo doc` Warnings

**Before**: 7 warnings (5 broken intra-doc links, 2 private intra-doc links).  
**After**: 0 warnings.  
**Command**: `cargo doc -p structural-analysis --no-deps`

### 10.2 Re-exports (`lib.rs`)

```rust
pub use crate::beam_fem::{BeamAnalysis, BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver, Dof, EndRelease, FemError};
pub use crate::frame::{EquilibriumReport, FrameAnalysisResult, FrameModel, FrameSolver, MemberHandle, NodeHandle, StructuralDiagnostic};
pub use crate::load::{LoadCase, LoadCombination};
pub use crate::postprocessing::{Envelope, EnvelopeSample};
pub use crate::truss::{TrussAnalysisResult, TrussDof, TrussElement, TrussEquilibriumReport, TrussModel, TrussNode, TrussSolver};
```

All public types are re-exported at crate root. ✅  
`BeamForceSample`, `BeamForceDiagram`, `SectionForces` are **not** re-exported at
crate root — they are accessible via `structural_analysis::beam_fem::`. This is
intentional: they are beam-module types, not top-level API.

**Verdict**: KEEP. The re-export structure reflects the module hierarchy.

---

## 11. Verification

| Check | Result |
|-------|--------|
| `cargo test --release -p structural-analysis --lib --bins --tests` | 538 tests pass |
| `cargo test --release -p structural-analysis --doc` | 10 doctests pass |
| `cargo doc -p structural-analysis --no-deps` | 0 warnings |
| `cargo fmt --check` | PASS |
| `cargo check --workspace` | PASS (3 pre-existing pardiso warnings) |

---

## 12. Summary

| Category | Count | Details |
|----------|-------|---------|
| P0 issues | 0 | — |
| P1 issues | 0 | — |
| P2 issues (fixed) | 7 | rustdoc broken/private intra-doc links |
| P2 issues (deferred) | 3 | D1: truss moment tolerance, D2: member_length on Beam/Truss, D3: serde derives |
| KEEP decisions | 9 | See §4.1 |
| DEFER decisions | 3 | See §4.3 |
| Production changes | 7 doc link fixes | No API changes, no math changes, no new features |

**Conclusion**: The Result / Envelope / Geometry / Equilibrium data model
established in Phases 102–103 is **stable** and ready to serve as the public
data foundation for future Visualization / Export / GUI layers. The only changes
in this phase are 7 rustdoc link fixes to achieve `cargo doc` 0 warnings.
