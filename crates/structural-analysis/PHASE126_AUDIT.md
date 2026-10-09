# Phase 126 — 3D Truss Envelope Topology Audit

**Date:** 2026-10-09  
**Base commit:** `385f1ab` (Phase 125)  
**Scope:** Audit `Truss3DEnvelope` topology validation path against the verified 2D `TrussEnvelope` behavior.

---

## 1. Objective

Determine whether `Truss3DEnvelope::from_results()` validates topology (node coordinates, element connectivity) to the same standard as `TrussEnvelope::from_results()` (2D), and add regression tests for any gaps.

---

## 2. 3D Support Confirmation

| Component | Status | Location |
|-----------|--------|----------|
| `Truss3DEnvelope` struct | ✅ Exists | `postprocessing.rs:492` |
| `Truss3DEnvelope::from_results()` | ✅ Exists | `postprocessing.rs:520` |
| `TrussAnalysisResult3D.node_coords` | ✅ `pub Vec<(f64,f64,f64)>` | `truss3d.rs:1004` |
| `TrussAnalysisResult3D.element_nodes` | ✅ `pub Vec<(usize,usize)>` | `truss3d.rs:1006` |
| `TrussAnalysisResult3D.constrained_dofs` | ✅ `pub Vec<(usize,usize)>` | `truss3d.rs:1014` |
| 3D truss element/solver | ✅ Exists | `truss3d.rs` |
| Existing envelope tests | ✅ 15 tests (p120) + 5 tests (p121) | `tests/truss3d.rs` |

**Conclusion:** Full 3D truss envelope support exists. Topology data (`node_coords`, `element_nodes`) is available as public fields on `TrussAnalysisResult3D`.

---

## 3. Topology Contract Audit

### 3.1 What the 3D envelope validates

`Truss3DEnvelope::from_results()` (`postprocessing.rs:520–559`) checks:

1. **Non-empty input** — rejects empty slice with `InvalidInput`.
2. **Node count agreement** — all results must have the same `n_nodes`.
3. **Element count agreement** — all results must have the same `n_elements`.
4. **Non-finite values** — displacements, reactions, and axial forces are checked with `is_finite()`.

### 3.2 What the 3D envelope does NOT validate

- **Node coordinates** — no comparison of `node_coords` across results.
- **Element connectivity** — no comparison of `element_nodes` across results.

### 3.3 What the 2D envelope validates (for comparison)

`TrussEnvelope::from_results()` (`postprocessing.rs:798–863`) checks all of the above **plus**:

5. **Node coordinates** — `validate_topology()` compares each node's `(x, y)` across results within a relative tolerance of `1e-9`.
6. **Element connectivity** — exact `Vec` equality of `element_nodes`.

### 3.4 NaN/∞ coordinate handling

| Model | Coordinate check? | NaN/∞ risk |
|-------|-------------------|------------|
| 2D `TrussEnvelope` | ✅ Yes (1e-9 rel tol) | NaN/∞ passes validation (IEEE 754), but no correctness impact — non-finite *values* are caught separately. (Phase 125 P3) |
| 3D `Truss3DEnvelope` | ❌ No | N/A — coordinates are never compared. |

### 3.5 Reverse endpoint ordering

Neither 2D nor 3D envelope checks for reversed element endpoints (e.g., `(0,1)` vs `(1,0)`). The 2D envelope requires exact `Vec` equality, so reversed ordering would be rejected. The 3D envelope does not check connectivity at all.

---

## 4. 2D vs 3D Comparison Table

| Feature | `TrussEnvelope` (2D) | `Truss3DEnvelope` (3D) |
|---------|----------------------|------------------------|
| DOF per node | 2 (ux, uy) | 3 (ux, uy, uz) |
| Empty input rejected | ✅ | ✅ |
| Node count check | ✅ | ✅ |
| Element count check | ✅ | ✅ |
| **Node coordinate check** | ✅ (1e-9 rel tol) | ❌ |
| **Element connectivity check** | ✅ (exact match) | ❌ |
| Non-finite value check | ✅ | ✅ |
| Reaction sampling | constrained_dofs | constrained_dofs |
| Source tracking | LoadSource | LoadSource |
| Tie behavior | first input | first input |
| `max_abs_axial()` | ✅ | ✅ |

---

## 5. Findings

### P0 — Crash / Panic
**Count: 0**

No panics or crashes found. The 3D envelope handles all inputs gracefully.

### P1 — Silent Wrong Result (contract violation)
**Count: 0**

The 3D envelope's acceptance of mismatched topology is **explicitly documented** as a design limitation:

- `p120_same_count_incompatible_topology` test (line 1499): comment states *"Same counts → envelope accepts (documented limitation)"* and *"Values differ because geometry differs — user responsibility"*.
- `TrussEnvelope` doc comment (line 740): *"Unlike [`Truss3DEnvelope`], which only checks node and element counts..."*

Since the behavior is documented and intentional, it is not a contract violation.

### P2 — Design Gap
**Count: 1**

**P2-1: Missing topology validation in 3D envelope.**

`Truss3DEnvelope::from_results()` does not validate node coordinates or element connectivity, while `TrussEnvelope::from_results()` (2D) does. The topology data (`node_coords`, `element_nodes`) is available as public fields on `TrussAnalysisResult3D` and could be used for validation.

**Impact:** A user passing results from different 3D models (different geometry or connectivity) gets a silently aggregated envelope with no error. The resulting extrema are physically meaningless.

**Root cause:** The 3D envelope was implemented in Phase 120 (commit `1181690`) with count-only validation. The 2D envelope was implemented later in Phase 124 (commit `7b97bc0`) with stronger topology validation. The 3D envelope was never upgraded to match.

**Classification:** P2 (design gap), not P1 (correctness defect), because:
1. The behavior is explicitly documented in both the test suite and the 2D envelope's doc comments.
2. No user contract is violated — the 3D envelope's contract is "same counts", not "same topology".
3. The 2D envelope's stronger validation was a Phase 124 enhancement, not a correction of a 3D bug.

**Recommendation:** Consider upgrading `Truss3DEnvelope::from_results()` to validate topology in a future phase, matching the 2D behavior. The data is available; only the validation logic needs to be ported from `TrussEnvelope::validate_topology()`.

### P3 — Edge Case / IEEE 754 Quirk
**Count: 0**

Since the 3D envelope does not compare coordinates, the NaN/∞ coordinate issue from Phase 125 (P3) does not apply.

---

## 6. Changes

### Production code
**No changes.** P2 is a documented design limitation, not a reproducible correctness defect.

### Tests added
**7 new tests** in `tests/truss3d.rs` (p126_*):

| Test | Description |
|------|-------------|
| `p126_all_positive_axial_extrema` | Both load cases produce positive axial force; verify min > 0, max > 0, sources correct. |
| `p126_all_negative_axial_extrema` | Both load cases produce negative axial force; verify min < 0, max < 0, sources correct. |
| `p126_independent_axial_governing_source` | Different elements governed by different load sources (tripod model). |
| `p126_zero_axial_force_extrema` | Zero-force case + tension case; verify min ≈ 0, max > 0. |
| `p126_independent_displacement_governing_sources` | ux/uy/uz each populated with governing sources from 3 orthogonal load cases. |
| `p126_different_connectivity_accepted` | Characterization: same counts, different element connectivity → accepted (documented limitation). |
| `p126_different_coordinates_accepted` | Characterization: same counts, different node coordinates (z-offset) → accepted (documented limitation). |

---

## 7. Verification

| Check | Result |
|-------|--------|
| `cargo fmt --all -- --check` | ✅ Clean |
| `cargo check -p structural-analysis` | ✅ Clean (3 pre-existing pardiso warnings) |
| `cargo test --release -p structural-analysis --test truss3d` | ✅ 60 passed, 0 failed |
| `cargo test --release -p structural-analysis --test truss` | ✅ 82 passed, 0 failed |
| `cargo doc -p structural-analysis --no-deps` | ✅ 2 pre-existing doc warnings |

---

## 8. Summary

| Metric | Value |
|--------|-------|
| P0 (crash) | 0 |
| P1 (silent wrong) | 0 |
| P2 (design gap) | 1 |
| P3 (edge case) | 0 |
| Production code changes | 0 |
| Tests added | 7 |
| Files modified | 1 (`tests/truss3d.rs`) |
| Files created | 1 (`PHASE126_AUDIT.md`) |

**Evidence-based next step:** The P2 finding (missing 3D topology validation) has a clear remediation path — port `validate_topology()` from `TrussEnvelope` to `Truss3DEnvelope`, adapting for 3D coordinates `(x, y, z)`. This would be a small, focused change (~30 lines) with no API impact. However, it would change the behavior documented in `p120_same_count_incompatible_topology` and `p126_different_connectivity_accepted` / `p126_different_coordinates_accepted`, which would need to be updated from "accepted" to "rejected". This should be a separate phase with its own spec.
