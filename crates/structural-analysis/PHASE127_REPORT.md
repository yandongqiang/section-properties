# Phase 127 — Add Topology Validation to Truss3DEnvelope

**Date:** 2026-10-09  
**Baseline HEAD:** `a9c8b1e` (Phase 126)  
**Scope:** Close the Phase 126 P2 finding by adding node-coordinate and element-connectivity validation to `Truss3DEnvelope::from_results()`, matching the 2D `TrussEnvelope` contract.

---

## 1. Baseline and HEAD

| | |
|---|---|
| Expected HEAD | `a9c8b1e` |
| Actual HEAD | `a9c8b1e` ✅ |
| Working tree | Clean (only untracked historical docs) |

---

## 2. Previous P2 Finding and Resolution

**Phase 126 P2-1:** `Truss3DEnvelope::from_results()` did not validate node coordinates or element connectivity, while `TrussEnvelope::from_results()` (2D) did. The topology data (`node_coords`, `element_nodes`) was available as public fields on `TrussAnalysisResult3D` but unused for validation.

**Resolution:** Added `Truss3DEnvelope::validate_topology()` — a private method that compares node coordinates (x, y, z) across all results using the same 1e-9 relative tolerance as the 2D envelope, and verifies element connectivity by exact `Vec` equality. Called after count checks, before any extrema computation. Non-finite coordinates (NaN, ±∞) are explicitly rejected.

**P2 status:** Resolved. P0=0, P1=0, P2=0, P3=0.

---

## 3. Validation Contract and Tolerance Formula

### Node coordinates

For each node `n` and each result `r[idx]` (idx ≥ 1), compared against the reference `results[0]`:

```
ref = (rx, ry, rz) = results[0].node_coords[n]
cand = (x, y, z) = r.node_coords[n]

// Non-finite check (3D enhancement over 2D)
if !rx.is_finite() || !ry.is_finite() || !rz.is_finite()
   || !x.is_finite() || !y.is_finite() || !z.is_finite()
    → reject with InvalidInput

// Relative tolerance (same formula as 2D, extended to z)
scale = |rx| ⊔ |ry| ⊔ |rz| ⊔ |x| ⊔ |y| ⊔ |z| ⊔ 1.0
if |rx - x| > 1e-9 * scale || |ry - y| > 1e-9 * scale || |rz - z| > 1e-9 * scale
    → reject with InvalidInput
```

Where `⊔` denotes `f64::max`. The tolerance is `1e-9` (relative), matching the 2D contract exactly. The `max(1.0)` floor ensures near-zero coordinates use an absolute tolerance of `1e-9`.

### Element connectivity

Exact `Vec<(usize, usize)>` equality: `r.element_nodes != *ref_elements → reject`.

Reversed endpoint ordering (e.g., `(0,1)` vs `(1,0)`) is rejected. Although the axial force sign is invariant under endpoint reversal (direction cosines and displacement differences both flip sign, product unchanged), exact match is the documented contract for both 2D and 3D envelopes. Accepting reversed endpoints would require remapping result values and is not supported.

---

## 4. Connectivity and Ordering Semantics

| Property | Contract |
|----------|----------|
| Node ordering | Must match exactly (node `i` in result 0 = node `i` in all results) |
| Element ordering | Must match exactly (element `j` in result 0 = element `j` in all results) |
| Endpoint ordering | Must match exactly (`(i, j)` ≠ `(j, i)`) |
| Node coordinates | Must match within 1e-9 relative tolerance |
| Element connectivity | Must match exactly (`Vec` equality) |

---

## 5. Non-Finite Coordinate Behavior

Unlike the 2D envelope (where NaN/∞ coordinates pass validation due to IEEE 754 semantics — Phase 125 P3), the 3D envelope **explicitly rejects** non-finite coordinates before applying the tolerance comparison. This is a deliberate enhancement:

- **NaN:** `f64::NAN` in any coordinate → `InvalidInput`
- **+∞:** `f64::INFINITY` in any coordinate → `InvalidInput`
- **−∞:** `f64::NEG_INFINITY` in any coordinate → `InvalidInput`

Both reference and candidate coordinates are checked. This prevents NaN from silently bypassing the mismatch check (IEEE 754: `NaN > x` is `false` for all `x`).

---

## 6. Tests Added or Updated

### Updated characterization tests (3)

| Test | Previous behavior | New behavior |
|------|-------------------|--------------|
| `p120_same_count_incompatible_topology` | Accepted (documented limitation) | Rejected with `InvalidInput` |
| `p126_different_connectivity_accepted` | Accepted (documented limitation) | Rejected with `InvalidInput` |
| `p126_different_coordinates_accepted` | Accepted (documented limitation) | Rejected with `InvalidInput` |

### New validation tests (13)

| Test | Description |
|------|-------------|
| `p127_identical_topology_accepted` | Two results from the same model → accepted |
| `p127_x_mismatch_rejected` | x-coordinate mismatch independently detected |
| `p127_y_mismatch_rejected` | y-coordinate mismatch independently detected |
| `p127_z_mismatch_rejected` | z-coordinate mismatch independently detected |
| `p127_near_zero_coordinate_tolerance` | Tiny perturbation (1e-12/1e-13/1e-14) passes |
| `p127_near_zero_coordinate_rejected` | Larger perturbation (1e-6) rejected |
| `p127_large_coordinate_tolerance` | Large coord (1e8) + small relative perturbation (1e-4) passes |
| `p127_large_coordinate_rejected` | Large coord (1e8) + excessive perturbation (1.0) rejected |
| `p127_nan_coordinate_rejected` | NaN coordinate rejected deterministically |
| `p127_positive_infinity_coordinate_rejected` | +∞ coordinate rejected deterministically |
| `p127_negative_infinity_coordinate_rejected` | −∞ coordinate rejected deterministically |
| `p127_later_mismatch_no_partial_contamination` | Mismatch in result[2] rejects entire envelope (no partial result) |
| `p127_reversed_endpoints_rejected` | Reversed endpoint ordering `(1,0)` vs `(0,1)` rejected |

**Total truss3d tests:** 60 (Phase 126) → 73 (Phase 127)

---

## 7. Files Changed

| File | Type | Changes |
|------|------|--------|
| `crates/structural-analysis/src/postprocessing.rs` | Production | +66 −18: added `validate_topology()`, called in `from_results()`, updated doc comments on `Truss3DEnvelope` and `TrussEnvelope` |
| `crates/structural-analysis/tests/truss3d.rs` | Tests | +243 −28: updated 3 characterization tests, added 13 new validation tests |

**No public API changes.** No new dependencies. No unrelated files.

---

## 8. Public API Compatibility Assessment

| Aspect | Status |
|--------|--------|
| `Truss3DEnvelope` struct fields | Unchanged |
| `Truss3DEnvelope::from_results()` signature | Unchanged (`&[&TrussAnalysisResult3D] → Result<Self, FemError>`) |
| `Truss3DEnvelope` accessor methods | Unchanged |
| Error type | `FemError::InvalidInput` (existing variant) |
| 2D `TrussEnvelope` | Unchanged (behavior, signature, tests) |
| `TrussAnalysisResult3D` | Unchanged |

**Behavioral change:** `from_results()` now rejects results with mismatched topology (previously accepted). This is a stricter contract, not an API change. Callers that previously passed results from different models would have gotten silently wrong envelopes; they now get a clear error.

---

## 9. Commands and Validation Results

| Command | Result |
|---------|--------|
| `cargo fmt --all -- --check` | ✅ Clean |
| `cargo check -p structural-analysis` | ✅ Clean (3 pre-existing pardiso warnings) |
| `cargo test -p structural-analysis --test truss3d` | ✅ 73 passed, 0 failed |
| `cargo test -p structural-analysis --test truss` | ✅ 82 passed, 0 failed |
| `cargo test -p structural-analysis` | ✅ All passed (all test binaries + 13 doctests) |
| `cargo test --workspace` | ⏱️ Timed out (pre-existing: section-properties FEM benchmarks) |
| `cargo doc -p structural-analysis --no-deps` | ✅ 1 pre-existing warning (`truss3d.rs:9` unresolved intra-doc link) |
| `cargo clippy -p structural-analysis --all-targets -- -D warnings` | ⚠️ 133 pre-existing `section-properties` lints + 1 pre-existing unused import in `tests/postprocessing.rs`; 0 new warnings from this phase |
| `git diff --check` | ✅ No whitespace errors (CRLF warnings only) |

---

## 10. Remaining Limitations and Next-Phase Recommendation

### Remaining limitations

1. **2D NaN/∞ gap (Phase 125 P3):** The 2D `TrussEnvelope::validate_topology()` does not explicitly reject non-finite coordinates. NaN/∞ passes due to IEEE 754 semantics. This has no correctness impact (non-finite *values* are caught separately), but is an asymmetry with the 3D envelope. A future phase could add the same `is_finite()` guard to 2D.

2. **Frame `Envelope`:** The Frame envelope (`Envelope::from_results()`) only validates node/element counts, not topology. This is the same gap that 3D had before Phase 127. A future phase could add topology validation to the Frame envelope if `FrameAnalysisResult` exposes topology data.

3. **Reversed endpoint equivalence:** Reversed endpoints `(i,j)` vs `(j,i)` produce the same axial force but are rejected by exact match. A future phase could optionally accept reversed endpoints by remapping, but this adds complexity for no clear user benefit.

### Evidence-based recommendation for Phase 128

**Option A:** Add `is_finite()` coordinate guard to 2D `TrussEnvelope::validate_topology()` to close the Phase 125 P3 asymmetry with 3D. Small, focused, ~5 lines.

**Option B:** Audit Frame `Envelope` topology validation — determine whether `FrameAnalysisResult` exposes `node_coords`/`element_nodes` and whether the Frame envelope should validate topology like 2D/3D Truss envelopes.

**Option C:** Move to a different area (e.g., 2D/3D truss self-weight, or frame envelope enhancements).

Option A is the smallest and most directly motivated by the work in this phase.
