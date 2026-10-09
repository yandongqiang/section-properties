# Phase 128 — Reject Non-Finite Coordinates in 2D Truss Envelope

**Date:** 2026-10-09  
**Baseline HEAD:** `ff9223c` (Phase 127)  
**Scope:** Close the Phase 125 P3 finding by adding `is_finite()` coordinate validation to 2D `TrussEnvelope::validate_topology()`, achieving parity with the 3D envelope.

---

## 1. Actual Baseline and HEAD

| | |
|---|---|
| Expected HEAD | `ff9223c` |
| Actual HEAD | `ff9223c` ✅ |
| Working tree | Clean (only untracked historical docs) |

---

## 2. Root Cause

Phase 125 identified that NaN and ±∞ coordinates could pass the 2D `TrussEnvelope::validate_topology()` check due to IEEE 754 semantics:

- `NaN - x = NaN`, `NaN.abs() = NaN`, `NaN > tol * scale` evaluates to `false` → mismatch silently passes
- `∞ - ∞ = NaN` → same bypass
- `∞ - finite = ∞`, but `scale` may also be `∞`, making `tol * ∞ = ∞` and `∞ > ∞` is `false` → bypass

The 2D `validate_topology()` (line 895) had no `is_finite()` guard — it proceeded directly to the tolerance comparison. The 3D `validate_topology()` (line 569, added in Phase 127) already had the guard.

---

## 3. Exact Production Change

**File:** `crates/structural-analysis/src/postprocessing.rs`  
**Lines:** +7 in `TrussEnvelope::validate_topology()`

Added an `is_finite()` check on all four coordinate components (reference rx, ry and candidate x, y) before the tolerance comparison:

```rust
if !rx.is_finite() || !ry.is_finite() || !x.is_finite() || !y.is_finite() {
    return Err(FemError::InvalidInput(format!(
        "node {node} has non-finite coordinates between result 0 and \
         result {idx}: ({rx}, {ry}) vs ({x}, {y})"
    )));
}
```

Also updated the `from_results()` doc comment to list the new error condition.

**No public API changes.** No signature changes. No struct layout changes.

---

## 4. Tests Added

**File:** `crates/structural-analysis/tests/truss.rs` — 7 new tests (`p128_*`)

| Test | Description |
|------|-------------|
| `p128_nan_x_rejected` | NaN in x-coordinate of candidate → `InvalidInput` |
| `p128_nan_y_rejected` | NaN in y-coordinate of candidate → `InvalidInput` |
| `p128_positive_infinity_rejected` | +∞ in x-coordinate → `InvalidInput` |
| `p128_negative_infinity_rejected` | −∞ in y-coordinate → `InvalidInput` |
| `p128_non_finite_reference_rejected` | NaN in reference result[0] → `InvalidInput` |
| `p128_non_finite_later_candidate_rejected` | +∞ in result[2] → entire envelope rejected, no partial contamination |
| `p128_finite_matching_topology_accepted` | Two results from same model → accepted (regression guard) |

**Total 2D truss tests:** 82 (Phase 127) → 89 (Phase 128)

---

## 5. 2D/3D Finite-Coordinate Validation Comparison

| Aspect | 2D `TrussEnvelope` (Phase 128) | 3D `Truss3DEnvelope` (Phase 127) |
|--------|-------------------------------|----------------------------------|
| `is_finite()` guard | ✅ Added | ✅ Already present |
| Components checked | rx, ry, x, y | rx, ry, rz, x, y, z |
| NaN rejected | ✅ | ✅ |
| +∞ rejected | ✅ | ✅ |
| −∞ rejected | ✅ | ✅ |
| Reference checked | ✅ | ✅ |
| Candidate checked | ✅ | ✅ |
| Error type | `FemError::InvalidInput` | `FemError::InvalidInput` |
| Guard position | Before tolerance comparison | Before tolerance comparison |

**Parity achieved.** Both 2D and 3D envelopes now reject non-finite coordinates with the same error type, at the same point in the validation sequence.

---

## 6. Tolerance Behavior Preserved

The tolerance formula for finite coordinates is unchanged:

```
scale = |rx| ⊔ |ry| ⊔ |x| ⊔ |y| ⊔ 1.0
if |rx - x| > 1e-9 * scale || |ry - y| > 1e-9 * scale → reject
```

The `is_finite()` guard runs **before** the tolerance comparison, so finite coordinates follow the exact same path as before. Existing Phase 125 tolerance tests (`p125_near_zero_coordinate_tolerance`, `p125_near_zero_coordinate_rejected`, `p125_large_coordinate_tolerance`, `p125_large_coordinate_rejected`) all pass unchanged.

---

## 7. Validation Results

| Command | Result |
|---------|--------|
| `cargo fmt --all -- --check` | ✅ Clean |
| `cargo check -p structural-analysis` | ✅ Clean (3 pre-existing pardiso warnings) |
| `cargo test -p structural-analysis --test truss` | ✅ 89 passed, 0 failed |
| `cargo test -p structural-analysis --test truss3d` | ✅ 73 passed, 0 failed |
| `cargo test -p structural-analysis` | ✅ All passed (all test binaries + 13 doctests) |
| `cargo doc -p structural-analysis --no-deps` | ✅ 1 pre-existing warning (`truss3d.rs:9` unresolved intra-doc link) |
| `cargo clippy -p structural-analysis --all-targets -- -D warnings` | ⚠️ Pre-existing lints only; 0 new warnings from this phase |
| `cargo test --workspace` | ⏱️ Timed out (pre-existing: section-properties FEM benchmarks) |
| `git diff --check` | ✅ No whitespace errors (CRLF warnings only) |

---

## 8. API Compatibility

| Aspect | Status |
|--------|--------|
| `TrussEnvelope` struct | Unchanged |
| `TrussEnvelope::from_results()` signature | Unchanged |
| Error type | `FemError::InvalidInput` (existing variant) |
| 3D `Truss3DEnvelope` | Unchanged |
| `TrussAnalysisResult` | Unchanged |

**Behavioral change:** `from_results()` now rejects results with non-finite node coordinates (previously silently accepted due to IEEE 754 semantics). This is a stricter contract, not an API change.

---

## 9. Remaining Findings and Recommended Phase 129

### Current finding counts

- P0 = 0 (no crash)
- P1 = 0 (no silent wrong result)
- P2 = 0 (Phase 126 P2 resolved in Phase 127)
- P3 = 0 (Phase 125 P3 resolved in this phase)

### Remaining limitations

1. **Frame `Envelope`:** The Frame envelope only validates node/element counts, not topology. This is the same gap that 3D Truss had before Phase 127. A future phase could audit whether `FrameAnalysisResult` exposes topology data and whether the Frame envelope should validate it.

2. **Reversed endpoint equivalence:** Both 2D and 3D Truss envelopes reject reversed endpoints `(i,j)` vs `(j,i)` by exact match. Although axial force is invariant under reversal, accepting reversed endpoints would require remapping and adds complexity for no clear user benefit.

### Evidence-based recommendation for Phase 129

**Option A:** Audit Frame `Envelope` topology validation — determine whether `FrameAnalysisResult` exposes `node_coords`/`element_nodes` and whether the Frame envelope should validate topology like 2D/3D Truss envelopes. This is the last envelope type that may lack topology validation.

**Option B:** Move to a different area (e.g., 2D/3D truss self-weight, frame envelope enhancements, or new engineering capabilities).

Option A is the most directly motivated by the envelope validation work in Phases 124–128.
