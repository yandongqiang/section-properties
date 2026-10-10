# Phase 138 — 3D Frame Result Completeness & Global End-Force Recovery

## Objective

Resolve P2-3 from the Phase 137 audit: `FrameAnalysisResult3D` did not store
member reference-orientation vectors (`ref_vec`), so consumers could not
reconstruct member local coordinate systems and therefore could not transform
local end forces into global coordinates.

## Background

`FrameElement3D` stores a `ref_vec: [f64; 3]` used to build the local
`(x_hat, y_hat, z_hat)` basis via `local_axes()`.  The solver cloned the model
internally, but `FrameAnalysisResult3D` — the public result struct returned to
callers — only stored `node_coords` and `member_nodes`.  Without `ref_vec`, a
consumer holding a `FrameAnalysisResult3D` had no way to recover the local axes
or transform the 12-component local end-force vector into global coordinates.

## Design Decision

Store **one `[f64; 3]` per member** (`member_ref_vecs: Vec<[f64; 3]>`) in the
result struct.  This is the minimal data needed to deterministically reconstruct
the local coordinate system via the same algorithm used by
`FrameElement3D::local_axes()`.

Alternatives considered:
- **Full 3×3 rotation matrix per member** — rejected: redundant data, the matrix
  is derivable from node coordinates + ref_vec.
- **12×12 transformation matrix per member** — rejected: even more redundant,
  and couples result storage to an implementation detail.
- **Store nothing, require caller to pass ref_vec** — rejected: defeats the
  purpose of a self-contained result struct.

## Implementation

### New field

```rust
pub struct FrameAnalysisResult3D {
    // ... existing fields ...
    /// Member reference orientation vectors (one per member).
    pub member_ref_vecs: Vec<[f64; 3]>,
    // ...
}
```

### `solve()` update

`FrameSolver3D::solve()` now populates `member_ref_vecs` by collecting each
member's `ref_vec` from the internal model.

### New method: `member_local_axes`

```rust
pub fn member_local_axes(&self, member_idx: usize)
    -> Result<([f64; 3], [f64; 3], [f64; 3]), FemError>
```

Computes the orthonormal right-handed `(x_hat, y_hat, z_hat)` basis for the
given member using the stored `node_coords`, `member_nodes`, and
`member_ref_vecs`.  The algorithm matches `FrameElement3D::local_axes()`:
- `x_hat` = unit vector from node_i to node_j
- `z_hat` = unit(`ref_vec − (ref_vec · x_hat) x_hat`)  (Gram-Schmidt)
- `y_hat` = `z_hat × x_hat`

Returns `FemError::InvalidMember` for out-of-bounds `member_idx`.

### New method: `member_end_forces_global`

```rust
pub fn member_end_forces_global(&self, member_idx: usize)
    -> Result<[f64; 12], FemError>
```

Transforms the 12-component local end-force vector into global coordinates.
The local vector is grouped into four 3-component blocks (force_i, moment_i,
force_j, moment_j).  Each block is transformed as:

```
f_global = R^T · f_local = f[0]·x_hat + f[1]·y_hat + f[2]·z_hat
```

where `R` is the 3×3 rotation matrix whose rows are the local axes expressed in
global coordinates.

Returns `FemError::InvalidMember` for out-of-bounds `member_idx`.

## API Compatibility

- **Backward compatible.**  `FrameAnalysisResult3D` is only constructed inside
  `solve()`; external code receives it by value and reads fields.  Adding a new
  `pub` field does not break existing readers.
- No existing method signatures changed.
- `structural-analysis` has `publish = false` — no external crate consumers.
- Migration: callers who want global end forces simply call
  `result.member_end_forces_global(idx)` instead of manually transforming
  `result.member_end_forces[idx]`.

## Tests

10 new tests in `tests/frame3d.rs` module `phase138`:

| Test | Description |
|------|-------------|
| `global_forces_aligned_x` | Member along global X; local == global |
| `global_forces_aligned_y` | Member along global Y; rotation about Z |
| `global_forces_inclined` | Inclined member with nontrivial ref_vec |
| `global_forces_general_3d` | General 3D orientation |
| `round_trip_local_global_local` | local → global → local identity |
| `local_axes_match_element` | Result axes match `FrameElement3D::local_axes()` |
| `global_forces_match_applied_load` | Global end forces match applied nodal load |
| `ref_vecs_stored_correctly` | `member_ref_vecs` populated correctly |
| `invalid_member_idx_errors` | Out-of-bounds returns `InvalidMember` error |
| `local_axes_orthonormal_from_result` | Axes form orthonormal right-handed basis |

## Validation

| Check | Result |
|-------|--------|
| `cargo fmt --all -- --check` | ✅ clean |
| `cargo check -p structural-analysis --tests` | ✅ passes |
| `cargo test -p structural-analysis --release --test frame3d` | ✅ 122 passed, 0 failed |
| `cargo test -p structural-analysis --release` (full suite) | ✅ all passed, 0 failed |
| `cargo doc -p structural-analysis --no-deps` | ✅ 3 pre-existing warnings (truss3d.rs, not from this change) |
| `cargo clippy -p structural-analysis --test frame3d` | ✅ 0 new warnings from Phase 138 code |

## Files Changed

| File | Changes |
|------|---------|
| `crates/structural-analysis/src/frame3d.rs` | +134 −1: new field, 2 new methods, `solve()` update, docs |
| `crates/structural-analysis/tests/frame3d.rs` | +293: 10 new tests in `phase138` module |

**Total: +426 −1 across 2 files.**

## Phase 137 P2-3 Resolution

P2-3 is now **resolved**.  `FrameAnalysisResult3D` is self-contained: a consumer
can recover member local axes and transform local end forces to global
coordinates without access to the original `FrameModel3D`.
