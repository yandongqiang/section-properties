# Phase 155 Audit — Member Loads Numerical & Integration Audit

## 1. HEAD / Baseline

| Item | Value |
|------|-------|
| HEAD | `87ca719` Phase 154: add 3D frame member point loads (concentrated forces) |
| Working tree | Clean (only untracked audit docs and global_matrix_exports) |
| Baseline tests | viewport-poc: 510 passed; structural-analysis: 894 passed |
| Persistence | V3 (CURRENT_FORMAT_VERSION = 3) |
| GUI Runtime | BLOCKED (no display) |

## 2. Audit Scope

This phase audits the consistency and correctness of member uniform loads (Phase 153) and member point loads (Phase 154) across:

1. **P0 — Solver numerical correctness** (frame3d.rs)
2. **P1 — EditModel data consistency** (edit_model.rs)
3. **P1 — Persistence V3** (persistence.rs)
4. **P1 — GUI & Viewport** (main.rs, scene.rs)
5. **P2 — Performance**

Principle: **Audit first. Fix only confirmed defects. No architecture refactor.**

## 3. Numerical Validation Matrix

### 3.1 Local Coordinate & Sign Convention Consistency

**Finding: UniformLoad and PointLoad use identical local coordinate and sign conventions.** ✅

Both produce 12-component local equivalent nodal load vectors with the same DOF ordering:
`[u_i, v_i, w_i, rx_i, ry_i, rz_i, u_j, v_j, w_j, rx_j, ry_j, rz_j]`

| Direction | Force DOFs | Moment DOFs | Moment sign |
|-----------|-----------|-------------|-------------|
| Local x (axial) | 0, 6 | — | — |
| Local y (transverse) | 1, 7 | 5, 11 | +qL²/12 at i, −qL²/12 at j |
| Local z (transverse) | 2, 8 | 4, 10 | −qL²/12 at i, +qL²/12 at j |

PointLoad f_y uses the same DOF pattern and sign convention as UniformY.
PointLoad f_z uses the same DOF pattern and sign convention as UniformZ (flipped moments).

### 3.2 Hermite Shape Functions

The PointLoad implementation uses standard Euler-Bernoulli cubic Hermite shape functions:

```
N1(ξ) = 1 − 3ξ² + 2ξ³       (translational at i)
N2(ξ) = L·ξ·(1−ξ)²          (rotational at i)
N3(ξ) = ξ²·(3−2ξ)           (translational at j)
N4(ξ) = −L·ξ²·(1−ξ)         (rotational at j)
```

Verified properties:
- N1 + N3 = 1 for all ξ (partition of unity) ✅
- N1(0)=1, N1(1)=0, N3(0)=0, N3(1)=1 ✅
- N2(0)=0, N2(1)=0, N4(0)=0, N4(1)=0 ✅

### 3.3 Special Positions

| ξ | N1 | N2 | N3 | N4 | Behavior |
|---|----|----|----|----|----------|
| 0 | 1 | 0 | 0 | 0 | All force at node i, no moments ✅ |
| 0.25 | 0.84375 | 0.140625L | 0.15625 | −0.046875L | Asymmetric distribution ✅ |
| 0.5 | 0.5 | L/8 | 0.5 | −L/8 | Symmetric: P/2 each, ±PL/8 moments ✅ |
| 0.75 | 0.15625 | 0.046875L | 0.84375 | −0.140625L | Asymmetric distribution ✅ |
| 1 | 0 | 0 | 1 | 0 | All force at node j, no moments ✅ |

### 3.4 Load Assembly & Transformation

- Each member load is processed exactly once in the assembly loop (frame3d.rs:1507)
- Local equivalent loads are transformed to global: `f_global = Tᵀ · f_local` (frame3d.rs:1554)
- T is the 12×12 block-diagonal transformation matrix with 4 copies of the 3×3 direction cosine matrix
- **No duplicate load application** ✅
- **No sign reversal in transformation** ✅

### 3.5 End Force Recovery

- `f_local = K_local · u_local − f_eq_local` (frame3d.rs:1724-1730)
- The subtraction of `f_eq_local` correctly accounts for member loads
- `member_equiv_loads[mid]` accumulates all loads for a member (superposition) ✅
- **No double-counting at end positions** (ξ=0/1 loads have no moments, just nodal forces) ✅

### 3.6 Reaction Recovery

- `R = K·u − F` (frame3d.rs:1686)
- F includes equivalent nodal loads from member loads
- At free DOFs: R = 0 by construction (equilibrium)
- At constrained DOFs: R gives support reactions ✅

### 3.7 Superposition

- Multiple loads on the same member accumulate in both `f_global` and `member_equiv_loads`
- Solver is linear → superposition holds ✅
- Verified by Phase 154 test `case_g_multiple_loads_superposition` ✅

### 3.8 Axial Force in PointLoad

- `if f_x != 0.0` guard handles axial component (frame3d.rs:904-907)
- Linear shape functions: f[0] = f_x·(1−ξ), f[6] = f_x·ξ
- **No omission of axial force** ✅

### 3.9 Existing Test Verification

Phase 154 numerical tests (all passing):
- Case A: Fixed-fixed midspan fy — reaction & moment equilibrium ✅
- Case B: Cantilever tip fy — deflection PL³/(3EIz), rotation PL²/(2EIz), reaction −P, moment PL ✅
- Case C: Cantilever midspan fy — deflection Pa²(3L−a)/(6EIz), rotation Pa²/(2EIz) ✅
- Case D: Simply supported ξ=0.25 fy — reactions Pb/L, Pa/L, moment equilibrium ✅
- Case E: Cantilever tip fx (axial) — deflection FL/(EA) ✅
- Case F: Cantilever tip fz — deflection (local z → global y with sign flip) ✅
- Case G: Superposition — 2×(P/2) = P ✅
- Case H: ξ=1 point load = nodal load ✅
- Case I: Total force equilibrium (fx, fy, fz with 2 loads) ✅

## 4. Persistence Compatibility Matrix

| Version | PointLoad field | UniformLoad field | Result |
|---------|----------------|-------------------|--------|
| V1 | Missing | Missing | → empty loads (backward compatible) ✅ |
| V2 | Missing | Present | → empty point loads (backward compatible) ✅ |
| V3 | Present | Present | → full data ✅ |
| Unknown | — | — | → UnsupportedVersion error ✅ |

### V3 Validation Checks

| Check | Implementation | Result |
|-------|---------------|--------|
| Member existence | `member_exists(pl.member_id)` | ✅ Rejects with Validation error |
| xi ∈ [0, 1] | `(0.0..=1.0).contains(&pl.xi)` | ✅ Rejects with Validation error |
| Finite values | `pl.xi.is_finite() && pl.fx.is_finite() && ...` | ✅ Rejects with Validation error |
| Duplicate uniform load | `seen_member_loads.insert(ml.member_id)` | ✅ Rejects with Validation error |
| Multiple point loads | No duplicate check (allowed) | ✅ By design |
| Unknown version | `found => Err(UnsupportedVersion)` | ✅ Rejects |

### Deserialization vs EditModel Validation

- `dto_v3_to_model` constructs `EditModel::from_parts()` directly, bypassing individual `add_*` methods
- However, validation in `dto_v3_to_model` covers the same checks as `add_member_point_load`:
  - Member existence ✅
  - xi ∈ [0, 1] ✅
  - Finite values ✅
- **No silent deletion of invalid data** — all invalid data is rejected with explicit errors ✅

### Save Safety

- `save_project` writes to a temporary file first, then renames atomically (persistence.rs:880-898)
- On rename failure, temp file is cleaned up
- Original file is not modified until rename succeeds ✅

## 5. EditModel State Consistency

### 5.1 Fingerprint Completeness

The fingerprint includes all load fields:
- Member uniform loads: `(member_id, qx, qy, qz)` ✅
- Member point loads: `(member_id, xi, fx, fy, fz)` ✅
- Any change to any load field changes the fingerprint ✅

### 5.2 Delete Member Cleanup

`delete_member` (edit_model.rs:666-675) removes:
- `self.members.retain(|m| m.id != id)` — member itself
- `self.member_uniform_loads.retain(|(mid, _)| *mid != id)` — uniform loads
- `self.member_point_loads.retain(|(mid, _)| *mid != id)` — point loads
- ✅ No dangling load references

### 5.3 to_frame_model Load Transfer

- All uniform loads transferred via `add_uniform_axial/y/z` (edit_model.rs:1263-1282) ✅
- All point loads transferred via `add_point_load` (edit_model.rs:1284-1291) ✅
- Stable MemberId → index mapping via `member_id_to_idx` ✅
- **No load omission** ✅

### 5.4 Undo/Redo

- `commit_edit` records old model before replacement (single transaction)
- `do_undo`/`do_redo` restore with `ensure_revision_at_least(current_rev + 1)`
- Point loads are part of the EditModel clone, so they are fully restored ✅

### 5.5 Stale Detection

- `results_are_stale()` compares fingerprint of current model vs fingerprint at solve time
- Any change to point loads changes fingerprint → results marked stale ✅

## 6. GUI / Viewport Findings

### 6.1 Inspector (Code-Level Audit)

| Check | Finding |
|-------|---------|
| Duplicate history records | No — `apply_member_edit` creates one `commit_edit` call for all changes ✅ |
| DragValue continuous editing | Edits buffer only; no model changes until "Apply Changes" ✅ |
| Selection switch cleanup | `reset_member_edit` syncs buffer from model, clearing stale state ✅ |
| xi range constraint | DragValue `.range(0.0..=1.0)` prevents out-of-range input ✅ |
| Add/Delete/Clear | Buffer-only operations; model unchanged until "Apply Changes" ✅ |
| No per-frame recreation | Point loads stored in `member_edit.point_loads`, modified in-place ✅ |

### 6.2 Viewport (Code-Level Audit)

| Check | Finding |
|-------|---------|
| xi position | `base = pi + xi * (pj - pi)` — correct position along member ✅ |
| Local-to-global direction | `f_global = fx·x_hat + fy·y_hat + fz·z_hat` — same local axes as solver ✅ |
| Local axes computation | Same algorithm as `FrameMember3D::local_axes` (x_hat, z_hat = cross, y_hat = z×x) ✅ |
| Camera invariance | Uses `camera.project()` for all points ✅ |
| Multiple loads | Iterates over all loads, draws each at its position ✅ |
| Zero/non-finite loads | Skipped (`pl.is_zero() || !pl.is_finite()`) ✅ |
| Zero force magnitude | Skipped (`fmag < 1e-12`) ✅ |
| Color consistency | Uses `MEMBER_LOAD_C` — same green as uniform loads ✅ |
| [P] label | `has_member_point_loads(member.id)` → " [P]" ✅ |

### 6.3 GUI Runtime

**BLOCKED** — no display available. All GUI findings are code-level only.

## 7. Performance Measurements

| Scenario | Build | Fingerprint | Convert | Solve |
|----------|-------|-------------|---------|-------|
| 100 members, 3 point loads each | 78.3µs | 13.0µs | 51.6µs | 1.63ms |
| 500 members, mixed loads | 380.3µs | 38.0µs | 113.2µs | 7.40ms |
| 100× fingerprint repeat | — | 7.62µs/call | — | — |

All operations are sub-10ms. No performance concerns. No optimization needed.

## 8. P0/P1/P2/P3 Findings

### P0 (Blocking — numerical errors, wrong results, data corruption)

**None found.**

### P1 (Must Fix — state inconsistency, persistence defects, functional errors)

**None found.**

### P2 (Improvement — performance, maintainability, non-critical interaction)

**P2-1: ref_vec not validated during deserialization**

- **Location**: `dto_v3_to_model` (persistence.rs:475-622)
- **Description**: The `validate_dto` function checks nodes, members, supports, and nodal loads but does not validate `ref_vec` (finite, nonzero, not parallel to member axis). An invalid ref_vec in a V3 file would be loaded successfully but fail at solve time with a `FrameModelBuild` error.
- **Impact**: Low — fails with clear error message at solve time, not silently. Pre-existing issue (not introduced by Phase 153/154).
- **Fix**: Not required for this audit phase. Would require adding ref_vec validation to `validate_dto` or `dto_v3_to_model`.

### P3 (Documentation, naming, non-critical optimization)

**None found.**

## 9. Regression Tests

All existing tests pass with no regressions:

| Test Suite | Count | Result |
|------------|-------|--------|
| viewport-poc (all) | 510 | ✅ All pass |
| structural-analysis (all) | 894 | ✅ All pass |
| section-properties doc tests | 3 | ✅ All pass |
| structural-analysis doc tests | 13 | ✅ All pass |

Phase 153 regression tests (within phase154_point_loads.rs):
- `regression_uniform_load_still_works` ✅
- `regression_both_load_types_solve` ✅
- `regression_fingerprint_unchanged_for_uniform_only` ✅

## 10. Validation Results

| Command | Result |
|---------|--------|
| `cargo fmt --all -- --check` | ✅ Clean |
| `cargo check -p structural-analysis` | ✅ No errors |
| `cargo check -p section-properties` | ✅ No errors |
| `cargo check --examples` | ✅ No errors |
| `cargo test -p structural-analysis --release` | ✅ 894 passed, 0 failed |
| `cargo test -p viewport-poc --release` | ✅ 510 passed, 0 failed |
| `cargo test --doc -p section-properties` | ✅ 3 passed, 0 failed |
| `cargo test --doc -p structural-analysis` | ✅ 13 passed, 0 failed |
| `cargo test -p section-properties --release` | ⛔ TIMEOUT (known: FEM benchmarks) |
| `cargo clippy -p structural-analysis --all-targets -- -D warnings` | ⛔ FAIL (133 pre-existing lint debt in section-properties) |
| `cargo clippy -p structural-analysis` | 150 warnings (all pre-existing) |
| `cargo clippy -p viewport-poc` | 194 warnings (all pre-existing) |
| GUI runtime | ⛔ BLOCKED (no display) |

## 11. Remaining Risks

1. **GUI runtime unverified** — Inspector and Viewport changes are code-complete but not runtime-tested
2. **section-properties test timeout** — Full test suite times out due to FEM benchmarks (known, pre-existing)
3. **ref_vec validation gap** (P2-1) — Invalid ref_vec in persisted file fails at solve, not at load
4. **clippy -D warnings** — Pre-existing lint debt in section-properties prevents strict clippy from passing (not introduced by Phase 153/154)

## 12. Phase 156 Readiness

All P0 and P1 checks pass. The member load system (uniform + point) is numerically correct, internally consistent, and properly integrated across solver, EditModel, persistence, and GUI. The only finding is a P2 (ref_vec validation gap) that is pre-existing and low-impact.

---

```text
P0: None
P1: None
P2: P2-1 (ref_vec not validated during deserialization, pre-existing, low impact)
P3: None

Numerical correctness: PASS
Persistence compatibility: PASS
EditModel consistency: PASS
GUI compile: PASS
GUI runtime: BLOCKED

Phase 156 readiness: READY
```
