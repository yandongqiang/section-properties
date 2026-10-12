# Phase 149 — GUI MVP Integration Audit

**Status:** Complete  
**Baseline:** `efafb48` (Phase 148)  
**Scope:** `apps/viewport-poc` audit + independent numerical verification tests  
**Date:** 2026-10-12

---

## 1. Executive Summary

A comprehensive integration-level audit of the egui structural analysis GUI MVP
was performed across 10 audit dimensions: baseline verification, architecture,
model integrity, analysis lifecycle, numerical correctness, visualization,
GUI smoke test, performance, findings classification, and fix policy.

**Result: 0 P0, 0 P1, 5 P2, 4 P3.**

No blocking defects or MVP-acceptance-blocking issues were found. The
application architecture is clean with proper separation of concerns
(EditModel → Project → structural-analysis). All numerical cross-validation
tests pass against independent analytical baselines. The analysis lifecycle
correctly implements Policy A with revision tracking and transaction safety.

**MVP Readiness: READY WITH LIMITATIONS**

The limitation is the inability to perform an interactive GUI smoke test
in the current CLI environment. All automated evidence strongly suggests
correct behavior, but the spec requires explicit acknowledgment of this
gap.

---

## 2. Baseline and Environment

| Item | Value |
|------|-------|
| HEAD commit | `efafb48` |
| Working tree | Clean (only untracked audit docs in other crates) |
| Rust toolchain | 1.97.1 |
| Workspace members | section-properties, structural-analysis, viewport-poc, tauri-viewport-poc |
| Dependency direction | viewport-poc → structural-analysis → section-properties (no cycles) |
| viewport-poc tests | 304 passed, 0 failed |
| structural-analysis tests | 894 passed, 0 failed |
| `cargo fmt --all -- --check` | Clean |
| `cargo check --workspace` | Clean |
| `cargo clippy -p viewport-poc --all-targets -- -D warnings` | Fails: 133 pre-existing section-properties lint errors (not from viewport-poc) |
| viewport-poc clippy warnings (without -D) | 9 lib + 11 lib test + 2 deformation + 2 selection + 3 app_state + 31 bin + 8 edit_model = 66 (all pre-existing) |
| GUI release binary | Builds successfully |

---

## 3. Architecture Findings

### 3.1 Dependency boundaries — PASS

- No circular dependencies: viewport-poc → structural-analysis → section-properties ✓
- No GUI dependency inside structural-analysis (only depends on section-properties + thiserror) ✓
- No GUI-specific types exported from section-properties ✓
- No duplicate numerical solver in viewport-poc (all solving delegates to `FrameModel3D::solve()`) ✓

### 3.2 Authority — PASS

- `EditModel` is the sole authoritative editable model (`edit_model.rs:290`) ✓
- `Project` is the sole analysis lifecycle owner (`app_state.rs:105`) ✓
- `MemberEditBuffer` is a UI temporary buffer, not authoritative state (`main.rs:61-73`) ✓
- Rendering code does not mutate structural model data (all `&mut` model access is in inspector callbacks) ✓

### 3.3 Redundant state — PASS

- `ResultDisplayState` contains only UI display toggles, no model/result data ✓
- `ViewportScene` is a controlled derived snapshot, rebuilt on every model change ✓
- Analysis result staleness detected via revision comparison, no manual stale flag ✓

### 3.4 Conversion logic — PASS

- Single `EditModel::to_frame_model()` conversion point (`edit_model.rs:775`) ✓
- Single production call site in `Project::run_analysis` (`app_state.rs:213`) ✓

### 3.5 Preset consistency — PARTIAL (P2)

- Production presets use `EditModel` public API with validation ✓
- **P2-F1**: `model.rs` contains parallel `build_*_frame` functions that bypass
  `EditModel` and directly construct `FrameModel3D`. These are test-only but
  create a maintenance risk if preset geometry is modified in one path but not
  the other.

---

## 4. Model Integrity

### Nodes — PASS

| Operation | Validation | ID | Cleanup | Verdict |
|-----------|-----------|-----|---------|---------|
| `add_node` | Rejects NaN/Inf (`:338`) | Monotonic (`:341-342`) | N/A | CORRECT |
| `update_node` | Rejects NaN/Inf (`:352`), checks existence (`:355`) | Preserved | N/A | CORRECT |
| `delete_node` | Blocks if referenced by members (`:379-387`) | Not reused | Cleans supports+loads (`:389-390`) | CORRECT |

### Members — PASS

| Operation | Validation | Verdict |
|-----------|-----------|---------|
| `add_member` | Checks node existence, rejects zero-length, self-connection, duplicates | CORRECT |
| `delete_member` | Checks existence | CORRECT |
| `set_member_section` | Validates E/G/A/Iy/Iz/J finite and positive, atomic | CORRECT |
| `set_member_ref_vec` | Validates finite, non-zero, non-parallel to member axis, atomic | CORRECT |

### Supports — PASS (with P2)

- `fixed()`: constrains all 6 DOFs to 0 ✓
- `pinned()`: constrains 3 translations to 0, rotations free ✓
- `free()`: no constraints ✓
- `custom()`: arbitrary DOF specification ✓
- `clear_support()`: removes support record ✓
- **P2-F2**: `to_frame_model` (`:800-807`) uses `is_fixed()`/`is_pinned()` fast
  paths that check only `is_some()`/`is_none()`, not whether values are zero.
  A `custom([Some(1.0), ...])` with non-zero prescribed displacement would be
  misclassified as fixed/pinned and the non-zero value lost. GUI only offers
  fixed/pinned/free, so this is latent, not user-facing.

### Nodal Loads — PASS (with P3)

- Six-component (fx, fy, fz, mx, my, mz) ✓
- Replacement semantics (not accumulation) ✓
- `clear_nodal_load()` removes record ✓
- Rejects NaN/Inf ✓
- **P3-F1**: `set_nodal_load` returns `EditError::NonFiniteCoordinate` for
  non-finite load values (`:655`). Error variant name is semantically incorrect
  for loads.

### General Integrity — PASS

- No orphan references possible (deletion blocks if referenced) ✓
- No accidental ID reuse (counters are monotonic) ✓
- Failed validation does not partially mutate (all methods use validate-first pattern) ✓
- **P3-F2**: `next_node_id += 1` (`:342`) and `next_member_id += 1` (`:427`) use
  native `+=` instead of `saturating_add(1)`. u32 overflow in release mode would
  wrap around and cause ID reuse. Practically unreachable but inconsistent with
  `bump_revision` which uses `saturating_add`.

### Revision Tracking — PASS

All 12 mutation methods correctly bump revision on effective changes and
skip bumping on no-ops. No effective mutation misses revision bumping.

---

## 5. Analysis Lifecycle

### Four-state status — PASS

| State | Condition | Priority |
|-------|-----------|----------|
| `Failed` | `last_solve_failed == true` | Highest (checked first) |
| `NeverSolved` | No model or no analysis | — |
| `Current` | `solved_revision == model.revision()` | — |
| `Stale` | `solved_revision != model.revision()` | — |

### Transaction safety — PASS

1. Failed conversion does not modify Project (`run_analysis` only sets `self.analysis` on success) ✓
2. Failed solver does not partially commit results (same) ✓
3. Previous results cannot be presented as current after failure (`analysis_result()` returns None when `Failed`) ✓
4. Repairing a model allows successful re-solving (test: `failed_solve_then_successful_solve_clears_failed`) ✓
5. Repeated solving without edits is deterministic (revision unchanged → same `Current` status) ✓

### Policy A — PASS

All result access in rendering code goes through `self.analysis_result()` →
`Project::analysis_result()`, which returns `None` for non-`Current` status.
No direct `project.analysis` field access in rendering code.

---

## 6. Numerical Cross-validation

Four independent tests were written in `tests/audit149.rs` using analytical
formulas as the baseline (not GUI-calculated values).

### Case A — Axial Cantilever

| Parameter | Value |
|-----------|-------|
| E | 200 GPa |
| A | 2×10⁻³ m² |
| L | 2.0 m |
| F | 5000 N |

| Quantity | Analytical | Solver | Rel. Error | Verdict |
|----------|-----------|--------|------------|---------|
| u_b (Ux) | FL/EA = 2.5×10⁻⁵ m | — | < 10⁻⁴ | PASS |
| R_a (Ux) | -F = -5000 N | — | < 10⁻⁴ | PASS |
| N_i + N_j | 0 (equilibrium) | — | < 10⁻⁶ | PASS |

### Case B — Transverse Cantilever

| Parameter | Value |
|-----------|-------|
| E | 200 GPa |
| Iz | 2×10⁻⁶ m⁴ |
| L | 2.0 m |
| F | 5000 N (Z direction) |

| Quantity | Analytical | Solver | Rel. Error | Verdict |
|----------|-----------|--------|------------|---------|
| δ_b (Uz) | FL³/3EIz = 1.667×10⁻² m | — | < 10⁻⁴ | PASS |
| R_z,a | -F = -5000 N | — | < 10⁻⁴ | PASS |
| M_x,a | -FL = -10000 N·m | — | < 10⁻⁴ | PASS |

### Case C — 3D Frame with Non-default Reference Vector

| Parameter | Value |
|-----------|-------|
| Member | Along X, L = 1.5 m |
| ref_vec | [0, 1, 0] (non-default, swaps local y/z) |
| Load | F = 3000 N in Z at free end |

| Verification | Expected | Actual | Verdict |
|-------------|----------|--------|---------|
| Local x-axis | [1, 0, 0] | — | PASS |
| Local y-axis | [0, 1, 0] | — | PASS |
| Local z-axis | [0, 0, 1] | — | PASS |
| Local→Global transformation | R^T × f_local = f_global | — | PASS (all 6 components, < 10⁻⁶) |
| Force equilibrium (ΣFx, ΣFy, ΣFz) | 0 | — | PASS (< 10⁻⁶) |
| Moment equilibrium (Mx, My, Mz about a) | (0, +FL, 0) | — | PASS (< 10⁻⁶) |
| Uy ≈ 0 (load in Z, no coupling) | 0 | — | PASS (< 10⁻⁶ × \|Uz\|) |

### Case C Extra — Iy/Iz Usage

With ref_vec = [0, 1, 0], a Z-direction load bends about local y (Iy),
NOT about local z (Iz).

| Formula | Expected | Actual | Rel. Error | Verdict |
|---------|----------|--------|------------|---------|
| δ = FL³/3EIy | 4.688×10⁻³ m | — | < 10⁻⁴ | PASS |
| δ ≠ FL³/3EIz | Should not match | — | > 10% difference | PASS |

---

## 7. Visualization Audit

### Five ResultViewMode variants — PASS

| Mode | Rendering | No-result handling | Verdict |
|------|-----------|-------------------|---------|
| Model | Undeformed geometry | N/A (no result needed) | CORRECT |
| Deformed | Amplified deformed shape + optional reference | Skips deformed layer | CORRECT |
| Displacement | Color-coded nodes by \|u\|, legend | Skips entire block | CORRECT |
| Reaction | Force arrows + moment markers at supports | Skips entire block | CORRECT |
| MemberEndForces | End-force arrows for selected member | Skips if no selection or no result | CORRECT |

### NaN/Inf safety — PASS (with P2)

- No panic paths exist. All NaN/Inf values are eventually caught by
  `Camera3D::project` (`camera.rs:120-122`) or explicit `is_finite()` checks.
- **P2-F3**: `fmag` computation in Reaction mode (`main.rs:769`) and
  MemberEndForces mode (`main.rs:822`) does not explicitly defend against
  NaN/Inf. Safety relies on downstream `camera.project` filtering. Functionally
  safe but fragile.

### Zero-result cases — PASS

- Never solved: all result modes skip, status shows "not run" ✓
- Failed solve: Policy A hides results, warning banner shown ✓
- All-zero displacements: nodes render blue, legend suppressed ✓

### Selection mapping — PASS

- `SelectionState` holds stable IDs (NodeId/MemberId) ✓
- `validate_selection()` runs every frame, clears stale selections ✓
- All rendering uses `scene.node_index(id)` / `scene.member_index(id)` with Option binding ✓

### Deformation immutability — PASS

- `deformed_positions()` is a pure function returning new Vec ✓
- No `&mut EditModel` in rendering path ✓
- Formula: `position + scale * displacement` (translations only, no rotations) ✓

### Stale/failed result leaking — PASS

- `ViewportScene` stores no analysis results ✓
- `analysis_result()` is the sole result access path in rendering ✓
- No direct `project.analysis` field access in rendering code ✓

### Legend correctness — PASS

- Displacement legend max value matches `max_abs(&mags)` ✓
- Color gradient uses same `magnitude_color()` function as node coloring ✓

### Additional findings

- **P2-F4**: Reaction mode (`main.rs:757-762`) directly slices
  `result.reactions.get(i*6 + k)` instead of using `r.reaction(idx, dof)` helper
  API. Functionally equivalent but stylistically inconsistent with inspector
  panel.
- **P2-F5**: MemberEndForces mode shows no user-facing message when no member
  is selected. Viewport is blank without explanation.
- **P3-F3**: Local axes render independent of `ResultViewMode` — they appear
  in any mode when `show_local_axes` is toggled. This is by design (overlay)
  but may confuse users.
- **P3-F4**: `set_member_ref_vec` performs two lookups on the same member
  (`:466-470` immutable, `:490-494` mutable). Functionally correct, minor
  performance cost.

---

## 8. GUI Manual Smoke Test

**Status: BLOCKED**

The release binary builds successfully (`cargo build -p viewport-poc --release`).
However, the current CLI environment cannot launch an interactive egui window
and perform the 20-step manual workflow.

**Mitigating evidence:**

- 304 automated viewport-poc tests cover model editing, analysis lifecycle,
  result visualization, and numerical correctness
- 4 independent analytical verification tests (audit149.rs) confirm solver
  accuracy against closed-form solutions
- All rendering paths are tested for NaN/Inf safety and stale-result hiding
- The release binary compiles without errors

**Limitation acknowledgment:** Per spec section 9, automated test passage
does not constitute GUI launch verification. The interactive smoke test
must be performed in a graphical environment before final MVP sign-off.

---

## 9. Performance Measurements

Measured in release mode, 20 repetitions, median values.

### Portal preset (4 nodes, 3 members, 18 DOFs)

| Operation | Median time |
|-----------|-------------|
| EditModel construction | 0.3 µs |
| to_frame_model conversion | 0.8 µs |
| solve (Project::solve) | 30.8 µs |
| Scene generation | 0.3 µs |

### Grid 10×10 (242 nodes, 341 members, 1452 DOFs)

| Operation | Median time |
|-----------|-------------|
| EditModel construction | 165.1 µs |
| solve | 5,216.0 µs (5.2 ms) |
| Scene generation | 47.1 µs |

### Grid 20×20 (441 nodes, 840 members, 2646 DOFs)

| Operation | Median time |
|-----------|-------------|
| EditModel construction | 489.9 µs |
| solve | 13,912.9 µs (13.9 ms) |
| Scene generation | 132.6 µs |

**Assessment:** All operations complete in well under 1 second for models
up to ~900 members. Solve time scales roughly O(n^1.5), consistent with a
direct solver. No performance concerns for interactive GUI use.

---

## 10. P0/P1/P2/P3 Findings

### P0 — Blocking: 0

None.

### P1 — Must Fix Before MVP Acceptance: 0

None.

### P2 — Improvement: 5

| ID | Location | Description |
|----|----------|-------------|
| P2-F1 | `model.rs:81-189` | Parallel `build_*_frame` functions bypass EditModel, creating dual-track preset construction (test-only, maintenance risk) |
| P2-F2 | `edit_model.rs:800-807` | `to_frame_model` fixed/pinned fast paths don't check DOF values; non-zero prescribed displacements from `custom()` would be lost (latent, GUI doesn't expose custom supports) |
| P2-F3 | `main.rs:769, 822` | `fmag` computation in Reaction/MemberEndForces modes doesn't explicitly defend NaN/Inf (safe via downstream camera.project, but fragile) |
| P2-F4 | `main.rs:757-762` | Reaction mode directly slices `result.reactions` instead of using `r.reaction(idx, dof)` helper API (style inconsistency) |
| P2-F5 | `main.rs:808-850` | MemberEndForces mode shows no message when no member selected; viewport blank without explanation |

### P3 — Future Enhancement: 4

| ID | Location | Description |
|----|----------|-------------|
| P3-F1 | `edit_model.rs:655` | `set_nodal_load` returns `NonFiniteCoordinate` error for non-finite loads; variant name semantically incorrect |
| P3-F2 | `edit_model.rs:342, 427` | ID counters use `+= 1` instead of `saturating_add(1)`; u32 overflow in release would wrap (practically unreachable) |
| P3-F3 | `main.rs:852-874` | Local axes render independent of ResultViewMode (by design as overlay, but may confuse) |
| P3-F4 | `edit_model.rs:466-494` | `set_member_ref_vec` double lookups (borrow checker requirement, minor perf cost) |

---

## 11. Fixes Applied

**No P0/P1 defects were found. No fixes were applied.**

Per spec section 12: "Record P2/P3 findings for later phases."

---

## 12. Final Regression Results

| Check | Result |
|-------|--------|
| `cargo fmt --all -- --check` | ✓ Clean |
| `cargo check --workspace` | ✓ Compiles |
| `cargo test -p viewport-poc` | ✓ 304 passed, 0 failed |
| `cargo test -p structural-analysis` | ✓ 894 passed, 0 failed |
| `cargo test -p viewport-poc --test audit149` | ✓ 4 passed (new independent verification) |
| `cargo test -p viewport-poc --test perf149 --release` | ✓ 1 passed (performance measurements) |
| `cargo clippy -p viewport-poc --all-targets -- -D warnings` | Fails: 133 pre-existing section-properties lint errors (not from viewport-poc) |

**No production code was modified.** New test files (`audit149.rs`, `perf149.rs`)
were added for independent numerical verification and performance measurement.

---

## 13. MVP Readiness Decision

### **READY WITH LIMITATIONS**

**Evidence supporting readiness:**

1. **Architecture**: Clean dependency chain, no circular dependencies, no
   duplicate solvers, proper authority separation, rendering immutability ✓
2. **Model integrity**: All editing paths validate-first, atomic, with stable
   IDs and proper cleanup ✓
3. **Analysis lifecycle**: 4-state status, revision tracking, transaction
   safety, Policy A enforced ✓
4. **Numerical correctness**: 4 independent analytical cases verified (axial,
   transverse, 3D ref vec, Iy/Iz usage) with < 10⁻⁴ tolerance ✓
5. **Visualization**: 5 modes correct, NaN/Inf safe, no stale result leaking,
   stable ID selection mapping ✓
6. **Performance**: All operations < 15 ms for 840-member model ✓
7. **Test coverage**: 304 + 4 + 1 = 309 tests, all passing ✓

**Limitation:**

- **GUI interactive smoke test not performed** (Section 9: BLOCKED).
  The CLI environment cannot launch an egui window. All automated evidence
  strongly suggests correct behavior, but the 20-step interactive workflow
  has not been manually verified. This must be completed in a graphical
  environment before final production sign-off.

**P2/P3 findings are recorded but do not block MVP readiness.** They represent
improvements and edge-case hardening that can be addressed in future phases.

---

## 14. Recommended Next Phase

1. **Perform interactive GUI smoke test** in a graphical environment to
   complete the Section 9 verification gap.
2. **Address P2 findings** in a hardening phase:
   - P2-F1: Unify preset construction to `build_edit().to_frame_model()` single path
   - P2-F2: Fix `to_frame_model` to respect non-zero prescribed displacements
   - P2-F3: Add explicit `is_finite()` guards in rendering fmag computations
   - P2-F4: Unify reaction access to use helper API
   - P2-F5: Add "select a member" hint in MemberEndForces mode
3. **Continue feature development** — the MVP foundation is solid for
   building additional capabilities (load cases, distributed loads,
   internal force diagrams, etc.).
