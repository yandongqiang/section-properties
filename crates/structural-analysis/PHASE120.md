# Phase 120 — 3D Truss Envelope Implementation

**Date:** 2026-10-09
**Base commit:** `2ff6ac9` (Phase 118)
**Scope:** Implement production-quality envelope API for 3D truss analysis

---

## 1. Baseline

```
HEAD = 2ff6ac9 (Phase 118: 3D Truss LoadCase/LoadCombination deep audit)
Working tree: clean (no uncommitted code changes)
```

---

## 2. API Design Decisions

### New types (in `postprocessing.rs`)

| Type | Purpose |
|------|---------|
| `Truss3DEnvelope` | Top-level multi-case envelope for 3D truss results |
| `Truss3DNodeDisplacementSample` | Per-node displacement envelope (ux, uy, uz) |
| `Truss3DNodeReactionSample` | Per-node reaction envelope (rx, ry, rz) |
| `Truss3DAxialForceSample` | Per-element axial-force envelope |

### Design choices

1. **Reuse `Extremum` and `LoadSource`** — no duplication of numeric aggregation or provenance logic.
2. **3D-specific physical types** — `Truss3DNodeDisplacementSample` has `ux/uy/uz` (not `ux/uy/rz` like Frame). No bending moments, shear, or rotations.
3. **`from_results` constructor** — accepts `&[&TrussAnalysisResult3D]`, consistent with Frame's `from_frame_results`.
4. **No generic traits** — no `Result` trait, no `Envelopeable` trait. Each model has its own constructor.
5. **`constrained_dofs` field added to `TrussAnalysisResult3D`** — smallest justified change to enable reaction envelope (see below).

### `TrussAnalysisResult3D` modification

Added `constrained_dofs: Vec<(usize, usize)>` field and `constrained_dofs()` accessor.

**Why:** The reaction envelope must distinguish physical support reactions from free-DOF round-off residuals. Without constraint metadata, the envelope would either (a) sample all DOFs including meaningless residuals, or (b) require the caller to pass external constraint info. Option (a) violates the spec; option (b) breaks encapsulation. Storing constrained DOFs in the result is the same pattern used by `FrameAnalysisResult::support_dofs()`.

**Migration cost:** Minimal — one field added, populated automatically in `TrussSolver3D::results()`. No existing API signatures changed. The field is `pub` with a accessor method.

---

## 3. Compatibility Policy

Same as Frame Envelope (Phase 119 audit confirmed sufficient):

- All results must have the same `n_nodes`
- All results must have the same `n_elements`
- Count mismatch → `FemError::InvalidInput`
- Same-count topology mismatch → accepted (documented limitation; user responsibility)

---

## 4. Reaction Envelope Semantics

- Reactions sampled **only at constrained DOFs** (`r.constrained_dofs()`)
- Free-DOF residuals **excluded** — no artificial zero extrema
- DOF constrained in some results but not others → populated only from results where constrained
- Sign convention: `R = K·u − f` (same as solver)
- Prescribed displacements add constraints → their reactions are physical and included

---

## 5. Tests Added

15 tests in `tests/truss3d.rs` (lines 1219–1621):

| # | Test | Description |
|---|------|-------------|
| 1 | `p120_single_result_envelope_matches_original` | Single result: min == max == original |
| 2 | `p120_multiple_results_component_wise_extrema` | Multiple results: component-wise ux/uy/uz |
| 3 | `p120_reaction_extrema_for_supported_dofs` | Reaction extrema at supported DOFs |
| 4 | `p120_free_dofs_no_artificial_reactions` | Free DOFs not populated |
| 5 | `p120_mixed_support_configurations` | Prescribed displacement adds constraint |
| 6 | `p120_axial_force_min_max` | Per-element axial-force min/max |
| 7 | `p120_governing_load_source_correct` | Governing LoadSource for every extremum |
| 8 | `p120_case_and_combination_sources` | solve_case + solve_combination provenance |
| 9 | `p120_ties_keep_first_source` | Deterministic tie behavior |
| 10 | `p120_empty_input_rejected` | Empty input → InvalidInput |
| 11 | `p120_incompatible_counts_rejected` | Node/element count mismatch rejected |
| 12 | `p120_same_count_incompatible_topology` | Same-count topology accepted (documented) |
| 13 | `p120_reorder_preserves_extrema` | Reordering preserves numeric extrema |
| 14 | `p120_analytical_benchmark` | Analytical axial bar: δ=FL/EA, N=F, R=-F |
| 15 | `p120_frame_envelope_regression` | Frame Envelope unchanged |

**All 15 tests pass.** Total truss3d tests: 48 (33 existing + 15 new).

---

## 6. Files Changed

| File | Changes |
|------|--------|
| `src/lib.rs` | Export 4 new types (+2 lines) |
| `src/postprocessing.rs` | 4 new types + `Truss3DEnvelope` impl (+272 lines) |
| `src/truss3d.rs` | `constrained_dofs` field + accessor + populate in `results()` (+24 lines) |
| `tests/truss3d.rs` | 15 new tests + helpers (+404 lines) |

**Total: 4 files, +701 insertions, -2 deletions**

**Unrelated files modified:** None
**Existing uncommitted changes preserved:** Yes (Phase 118 was already committed)
**`section-properties` modified:** No

---

## 7. Validation Results

| Command | Result |
|---------|--------|
| `cargo fmt --all -- --check` | ✅ PASS |
| `cargo check -p structural-analysis` | ✅ PASS |
| `cargo test -p structural-analysis` | ✅ PASS (all suites, 0 failures) |
| `cargo test -p structural-analysis --doc` | ✅ PASS (13 doctests, 0 failures) |
| `cargo test --examples -p structural-analysis` | ✅ PASS (0 examples) |
| `cargo clippy -p structural-analysis --all-targets` | ✅ PASS (0 errors; pre-existing warnings only) |
| `cargo clippy -p structural-analysis --all-targets -- -D warnings` | ❌ FAIL (pre-existing `section-properties` lint debt — not from this change) |
| `cargo test --workspace` | ⏱️ TIMEOUT — PRE-EXISTING CONFIRMED (`section-properties` FEM benchmarks) |

---

## 8. Problem List

| Category | Count | Details |
|----------|-------|---------|
| P0 | 0 | — |
| P1 | 0 | — |
| P2 | 0 | — |
| P3 | 0 | — |

---

## 9. Known Limitations

1. **Same-count topology mismatch not detected** — documented limitation, same as Frame Envelope. Count check catches size mismatches; topology identity is user responsibility.
2. **2D Truss Envelope not implemented** — 2D Truss lacks LoadCase/LoadCombination support (prerequisite gap). Out of scope for Phase 120.
3. **`clippy -D warnings` fails workspace-wide** — pre-existing `section-properties` lint debt, not from this change.

---

## 10. Recommended Next Phase

**Phase 121 candidate:** 2D Truss LoadCase/LoadCombination support — the prerequisite for 2D Truss Envelope. This would close the last gap in cross-model load case coverage.
