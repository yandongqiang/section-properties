# Phase 157 — Analysis Workflow & Result State Integration

**Status:** COMPLETE  
**Date:** 2026-10-13  
**Baseline:** `da91a4f` (Phase 156)  
**Branch:** main  

---

## 1. Baseline HEAD

```
da91a4f Phase 156: 3D frame support editing verification and viewport symbol improvement
```

Working tree clean (only pre-existing untracked files).

---

## 2. Existing Workflow Audit (Phase 157A)

### Audit Matrix

| Capability | Existing | Missing | Action |
|------------|----------|---------|--------|
| Model validation | ✅ Solver validates (FemError) | — | Verified by tests |
| Analyze command | ✅ Solve button (main.rs:614) | — | Verified by tests |
| Solver error handling | ✅ Err → Failed state (app_state.rs:202) | — | Verified by tests |
| Result storage | ✅ AnalysisResult with revision (app_state.rs:84) | — | Verified by tests |
| Stale tracking | ✅ Revision comparison (app_state.rs:144) | — | Verified by tests |
| Undo/Redo integration | ✅ replace_model invalidates (main.rs:178) | — | Verified by tests |
| Result display | ✅ Policy A, 5 modes (app_state.rs:173) | — | Verified by tests |
| Save/Load | ✅ Model only, NeverSolved on load | — | Verified by tests |

### Call Chain

```
EditModel → Project.solve()
    → EditModel.to_frame_model() → FrameModel3D
    → FrameSolver3D::from_model() → solve()
    → FrameAnalysisResult3D
    → Project.analysis = AnalysisResult { result, solved_revision }
    → analysis_status() checks revision vs model.revision()
    → analysis_result() returns Some only if Current (Policy A)
    → ViewportScene renders deformed/displacement/reaction
```

### Key Findings

1. **AnalysisStatus** enum (4 variants) already exists: `NeverSolved`, `Current`, `Stale`, `Failed`
2. **Failed takes priority** over all other states (app_state.rs:138)
3. **Display Policy A**: only `Current` results returned; Stale/Failed hidden (app_state.rs:173)
4. **Solve is synchronous** on button click — no re-triggering concern
5. **Undo/redo invalidates** results via `replace_model()` (conservative strategy)
6. **Persistence saves model only** — results recomputed on load
7. **Fingerprint** includes all engineering content, excludes revision (edit_model.rs:481)
8. **No new production code needed** — implementation already complete

---

## 3. Files Changed

| File | Action | Description |
|------|--------|-------------|
| `apps/viewport-poc/tests/phase157_analysis_workflow.rs` | **NEW** | 43 integration tests |

No production code changes.

---

## 4. Analysis State Design

Existing `AnalysisStatus` enum (app_state.rs:66):

```rust
pub enum AnalysisStatus {
    NeverSolved,  // No analysis ever performed
    Current,      // Results match current model revision
    Stale,        // Model edited since solve
    Failed,       // Last solve attempt failed
}
```

**Priority:** `Failed` > `NeverSolved` > `Current`/`Stale` (by revision match)

**Staleness detection:** `solved_revision == model.revision()` → Current, else Stale

**Fingerprint:** Used for dirty/save state, NOT for staleness (revision is sufficient)

---

## 5. Model Validation

- **Input validation:** EditModel validates all inputs (finite values, valid references, non-zero length, non-parallel ref_vec)
- **Solver/stability:** FrameSolver3D detects singular matrices, mechanisms
- **No auto-fix:** Solver never modifies EditModel, never adds supports, never fabricates zero results
- **Error propagation:** `Project::solve()` returns `Err(String)` with descriptive message

---

## 6. Analyze Workflow

```
User clicks Solve
    → Project.solve()
    → EditModel.to_frame_model() (convert + validate)
    → FrameSolver3D.solve() (stiffness + solve)
    → Ok → store AnalysisResult, clear failed flag
    → Err → set failed flag, leave prior result intact (hidden by Policy A)
```

- Solve executes once per button click (no per-frame auto-solve)
- No partial result writes on failure
- Success clears error messages in GUI

---

## 7. Error Handling

- `Project::solve()` returns `Result<(), String>` — all errors captured
- GUI displays error via `AppMessage::error()` (main.rs:338)
- Failed state shows "⚠ Last solve failed" in toolbar (main.rs:640)
- Status bar shows "FAILED" indicator (main.rs:680)
- No panic on mechanism/singular — returns Err

---

## 8. Result Display Integration

- **5 modes:** Model, Deformed, Displacement, Reaction, MemberEndForces
- **Policy A:** Only `Current` results displayed; Stale/Failed return `None`
- **Deformation scale:** Display-only, doesn't modify calculations
- **Node/Member ID mapping:** Via stable ID → index lookup in ViewportScene
- **Empty model:** ViewportScene handles gracefully (empty vectors, no crash)
- **Stale results:** Hidden with "re-solve" prompt in UI
- **Failed results:** Hidden with "fix and re-solve" prompt in UI

---

## 9. History / Persistence

- **Save:** `persistence::save_project()` saves EditModel only (no results)
- **Load:** `Project::load_model()` resets to NeverSolved
- **Undo:** `replace_model()` clears analysis → NeverSolved (conservative)
- **Redo:** Same as undo — requires re-solve
- **Preset switch:** `load_model()` resets to NeverSolved
- **No result reuse:** Each `load_model`/`replace_model` clears `analysis = None`
- **No V4/V5 needed:** Persistence schema unchanged

---

## 10. Integration Tests

**File:** `apps/viewport-poc/tests/phase157_analysis_workflow.rs`  
**Tests:** 43 (all passing)

### Case A — Successful Analysis (6 tests)
- Portal preset solve success
- Result exists after solve
- Displacements non-zero
- Reactions non-zero
- Scene builds from solved model
- All 5 presets solve

### Case B — Model Modification (5 tests)
- Fingerprint changes on edit
- Revision changes on edit
- Result becomes stale after edit
- Stale result not displayed (Policy A)
- Re-solve after edit produces different result

### Case C — Undo/Redo (3 tests)
- Undo after solve invalidates result
- Redo after undo restores model
- Solve after undo produces valid results

### Case D — Invalid Model (4 tests)
- Empty model solve fails
- No-nodes model solve fails
- Failed does not show result
- Error message is descriptive

### Case E — Mechanism (4 tests)
- No supports → singular failure
- Mechanism no panic
- No auto support added
- Failed → fix → solve success

### Case F — Preset Switching (2 tests)
- Switch preset discards old result
- Multiple presets independent

### Case G — Save/Load (5 tests)
- Save/load preserves model
- Load resets to NeverSolved
- Save does not make stale valid
- Filesystem round-trip
- No result reuse across projects

### Case H — Mixed Loads (4 tests)
- Mixed loads solve
- Displacements non-zero
- Vertical equilibrium
- Superposition (nodal + member = mixed)

### State Transition Coverage (10 tests)
- NeverSolved initial
- Current after solve
- Stale after edit
- Failed after error
- Failed overrides Stale
- Clear resets
- Replace model resets
- Display no crash on empty
- Display no crash on failed
- Persistence does not save results

---

## 11. Regression Results

### Format
```
cargo fmt --all -- --check  →  PASS
```

### Compilation
```
cargo check -p viewport-poc --tests  →  PASS
```

### Tests
```
cargo test -p viewport-poc --release  →  601 passed, 0 failed
```

Breakdown:
- 194 unit tests (lib)
- 15 app_state integration
- 4 audit149
- 6 deformation
- 72 edit_model integration
- 6 geometry
- 1 perf149
- 1 performance
- 31 persistence_integration
- 14 phase151_regression
- 47 phase152_undo_redo
- 35 phase153_member_loads
- 51 phase154_point_loads
- 48 phase156_support_editing
- **43 phase157_analysis_workflow** (NEW)
- 17 robustness
- 12 result_viz
- 4 scene_render
- 0 main binary

### Clippy
```
cargo clippy -p viewport-poc --tests  →  No warnings from Phase 157 code
```

### GUI Runtime
**BLOCKED** — No GUI test harness available.

---

## 12. Remaining Risks

1. **GUI Runtime untested** — No automated GUI testing; manual verification needed
2. **Solve keyboard shortcut** — Not implemented (minor UX gap)
3. **Async solving** — Not implemented (synchronous sufficient for MVP)
4. **Analysis result persistence** — By design, results are recomputed on load

---

## 13. Phase 158 Readiness

```
P0: 0
P1: 0
P2: 0
P3: 0

Model validation:     ✅ Complete
Analyze workflow:      ✅ Complete
Solver diagnostics:    ✅ Complete
Result state:          ✅ Complete
Stale tracking:        ✅ Complete
Undo/Redo:             ✅ Complete
Persistence:           ✅ Complete
Viewport integration:  ✅ Complete
GUI compile:           ✅ PASS
GUI runtime:           ⛔ BLOCKED (no display)

Existing tests: 558
New tests: 43
Total tests: 601

Phase 158 readiness: READY
```
