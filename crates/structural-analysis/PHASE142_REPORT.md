# Phase 142 — egui Desktop Application Foundation

**Date:** 2026-10-10  
**Baseline HEAD:** `2e1288b` (Phase 141)  
**Final HEAD:** TBD (this commit)  
**Status:** Complete  

---

## 1. Baseline

| Item | Value |
|------|-------|
| Baseline HEAD | `2e1288b` (Phase 141) |
| Working tree at start | Clean (only untracked audit docs) |
| Rust toolchain | 1.97.1, edition 2024 |
| Platform | Windows (win32) |

### Commits verified

```
2e1288b Phase 141: audit viewport architecture and GUI options
654aeb3 Phase 140: add Tauri Three.js viewport comparison PoC
b9335d8 Phase 139: add 3D viewport proof of concept
```

### Decision context

Phase 141 recommended Tauri + Three.js (verdict A). The user overrode this recommendation and chose **egui + eframe as the main GUI line**. Tauri PoC is retained as a reference only.

---

## 2. Existing egui Architecture Audit

### Module structure (before Phase 142)

```
apps/viewport-poc/
├── src/
│   ├── lib.rs       — module declarations + re-exports
│   ├── main.rs      — ViewportApp, eframe::App impl, rendering, side panel
│   ├── camera.rs    — Camera3D: orbit/pan/zoom/project
│   ├── scene.rs     — ViewportScene: node_positions, member_nodes, member_ref_vecs
│   ├── selection.rs — SelectionState enum: None/Node/Member
│   ├── display.rs   — ResultDisplayState: deformation_scale, show_* toggles
│   ├── model.rs     — build_portal_frame, build_inclined_frame, build_grid_frame
│   └── project.rs   — Vec3 math helpers
└── tests/
    ├── geometry.rs, deformation.rs, selection.rs, robustness.rs, performance.rs
```

### ViewportApp struct (before)

```rust
struct ViewportApp {
    model: FrameModel3D,              // hardcoded, not Option
    result: Option<FrameAnalysisResult3D>,
    scene: Option<ViewportScene>,
    camera: Camera3D,
    selection: SelectionState,
    display: ResultDisplayState,
}
```

### Issues identified

| # | Issue | Impact |
|---|-------|--------|
| 1 | No top menu/toolbar — only right side panel | No global actions accessible |
| 2 | No left model browser — model hardcoded | Cannot switch models from UI |
| 3 | No bottom status/diagnostic panel | No error/message display |
| 4 | No empty project state — always loads portal at startup | Cannot represent "no project" |
| 5 | No staleness tracking — result is Option but no "stale" marker | Old results can be mistaken for current |
| 6 | Model and result tightly coupled in ViewportApp | No clear separation of concerns |
| 7 | No project concept — just a single hardcoded model | No foundation for multi-model workflow |
| 8 | Re-solve button buried in side panel | Poor UX for primary action |
| 9 | Errors silently discarded with `.ok()` | User never sees failure reason |

---

## 3. New and Modified Files

### New files

| File | Lines | Responsibility |
|------|-------|----------------|
| `src/app_state.rs` | 215 | `Project`, `AnalysisResult`, `AppMessage`, `MessageLevel` — application state types separating engineering data from rendering |
| `tests/app_state.rs` | 165 | Integration tests for project lifecycle, staleness, selection validation, deformation safety |

### Modified files

| File | Change | Lines |
|------|--------|-------|
| `src/lib.rs` | Added `pub mod app_state` + re-exports | +2 |
| `src/model.rs` | Added `PresetModel` enum + 3 tests | +70 |
| `src/main.rs` | Restructured to 5-panel layout, `Project` integration, empty state, messages, staleness, selection validation | +274/-61 |

### Unchanged files

All existing source files (`camera.rs`, `scene.rs`, `selection.rs`, `display.rs`, `project.rs`) and all existing test files (`geometry.rs`, `deformation.rs`, `selection.rs`, `robustness.rs`, `performance.rs`) are **unchanged**. All existing tests pass without modification.

---

## 4. State Boundaries and Data Flow

### Type hierarchy

```
Project
├── name: String
├── model: Option<FrameModel3D>           — the structural model
└── analysis: Option<AnalysisResult>
    ├── result: FrameAnalysisResult3D     — immutable analysis snapshot
    └── is_stale: bool                    — staleness flag

ViewportScene                              — derived display geometry (not in Project)
Camera3D                                   — viewport camera (not in Project)
SelectionState                             — current selection (not in Project)
ResultDisplayState                         — display options (not in Project)
Vec<AppMessage>                            — diagnostic messages (not in Project)
```

### Responsibility boundaries

| Type | Role | Who mutates it |
|------|------|----------------|
| `Project` | User's engineering data | `load_model()`, `solve()`, `clear()` — explicit operations only |
| `AnalysisResult` | Immutable result + staleness | `Project::solve()` creates fresh; `Project::load_model()` marks stale |
| `ViewportScene` | Derived rendering geometry | Rebuilt only in `run_solve()` — never on camera/selection/display changes |
| `Camera3D` | Viewport camera | Mouse/keyboard input in central panel |
| `SelectionState` | Current selection | Click in viewport, `validate_selection()` |
| `ResultDisplayState` | Display toggles/scale | Inspector panel controls |
| `AppMessage` | Diagnostic messages | Any operation that produces info/warning/error |

### Data flow

```
User action          →  Method called         →  What changes
─────────────────────────────────────────────────────────────
Click model in browser → load_preset()       →  Project.load_model() + run_solve()
Click Solve button     → run_solve()         →  Project.solve() + scene rebuild + camera fit
Click "New (empty)"    → clear_project()     →  Project.clear() + scene = None
Drag mouse in viewport → camera.orbit/pan    →  Camera only (no solve, no scene rebuild)
Scroll in viewport     → camera.zoom         →  Camera only
Click node/member      → pick()              →  Selection only (no solve)
Change display toggle  → checkbox binding    →  DisplayState only (no solve, no scene rebuild)
Change deform scale    → slider binding      →  DisplayState only (deformed_positions computed per frame)
```

### Key invariants enforced

1. **GUI does not modify analysis results** — `AnalysisResult` is only created by `Project::solve()`
2. **Viewport does not run FEM** — rendering code only reads `self.scene` and `self.analysis_result()`
3. **Model edit marks results stale** — `Project::load_model()` calls `mark_stale()` on existing results
4. **Camera/scale changes don't trigger solve** — no call path from camera/display mutation to `run_solve()`
5. **No unnecessary scene rebuild** — scene is only rebuilt in `run_solve()`, not on display/camera/selection changes
6. **No unnecessary model copy** — `Project` owns the model; `ViewportScene` borrows from result to build display geometry

---

## 5. Desktop Application Shell

### 5-panel layout

```
┌──────────────────────────────────────────────────────┐
│  Toolbar: [File ▾] [Solve] [Clear selection] [⚠ stale]│  Top
├──────────┬────────────────────────────┬──────────────┤
│          │                            │              │
│  Model   │       3D Viewport           │  Inspector   │
│  Browser │       (central)             │  (right)     │
│  (left)  │                            │              │
│          │                            │              │
├──────────┴────────────────────────────┴──────────────┤
│  Model: Portal | Nodes: 4 | Members: 3 | Analysis: ✓  │  Bottom
└──────────────────────────────────────────────────────┘
```

### Panel details

| Panel | egui widget | Default size | Resizable | Content |
|-------|-------------|-------------|-----------|---------|
| Top | `TopBottomPanel::top` | auto | No | File menu, Solve button, Clear selection, staleness indicator |
| Bottom | `TopBottomPanel::bottom` | 28px | No | Model name, node/member counts, analysis status, last message |
| Left | `SidePanel::left` | 180px | Yes | Preset model list (5 models), Clear project button |
| Right | `SidePanel::right` | 350px | Yes | Display controls, Selection details, Camera info |
| Center | `CentralPanel` | remaining | — | 3D viewport (existing rendering code) |

### Empty state

When no model is loaded:
- Viewport shows "No model loaded" centered text
- Status panel shows "No model loaded"
- Inspector panels show empty sections (no crash, no fake data)
- Solve button is disabled
- Help text still visible

### Staleness indicator

When results are stale (model changed after last solve):
- Toolbar shows "⚠ Results are stale — re-solve to update" in yellow
- Status panel shows "Analysis: STALE" in yellow
- Old results remain accessible (display shows old data with warning)

---

## 6. Testing Results

### Automated tests

| Command | Result | Count |
|---------|--------|-------|
| `cargo fmt --all -- --check` | ✅ Pass | — |
| `cargo check --workspace` | ✅ Pass (0 warnings) | — |
| `cargo test -p viewport-poc` | ✅ All pass | 79 tests |
| `cargo test -p structural-analysis --release --lib` | ✅ All pass | 26 tests |
| `cargo test -p tauri-viewport-poc --release` | ✅ All pass | 36 tests (unchanged) |

### Test breakdown (viewport-poc: 79 tests)

| Suite | Before | After | New tests |
|-------|--------|-------|-----------|
| lib (inline) | 26 | 39 | +10 `app_state` tests, +3 `PresetModel` tests |
| app_state (integration) | 0 | 11 | +11 project lifecycle/staleness/selection tests |
| geometry | 6 | 6 | unchanged |
| deformation | 6 | 6 | unchanged |
| selection | 4 | 4 | unchanged |
| robustness | 12 | 12 | unchanged |
| performance | 1 | 1 | unchanged |
| **Total** | **55** | **79** | **+24** |

### New test coverage

| Test | What it verifies |
|------|-----------------|
| `empty_project` | Empty state is valid |
| `default_is_empty` | Default == empty |
| `load_model_marks_existing_results_stale` | Staleness on model switch |
| `load_model_without_prior_result_does_not_mark_stale` | No false staleness |
| `solve_produces_fresh_result` | Solve clears staleness |
| `solve_after_stale_produces_fresh` | Re-solve works after stale |
| `solve_without_model_errors` | Error on empty solve |
| `clear_resets_to_empty` | Clear works correctly |
| `message_constructors` | AppMessage levels |
| `message_clone` | AppMessage is Clone |
| `preset_names_are_unique` | No duplicate model names |
| `preset_all_builds_solve` | All 5 presets solve |
| `preset_grid_10x10_node_count` | Grid 10×10 has 242 nodes, 341 members |
| `deformation_scale_does_not_trigger_solve` | Scale change doesn't modify results |
| `selection_validation_*` | Invalid selection detected |
| `all_presets_solve_and_produce_scenes` | All presets produce valid scenes |
| `stale_results_still_accessible` | Old results available after model switch |

### Manual GUI verification

**UNVERIFIED** — No GUI environment available in this session. The application compiles and all automated tests pass, but runtime behavior (window display, rendering, mouse interaction) has not been manually verified.

---

## 7. Regression Protection

| Constraint | Status |
|------------|--------|
| No modification to structural-analysis numerical algorithms | ✅ Verified — no files in `crates/structural-analysis/src/` modified |
| No modification to section-properties public API | ✅ Verified — no files in `crates/section-properties/` modified |
| Tauri PoC retained | ✅ Verified — `apps/tauri-viewport-poc/` unchanged, 36 tests pass |
| Existing egui viewport behavior preserved | ✅ All 55 original tests pass unchanged |
| No new GUI framework | ✅ Only `eframe`/`egui` 0.29 (existing dependencies) |
| No new dependencies | ✅ `Cargo.toml` unchanged |
| No push to GitHub | ✅ Not pushed |
| No unauthorized commit | ✅ Not committed (awaiting user approval) |

---

## 8. Solved Problems and Remaining Risks

### Solved

| # | Problem | Solution |
|---|---------|----------|
| 1 | No empty project state | `Project::empty()` + `Option<FrameModel3D>` |
| 2 | No staleness tracking | `AnalysisResult.is_stale` flag + `Project::load_model()` marks stale |
| 3 | No error display | `Vec<AppMessage>` + bottom status panel |
| 4 | No model browser | `PresetModel` enum + left panel |
| 5 | No layout structure | 5-panel egui layout |
| 6 | Selection to non-existent object | `validate_selection()` called each frame |
| 7 | Solve button accessibility | Toolbar with Solve button |
| 8 | Silent error swallowing | All errors push `AppMessage::error()` |

### Remaining risks

| # | Risk | Severity | Mitigation |
|---|------|----------|------------|
| 1 | GUI not manually verified | Medium | User should run `cargo run -p viewport-poc` and verify |
| 2 | FEM solve blocks UI thread | Low | Acceptable for current model sizes (< 10ms). Async infrastructure deferred to future phase per spec. |
| 3 | No project serialization | Low | Out of scope for Phase 142 (foundation only) |
| 4 | No model editing | Low | Out of scope for Phase 142 (foundation only) |
| 5 | Messages grow unbounded | Low | `Vec<AppMessage>` has no cap; for long sessions, could add truncation |
| 6 | Grid range hardcoded (-2 to 8) | Low | Existing behavior; should be derived from model bounding box in future |

---

## 9. Summary

### Implemented

- 5-panel desktop application shell (toolbar, model browser, viewport, inspector, status)
- `Project` type with model + analysis result + staleness tracking
- `AppMessage` / `MessageLevel` for diagnostic messages
- `PresetModel` enum with 5 available models
- Empty project state with appropriate UI
- Selection validation (clears invalid node/member references)
- Staleness indicator in toolbar and status panel
- Error messages captured and displayed

### Automated verification

- 79 viewport-poc tests pass (55 existing + 24 new)
- 26 structural-analysis lib tests pass
- 36 tauri-viewport-poc tests pass (unchanged)
- `cargo fmt --all -- --check` passes
- `cargo check --workspace` passes (0 warnings)

### Human verification

- **UNVERIFIED** — desktop GUI not launched in this session

### Not yet verified

- Window startup/shutdown
- Empty scene display
- Camera interaction (orbit/pan/zoom/reset)
- Node/member selection
- Window resize behavior
- Panel layout stability
- Real analysis result display
- Deformation scale change preserving original data

### Recommended Phase 143

**Model editing and project persistence**

- Node editor (add/move/delete nodes)
- Member editor (add/delete members, assign sections)
- Load editor (nodal loads, member loads)
- Constraint editor (fix nodes, fix DOFs)
- Project save/load (serialization to file)
- Section library integration

This builds on the Phase 142 foundation by adding the first real engineering workflow (model editing) while the GUI shell and state boundaries are fresh and stable.
