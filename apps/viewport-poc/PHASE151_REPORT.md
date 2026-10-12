# Phase 151 — GUI Runtime Acceptance & Interaction Hardening

## 1. Baseline

| Item | Value |
|---|---|
| HEAD | `5fbbf86` (Phase 150) |
| Working tree | Only untracked audit docs from other crates |
| Rust toolchain | 1.97.1 |
| Host target | `x86_64-pc-windows-msvc` |
| Launch command | `cargo run -p viewport-poc` |
| Baseline tests | 355 (viewport-poc) |
| Clippy warnings | 61 in viewport-poc (pre-existing, style only) |

## 2. Runtime Environment

| Acceptance level | Status |
|---|---|
| Compile verification | PASS |
| Automated test verification | PASS (369 tests) |
| Process launch verification | PASS (process starts, runs until killed) |
| Visible-window verification | **BLOCKED** (no graphical environment) |
| Manual interaction verification | **BLOCKED** (no graphical environment) |

The application binary compiles and starts. The process runs without
panic until terminated by timeout. However, the CodeBot execution
environment has no graphical display, so visible-window and manual
interaction verification could not be performed.

## 3. Launch Verification

| Check | Result |
|---|---|
| Binary compiles | PASS |
| Process starts | PASS |
| No startup panic | PASS (ran 10s without crash) |
| Window opens | BLOCKED |
| Toolbar renders | BLOCKED |
| Model browser renders | BLOCKED |
| Viewport renders | BLOCKED |
| Inspector renders | BLOCKED |
| Status bar renders | BLOCKED |
| Window resizing works | BLOCKED |
| Window closing works | BLOCKED |

## 4. Engineering Workflow Results

All 20 steps: **BLOCKED** — requires graphical interaction.

The engineering workflow (load preset → select node → edit coordinates →
select member → edit section → edit ref vec → apply → modify support →
apply load → solve → inspect results → change amplification → modify
load → confirm stale → re-solve → confirm updated results) requires
actual GUI interaction, which is not available in this environment.

## 5. Persistence Workflow Results

All persistence workflow steps: **BLOCKED** for runtime, **VERIFIED**
for logic via static audit and automated tests.

| Sub-test | Runtime | Static/Logic |
|---|---|---|
| Save As — file creation | BLOCKED | PASS (Phase 150 tests) |
| Save As — dirty clears | BLOCKED | PASS (Phase 150 tests) |
| Save — dirty appears on edit | BLOCKED | PASS (P1-2 regression) |
| Save — file updates | BLOCKED | PASS (Phase 150 tests) |
| Open — geometry restored | BLOCKED | PASS (Phase 150 tests) |
| Open — NeverSolved status | BLOCKED | PASS (Phase 150 tests) |
| Open — old overlays absent | BLOCKED | PASS (Phase 150 tests) |
| Unsaved — Save choice | BLOCKED | PASS (P1-1 regression) |
| Unsaved — Discard choice | BLOCKED | PASS (code inspection) |
| Unsaved — Cancel preserves | BLOCKED | PASS (code inspection) |
| Failed Save → no destructive op | BLOCKED | **PASS (P1-1 fix + regression)** |
| Invalid project — error shown | BLOCKED | PASS (Phase 150 tests) |
| Invalid project — current intact | BLOCKED | PASS (Phase 150 tests) |

## 6. Interaction Integrity Findings

### Selection

| Check | Finding |
|---|---|
| Selecting nodes and members | OK — `pick()` correctly identifies nearest entity |
| Switching between selections | OK — `validate_selection()` clears stale selections |
| Deleting selected entities | OK — selection set to None, scene rebuilt |
| Selection after loading another project | OK — `do_open()` sets `selection = None` |
| Inspector buffer reset after selection changes | OK — `sync_member_edit_buffer()` resets on ID change |

### Editing

| Check | Finding |
|---|---|
| DragValue precision | OK — ranges and speeds are reasonable |
| Negative values | OK — ranges include negative values |
| Very small engineering values | P2 — speed=0.1 may make small values hard to set |
| Invalid reference vectors | OK — `set_member_ref_vec` rejects, buffer reset |
| Invalid section values | OK — `set_member_section` rejects, error shown |
| Incomplete user input | OK — text parse errors show messages |
| Apply/Reset semantics | OK — Apply commits, Reset restores from model |

### Viewport

| Check | Finding |
|---|---|
| Camera orbit | OK — left drag |
| Pan | OK — right drag |
| Zoom | OK — scroll |
| Fit to model | OK — `rebuild_scene()` calls `Camera3D::fit` |
| Selection highlighting | OK — selected entities rendered with distinct color/size |
| Support symbols | OK — circle + label (F/P/Cn) |
| Nodal load arrows | OK — force arrow + moment circle |
| Result overlays | OK — deformed shape, displacement color, reactions, member forces |

### Layout

| Check | Finding |
|---|---|
| Small window sizes | OK — viewport guard `w < 10 || h < 10` |
| High-DPI scaling | UNVERIFIED — egui handles automatically |
| Long error messages | P2 — status bar may truncate |
| Inspector scrolling | OK — `ScrollArea::vertical()` |
| Toolbar overflow | P2 — no overflow handling for very narrow windows |
| Dialog focus and modality | P2 — `egui::Window` is not strictly modal |

## 7. Static Audit Results

### State transitions traced

| Transition | Finding |
|---|---|
| New → unsaved confirmation → Save → New | **P1-1 FIXED**: failed Save no longer proceeds to New |
| Open → invalid file → preserve current project | OK — `do_open()` error path preserves all state |
| Save As → cancel dialog | OK — Cancel sets `pending_action = None` |
| Member selection → edit buffer → selection change | OK — `sync_member_edit_buffer` resets buffer |
| Solve → edit → stale → re-solve | OK — revision tracking, `analysis_status()` |
| Delete selected node → selection cleanup | OK — `selection = None`, `rebuild_scene()` |
| Preset replacement → scene rebuild | **P1-3 FIXED**: preset load now checks `is_dirty()` |
| File load → inspector buffer reset | P2 — buffer not explicitly reset (harmless, reset on next selection) |

### Issues found

| Issue | Severity | Status |
|---|---|---|
| Failed Save in dialog proceeds to destructive action | P1 | **FIXED** |
| Clear project doesn't check unsaved changes | P1 | **FIXED** |
| Preset loading doesn't check unsaved changes | P1 | **FIXED** |
| Camera resets on every model edit | P2 | Documented |
| Member edit buffer not reset on model replacement | P2 | Documented (harmless) |
| apply_member_edit partial application doesn't rebuild scene | P2 | Documented (section change doesn't affect geometry) |
| Status bar may truncate long error messages | P2 | Documented |
| Dialog is not strictly modal | P2 | Documented (egui limitation) |

## 8. P0/P1/P2/P3 Findings

### P0 — Blocking: 0

No P0 defects found.

### P1 — Must Fix: 3 (all fixed)

| ID | Description | Fix |
|---|---|---|
| P1-1 | Failed Save in unsaved-changes dialog proceeds to destructive New/Open action | `do_save`/`save_to_path`/`do_save_as` now return `bool`; dialog only calls `execute_pending()` on success |
| P1-2 | "Clear project" doesn't check for unsaved changes | Added `PendingAction::Clear`; Clear project now checks `is_dirty()` before proceeding |
| P1-3 | Preset loading doesn't check for unsaved changes | Added `PendingAction::LoadPreset(PresetModel)`; preset loading now checks `is_dirty()` before proceeding |

### P2 — Improvement: 6

| ID | Description |
|---|---|
| P2-1 | Camera resets on every model edit (`rebuild_scene` calls `Camera3D::fit`) |
| P2-2 | Member edit buffer not reset on model replacement (harmless — reset on next selection) |
| P2-3 | `apply_member_edit` partial application (section OK, ref_vec fail) doesn't rebuild scene |
| P2-4 | Status bar may truncate long error messages |
| P2-5 | Unsaved-changes dialog is not strictly modal (egui limitation) |
| P2-6 | 61 pre-existing clippy warnings (style, not correctness) |

### P3 — Future: 4

| ID | Description |
|---|---|
| P3-1 | Undo/redo |
| P3-2 | Autosave / crash recovery |
| P3-3 | Recent files list |
| P3-4 | Native file dialog (currently text input only) |

## 9. Fixes Applied

### P1-1: Failed save no longer proceeds to destructive action

**Files**: `src/main.rs`

- `do_save()` → returns `bool` (true = success)
- `do_save_as()` → returns `bool` (true = success)
- `save_to_path()` → returns `bool` (true = success)
- Unsaved-changes dialog: only calls `execute_pending()` when save returns `true`

**Failure mode prevented**: User has unsaved changes, clicks "New", dialog
appears, clicks "Save", save fails (disk full / permission denied) →
previously the New action would execute, losing unsaved changes. Now the
dialog stays open and the error is shown.

### P1-2: Clear project checks unsaved changes

**Files**: `src/main.rs`

- Added `PendingAction::Clear` variant
- "Clear project" in toolbar and left panel now checks `is_dirty()` before proceeding
- `execute_pending()` handles `Clear` variant

**Failure mode prevented**: User has unsaved changes, clicks "Clear
project" → previously the model was destroyed without warning. Now the
unsaved-changes dialog appears.

### P1-3: Preset loading checks unsaved changes

**Files**: `src/main.rs`

- Added `PendingAction::LoadPreset(PresetModel)` variant
- Preset buttons in left panel now check `is_dirty()` before loading
- `execute_pending()` handles `LoadPreset` variant

**Failure mode prevented**: User has unsaved changes, clicks a preset
button → previously the model was replaced without warning. Now the
unsaved-changes dialog appears.

## 10. Regression Results

| Test file | Tests | Status |
|---|---|---|
| `tests/phase151_regression.rs` | 14 | ALL PASS |
| All Phase 150 tests | 355 | ALL PASS |
| **Total** | **369** | **ALL PASS** |

### Regression test details

| Test | Prevents |
|---|---|
| `p1_1_failed_save_returns_error_not_success` | Save failure must return Err |
| `p1_1_successful_save_returns_ok` | Save success must return Ok |
| `p1_1_failed_save_preserves_revision` | Model revision unchanged after failed save |
| `p1_2_edited_model_is_dirty_blocks_clear` | Edited model detected as dirty |
| `p1_2_unchanged_model_is_not_dirty_allows_clear` | Unchanged model not dirty |
| `p1_3_dirty_model_blocks_preset_load` | Dirty model blocks preset load |
| `p1_3_clean_model_allows_preset_load` | Clean model allows preset load |
| `dirty_state_separate_from_analysis_staleness` | Dirty ≠ stale |
| `loaded_model_is_not_dirty` | Freshly loaded model not dirty |
| `never_saved_model_is_dirty` | Never-saved model is dirty |
| `empty_project_is_not_dirty` | No model = not dirty |
| `pending_action_variants_are_distinct` | All action variants are distinct |
| `model_replacement_invalidates_old_selections` | Old selections invalid after replacement |
| `inspector_buffer_is_independent_of_model` | Buffer holds copy, not reference |

## 11. Remaining Runtime Acceptance Items

The following items require manual verification in a graphical
environment and could not be completed in this phase:

1. **Visible window verification**: Confirm window opens with correct title and size
2. **Toolbar rendering**: Confirm all buttons and menus render correctly
3. **Model browser rendering**: Confirm preset buttons and file operations render
4. **Viewport rendering**: Confirm 3D model renders with correct colors
5. **Inspector rendering**: Confirm all editing controls render
6. **Status bar rendering**: Confirm model info and messages display
7. **Window resizing**: Confirm panels resize correctly
8. **Engineering workflow** (20 steps): Full preset → edit → solve → inspect cycle
9. **Persistence workflow**: Save As / Save / Open / Unsaved dialog / Invalid file
10. **Camera interactions**: Orbit, pan, zoom, reset
11. **Selection interactions**: Click to select, visual highlighting
12. **Editing interactions**: DragValue precision, Apply/Reset
13. **Dialog modality**: Confirm dialog blocks underlying interactions

## 12. Final MVP Decision

### **READY WITH LIMITATIONS**

**Rationale:**

- 0 P0 defects
- 3 P1 defects found and fixed
- 6 P2 issues documented (non-blocking)
- 369 automated tests pass
- Process launches without panic
- Static audit confirms correct state transitions
- GUI runtime verification BLOCKED (no graphical environment)

**Limitations:**

1. GUI runtime acceptance is incomplete — visible-window and manual
   interaction verification could not be performed
2. 6 P2 cosmetic/UX issues remain
3. 4 P3 future enhancements not implemented
4. No native file dialog (text input only)
5. Camera resets on model edits (disorienting)

## 13. Recommended Next Phase

**Phase 152: GUI Runtime Verification**

Prerequisites: graphical environment with display

- Execute the 20-step engineering workflow (Section 4)
- Execute the persistence workflow (Section 5)
- Verify all interaction integrity items (Section 6)
- Confirm P1 fixes behave correctly in real interactions
- Classify any newly discovered runtime defects
- Update MVP decision to READY if all checks pass

Alternative if graphical environment remains unavailable:

**Phase 152: Headless GUI Test Framework**

- Introduce a test harness that can simulate egui interactions
- Add automated GUI interaction tests
- Verify state transitions through simulated events
