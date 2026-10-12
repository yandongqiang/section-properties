# Phase 152 Report: Bounded Undo/Redo for GUI Model Editing

**Baseline commit:** `dc747a0` (Phase 151)  
**Status:** COMPLETE  
**GUI runtime verification:** BLOCKED (no graphical environment)

## Objective

Implement bounded snapshot-based Undo/Redo for engineering model edits in
the `viewport-poc` egui desktop application.  The system must support
Edit → Undo → Redo → Save → Edit → Undo without corrupting the model,
stable IDs, analysis-result validity, dirty-state, or file persistence.

## Architecture

### EditHistory (new module: `edit_history.rs`)

```rust
pub struct EditHistory {
    undo: Vec<EditModel>,
    redo: Vec<EditModel>,
    capacity: usize,  // default 50
}
```

- **Ownership:** `ViewportApp` (GUI controller), not `Project`.  Rationale:
  `Project` is a pure data type; edit history is a controller concern.
- **Snapshot-based:** each committed edit stores a clone of the `EditModel`
  *before* the edit.  `EditModel` already derives `Clone`.
- **Bounded:** default capacity 50; oldest entry discarded when full.

### Edit Transaction (candidate-model approach)

All 10 GUI edit entry points now follow the same pattern:

1. Clone the current model (`let mut candidate = model.clone()`)
2. Apply the mutation to the candidate
3. If effective (revision changed), call `commit_edit(candidate)`
4. `commit_edit` records the old model in undo, replaces the current
   model, clears redo, invalidates analysis, rebuilds the scene

No-op edits (where the mutation has no effect) do not create history
entries — detected by comparing `candidate.revision() != model.revision()`.

### Revision Invariant

`EditModel::ensure_revision_at_least(min_rev)` — when restoring a model
via Undo/Redo, the restored model's revision is set to at least
`current_rev + 1`.  This ensures:

- The revision counter is monotonic across the entire edit session
- A restored model's revision never matches a prior `solved_revision`
- Stale analysis results are never mistaken for current
- The user must explicitly re-solve after Undo/Redo

### Dirty-State (fingerprint-based)

`EditModel::fingerprint() -> u64` — hashes all engineering data (nodes,
members, supports, loads, ID allocation state), excluding the revision
counter.  `saved_fingerprint: Option<u64>` replaces the prior
`saved_revision: Option<u64>`.

This correctly handles the spec scenarios:
- Save A → Edit B → Undo A → dirty=false (fingerprint matches saved)
- Save A → Edit B → Save B → Undo A → dirty=true (fingerprint differs)

### Project::replace_model

New method that replaces the model while preserving the project name and
invalidating analysis.  Does NOT reset the model's revision (caller is
responsible for the revision invariant).

## GUI Integration

### Toolbar buttons

- **Undo** button (disabled when `!can_undo()`)
- **Redo** button (disabled when `!can_redo()`)

### Keyboard shortcuts

- **Ctrl+Z** — Undo
- **Ctrl+Y** — Redo
- **Ctrl+Shift+Z** — Redo

Shortcuts are suppressed when any widget has focus
(`ctx.memory(|m| m.focused().is_none())`) to avoid interfering with
DragValue / TextEdit keyboard handling.

## Edit Entry Points Routed Through `commit_edit`

All 10 model mutation entry points in `main.rs` were converted from
direct `self.project.model.as_mut().<method>()` to the candidate-model
approach:

1. `add_node_from_input()` — `add_node`
2. `add_member_from_input()` — `add_member`
3. Node coordinate DragValue — `update_node`
4. Node delete button — `delete_node`
5. Support set — `set_support`
6. Support clear — `clear_support`
7. Nodal load set — `set_nodal_load`
8. Nodal load clear — `clear_nodal_load`
9. Member delete button — `delete_member`
10. `apply_member_edit()` — `set_member_section` + `set_member_ref_vec`
    (grouped as one commit)

## History Boundaries

| Action | History effect |
|--------|---------------|
| New | Clear |
| Open | Clear |
| Preset | Clear |
| Clear project | Clear |
| Failed Open | Preserve |
| Failed Save | Preserve |
| Save / Save As | Preserve + update savepoint |
| Solve | Preserve |
| Selection | No entry |
| Camera | No entry |
| Result mode | No entry |

## Files Modified

| File | Change |
|------|--------|
| `src/edit_history.rs` | **New** — `EditHistory` struct + 8 unit tests |
| `src/edit_model.rs` | Added `ensure_revision_at_least()`, `fingerprint()` |
| `src/app_state.rs` | Added `Project::replace_model()` |
| `src/lib.rs` | Added `edit_history` module + export |
| `src/main.rs` | Integrated undo/redo: `commit_edit`, `do_undo`, `do_redo`, fingerprint-based dirty state, routed all 10 edit sites, toolbar buttons, keyboard shortcuts |
| `tests/phase152_undo_redo.rs` | **New** — 47 regression tests + 3 integration scenarios |

## Test Results

| Category | Count | Status |
|----------|-------|--------|
| Library tests (incl. 8 new `edit_history`) | 194 | PASS |
| Phase 152 regression tests | 47 | PASS |
| All other integration tests | 183 | PASS |
| **Total** | **424** | **PASS** |

### New test breakdown (55 total)

- `edit_history.rs` unit tests: 8
- Fingerprint tests: 8
- Revision invariant tests: 3
- Project::replace_model tests: 2
- Undo/redo flow tests: 5
- Dirty-state semantics tests: 7
- History boundary tests: 6
- No-op edit tests: 2
- Bounded capacity test: 1
- Multiple edit chain test: 1
- Node/member preservation tests: 4
- Analysis invalidation tests: 2
- Integration scenarios A/B/C: 3
- Misc (undo_when_empty, redo_when_nothing, etc.): 2

### Integration scenarios

**Scenario A:** Save A → Edit B → Undo A → Redo B → Solve — verifies dirty state, fingerprint, and analysis correctness through the full cycle.

**Scenario B:** Solve A → Edit B → Undo A — verifies the restored model's revision is greater than the solved revision, so historical results are not exposed as Current.

**Scenario C:** Edit A → Edit B → Undo → Edit C — verifies Redo is cleared after a new edit following an undo.

## Validation

| Check | Result |
|-------|--------|
| `cargo fmt --check` | PASS |
| `cargo check` | PASS |
| `cargo test` (424 tests) | PASS |
| `cargo clippy` | 32 warnings (all pre-existing in `section-properties/`, `model.rs`, `selection.rs`, `main.rs`; **0 in new files**) |
| GUI runtime | BLOCKED (no graphical environment) |

## Scope Compliance

- No command-pattern hierarchy ✓
- No persistent undo history ✓
- No autosave ✓
- No recovery journal ✓
- No multi-project ✓
- No new element types ✓
- No distributed loads / load combos ✓
- No material/section DB ✓
- No Tauri changes ✓
- No broad refactoring ✓
- Changes confined to `viewport-poc` ✓

## Known Limitations

1. **DragValue grouping:** each DragValue change creates a separate
   history entry.  The spec recommends grouping logically related
   changes; this is a future enhancement (P2 from Phase 151).
2. **GUI runtime not verified:** no graphical environment available;
   process launch and static analysis confirmed.
3. **Pre-existing clippy warnings:** 32 warnings in pre-existing code,
   none introduced by this phase.
