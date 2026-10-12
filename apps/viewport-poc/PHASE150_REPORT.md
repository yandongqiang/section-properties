# Phase 150 — Versioned Project Persistence

## Status

**COMPLETE** — all deliverables produced, all tests pass, committed.

## Mission

Implement versioned project persistence for the egui GUI MVP: save/load
`.saproject` JSON files with stable IDs, full validation, and transaction
safety.  The user can create a model, save it, close the application,
reopen it, load the file, and solve — obtaining identical results.

## Scope

- **In scope**: `apps/viewport-poc` only.
- **Out of scope**: `section-properties` (frozen, published), `structural-analysis`
  (frozen, `publish = false`), `apps/tauri-viewport-poc` (retained for
  comparison, not modified).

## Baseline

- HEAD: `b8ad5c0` (Phase 149 audit, READY WITH LIMITATIONS).
- 309 tests passing at baseline.

## Architecture

### Dedicated versioned DTO

A dedicated set of DTO types (`ProjectFileV1`, `ModelFileV1`, `NodeFileV1`,
`MemberFileV1`, `SupportFileV1`, `NodalLoadFileV1`) is used for
serialization.  The `EditModel` and its components are **not** directly
serialized — this decouples the file format from internal representation
and allows future format migration.

### File format

- JSON, pretty-printed, with explicit `format_version: 1`.
- Extension: `.saproject`.
- Only version 1 is supported; other versions are rejected with
  `PersistenceError::UnsupportedVersion`.

### Serialization API

| Function | Purpose |
|---|---|
| `serialize_project(&EditModel) -> Result<String>` | Model → JSON string |
| `deserialize_project(&str) -> Result<EditModel>` | JSON string → Model (with validation) |
| `save_project(&Path, &EditModel) -> Result<()>` | Write to temp file, then rename |
| `load_project(&Path) -> Result<EditModel>` | Read file, then deserialize |

### Validation (14 checks)

1. Format version == 1
2. Unique node IDs
3. Unique member IDs
4. Valid member endpoint references
5. No self-connected members
6. No zero-length members
7. Finite node coordinates
8. Valid section properties (finite, positive)
9. Valid reference vectors (non-zero, not parallel to member axis)
10. Valid support node references
11. Finite support DOF values
12. No duplicate supports
13. Finite nodal loads, no duplicate loads
14. ID counters > all existing IDs

### Stable ID preservation

Node and member IDs are preserved exactly across save/load cycles.
`next_node_id` and `next_member_id` counters are serialized and restored,
so new entities created after loading do not collide with existing IDs.

### Transaction safety

- **Save**: serialize → write temp file → rename.  On rename failure, the
  temp file is cleaned up.  The original file is not deleted before the
  replacement is ready.  On Windows, rename-over-existing is not
  guaranteed atomic, but is the best available strategy without
  filesystem locks.
- **Load**: read → parse → validate version → validate data → construct
  candidate `EditModel`.  The caller is responsible for transactional
  replacement of the current model.  On failure, the current model, path,
  and dirty state are preserved.

### GUI integration

- **File menu** (toolbar): New, Open..., Save, Save As..., Clear project.
- **File section** (left panel): path text input + Open/Save/Save As
  buttons + dirty/saved status indicator.
- **Dirty state**: tracked via `saved_revision: Option<u64>`, separate
  from analysis staleness.  A model is dirty if it exists and its
  revision differs from the last saved revision (or has never been saved).
- **Unsaved changes dialog**: floating `egui::Window` shown before
  destructive operations (New, Open) when the model is dirty.  Options:
  Save, Don't Save, Cancel.
- **Failed Open**: current model, path, and dirty state are preserved.
- **Failed Save**: `saved_revision` is not updated; existing file on disk
  is not modified.

## Changes to existing code

### `apps/viewport-poc/src/edit_model.rs`

- Added `next_node_id() -> u32` and `next_member_id() -> u32` accessors
  (needed for serialization).
- Added `from_parts(nodes, members, supports, nodal_loads, next_node_id,
  next_member_id) -> Self` constructor (used by `dto_to_model` to
  construct from pre-validated data; `revision` starts at 0).

### `apps/viewport-poc/src/main.rs`

- Added fields to `ViewportApp`: `project_path`, `saved_revision`,
  `file_path_input`, `show_unsaved_dialog`, `pending_action`.
- Added `PendingAction` enum.
- Added methods: `is_dirty()`, `do_new()`, `do_open()`, `do_save()`,
  `do_save_as()`, `save_to_path()`, `execute_pending()`.
- Updated `load_preset()` to reset `project_path` and `saved_revision`.
- Updated `clear_project()` to reset `project_path` and `saved_revision`.
- Expanded File menu with Open/Save/Save As.
- Added file path input section to left panel.
- Added unsaved-changes dialog.
- Updated window title to "Phase 150".

### `apps/viewport-poc/Cargo.toml`

- Added `serde` and `serde_json` dependencies.

### `apps/viewport-poc/src/lib.rs`

- Added `pub mod persistence;`.

## New files

| File | Purpose |
|---|---|
| `src/persistence.rs` | Persistence module: DTOs, serialization, validation, file I/O, 15 unit tests |
| `tests/persistence_integration.rs` | 31 integration tests: preset round-trips, numerical round-trips, transaction safety, golden file, dirty-state, additional validation |
| `tests/golden_v1.saproject` | Hand-written V1 golden fixture: 2-node cantilever with fixed support and tip load |

## Test summary

| Category | Count | Status |
|---|---|---|
| Persistence unit tests (`persistence.rs`) | 15 | PASS |
| Persistence integration tests (`persistence_integration.rs`) | 31 | PASS |
| Phase 149 baseline tests | 309 | PASS |
| **Total** | **355** | **ALL PASS** |

### Integration test breakdown

- Preset round-trip: 5 (Portal, Inclined, Grid1x1, Grid3x3, Grid10x10)
- Numerical round-trip: 2 (Portal, Inclined — displacements + reactions match)
- Transaction safety: 5 (failed load preserves model, failed save preserves file, idempotent save→load→save, nonexistent file, invalid path)
- Golden file: 3 (loads successfully, solves with expected cantilever tip displacement, round-trips)
- Dirty-state: 5 (new=dirty, saved=clean, edited=dirty, loaded=clean, empty=clean)
- Additional validation: 11 (self-connected, zero-length, non-finite coords, duplicate member ID, duplicate load, non-finite load, non-finite section, parallel ref vec, zero ref vec, negative section, next_id below existing)

### Golden file verification

The golden fixture is a 2-node cantilever beam (3m, fixed at node 0,
−1000N vertical load at node 1, steel section).  The tip displacement
matches the analytical cantilever formula:

$$\delta = \frac{PL^3}{3EI} = \frac{1000 \times 3^3}{3 \times 200\times10^9 \times 2\times10^{-6}} \approx -11.25 \text{ mm}$$

## GUI manual smoke test

**BLOCKED** — no graphical environment available in the CLI test
context.  The GUI code compiles and all non-GUI tests pass.  Manual
verification requires a desktop environment with a display server.

## Validation

| Check | Result |
|---|---|
| `cargo fmt -p viewport-poc --check` | PASS |
| `cargo check -p viewport-poc` | PASS |
| `cargo clippy -p viewport-poc --lib` (new code) | No new warnings |
| `cargo clippy -p viewport-poc --test persistence_integration` | No new warnings |
| `cargo test -p viewport-poc --release` | 355 passed, 0 failed |
| `cargo check --workspace` | PASS |

## Scope restrictions (not implemented)

- Undo/redo
- Autosave / crash recovery
- Recent-projects list
- Cloud storage / multi-project workspace
- Material database / section database
- Distributed loads / load combinations
- Analysis result serialization (results are recomputed on load)
- New solvers or solver selection
- Tauri integration
- Broad refactoring of existing modules

## Deliverables

- `apps/viewport-poc/src/persistence.rs` — persistence module + 15 unit tests
- `apps/viewport-poc/tests/persistence_integration.rs` — 31 integration tests
- `apps/viewport-poc/tests/golden_v1.saproject` — golden fixture
- `apps/viewport-poc/PHASE150_REPORT.md` — this report
- Git commit (not pushed)
