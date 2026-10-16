# Phase 159 — Zed-inspired redesign (final report: A · B · B2 · C · D · 方案 B)

**Scope (all landed):** replace the CAD ribbon chrome with a compact, Zed-like
shell — title bar, Project panel, three-mode right panel, centre workspace and a
centre-scoped Output dock — then finalise it with a shared typography scale,
consistent panel headers, hover/focus/selection states, a light-theme pass, and
the closing validation sweep.

> This report describes the app at `apps/viewport-poc` at the **end of Phase
> 159 plus the follow-up "方案 B" chrome revision**. Earlier sections are
> consolidated; §12 is the current, verbatim test output.

## 0. 方案 B chrome revision (post-159-D)

Three user-requested chrome changes, landed after 159-D:

1. **The title-bar command-palette trigger was removed.** The title bar no
   longer carries a search-field-looking button (nor its `Ctrl+Shift+P` hint).
   The palette itself is unchanged and stays reachable by its `Ctrl+Shift+P`
   shortcut (`main.rs`), which is listed in **Help → Shortcuts**, and from the
   **Help → Command Palette… (Ctrl+Shift+P)** menu entry.
2. **The menu-bar `◀ Inspector` toggle was removed.** The right panel's
   visibility remains controllable from the title bar's panel toggles, the
   **View → Right panel (Ctrl+I)** checkbox and the `Ctrl+I` shortcut.
3. **The centre workspace tab bar was removed; the centre is now module-driven
   (方案 B).** `WorkspaceTab { Model, Analysis, Results }` and its tab strip are
   gone. The **top module strip** (`Model | Section | Load | Analysis |
   Results`, `app.module`) is the single switcher and drives the centre content
   via `ui::workspace::CentreContent::for_module`:

   | active module | centre content |
   |---|---|
   | `Model` / `Section` / `Load` | the 3-D viewport |
   | `Analysis` | the analysis job panel (`inspector::show_analysis_section`) |
   | `Results` | the tabular results (nodal displacements / member end forces) |

   The centre-scoped bottom dock (`TopBottomPanel::bottom("workspace_output")
   .show_inside(ui, …)`) is **unchanged**: it stays nested inside the centre
   `CentralPanel`, so its width still equals the centre width. Switching
   modules is pure view state and preserves camera, display state, selection,
   model revision and analysis status (asserted by
   `ui::workspace::tests::switching_module_preserves_state`).

## 1. Git baseline

| | |
|---|---|
| HEAD | `d0ed2d9854089d1d1e4e740e7cbf04a73fafb9a6` (`feat(gui): wire Boundary and View ribbon placeholders`) |
| Branch | `main` |
| Working tree | carries **all of Phases 159-A → 159-D plus the 方案 B chrome revision** un-committed (project is under a no-commit instruction). New files: `src/commands.rs`, `src/ui/commands.rs`, `src/ui/palette.rs`, `src/ui/titlebar.rs`, `src/ui/workspace.rs` (now the module-driven centre composer). Deleted: `src/ui/ribbon.rs`. |

Nothing was committed. The window title, the `.saproject` **model** format
(v3) and `crates/**` are untouched; only the *settings* file grew additively
(§7).

## 2. Final layout

```
┌──────────────────────────────────────────────────────────────────────────┐
│ My Project ●                                         ▤ ▥ ▨               │  title bar (B1/D1)
├──────────────────────────────────────────────────────────────────────────┤
│ File Edit View Model Boundary Analysis Results Help                      │  menu bar
├──────────────────────────────────────────────────────────────────────────┤
│ Model │ Section │ Load │ Analysis │ Results                              │  module strip (single switcher)
├───────────────┬──────────────────────────────────────┬───────────────────┤
│ ▣ Project ◀   │            3-D viewport              │ ▤ Inspector ────  │
│  Project file │           (+ view cube)              │ ▥ Agent           │
│  Model        │       (or Analysis panel /           │ ▦ Outline   ▶Hide │
│   Nodes (n)   │        Results tables per module)    │                   │
│   Members (n) │                                      │  (mode body)      │
│   Supports(n) │                                      │                   │
│   Loads (n)   │                                      │                   │
├───────────────┴──────────────────────────────────────┴───────────────────┤
│ ⚠ Problems (n) │ ⏱ Solver Log │ ▤ History (n)                ▼ Hide       │  Output dock (centre-scoped)
├──────────────────────────────────────────────────────────────────────────┤
│ ▼ Output  Model: … ✗1 ⚠2  Module: …  Mode: …  Selection: …                │  status line (full width)
└──────────────────────────────────────────────────────────────────────────┘
```

* Title bar: project name + `●` dirty marker; three panel toggles (Project /
  right panel / Output dock). No palette trigger (see §0.1).
* Menu bar: `File Edit View Model Boundary Analysis Results Help`. No trailing
  Inspector toggle (see §0.2).
* Module strip (the single switcher): `Model | Section | Load | Analysis |
  Results`. It drives both the inspector sections and the centre content.
* Left **Project** panel: header `▣ Project` + `◀ Hide`; `Project file`
  section; `Model Explorer` → `Nodes / Members / Supports / Loads`.
* Centre workspace: the active module's content (viewport / Analysis panel /
  Results tables) plus the centre-scoped Output dock — no tab bar.
* Right panel switcher: `Inspector | Agent | Outline`; body per mode.
* Output dock tabs: `Problems (n) | Solver Log | History (n)`.
* Status line: model + counts, `✗ n` / `⚠ n` badges, module, interaction mode,
  selection summary.

## 3. Architecture / file map

```
Status Line                        root TopBottomPanel::bottom("status_line")   — full width
Title bar / menu bar / module strip root TopBottomPanel::top(…) rows (added first)
Main workspace (horizontal):
  Left Project Panel               root SidePanel::left("model_tree")           — full height
  Centre Workspace                 root CentralPanel:
      Flexible content             the active module's body (viewport / analysis / results)
      Bottom Output Dock           nested TopBottomPanel::bottom::show_inside, resizable
  Right Auxiliary Panel            root SidePanel::right("inspector")           — full height
```

`main.rs::update` adds the root panels in the order **title bar → menu bar →
module strip → status line → left panel → right panel → centre workspace**, so
the status bar spans the full window and the dock (drawn with nested
`show_inside` inside the centre `CentralPanel`) is exactly as wide as the
centre region — **geometry preserved from 159-B2/C**: left/right panels full
height, dock width == centre width, dock ≤ 70 % of the centre height.

`ui/viewport.rs` is **untouched** (its `show_viewport(app, ui)` signature and
the `show_inside` nesting are as landed in 159-B2/C; the 方案 B revision did not
edit it, so its module doc still mentions the old tab bar — a doc-only nit).

## 4. Visual language (D1)

All typography + chrome metrics are centralised in `ui/theme.rs` (the previous
ad-hoc `RichText::new(..).size(..)` / `FontId::proportional(..)` sprinkles are
gone):

| token | value | use |
|---|---|---|
| `TEXT_HEADER` | 13.0 | title bar, panel headers, tab labels, section headings |
| `TEXT_BODY` | 12.5 | body / control text, status line |
| `TEXT_SMALL` | 11.0 | hints, metadata, badges, secondary notes |
| `TEXT_MONO` | 12.0 | numeric readouts (results grid) |
| `ROW_HEIGHT` | 24.0 | every compact chrome row (headers, module strip, toggles) |
| `ICON_SIZE` | 14.0 | inline chrome glyphs |

Helpers: `theme::{header, body, small, mono}` (RichText) and
`theme::body_font` (FontId); `theme::accent(dark)` /
`theme::on_accent(dark)` / `theme::scrim(dark)`; and the shared
`theme::panel_header(ui, icon, label, trailing_closure)`.

* **Typography scale** applied to: title bar (`titlebar.rs`), module strip
  (`module.rs`), results header (`workspace.rs`), panel headers + dock tabs
  (`tree.rs`, `inspector.rs`, `status.rs`), status bar (`status.rs`), palette
  rows (`palette.rs`).
* **Consistent panel headers**: the left Project header uses
  `theme::panel_header` (`tree.rs`); the right-panel switcher
  (`inspector.rs`) and the dock tab strip (`status.rs`) all set
  `set_min_height(theme::ROW_HEIGHT)` and share the same header font, glyph
  size and `separator()` — one header pattern, one height.
* **Hover / focus / selection**: `theme::apply_theme` now installs a single
  restrained accent (`theme::accent`) as `Visuals::selection.bg_fill`, a
  translucent tint of the same accent for `widgets.hovered` / `active`, the
  accent as `hyperlink_color` and a faded non-interactive stroke for quieter
  separators. Every interactive surface reads from `Visuals` (via
  `icons::icon_toggle` / `icons::icon_text_button` and egui widgets), so hover
  vs. selected vs. resting read clearly in **both** themes. The primary Run
  button uses `theme::accent`/`theme::on_accent` as a paired fill/foreground
  (`inspector.rs:362-368`).
* **Restraint**: no oversized buttons (tabs are `ROW_HEIGHT` = 24 pt), minimal
  borders, subtle separators, and the viewport keeps the visual priority.

## 5. Light-theme pass (D1)

Every chrome element added in 159-A/B/B2/C was audited for hardcoded dark
colours. Result:

* **Fixed**: the palette backdrop used a fixed `from_black_alpha(110)`; it is
  now `theme::scrim(app.dark_mode)` (heavier in dark, lighter in light)
  (`palette.rs:149`). `icons::icon_text_button` measured its galley with a
  fixed `Color32::WHITE`; now `ui.visuals().text_color()` (`icons.rs:559`).
* **Verified already theme-correct**: the title bar, module strip, dock tabs,
  right-mode switcher and status bar all drew from `Visuals` / `TextPalette`;
  the Agent and Outline panels use `TextPalette`. No remaining hardcoded dark
  colour in any owned chrome file (grep-clean).
* **Agent panel** is now a properly styled empty state: `theme::header`
  heading, a `● Not connected` line from `TextPalette`, themed hint text,
  disabled input / preview fields and a disabled Run/Cancel pair, with a note
  pointing at the real command layer.

## 6. Command layer & palette (159-A, unchanged)

The typed command layer is the single execution path used by the menus, the
palette and the viewport context menu:

* `src/commands.rs` (lib) — the fuzzy/substring matcher (`best_score`).
* `src/ui/commands.rs` — `Command`, the 64-entry `Command::ALL` registry,
  `id()` / `label()` / `category()` / `shortcut()`, `search()`, the enablement
  predicate (`command_is_enabled` / `command_disabled_reason`) and the **one
  dispatcher** `ViewportApp::execute_command`.
* `src/ui/palette.rs` — the `Ctrl+Shift+P` overlay (ranked list, keyboard +
  mouse, disabled rows listed but inert).

There is no second model-editing implementation: every command ends in the
same `EditModel` / `Project` / `EditHistory` lifecycle as the menus.

## 7. Module-driven centre, modes & persistence (D2 + 方案 B)

* **No `WorkspaceTab`.** The centre is driven by the active work module via
  `ui::workspace::CentreContent::for_module`: `Model` / `Section` / `Load` →
  the 3-D viewport, `Analysis` → the analysis job panel (the same body as the
  inspector's Analysis module, mirrored not duplicated), `Results` → the
  tabular results. Switching a module touches nothing but `app.module`.
* `RightPanelMode { Inspector, Agent, Outline }` — pure view state.
* **Persisted** (159-D): `LayoutPrefs` gained the additive,
  `#[serde(default)]` field `right_panel_mode: Option<u8>` (index into the
  `ALL` array; `Option<u8>` keeps `LayoutPrefs: Copy`). `main.rs` writes it
  onto the captured preset each frame and restores it at launch via
  `RightPanelMode::from_index`, which falls back to the default for an unset
  or out-of-range value. The former `workspace_tab: Option<u8>` field was
  **removed** in the 方案 B revision; because `LayoutPrefs` does not use
  `deny_unknown_fields`, a legacy settings file that still carries the
  `workspace_tab` key loads cleanly with the key ignored (proved by
  `persistence::tests::legacy_layout_with_retired_workspace_tab_key_still_loads`).
* Panel visibility (`tree_collapsed` / `inspector_collapsed` /
  `dock_collapsed`) and widths/heights continue to persist as before.
* The work module itself is deliberately **not** persisted (it is the single
  live switcher); the centre reopens on the default `Model` module.

## 8. AI readiness

The Agent mode is a deliberate, honest placeholder — no networking, no model,
no fake behaviour. The integration path is already prepared: because
*everything* runs through `Command` + `execute_command`, a future assistant
needs only to emit `Command` values (or command ids) and dispatch them; it
never edits `EditModel` directly. The Agent panel documents this in-app.

## 9. Validation sweep (D3)

Required list → result → the focused test(s) that prove it. Tests marked *(new)*
were added this phase; everything else already existed.

| # | Item | Result | Test(s) |
|---|---|---|---|
| 1 | Module switching (centre content) + dock tabs | **PASS** | `ui::workspace::tests::{centre_content_follows_the_active_module, switching_module_preserves_state, all_modules_draw_headlessly_without_mutation}` *(new)*; `ui::status::tests::{tab_labels_are_distinct_and_complete, messages_partition_into_problems_and_log, switching_dock_tab_does_not_mutate_state}` |
| 2 | Panel visibility (left/right/bottom toggles; pure view state) | **PASS** | `ui::commands::tests::{dispatch_panel_toggles_are_pure_view_state, dispatch_view_mode_module_theme_and_panels}` |
| 3 | Selection persistence (across module switches, panel toggles, mode switches) | **PASS** | `ui::workspace::tests::switching_module_preserves_state` *(new)*; `ui::commands::tests::dispatch_panel_toggles_are_pure_view_state`; `ui::inspector::tests::switching_mode_preserves_selection_and_model` |
| 4 | Command Palette dispatch (enabled executes, disabled refused, one dispatcher) | **PASS** | `ui::commands::tests::{palette_flow_executes_the_highlighted_enabled_command, palette_flow_refuses_the_highlighted_disabled_command}`; `ui::palette::tests::{palette_typing_narrows_results_and_enter_executes, palette_lists_disabled_commands_but_refuses_to_run_them, palette_escape_closes_without_running}` |
| 5 | Model revision consistency (no mutation from layout / module ops) | **PASS** | `ui::workspace::tests::switching_module_preserves_state` (asserts camera, display, `fingerprint()` + `revision()`, analysis status); `ui::status::tests::switching_dock_tab_does_not_mutate_state`; `ui::commands::tests::dispatch_panel_toggles_are_pure_view_state` |
| 6 | Undo / redo (labels + semantics) | **PASS** | `edit_history::tests::{labels_are_listed_newest_first, labels_ride_along_through_undo_and_redo, legacy_record_yields_an_empty_label, labels_are_dropped_with_the_capacity}` (lib); `ui::commands::tests::dispatch_undo_redo_through_commands`; `ui::actions::tests::editing_workflow_add_member_apply_section_is_undoable` *(new)* |
| 7 | Result stale tracking (edit after solve → Stale; hidden while Stale) | **PASS** | `ui::commands::tests::dispatch_edit_marks_a_solved_result_stale`; `app_state::tests::analysis_result_policy_a_hides_non_current` (lib) |
| 8 | Inspector / Agent switching | **PASS** | `ui::inspector::tests::{mode_labels_default_and_icons, switching_mode_preserves_selection_and_model, all_modes_draw_headlessly, mode_index_round_trips_and_tolerates_bad_values}` *(last one new)* |
| 9 | Existing editing workflows (add node/member, supports, loads, section apply, delete) | **PASS** | `ui::actions::tests::editing_workflow_add_member_apply_section_is_undoable` *(new)*; `ui::commands::tests::dispatch_delete_selection_is_one_undo_step`; `ui::actions::tests::support_quick_*` / `nodal_load_quick_*`; integration `phase152/156/157/158` (§12) |
| 10 | Visual language / theme (D1) | **PASS (tests)** | `ui::theme::tests::{typography_scale_is_ordered_and_centralised, accent_is_theme_specific_with_legible_foreground, apply_theme_installs_accent_hover_and_selection_in_both_themes, panel_header_paints_headlessly}` *(new)* |
| 11 | Mode persistence + legacy-settings compatibility (D2 + 方案 B) | **PASS (tests)** | `persistence::tests::{layout_mode_round_trips_and_tolerates_out_of_range, legacy_layout_without_mode_loads, legacy_layout_with_retired_workspace_tab_key_still_loads}` *(new/updated)*; `ui::inspector::tests::mode_index_round_trips_and_tolerates_bad_values` |

**Deliberately not verified by hand** (see §13): every visual / geometry /
resize-by-drag acceptance item. No click-testing is possible for this agent, so
those remain **NOT VERIFIED interactively**; they are covered only as far as
the headless draw + state tests above reach.

### New tests added this phase (159-D)

* `ui::theme::tests` ×4 — scale ordering, accent contrast, `apply_theme` states
  in both themes, `panel_header` headless draw.
* `ui::inspector::tests::mode_index_round_trips_and_tolerates_bad_values`.
* `ui::actions::tests::editing_workflow_add_member_apply_section_is_undoable`.
* `persistence::tests::layout_tab_and_mode_round_trip_and_tolerate_out_of_range`
  and `legacy_layout_without_tab_or_mode_loads` — superseded in 方案 B by
  `layout_mode_round_trips_and_tolerates_out_of_range`,
  `legacy_layout_without_mode_loads` and
  `legacy_layout_with_retired_workspace_tab_key_still_loads`.

### 方案 B test changes

* **Removed** the four `WorkspaceTab` tests
  (`tab_labels_are_distinct_and_complete`,
  `tab_index_round_trips_and_tolerates_bad_values`,
  `switching_workspace_tab_preserves_state`,
  `all_tabs_draw_headlessly_without_mutation`) together with the enum.
* **Added** `ui::workspace::tests::{centre_content_follows_the_active_module,
  switching_module_preserves_state, all_modules_draw_headlessly_without_mutation}`.
* **Replaced** the two `LayoutPrefs` tab/mode tests with `layout_mode_…` /
  `legacy_layout_without_mode_loads` and **added**
  `legacy_layout_with_retired_workspace_tab_key_still_loads` (proves the
  retired `workspace_tab` key is ignored, not rejected).

No existing test was duplicated. **Warning fix (D2):** the previously
reported `unused return value of egui::Context::run` build warning was already
resolved — every `ctx.run(...)` in the crate now binds the result with
`let _ = …` (verified: a `cargo test --no-run` of the whole crate reports no
such warning).

## 10. Test results (verbatim)

Forced fresh check (exe killed, `cargo clean -p viewport-poc`, then
`cargo check -p viewport-poc`) after the 方案 B revision:

```
    Checking viewport-poc v0.1.0 (D:\code\section-properties\apps\viewport-poc)
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 1.68s
```

`warning:` count = **0**, `error` count = **0**.

Required suites:

```
cargo test -p viewport-poc --bin viewport-poc     118 passed; 0 failed
cargo test -p viewport-poc --lib -- commands        8 passed; 0 failed
cargo test -p viewport-poc --lib -- persistence    40 passed; 0 failed
cargo test -p viewport-poc --lib -- edit_history   12 passed; 0 failed
```

(The bin count fell from 119 to 118: the four `WorkspaceTab` tests were
removed and three module-driven tests added; persistence rose 39 → 40 with the
`workspace_tab` → `right_panel_mode` + retired-key test swap.)

Regression / integration targets (editing workflows), re-run after the revision:

```
phase152_undo_redo            47 passed; 0 failed
phase156_support_editing      48 passed; 0 failed
phase157_analysis_workflow    43 passed; 0 failed
phase158_e2e_state            80 passed; 0 failed
persistence_integration       31 passed; 0 failed
```

The full unfiltered **lib** suite was intentionally not run (pre-existing
pathologically slow targets). Pre-existing warnings in unrelated integration
files (`perf149`, `phase152`, `phase154`) are untouched.

`cargo build`:

```
   Compiling viewport-poc v0.1.0 (D:\code\section-properties\apps\viewport-poc)
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 3.34s
```

## 11. Runtime relaunch

Killed the previous instance, rebuilt, relaunched `target/debug/viewport-poc.exe`
with **no stdout/stderr redirection**, waited ~8 s:

| Field | Value |
|---|---|
| PID | **13140** |
| Process | viewport-poc |
| Responding | **True** |
| MainWindowTitle | `Section Properties & Frame Analysis` |

Window title unchanged.

## 12. Honest runtime validation status

**The UI was NOT click-tested interactively.** The process is alive and
responding, and every draw path (title bar, menu, module strip, viewport,
Analysis panel, Results tables, Output dock, right-panel modes, palette,
dialogs) is exercised by the headless egui tests in §10 — but no human clicked
the running GUI, so:

* the *visual* result of the typography scale, the accent hover/selection
  states and the light-theme pass is **unverified by eye**;
* resize-by-drag behaviour (side panels, Output dock) and window-resize reflow
  are **unverified by hand** (the geometry is test-covered only where a test
  asserts a stored size / clamp);
* whether the machine actually has a GPU renderer (glow) at runtime is
  environment-dependent and was not observed this phase.

Everything claimed as **PASS** above is backed by a named automated test; the
geometry items that cannot be reduced to a headless assertion are marked
**NOT VERIFIED** and were not confirmed by hand.

## 13. Known limitations

* **Headless-only verification** (see §12) — no interactive click-through.
* **Agent** is a non-functional placeholder by design (no LLM / HTTP / thread).
* There is exactly **one** switcher (the module strip); the former centre tab
  row is gone, so no more visually adjacent duplicate tab rows.
* `ui/icons.rs` keeps its full glyph library (some glyphs unused pending future
  toolbars); the module opts out of `dead_code`. The `search` glyph is now only
  referenced from the icon test (the title-bar trigger that drew it was
  removed).
* `ui/viewport.rs`'s module doc still mentions "the workspace tab bar" (the
  file is off-limits for this revision and was not edited).
* Tab **size** for the right-panel switcher is text-driven, so a very narrow
  panel will wrap/overflow before the `Min` width is reached (unchanged).
* Pre-existing `cargo test` warnings in unrelated integration files
  (`perf149`, `phase152`, `phase154`) are untouched.

## 14. Recommended Phase 160

1. **Keyboard parity**: bind `Ctrl+B` (Project panel) and expose the right-panel
   modes as palette commands (module switching already has `SwitchModule`
   commands).
2. **Runtime smoke harness**: a scripted egui interaction test (synthetic
   pointer events) to move the "NOT VERIFIED" geometry items into automated
   coverage.
3. **Agent (future)**: implement a real dispatcher on top of `Command` and
   replace the placeholder body, keeping the same command path.
4. **Inspector/Outline polish**: continue the D1 pass into the Inspector body
   (grid spacing, numeric alignment) once the runtime look can be reviewed.
