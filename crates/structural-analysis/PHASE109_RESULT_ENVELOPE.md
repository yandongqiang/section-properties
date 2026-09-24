# Phase 109 — Reaction / Displacement Envelope

## Summary

Extended the existing `Envelope` type (Phase 103) to track **per-node
displacement** and **per-node support reaction** envelopes across multiple
load cases and load combinations. Each extreme value records the governing
load source, enabling engineers to identify which load case controls each
DOF.

## Motivation

Phase 103 introduced member-force envelopes (`N`, `V`, `M`), but displacement
and reaction envelopes were absent. In practice, engineers need:

- **Displacement envelopes**: maximum deflection across all load cases for
  serviceability limit-state checks.
- **Reaction envelopes**: maximum support reactions for footing and connection
  design.
- **Governing load source**: which load case or combination produces the
  extreme value — critical for load-case attribution in design reports.

## Design

### New types

| Type | Purpose |
|------|---------|
| `Extremum` | Min/max of a scalar quantity with governing `load_source`. |
| `NodeEnvelopeSample` | Per-node envelope for `ux`, `uy`, `rz`, each an `Extremum`. |

### `Envelope` extensions

The `Envelope` struct gains three fields:

```rust
pub struct Envelope {
    // existing fields ...
    pub node_displacements: Vec<NodeEnvelopeSample>,
    pub support_reactions:  Vec<NodeEnvelopeSample>,
    pub n_nodes: usize,
}
```

### `from_frame_results` changes

- **New validation**: all results must agree on `n_nodes` (in addition to
  the existing `n_members` check).
- **New computation**: after building member-force samples, calls
  `compute_node_envelope` twice — once for displacements, once for reactions.
- `compute_node_envelope` iterates over all nodes and all results, calling
  `Extremum::update` with the DOF value and `load_source`.

### Governing load source

`Extremum::update(value, source)` updates `min` / `max` and stores the
`source` string (e.g. `"case:dead"`, `"combination:1.2D+1.6L"`) when a new
extreme is found. The source is `Option<String>` — `None` if the result had
no `load_source`.

### Accessors

- `Envelope::node_displacement(node_index) -> Option<&NodeEnvelopeSample>`
- `Envelope::support_reaction(node_index) -> Option<&NodeEnvelopeSample>`

## API additions

```rust
// New public types
pub struct Extremum { pub min: f64, pub min_source: Option<String>,
                     pub max: f64, pub max_source: Option<String> }
pub struct NodeEnvelopeSample { pub node_index: usize,
                                pub ux: Extremum, pub uy: Extremum, pub rz: Extremum }

// New Envelope fields
pub node_displacements: Vec<NodeEnvelopeSample>,
pub support_reactions:  Vec<NodeEnvelopeSample>,
pub n_nodes: usize,

// New Envelope methods
pub fn node_displacement(&self, node_index: usize) -> Option<&NodeEnvelopeSample>
pub fn support_reaction(&self, node_index: usize) -> Option<&NodeEnvelopeSample>
```

## Files changed

| File | Change |
|------|--------|
| `src/postprocessing.rs` | `Extremum`, `NodeEnvelopeSample`, `compute_node_envelope`, extended `Envelope` |
| `src/lib.rs` | Re-export `Extremum`, `NodeEnvelopeSample` |
| `tests/result_envelope.rs` | 10 new tests (new file) |

## Tests

10 tests in `tests/result_envelope.rs`:

| Test | Verifies |
|------|----------|
| `a_node_displacement_envelope` | Cantilever under two load cases: min/max `uy` at free end |
| `b_reaction_envelope` | Cantilever support reactions: min/max `Ry` and `Mz` |
| `c_governing_load_source` | `min_source` / `max_source` correctly attributed |
| `d_three_load_cases` | Middle load case produces the extreme value |
| `e_empty_input_returns_error` | Empty results slice → `InvalidInput` |
| `f_incompatible_node_count_rejected` | Mismatched `n_nodes` → `InvalidInput` |
| `g_member_force_envelope_regression` | Existing N/V/M envelope behavior unchanged |
| `combination_provenance_in_envelope` | `LoadCombination` source string preserved |
| `envelope_node_count_matches_model` | `n_nodes` matches model node count |
| `single_result_envelope_min_equals_max` | Single result → `min == max` for all DOFs |

## Verification

```
cargo test -p structural-analysis --test result_envelope --release  → 10 passed
cargo test -p structural-analysis --release                         → all passed (0 failed)
cargo check --workspace --all-targets                               → OK (no new warnings)
cargo doc -p structural-analysis --no-deps                          → OK
```

## Non-goals

- No absolute-value envelope mode (min/max are signed, as in Phase 103).
- No tracking of governing load case *index* — only the `load_source` string.
- No envelope for member-level displacements (only node-level).
- No changes to `section-properties` crate.
