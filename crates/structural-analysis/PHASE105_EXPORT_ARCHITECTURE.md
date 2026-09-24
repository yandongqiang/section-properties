# Phase 105 — Export / Serialization Architecture Audit

**Date**: 2026-09-24  
**Baseline**: commit `1306a4f` (Phase 104)  
**Status**: Complete  
**Production changes**: None  
**New features**: None  

---

## 1. Baseline

```text
HEAD: 1306a4f Phase 104: audit result API and fix rustdoc links
working tree: clean (only untracked PHASE*.md docs from prior phases)
tests: 538 pass
doctests: 10 pass
rustdoc: 0 warnings
fmt: PASS
check: PASS (pre-existing pardiso warnings only)
```

---

## 2. Current Result Model

### 2.1 Three Result Types

| Type | Ownership | Clone | Lifetime |
|------|-----------|-------|----------|
| `FrameAnalysisResult` | owned (`BeamSolver` + `FrameModel`) | NO | independent |
| `BeamAnalysisResult<'a>` | borrows `&'a BeamSolver` | YES | tied to solver |
| `TrussAnalysisResult` | owned pure snapshot | YES | independent |

### 2.2 Post-processing Types

| Type | Ownership | Clone | Notes |
|------|-----------|-------|-------|
| `Envelope` | owned (`Vec<EnvelopeSample>`) | YES | pure data, no solver ref |
| `EnvelopeSample` | owned (`SectionForces` × 2) | YES, Copy | pure data |
| `BeamForceSample` | owned (`SectionForces` + coords) | YES, Copy | pure data |
| `BeamForceDiagram` | owned (`Vec<BeamForceSample>`) | YES | pure data |
| `SectionForces` | owned (`f64` × 3) | YES, Copy | pure data |

### 2.3 Equilibrium Types

| Type | Ownership | Clone | Private fields |
|------|-----------|-------|----------------|
| `EquilibriumReport` | owned (`f64` × 12) | YES, Copy | `tolerance`, `f_mag`, `m_mag`, `l_char`, `cond_rel_floor` |
| `TrussEquilibriumReport` | owned (`f64` × 10) | YES, Copy | none (all public) |

---

## 3. Existing Serialization in `section-properties`

### 3.1 Dependencies

`section-properties` v0.4.0 has **direct dependencies** on:

```toml
serde = { version = "1.0", features = ["derive"] }
serde_json = "1.0"
csv = "1.3"
```

`structural-analysis` does **not** directly depend on serde/csv — it inherits
them transitively through `section-properties` but has **zero** usage in source.

### 3.2 The `io` Module Pattern

`section-properties` has a mature `src/io/` module:

```text
src/io/
├── json.rs         — JSON export via dedicated DTO types
├── csv.rs          — CSV export with configurable options
├── dxf.rs          — DXF export
├── mesh_export.rs  — Nastran BDF + VTK export
└── svg.rs          — SVG export
```

### 3.3 The DTO Pattern (Established Precedent)

`section-properties` does **not** directly `#[derive(Serialize)]` on its core
types (`Section`, `Material`, `SectionProperties`). Instead, it uses **dedicated
DTO types**:

```rust
// io/json.rs
#[derive(Serialize, Deserialize)]
pub struct JsonSection { ... }          // ← DTO, not Section

#[derive(Serialize, Deserialize)]
pub struct JsonSectionProperties { ... } // ← DTO, not SectionProperties

#[derive(Serialize, Deserialize)]
pub struct JsonMaterial { ... }          // ← DTO, not Material

pub fn to_json(section: &Section, ...) -> String { ... }
```

This is the **architectural precedent**: core types → DTO → format-specific
serialization. The DTO decouples internal representation from file format.

---

## 4. Serialization Suitability — Per-Type Assessment

### 4.1 `FrameAnalysisResult`

```rust
pub struct FrameAnalysisResult {
    beam: BeamSolver,      // SparseMatrix, solver state, model internals
    model: FrameModel,     // BeamModel with nodes, elements, loads, constraints
    load_source: Option<String>,
}
```

**Direct `#[derive(Serialize)]`? NO.**

Rationale:
- `BeamSolver` contains `SparseMatrix` (CSR storage), solver backend selection,
  reduced system, prescribed values — all internal implementation details.
- Serializing these would bind the solver's internal representation to a public
  file format. Any future solver implementation change (e.g., different matrix
  storage, iterative solver state) would become a serialization compatibility
  problem.
- `FrameModel` contains the full structural model (geometry, material, section,
  loads, boundary conditions) — useful for persistence, but mixed with solver
  state in the same struct.
- The struct is not even `Clone` (because `SparseMatrix` is not `Clone`).

**Verdict**: Requires a **snapshot DTO** that extracts displacements, reactions,
member forces, and geometry into plain owned data — similar to what
`TrussAnalysisResult` already is.

### 4.2 `BeamAnalysisResult<'a>`

```rust
pub struct BeamAnalysisResult<'a> {
    solver: &'a BeamSolver,
    reactions: Vec<f64>,
}
```

**Direct `#[derive(Serialize)]`? NO.**

Rationale:
- Borrows `&'a BeamSolver` — a lifetime-bound reference type cannot be
  serialized directly (serde requires owned data).
- The result is a **runtime API** for inspecting a solved beam while the solver
  is alive. It is not a persistence type.
- A snapshot DTO would be needed for persistence.

**Verdict**: Runtime API only. Persistence requires a separate snapshot DTO.

### 4.3 `TrussAnalysisResult`

```rust
pub struct TrussAnalysisResult {
    pub displacements: Vec<f64>,
    pub reactions: Vec<f64>,
    pub axial_forces: Vec<f64>,
    pub solver_name: Option<String>,
    pub node_coords: Vec<(f64, f64)>,
    pub element_nodes: Vec<(usize, usize)>,
    pub nodal_forces: Vec<(usize, usize, f64)>,
    n_nodes: usize,
    n_elements: usize,
}
```

**Direct `#[derive(Serialize)]`? YES (structurally).**

Rationale:
- Already an owned snapshot — all data is plain `Vec<f64>`, `Option<String>`,
  `Vec<(f64, f64)>`, `Vec<(usize, usize)>`.
- All fields except `n_nodes`/`n_elements` are already public.
- `n_nodes`/`n_elements` are derivable from other fields (`displacements.len() / 2`,
  `axial_forces.len()`), so they could be `#[serde(skip)]` or made public.
- Includes geometry (`node_coords`, `element_nodes`) — self-contained for
  export/visualization.

**Verdict**: Already close to a serializable DTO. Adding `#[derive(Serialize)]`
would be trivial. But **DEFER** — no actual persistence demand.

### 4.4 `Envelope`

```rust
pub struct Envelope {
    pub samples: Vec<EnvelopeSample>,
    pub n_results: usize,
    pub n_members: usize,
    pub n_per_member: usize,
}
```

**Direct `#[derive(Serialize)]`? YES (structurally).**

Rationale:
- Pure data, owned, `Clone`. No solver/model references.
- All fields public. `EnvelopeSample` is `Copy + PartialEq`.
- `SectionForces` (inside `EnvelopeSample`) is plain `f64 × 3`.
- Deterministic ordering: samples in member-major, xi-minor order.

**Verdict**: Structurally serde-ready. **DEFER** actual implementation.

### 4.5 `BeamForceSample` / `BeamForceDiagram`

Both are pure data containers (`Copy`, `Clone`, `PartialEq`). Structurally
serde-ready. **DEFER**.

### 4.6 `SectionForces`

Plain `f64 × 3`, `Copy + Clone + PartialEq`. Trivially serde-ready. **DEFER**.

### 4.7 `EquilibriumReport`

```rust
pub struct EquilibriumReport {
    pub fx_residual: f64, pub fy_residual: f64, pub mz_residual: f64,
    pub applied_fx: f64, pub applied_fy: f64, pub applied_mz: f64,
    pub reaction_fx: f64, pub reaction_fy: f64, pub reaction_mz: f64,
    tolerance: f64,    // private
    f_mag: f64,        // private
    m_mag: f64,        // private
    l_char: f64,       // private
    cond_rel_floor: f64, // private
}
```

**Direct `#[derive(Serialize)]`? NO (without exposing private fields).**

Rationale:
- 5 private fields feed `is_balanced()`. Exposing them for serde would break
  encapsulation — they are implementation details of the tolerance logic.
- A DTO could expose only the public fields + `force_tolerance()` +
  `moment_tolerance()` + `is_balanced()` verdict.

**Verdict**: Needs a DTO for serialization. **DEFER**.

### 4.8 `TrussEquilibriumReport`

All 10 fields are public `f64`. Structurally serde-ready. **DEFER**.

### 4.9 Summary Table

| Type | Core API? | Suitable for direct Serialize? | Should be directly exported? |
|------|-----------|-------------------------------|------------------------------|
| `FrameAnalysisResult` | YES | NO (solver internals) | NO — use DTO snapshot |
| `BeamAnalysisResult<'a>` | YES | NO (lifetime-bound) | NO — use DTO snapshot |
| `TrussAnalysisResult` | YES | YES (already a snapshot) | DEFER |
| `Envelope` | YES (post-processing) | YES (pure data) | DEFER |
| `BeamForceSample` | YES (post-processing) | YES (pure data) | DEFER |
| `BeamForceDiagram` | YES (post-processing) | YES (pure data) | DEFER |
| `SectionForces` | YES (core value type) | YES (plain data) | DEFER |
| `EquilibriumReport` | YES | NO (private fields) | NO — use DTO |
| `TrussEquilibriumReport` | YES | YES (all public) | DEFER |

---

## 5. Export DTO Decision

### 5.1 Do we need Export DTOs now?

**DEFER.** No actual persistence/export demand exists. The current API is
sufficient for runtime consumers (tests, examples, equilibrium checks).

### 5.2 When needed, what shape?

Follow the `section-properties` precedent: **dedicated DTO types in an `io`
module**, not direct `#[derive(Serialize)]` on core types.

```text
structural-analysis/src/io/
├── json.rs    — FrameResultJson, BeamResultJson, TrussResultJson, EnvelopeJson
├── csv.rs     — tabular export for force samples, envelope samples
└── mod.rs
```

The DTOs would be **snapshot types** — owned, plain data, no solver references:

```rust
// Example future shape (NOT implemented now):
#[derive(Serialize, Deserialize)]
pub struct FrameResultJson {
    pub schema_version: u32,
    pub nodes: Vec<NodeData>,
    pub members: Vec<MemberData>,
    pub displacements: Vec<f64>,
    pub reactions: Vec<f64>,
    pub member_end_forces: Vec<[f64; 6]>,
    pub load_source: Option<String>,
    pub solver_name: Option<String>,
    pub equilibrium: EquilibriumData,
}
```

### 5.3 Conversion functions

```rust
impl FrameAnalysisResult {
    pub fn to_export(&self) -> FrameResultJson { ... }
}
```

The conversion is one-way: `Result → DTO`. There is no `from_export()` —
reconstruction from a file would require re-building a model and re-solving,
which is a different workflow.

### 5.4 Why not directly serialize TrussAnalysisResult?

Although `TrussAnalysisResult` is structurally serde-ready, adding
`#[derive(Serialize)]` directly would:
1. Add a `serde` dependency to `structural-analysis` (currently not a direct dep).
2. Couple the public field layout to a file format schema.
3. Make future field additions a serialization compatibility concern.

The DTO pattern avoids all three. The cost is one extra type + conversion fn,
which is minimal.

**Decision**: **DEFER**. When export is needed, use DTOs for all types,
including `TrussAnalysisResult`.

---

## 6. JSON Decision

**DEFER.**

No actual JSON export demand. When needed:
- Add `serde` + `serde_json` as direct dependencies of `structural-analysis`
  (or as a feature-gated opt-in).
- Create `io::json` module with dedicated DTO types.
- Include `schema_version: u32` field for forward compatibility.
- Handle NaN/Infinity: use `serde_json`'s default (serialize as `null` or
  error) or a custom serializer.
- Provide both `to_json()` (compact) and `to_json_pretty()` options.

**Engineering reason for deferral**: No consumer needs JSON output. Adding it
now would be speculative development — violating the principle established in
Phase 79 ("identify real user capability gaps, don't build speculative
infrastructure").

---

## 7. CSV Decision

**DEFER.**

CSV is suitable for **tabular** data (force samples, envelope samples, nodal
results) but not for complete analysis results (nested structure, variable
member counts, equilibrium reports).

When needed:
- Add `io::csv` module.
- Follow `section-properties`' `CsvExportOptions` pattern (configurable
  delimiter, precision, header, units).
- Export specific views: `export_force_diagram_csv(&diagram)`,
  `export_envelope_csv(&envelope)`, `export_nodal_results_csv(&result)`.

**Engineering reason for deferral**: No consumer needs CSV output.

---

## 8. Geometry Decision

### 8.1 Does Export need Geometry?

**YES** (when export is implemented). A serialized result without geometry is
useless — the consumer needs to know where nodes are and how members connect.

### 8.2 Current Geometry API is sufficient

The Phase 103 geometry accessors provide everything needed:

| Data | Frame | Beam | Truss |
|------|-------|------|-------|
| Node positions | `node_position(NodeHandle) → Option<Point>` | `node_position(usize) → Option<Point>` | `node_position(usize) → Option<(f64, f64)>` |
| Connectivity | `member_nodes(MemberHandle) → Option<(NodeHandle, NodeHandle)>` | `element_nodes(usize) → Option<(usize, usize)>` | `element_endpoints(usize) → Option<(usize, usize)>` |
| Member length | `member_length(MemberHandle) → Option<f64>` | N/A | N/A |

### 8.3 Does Export need a separate Geometry DTO?

**NO.** Geometry data should be **embedded** in the result DTO, not separated.
`TrussAnalysisResult` already demonstrates this: `node_coords` and
`element_nodes` are fields of the result struct. A `FrameResultJson` DTO would
similarly include `nodes: Vec<NodeData>` and `members: Vec<MemberData>`.

**Decision**: Geometry is part of the result snapshot, not a separate export
type. **DEFER** implementation.

---

## 9. Load Provenance

### 9.1 Current: `Option<String>`

```rust
FrameAnalysisResult::load_source() → Option<&str>
// None | "case:{name}" | "combination:{name}"
```

### 9.2 Should this be a strong enum?

```rust
enum LoadSource {
    Direct,
    Case(String),
    Combination(String),
}
```

**DEFER.** Rationale:
1. The value is only set by the solver, never constructed by the user.
2. The only consumer use case is display/logging — a string is adequate.
3. For export/persistence, the DTO can use a string field or a typed enum —
   either way, it's a DTO decision, not a core API change.
4. No machine-parsing of the `"case:{name}"` format exists or is planned.
5. Changing to an enum would be a breaking API change for no engineering value.

**Decision**: KEEP `Option<String>`. Revisit only if machine parsing of load
source becomes a real export requirement.

---

## 10. Visualization Compatibility

### 10.1 Can Visualization work without solver internals?

**YES.** The current API provides everything a visualization layer needs:

```text
Geometry:     node_position(), member_nodes(), member_length()
Displacements: displacement(), displacements()
Reactions:    reaction(), reactions()
Forces:       member_end_forces(), section_forces(), axial_force()
Diagrams:     sample_forces(), diagram()
Envelope:     Envelope::from_frame_results(), member_samples(), max_abs_*()
Equilibrium:  equilibrium().is_balanced()
```

No method requires access to `BeamSolver`, `FrameModel.inner`, or
`TrussSolver` internals.

### 10.2 Two consumption paths

```text
Path A (runtime):
  AnalysisResult + Geometry API → Visualization (in-process)

Path B (persistence):
  AnalysisResult → Export DTO → JSON/CSV → Visualization (cross-process)
```

Both paths are viable. Path A is for embedded visualization (e.g., a GUI in the
same process). Path B is for file-based export to external tools.

### 10.3 Target architecture

```text
                    ┌── Export DTO ── JSON / CSV
                    │
AnalysisResult ─────┤
                    │
                    └── Visualization (direct API consumption)
```

**Not**:

```text
Solver ─────────────┬── Export
                    └── Visualization
```

---

## 11. Architecture

```text
section-properties
        ↑
        │ (one-way dependency)
        │
structural-analysis
        │
        ├── Model
        │     ├── FrameModel
        │     ├── BeamModel
        │     └── TrussModel
        │
        ├── Solver
        │     ├── FrameSolver
        │     ├── BeamSolver
        │     └── TrussSolver
        │
        ├── AnalysisResult
        │     ├── FrameAnalysisResult    (owned, not Clone)
        │     ├── BeamAnalysisResult<'a> (borrowed, Clone)
        │     └── TrussAnalysisResult    (owned snapshot, Clone)
        │
        ├── Post-processing
        │     ├── Envelope / EnvelopeSample
        │     ├── BeamForceSample / BeamForceDiagram
        │     └── SectionForces
        │
        ├── Equilibrium
        │     ├── EquilibriumReport      (Frame / Beam)
        │     └── TrussEquilibriumReport (Truss)
        │
        └── Future Export (DEFER)
                │
                ├── io::json  (DTO types → JSON)
                ├── io::csv   (tabular export)
                └── (Visualization consumes Result + Geometry API directly)
```

---

## 12. Binary Serialization

**Not considered.** `bincode`, `postcard`, `rkyv` — all deferred. No demand,
no dependency. If binary serialization is ever needed (e.g., for high-performance
caching of results), it would follow the same DTO pattern with a feature gate.

---

## 13. Versioning

**Not implemented.** When JSON export is added, the DTO should include:

```rust
#[derive(Serialize, Deserialize)]
pub struct FrameResultJson {
    pub schema_version: u32,  // ← version field
    // ... data fields
}
```

The version field allows future schema evolution without breaking existing
files. `schema_version: 1` for the initial format.

**Decision**: Design consideration only. **DEFER** implementation.

---

## 14. LoadCase / LoadCombination in Export

### 14.1 Distinction

```text
LoadCase / LoadCombination = analysis INPUT
FrameAnalysisResult       = analysis OUTPUT
Envelope                  = cross-result POST-PROCESSING
```

Export must not conflate these. A complete analysis archive would include both
input (model + loads) and output (results), but they should be separate
sections in the export format, not a single flat struct.

### 14.2 Decision

**DEFER.** When export is needed, the DTO would have separate input/output
sections:

```rust
pub struct AnalysisArchive {
    pub schema_version: u32,
    pub model: ModelData,           // input
    pub load_cases: Vec<LoadCaseData>, // input
    pub results: Vec<ResultData>,   // output
    pub envelopes: Vec<EnvelopeData>, // post-processing
}
```

---

## 15. Final Answers

### Q1: Should `FrameAnalysisResult` be directly Serialize?

**NO.** Contains `BeamSolver` with `SparseMatrix`, solver backend, model
internals. Direct serialization would bind internal implementation to public
file format. Use a snapshot DTO.

### Q2: Should `BeamAnalysisResult<'a>` be directly Serialize?

**NO.** Lifetime-bound to solver. Not a persistence type. A snapshot DTO would
be needed.

### Q3: Is `TrussAnalysisResult` already close to a serializable DTO?

**YES.** It is already an owned snapshot with all-public fields (except
`n_nodes`/`n_elements` which are derivable). Could be serde-derive-ready with
minimal effort. But **DEFER** — use a DTO for consistency with the
`section-properties` pattern.

### Q4: Should `Envelope` be independently persistable?

**YES** (structurally). Pure data, owned, `Clone`, deterministic ordering.
**DEFER** actual implementation.

### Q5: Do we need Export DTOs?

**DEFER.** No actual demand. When needed, follow `section-properties` pattern:
dedicated DTO types in an `io` module, not direct derives on core types.

### Q6: Do we need a JSON API?

**DEFER.** No demand. When needed, add `io::json` with DTO types + schema
versioning.

### Q7: Do we need a CSV API?

**DEFER.** No demand. When needed, add `io::csv` for tabular views (force
samples, envelope samples, nodal results).

### Q8: Do we need strongly-typed `LoadSource`?

**DEFER.** `Option<String>` is adequate. Change only if machine parsing becomes
a real requirement.

### Q9: Should Geometry enter Export data?

**YES** (when export is implemented). Geometry is embedded in the result
snapshot, not a separate export type. `TrussAnalysisResult` already demonstrates
this pattern.

### Q10: Can future Visualization work without solver internals?

**YES.** The Geometry API + Result API + Envelope provide all needed data.
No access to `BeamSolver`, `FrameModel.inner`, or `TrussSolver` internals
is required.

---

## 16. Verification

```text
cargo fmt --all -- --check          PASS
cargo check --workspace --tests --examples  PASS (pre-existing pardiso warnings)
cargo test --release -p structural-analysis  538 tests PASS
cargo test --release -p structural-analysis --doc  10 doctests PASS
cargo doc -p structural-analysis --no-deps    0 warnings
```

No code changes. No commit required.

---

## 17. Summary

| Category | Count | Details |
|----------|-------|---------|
| P0 issues | 0 | — |
| P1 issues | 0 | — |
| Production changes | 0 | Audit-only phase |
| KEEP decisions | 2 | LoadSource as `Option<String>`, Geometry in result snapshot |
| DEFER decisions | 7 | Export DTOs, JSON, CSV, binary, versioning, LoadCase export, direct serde on Truss |
| No-commit decisions | 1 | `TrussAnalysisResult` serde-ready but DEFER for consistency |

**Conclusion**: The Result / Post-processing / Geometry API established in
Phases 102–104 is **stable and sufficient** for both runtime consumption and
future export. The `section-properties` `io` module provides the architectural
precedent: **dedicated DTO types, not direct core-type serialization**. No
serialization infrastructure should be added to `structural-analysis` until a
real export/persistence demand is identified. The existing API already allows
future Visualization to operate entirely without solver internals.
