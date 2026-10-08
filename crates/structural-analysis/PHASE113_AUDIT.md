# Phase 113 — 3D Readiness / API Lock-in Audit

**Date:** 2026-10-08
**Baseline:** `304a03f` (Phase 112)
**Production code changes:** 0
**Output:** this document only

---

## 1. Objective

Not to implement 3D. Not to add features. Not to create a generic 3D
abstraction. This audit answers one question:

> Which public APIs can safely survive a future 3D Truss implementation,
> which are intrinsically 2D, and which would force an unnecessary breaking
> change?

The audit also assesses 3D Frame readiness and locks in the API surface so
that future 3D work does not inadvertently break the 2D contract.

---

## 2. Baseline

| Item | Value |
|---|---|
| HEAD | `304a03f` Phase 112: audit result and post-processing consistency |
| Working tree | clean (only untracked old `PHASE*.md` docs) |
| `cargo fmt --all -- --check` | pass |
| `cargo check --workspace` | pass (3 pre-existing pardiso warnings) |
| Crate | `structural-analysis` v0.1.0, `publish = false` |
| Dependency | `section-properties` v0.4.0 (published, frozen public API) |

---

## 3. Complete Public API Inventory

### 3.1 `lib.rs` re-exports

```
pub use beam_fem::{
    BeamAnalysis, BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver,
    Dof, EndRelease, FemError,
};
pub use frame::{
    EquilibriumReport, FrameAnalysisResult, FrameModel, FrameSolver,
    MemberHandle, NodeHandle, PreparedFrameAnalysis, StructuralDiagnostic,
};
pub use load::{LoadCase, LoadCombination, LoadCombinationTerm, LoadSource};
pub use postprocessing::{
    Envelope, EnvelopeSample, Extremum, NodeDisplacementSample,
    NodeReactionSample, SectionForceSources,
};
pub use truss::{
    TrussAnalysisResult, TrussDof, TrussElement, TrussEquilibriumReport,
    TrussModel, TrussNode, TrussSolver,
};
```

### 3.2 Type inventory by module

**`beam_fem`** (lower-level, 4489 lines):

| Type | Kind | Signature highlights |
|---|---|---|
| `Dof` | enum | `Ux, Uy, Rz`; `ALL: [Dof; 3]`; `index() -> 0/1/2` |
| `FemError` | enum | `InvalidModel`, `SolverError`, `InvalidInput`, `InvalidNode`, `InvalidMember`, `ZeroLengthMember`, `DuplicateMember`, `OrphanNode`, `DisconnectedStructure`, `ConflictingPrescribedDisplacement` |
| `BeamSection` | struct | `area: f64`, `second_moment: f64` |
| `SectionForces` | struct | `axial: f64`, `shear: f64`, `moment: f64` |
| `BeamForceSample` | struct | `element_index`, `xi`, `x`, `section_forces: SectionForces` |
| `BeamForceDiagram` | struct | `samples: Vec<BeamForceSample>` |
| `BeamNodalDisplacement` | struct | `ux, uy, rz: f64` |
| `BeamReaction` | struct | `fx, fy, mz: f64` |
| `BeamAnalysisResult<'a>` | struct | borrows `BeamSolver`; `displacement()`, `reaction()`, `element_end_forces() -> [f64; 6]`, `section_forces() -> SectionForces` |
| `EndRelease` | struct | `start_rotation: bool`, `end_rotation: bool` |
| `BeamElement` | struct | `node_i, node_j, material, section, end_release`; `local_stiffness() -> [[f64; 6]; 6]`, `transformation_matrix() -> [[f64; 6]; 6]`, `global_stiffness() -> [[f64; 6]; 6]` |
| `BeamNode` | struct | `id: usize, x: f64, y: f64` |
| `DistributedLoad` | struct | `element_idx, qx, qy, qx_end, qy_end: f64` |
| `PointLoad` | struct | `element_idx, position, fx, fy, mz: f64` |
| `AppliedMoment` | struct | `node_idx: usize, value: f64` |
| `BeamModel` | struct | `nodes, elements, nodal_forces, distributed_loads, point_loads, applied_moments, fixed_dofs, spring_supports, inclined_rollers`; `n_dof() = nodes * 3` |
| `BeamSolver` | struct | `from_model()`, `solve()`, `solve_configured()`, `displacement()`, `reactions()`, `element_end_forces()`, `results()` |
| `BeamAnalysis` | struct | `cantilever_tip_load()`, `simply_supported_central_load()` |

**`frame`** (higher-level façade, 2602 lines):

| Type | Kind | Signature highlights |
|---|---|---|
| `NodeHandle` | newtype | `NodeHandle(usize)`; `index()`, `from_index()` |
| `MemberHandle` | newtype | `MemberHandle(usize)`; `index()`, `from_index()` |
| `EquilibriumReport` | struct | `fx_residual, fy_residual, mz_residual, applied_fx/fy/mz, reaction_fx/fy/mz`; `is_balanced()` |
| `FrameModel` | struct | wraps `BeamModel`; `add_node(x, y)`, `add_member(a, b, material, section)`, `fix/pin/roller_x/roller_y/restrain/spring/inclined_roller`, `nodal_load(node, fx, fy)`, `nodal_moment(node, mz)`, `member_udl/trapezoidal/point_load`, `self_weight(gx, gy)`, `solve/solve_with/solve_case/solve_combination`, `prepare/prepare_with/solve_cases`, `validate`, `diagnostic` |
| `FrameSolver<'a>` | struct | `new(model)`, `with_selection()`, `solve()` |
| `FrameAnalysisResult` | struct | owns `BeamSolver + FrameModel + reactions + load_source`; `displacement(node, dof)`, `reaction(node, dof)`, `member_end_forces() -> [f64; 6]`, `section_forces() -> SectionForces`, `equilibrium() -> EquilibriumReport` |
| `PreparedFrameAnalysis<'a>` | struct | borrows `FrameModel`; `solve_case`, `solve_cases`, `solve_combination` |

**`load`** (510 lines):

| Type | Kind | Signature highlights |
|---|---|---|
| `LoadCase` | struct | `name`, `nodal_forces, distributed_loads, point_loads, applied_moments, prescribed_displacements`; `nodal_load(node, fx, fy)`, `nodal_moment(node, mz)`, `member_udl/trapezoidal/point_load`, `add_self_weight(model, gx, gy)`, `prescribed_displacement(node, dof, value)` |
| `LoadCombination` | struct | `name, terms: Vec<(LoadCase, f64)>`; `add_case(case, factor)` |
| `LoadCombinationTerm` | struct | `case_name: String, factor: f64` |
| `LoadSource` | enum | `ModelLoads`, `LoadCase { name, has_prescribed_displacements }`, `LoadCombination { name, terms }` |

**`postprocessing`** (410 lines):

| Type | Kind | Signature highlights |
|---|---|---|
| `SectionForceSources` | struct | `axial, shear, moment: Option<usize>` |
| `EnvelopeSample` | struct | `member_index, xi, min: SectionForces, max: SectionForces, min_sources, max_sources: SectionForceSources` |
| `Extremum` | struct | `min, max: f64, min_source, max_source: Option<usize>` |
| `NodeDisplacementSample` | struct | `node_index, ux: Extremum, uy: Extremum, rz: Extremum` |
| `NodeReactionSample` | struct | `node_index, rx: Extremum, ry: Extremum, mz: Extremum` |
| `Envelope` | struct | `samples, n_results, n_members, n_per_member, node_displacements, support_reactions, n_nodes, sources`; `from_frame_results()`, `member_samples()`, `node_displacement()`, `support_reaction()` |

**`truss`** (946 lines):

| Type | Kind | Signature highlights |
|---|---|---|
| `TrussDof` | enum | `Ux, Uy`; `ALL: [TrussDof; 2]`; `index() -> 0/1` |
| `TrussNode` | struct | `id: usize, x: f64, y: f64` |
| `TrussElement` | struct | `node_i, node_j, E: f64, A: f64`; `global_stiffness() -> [[f64; 4]; 4]`, `axial_force(u: [f64; 4]) -> f64` |
| `TrussModel` | struct | `nodes, elements, nodal_forces, fixed_dofs`; `n_dof() = nodes * 2` |
| `TrussSolver` | struct | `from_model()`, `solve/solve_configured`, `displacement(node, dof)`, `reactions()`, `axial_force()`, `diagnostic()` |
| `TrussAnalysisResult` | struct | `displacements: Vec<f64>`, `reactions: Vec<f64>`, `axial_forces: Vec<f64>`, `node_coords: Vec<(f64, f64)>`, `element_nodes`, `nodal_forces` |
| `TrussEquilibriumReport` | struct | `fx_residual, fy_residual, mz_residual, applied_fx/fy/mz, reaction_fx/fy/mz, tolerance` |

**`mechanism`** (578 lines):

| Type | Kind | Signature highlights |
|---|---|---|
| `DiagnosticLimit` | enum | `SystemTooLarge, NotSymmetric, NotPositiveSemidefinite` |
| `StructuralDiagnostic` | enum | `Stable, RigidBodyMode { n_free, rank, rigid_modes }, Mechanism { n_free, rank }, IllConditioned { n_free, rank }, Indeterminate { n_free, reason }` |
| `diagnose_reduced` | fn | `(k_ff: &SparseMatrix, rigid_candidates: &[Vec<f64>]) -> StructuralDiagnostic` |
| `MAX_DENSE_PROBE_DOF` | const | `500` |

---

## 4. DOF Architecture Audit

### 4.1 `Dof` (frame/beam)

```rust
pub enum Dof { Ux, Uy, Rz }
pub const ALL: [Dof; 3] = [Dof::Ux, Dof::Uy, Dof::Rz];
pub const fn index(self) -> usize  // 0, 1, 2
```

**Classification: INTRINSICALLY 2D.**

The enum has exactly three variants corresponding to the three global DOFs of
a planar frame node: two translations and one rotation about the out-of-plane
axis. A 3D frame node has six DOFs (`Ux, Uy, Uz, Rx, Ry, Rz`). There is no
way to extend this enum without changing the number of variants, which is a
breaking change to every caller that matches on `Dof` or reads `Dof::ALL`.

The DOF index mapping `dof(node, d) = 3*node + d` is hardcoded as `3` in
`BeamModel::n_dof()` and throughout the assembly/solve path. This constant
would become `6` for a 3D frame.

### 4.2 `TrussDof` (truss)

```rust
pub enum TrussDof { Ux, Uy }
pub const ALL: [TrussDof; 2] = [TrussDof::Ux, TrussDof::Uy];
pub const fn index(self) -> usize  // 0, 1
```

**Classification: INTRINSICALLY 2D.**

A 3D truss node has three translational DOFs (`Ux, Uy, Uz`). The enum would
need a third variant, and `ALL` would change from `[TrussDof; 2]` to
`[TrussDof; 3]`.

The DOF index mapping `dof(node, d) = 2*node + d` is hardcoded as `2` in
`TrussModel::n_dof()` and `dof_index()`.

### 4.3 Assessment

| Item | 2D | 3D Frame | 3D Truss | Verdict |
|---|---|---|---|---|
| `Dof` variants | 3 | 6 | — | intrinsically 2D |
| `Dof::ALL` size | 3 | 6 | — | intrinsically 2D |
| `TrussDof` variants | 2 | — | 3 | intrinsically 2D |
| `TrussDof::ALL` size | 2 | — | 3 | intrinsically 2D |
| Frame DOF stride | 3 | 6 | — | hardcoded constant |
| Truss DOF stride | 2 | — | 3 | hardcoded constant |

No accidental 2D here — the DOF types are correctly and irreducibly
dimension-specific. A generic `Dof<N>` abstraction would add complexity for
no benefit; the recommendation is separate types for 3D (§12–13).

---

## 5. Truss 3D Readiness Audit

### 5.1 `TrussElement`

```rust
pub struct TrussElement {
    pub node_i: usize,
    pub node_j: usize,
    pub E: f64,
    pub A: f64,
}
pub fn global_stiffness(&self, pi: Point, pj: Point) -> [[f64; 4]; 4]
pub fn axial_force(&self, pi: Point, pj: Point, u: [f64; 4]) -> f64
```

The 4×4 stiffness uses 2D direction cosines `c = dx/L, s = dy/L`. A 3D
truss element has a 6×6 stiffness with three direction cosines
`c = dx/L, s = dy/L, t = dz/L`. The return type `[[f64; 4]; 4]` would become
`[[f64; 6]; 6]` and `axial_force` input would change from `[f64; 4]` to
`[f64; 6]`.

`TrussElement` stores `E` and `A` directly (not `Material`), so there is no
density field and no self-weight support — this is unchanged in 3D.

**Verdict: INTRINSICALLY 2D.** The element stiffness shape and direction
cosine count are hardcoded for 2D.

### 5.2 `TrussModel`

```rust
pub fn n_dof(&self) -> usize { self.nodes.len() * 2 }
pub(crate) fn dof_index(&self, node_idx: usize, dof: usize) -> usize { node_idx * 2 + dof }
```

The stride `2` is hardcoded. `TrussNode` has `x, y` but no `z`.

**Verdict: INTRINSICALLY 2D.**

### 5.3 `TrussSolver`

The solver reuses `SparseMatrix`, `LinearSolver`, `SolverRegistry` — all
DOF-agnostic. The 2D-specific parts are:

- Assembly: iterates 4×4 element stiffness, maps 4 DOFs per element
  (`dof_index(node, 0/1)`).
- `rigid_candidates()`: builds 3 modes (Tx, Ty, Rz) using `global / 2` for
  node index and `global % 2` for DOF. A 3D truss has 6 rigid-body modes
  (Tx, Ty, Tz, Rx, Ry, Rz) and the stride would be 3.

**Verdict: assembly + rigid candidates are INTRINSICALLY 2D; solver
infrastructure is SAFE.**

### 5.4 `TrussAnalysisResult`

```rust
pub displacements: Vec<f64>,   // 2 per node
pub reactions: Vec<f64>,       // 2 per node
pub node_coords: Vec<(f64, f64)>,  // (x, y)
```

All layouts assume 2 DOF/node and 2D coordinates.

**Verdict: INTRINSICALLY 2D.**

### 5.5 `TrussEquilibriumReport`

```rust
pub fx_residual, fy_residual, mz_residual: f64
```

Three equilibrium equations (ΣFx, ΣFy, ΣMz). A 3D truss has six (ΣFx, ΣFy,
ΣFz, ΣMx, ΣMy, ΣMz).

**Verdict: INTRINSICALLY 2D.**

### 5.6 Truss summary

| Component | 3D-ready? | What changes |
|---|---|---|
| `TrussElement` | no | 6×6 stiffness, 3 direction cosines |
| `TrussModel` | no | stride 2→3, `TrussNode` needs `z` |
| `TrussSolver` assembly | no | 6×6 element, 6-DOF mapping |
| `TrussSolver` solve path | yes | DOF-agnostic (SparseMatrix + LinearSolver) |
| `rigid_candidates` | no | 3 modes → 6 modes, stride 2→3 |
| `TrussAnalysisResult` | no | 3 DOF/node, 3D coords |
| `TrussEquilibriumReport` | no | 3 equations → 6 |
| `diagnose_reduced` | yes | takes any matrix + candidates |
| `SparseMatrix` / `LinearSolver` | yes | fully DOF-agnostic |

---

## 6. Frame / Beam 3D Readiness Audit

### 6.1 Element stiffness

`BeamElement::local_stiffness()` returns the standard 6×6 2D Euler-Bernoulli
stiffness with `EA/L` (axial) and `EI/L³`, `EI/L²`, `EI/L` (bending). A 3D
frame element has a 12×12 stiffness with:

- Axial: `EA/L` (same)
- Biaxial bending: `EIy/L³`, `EIz/L³` (two independent bending planes)
- Torsion: `GJ/L` (new — requires shear modulus `G` and torsion constant `J`)
- No coupling between bending planes in the local frame

**Verdict: INTRINSICALLY 2D.** The 6×6 shape and single `I` are fundamental.

### 6.2 Transformation matrix

```rust
pub fn transformation_matrix(&self, ...) -> [[f64; 6]; 6]
// 2D rotation: c = dx/L, s = dy/L
// T = diag(R, 1, R, 1) where R = [[c, s], [-s, c]]
```

A 3D frame transformation is 12×12 with a full 3×3 rotation matrix (from two
or three direction vectors). The 2D rotation is a single angle; 3D requires
three angles or a rotation matrix from member orientation + reference vector.

**Verdict: INTRINSICALLY 2D.**

### 6.3 Section properties

```rust
pub struct BeamSection {
    pub area: f64,
    pub second_moment: f64,  // single I
}
```

A 3D frame section needs `area`, `second_moment_y` (Iy), `second_moment_z`
(Iz), and `torsion_constant` (J). The single `second_moment` field is
irreducibly 2D.

**Verdict: INTRINSICALLY 2D.**

### 6.4 Section forces

```rust
pub struct SectionForces {
    pub axial: f64,
    pub shear: f64,
    pub moment: f64,
}
```

A 3D frame has six section forces: `axial`, `shear_y`, `shear_z`,
`moment_y`, `moment_z`, `torque`. The three-component struct is 2D.

**Verdict: INTRINSICALLY 2D.**

### 6.5 End releases

```rust
pub struct EndRelease {
    pub start_rotation: bool,
    pub end_rotation: bool,
}
```

A 3D frame can release any of the three rotational DOFs at each end. The
single boolean per end is 2D.

**Verdict: INTRINSICALLY 2D.**

### 6.6 Loads

| Load type | 2D fields | 3D fields needed |
|---|---|---|
| `DistributedLoad` | `qx, qy` (+ end values) | `qx, qy, qz` (+ end values) |
| `PointLoad` | `fx, fy, mz` | `fx, fy, fz, mx, my, mz` |
| `AppliedMoment` | `value` (Mz) | `mx, my, mz` |
| `LoadCase::nodal_load` | `(node, fx, fy)` | `(node, fx, fy, fz)` |
| `LoadCase::nodal_moment` | `(node, mz)` | `(node, mx, my, mz)` |
| `FrameModel::self_weight` | `(gx, gy)` | `(gx, gy, gz)` |

**Verdict: ALL INTRINSICALLY 2D.**

### 6.7 Boundary conditions

| Support | 2D | 3D |
|---|---|---|
| `fix(node)` | ux, uy, rz (3 DOF) | ux, uy, uz, rx, ry, rz (6 DOF) |
| `pin(node)` | ux, uy (2 DOF) | ux, uy, uz (3 DOF) |
| `roller_x(node)` | fixes ux | fixes ux (same concept) |
| `roller_y(node)` | fixes uy | fixes uy (same concept) |
| `inclined_roller(node, nx, ny, value)` | 2D direction | 3D needs plane or direction vector |
| `spring(node, dof, k)` | `Dof` is 2D | `Dof3D` needed |
| `restrain(node, dof, value)` | `Dof` is 2D | `Dof3D` needed |

**Verdict: ALL INTRINSICALLY 2D** (through `Dof` and 2D coordinate assumptions).

### 6.8 Hermite formulation

The 6×6 local stiffness uses the standard cubic Hermite shape functions for
transverse displacement, which assume a single bending plane. In 3D, the
Hermite formulation is applied independently in two bending planes (xz and
yz), but the element stiffness must also include torsion — a fundamentally
different matrix structure.

**Verdict: INTRINSICALLY 2D.**

### 6.9 Reactions

`BeamReaction` has `fx, fy, mz`. `EquilibriumReport` has 3 residuals. Both
are 2D. 3D needs 6 components.

**Verdict: INTRINSICALLY 2D.**

### 6.10 Frame/Beam summary

| Component | 3D-ready? | What changes |
|---|---|---|
| `BeamElement` stiffness | no | 6×6 → 12×12, biaxial bending + torsion |
| `BeamElement` transformation | no | 2D rotation → 3D rotation |
| `BeamSection` | no | single I → Iy, Iz, J |
| `SectionForces` | no | 3 components → 6 |
| `EndRelease` | no | 1 rotation → 3 rotations per end |
| `DistributedLoad` | no | qx, qy → qx, qy, qz |
| `PointLoad` | no | fx, fy, mz → fx, fy, fz, mx, my, mz |
| `AppliedMoment` | no | single Mz → mx, my, mz |
| `BeamModel` DOF stride | no | 3 → 6 |
| `BeamSolver` assembly | no | 6×6 element → 12×12 |
| `BeamSolver` solve path | yes | DOF-agnostic |
| `rigid_candidates` | no | 3 modes → 6 modes |
| `FrameModel::add_node` | no | (x, y) → (x, y, z) |
| `FrameModel` support vocabulary | no | all use `Dof` (2D) |
| `FrameAnalysisResult` | no | [f64; 6] → [f64; 12], SectionForces |
| `EquilibriumReport` | no | 3 residuals → 6 |
| `BeamAnalysisResult` | no | same as FrameAnalysisResult |
| `diagnose_reduced` | yes | DOF-agnostic |
| `SparseMatrix` / `LinearSolver` | yes | DOF-agnostic |

---

## 7. Load Architecture Audit

### 7.1 Dimension-agnostic components

| Component | Why it survives |
|---|---|
| `LoadCase::new(name)`, `name()`, `is_empty()` | no dimension dependency |
| `LoadCombination` | just factors + cases, no geometry |
| `LoadCombinationTerm` | just name + factor |
| `LoadSource` | just provenance metadata |
| `LoadCase::prescribed_displacement(node, dof, value)` | takes `Dof` — if a 3D `Dof3D` is used with a 3D `LoadCase3D`, the method signature is identical |

### 7.2 Intrinsically 2D components

| Component | Why |
|---|---|
| `LoadCase::nodal_load(node, fx, fy)` | 2 force components; 3D needs 3 |
| `LoadCase::nodal_moment(node, mz)` | 1 moment; 3D needs 3 |
| `LoadCase::member_udl(member, qx, qy)` | 2 load components; 3D needs 3 |
| `LoadCase::member_trapezoidal(member, qx, qy, qx_end, qy_end)` | 4 values; 3D needs 6 |
| `LoadCase::member_point_load(member, xi, fx, fy, mz)` | 3 components; 3D needs 6 |
| `LoadCase::add_self_weight(model, gx, gy)` | 2 gravity components; 3D needs 3 |
| `DistributedLoad` | `qx, qy` fields |
| `PointLoad` | `fx, fy, mz` fields |
| `AppliedMoment` | single `value` field |

### 7.3 Assessment

The `LoadCase` / `LoadCombination` / `LoadSource` **architecture** (named
cases, linear combination, typed provenance, prescribed displacement
isolation) is dimension-agnostic and would be reused verbatim in 3D. Only
the **load-adding method signatures** and **load data structs** are 2D. A
3D `LoadCase3D` would have the same internal structure
(`nodal_forces: Vec<(usize, usize, f64)>`, etc.) but different builder
methods (`nodal_load(node, fx, fy, fz)` instead of `(node, fx, fy)`).

---

## 8. Result Architecture Audit

### 8.1 `FrameAnalysisResult`

| Method | 2D | 3D |
|---|---|---|
| `displacement(node, dof) -> f64` | `Dof` (3 variants) | `Dof3D` (6 variants) — same signature shape |
| `reaction(node, dof) -> f64` | `Dof` | `Dof3D` |
| `member_end_forces() -> [f64; 6]` | 6 local forces | `[f64; 12]` |
| `member_end_forces_global() -> [f64; 6]` | 6 global forces | `[f64; 12]` |
| `section_forces() -> SectionForces` | 3 components | `SectionForces3D` (6 components) |
| `equilibrium() -> EquilibriumReport` | 3 residuals | `EquilibriumReport3D` (6 residuals) |
| `displacements() -> &[f64]` | 3 per node | 6 per node |
| `reactions() -> &[f64]` | 3 per node | 6 per node |
| `n_nodes()`, `n_members()` | dimension-agnostic | same |
| `load_source() -> &LoadSource` | dimension-agnostic | same |
| `solver_name()` | dimension-agnostic | same |

**Verdict: INTRINSICALLY 2D** (array sizes, `SectionForces`, `EquilibriumReport`).

### 8.2 `BeamAnalysisResult`

Same issues as `FrameAnalysisResult` — it borrows `BeamSolver` and returns
`[f64; 6]` end forces and `SectionForces`.

**Verdict: INTRINSICALLY 2D.**

### 8.3 `TrussAnalysisResult`

| Field | 2D | 3D |
|---|---|---|
| `displacements: Vec<f64>` | 2 per node | 3 per node |
| `reactions: Vec<f64>` | 2 per node | 3 per node |
| `axial_forces: Vec<f64>` | same (1 per element) | same |
| `node_coords: Vec<(f64, f64)>` | (x, y) | `(f64, f64, f64)` |
| `element_nodes: Vec<(usize, usize)>` | same | same |
| `nodal_forces: Vec<(usize, usize, f64)>` | dof 0/1 | dof 0/1/2 |

**Verdict: INTRINSICALLY 2D** (coordinate pairs, DOF stride).

### 8.4 Separateness of result types

`FrameAnalysisResult`, `BeamAnalysisResult`, and `TrussAnalysisResult` are
three independent types. They share no common trait or generic base. This is
intentional (Phase 112 confirmed): each type's API is tailored to its
element's result shape. A 3D version would add `FrameAnalysisResult3D`,
`TrussAnalysisResult3D`, etc. — continuing the same pattern.

**Verdict: KEEP separate. No `Result` trait.**

---

## 9. Envelope Architecture Audit

### 9.1 Dimension-agnostic components

| Component | Why |
|---|---|
| `Extremum` | just `min/max: f64` + source indices |
| `Envelope::n_results`, `n_members`, `n_per_member`, `n_nodes` | counts |
| `Envelope::sources: Vec<LoadSource>` | dimension-agnostic |
| `Envelope::from_frame_results(results, n_per_member)` | takes `&[&FrameAnalysisResult]` — if a 3D result type exists, a parallel `from_frame_results_3d` would follow the same pattern |

### 9.2 Intrinsically 2D components

| Component | Why |
|---|---|
| `EnvelopeSample` | `min/max: SectionForces` (3 components) |
| `SectionForceSources` | `axial, shear, moment` (3 indices) |
| `NodeDisplacementSample` | `ux, uy, rz: Extremum` (3 DOF) |
| `NodeReactionSample` | `rx, ry, mz: Extremum` (3 DOF) |
| `Envelope::max_abs_moment/shear/axial` | operates on `SectionForces` fields |

### 9.3 Assessment

The envelope **architecture** (multi-case min/max tracking with governing
source indices, per-node displacement/reaction envelopes) is
dimension-agnostic. The **data types** (`EnvelopeSample`,
`NodeDisplacementSample`, `NodeReactionSample`) are 2D through
`SectionForces` and the 3-DOF assumption. A 3D envelope would need
`EnvelopeSample3D` with `SectionForces3D` and `NodeDisplacementSample3D`
with 6 `Extremum` fields.

**Verdict: architecture SAFE, data types INTRINSICALLY 2D.**

---

## 10. Boundary-Condition Architecture Audit

### 10.1 How BCs are stored

**Frame/Beam** (`BeamModel`):
- `fixed_dofs: Vec<(usize, usize, f64)>` — `(node_idx, dof_idx, value)`, `dof_idx` is 0/1/2
- `spring_supports: Vec<(usize, usize, f64)>` — `(node_idx, dof_idx, stiffness)`
- `inclined_rollers: Vec<(usize, f64, f64, f64)>` — `(node_idx, nx, ny, value)`

**Truss** (`TrussModel`):
- `fixed_dofs: Vec<(usize, usize, f64)>` — `(node_idx, dof_idx, value)`, `dof_idx` is 0/1

### 10.2 Static condensation

Both Frame and Truss enforce BCs by static condensation (no penalties):
`K_ff u_f = f_f - K_fc u_c`. This logic is in `BeamSolver::condense()` and
`TrussSolver::apply_boundary_conditions()`. The condensation itself is
DOF-agnostic — it just partitions the matrix into free/constrained indices
and adjusts the RHS. The 2D-specific part is only the DOF stride used to
build the free/constrained index lists.

### 10.3 Support vocabulary

`FrameModel::fix/pin/roller_x/roller_y/restrain/spring/inclined_roller` —
all translate to entries in `fixed_dofs` / `spring_supports` /
`inclined_rollers`. The translation is 2D-specific (e.g., `fix` constrains
DOFs 0, 1, 2; `pin` constrains 0, 1). A 3D `FrameModel3D` would have
`fix` constrain 0–5, `pin` constrain 0–2, etc.

### 10.4 Assessment

| Component | 3D-ready? |
|---|---|
| Static condensation logic | yes (DOF-agnostic) |
| `fixed_dofs` / `spring_supports` storage format | yes (raw indices) |
| `inclined_rollers` storage | no (2D `nx, ny`) |
| Support vocabulary methods | no (hardcoded DOF indices) |
| `try_fix_dof` / `try_override_dof` | yes (take raw `dof: usize`) |

**Verdict: storage and condensation SAFE; vocabulary methods INTRINSICALLY 2D.**

---

## 11. Mechanism Diagnostics Audit

### 11.1 `diagnose_reduced(k_ff, rigid_candidates)`

This function takes a `SparseMatrix` and a slice of rigid-body candidate
vectors. It performs:

1. Symmetry check
2. Equilibration (Jacobi scaling)
3. Pivoted Cholesky rank probe
4. Rigid-body nullity test
5. Raw system conditioning probe

None of these steps depend on the DOF count, element type, or dimension.
The function is **fully DOF-agnostic**. The only dimension-specific part is
the **caller's construction** of `rigid_candidates`:

- `FrameModel::rigid_candidates()` — builds 3 modes (Tx, Ty, Rz) with stride 3
- `TrussSolver::rigid_candidates()` — builds 3 modes (Tx, Ty, Rz) with stride 2

A 3D frame would build 6 modes (Tx, Ty, Tz, Rx, Ry, Rz) with stride 6; a 3D
truss would build 6 modes with stride 3. The `diagnose_reduced` function
itself would not change.

### 11.2 `StructuralDiagnostic`

The enum variants (`Stable`, `RigidBodyMode`, `Mechanism`, `IllConditioned`,
`Indeterminate`) carry `n_free`, `rank`, `rigid_modes` — all dimensionless
integers. The type is fully dimension-agnostic.

### 11.3 Assessment

**Verdict: FULLY SAFE.** `diagnose_reduced`, `StructuralDiagnostic`,
`DiagnosticLimit`, and `MAX_DENSE_PROBE_DOF` all survive 3D without change.
Only the rigid-body candidate construction (in the model types) is 2D.

---

## 12. Solver Infrastructure Audit

### 12.1 `section-properties::fea`

| Component | Location | 3D-ready? |
|---|---|---|
| `SparseMatrix` | `section-properties::fea` | yes — stores `n × n` matrix, no DOF assumption |
| `LinearSolver` trait | `section-properties::fea::solver` | yes — `factor(&SparseMatrix)`, `solve(&[f64])` |
| `SolverRegistry` | `section-properties::fea::solver` | yes — selects by matrix size/properties |
| `SolverSelection` | `section-properties::fea::solver` | yes — enum of backend choices |
| `SolverError` | `section-properties::fea::solver` | yes — error types |
| `PIVOT_TOL_BASE` | `section-properties::fea` | yes — scalar tolerance |

All solver infrastructure lives in `section-properties` and is fully
DOF-agnostic. It operates on raw matrices and vectors with no knowledge of
nodes, elements, DOFs, or dimensions.

### 12.2 Assembly path

The assembly path (`BeamSolver::from_model`, `TrussSolver::from_model`) is
where dimension enters:

- Element stiffness is computed (6×6 for 2D frame, 4×4 for 2D truss)
- Element DOFs are mapped to global DOFs (`3*node + d` or `2*node + d`)
- The global stiffness matrix is assembled from element contributions

Once assembled, the rest of the pipeline (condensation → factorisation →
back-substitution → reaction recovery) is DOF-agnostic.

### 12.3 Assessment

**Verdict: solver infrastructure FULLY SAFE. Assembly path INTRINSICALLY 2D.**

---

## 13. API Lock-in Assessment

### 13.1 Classification table

| API | Category | Survives 3D? |
|---|---|---|
| **Error / diagnostic types** | | |
| `FemError` | dimension-agnostic | yes |
| `StructuralDiagnostic` | dimension-agnostic | yes |
| `DiagnosticLimit` | dimension-agnostic | yes |
| `diagnose_reduced()` | dimension-agnostic | yes |
| `MAX_DENSE_PROBE_DOF` | dimension-agnostic | yes |
| **Handle types** | | |
| `NodeHandle` | dimension-agnostic | yes |
| `MemberHandle` | dimension-agnostic | yes |
| **Load architecture** | | |
| `LoadCombination` | dimension-agnostic | yes |
| `LoadCombinationTerm` | dimension-agnostic | yes |
| `LoadSource` | dimension-agnostic | yes |
| `LoadCase` (architecture) | dimension-agnostic | yes (structure) |
| `LoadCase::new/name/is_empty` | dimension-agnostic | yes |
| `LoadCase::prescribed_displacement` | takes `Dof` | yes (with `Dof3D`) |
| `LoadCase::has_prescribed_displacements` | dimension-agnostic | yes |
| `LoadCase::nodal_load(node, fx, fy)` | 2D signature | no |
| `LoadCase::nodal_moment(node, mz)` | 2D signature | no |
| `LoadCase::member_udl/trapezoidal/point_load` | 2D signature | no |
| `LoadCase::add_self_weight(model, gx, gy)` | 2D signature | no |
| `DistributedLoad` | 2D fields | no |
| `PointLoad` | 2D fields | no |
| `AppliedMoment` | 2D fields | no |
| **Envelope (architecture)** | | |
| `Extremum` | dimension-agnostic | yes |
| `Envelope` (architecture) | dimension-agnostic | yes (structure) |
| `Envelope::from_frame_results` | takes `FrameAnalysisResult` | yes (with 3D result) |
| `Envelope::source/member_samples/node_displacement/support_reaction` | dimension-agnostic | yes |
| `Envelope::max_abs_moment/shear/axial` | 2D `SectionForces` | no |
| `EnvelopeSample` | 2D `SectionForces` | no |
| `SectionForceSources` | 2D (3 components) | no |
| `NodeDisplacementSample` | 2D (3 DOF) | no |
| `NodeReactionSample` | 2D (3 DOF) | no |
| **DOF types** | | |
| `Dof` | intrinsically 2D | no |
| `TrussDof` | intrinsically 2D | no |
| **Frame/Beam types** | | |
| `BeamSection` | intrinsically 2D | no |
| `SectionForces` | intrinsically 2D | no |
| `BeamElement` | intrinsically 2D | no |
| `EndRelease` | intrinsically 2D | no |
| `BeamNode` | intrinsically 2D | no |
| `BeamModel` | intrinsically 2D | no |
| `BeamSolver` | intrinsically 2D | no |
| `BeamAnalysisResult` | intrinsically 2D | no |
| `BeamForceSample/Diagram` | intrinsically 2D | no |
| `BeamNodalDisplacement` | intrinsically 2D | no |
| `BeamReaction` | intrinsically 2D | no |
| `BeamAnalysis` | intrinsically 2D | no |
| `FrameModel` | intrinsically 2D | no |
| `FrameSolver` | intrinsically 2D | no |
| `FrameAnalysisResult` | intrinsically 2D | no |
| `PreparedFrameAnalysis` | intrinsically 2D | no |
| `EquilibriumReport` | intrinsically 2D | no |
| **Truss types** | | |
| `TrussNode` | intrinsically 2D | no |
| `TrussElement` | intrinsically 2D | no |
| `TrussModel` | intrinsically 2D | no |
| `TrussSolver` | intrinsically 2D | no |
| `TrussAnalysisResult` | intrinsically 2D | no |
| `TrussEquilibriumReport` | intrinsically 2D | no |
| **Solver infrastructure** | | |
| `SparseMatrix` | dimension-agnostic | yes |
| `LinearSolver` | dimension-agnostic | yes |
| `SolverRegistry` | dimension-agnostic | yes |
| `SolverSelection` | dimension-agnostic | yes |

### 13.2 Summary counts

| Category | Count |
|---|---|
| Dimension-agnostic (survives 3D) | ~20 types/functions |
| Intrinsically 2D (needs 3D replacement) | ~35 types |
| Accidentally 2D (could be generic but shouldn't) | 0 |

No accidentally 2D APIs were found. Every 2D-specific type is 2D for a
fundamental physical reason (DOF count, stiffness matrix size, section
property count, load component count), not for an incidental implementation
choice.

---

## 14. 3D Truss Strategy

### 14.1 Option A: Extend current Truss types

Add `Uz` to `TrussDof`, change stride to 3, change stiffness to 6×6, add 3
rigid modes, add `z` to `TrussNode`.

**Breaking changes:**
- `TrussDof::ALL` type changes (`[TrussDof; 2]` → `[TrussDof; 3]`)
- `TrussElement::global_stiffness` return type (`[[f64; 4]; 4]` → `[[f64; 6]; 6]`)
- `TrussElement::axial_force` input (`[f64; 4]` → `[f64; 6]`)
- `TrussModel::n_dof()` return value changes
- `TrussAnalysisResult` field layouts change
- `TrussEquilibriumReport` gains 3 new fields
- `TrussNode` gains `z` field

**Verdict: REJECTED.** Breaks every Trust public type. Not viable without
major version bump, and even then the API churn is excessive.

### 14.2 Option B: Separate `TrussModel3D` / `TrussElement3D` / etc.

New module `truss3d` (or `truss_3d`) with:

```
TrussDof3D { Ux, Uy, Uz }
TrussNode3D { id, x, y, z }
TrussElement3D { node_i, node_j, E, A }  // 6×6 stiffness
TrussModel3D { nodes, elements, nodal_forces, fixed_dofs }  // stride 3
TrussSolver3D  // reuses SparseMatrix + LinearSolver
TrussAnalysisResult3D { displacements, reactions, axial_forces, node_coords: Vec<(f64,f64,f64)>, ... }
TrussEquilibriumReport3D { fx, fy, fz, mx, my, mz residuals }
```

**Shared infrastructure (no duplication):**
- `SparseMatrix`, `LinearSolver`, `SolverRegistry`, `SolverSelection`
- `diagnose_reduced`, `StructuralDiagnostic`, `DiagnosticLimit`
- `FemError`

**Duplicated code (small, mechanical):**
- Element stiffness (4×4 → 6×6 — ~20 lines)
- DOF mapping (`2*node + d` → `3*node + d`)
- Rigid-body candidates (3 modes → 6 modes — ~30 lines)
- Assembly loop (4 DOFs/element → 6 DOFs/element)

**No breaking changes to existing 2D API.**

**Verdict: RECOMMENDED.** Minimal duplication (~100 lines), no breaking
changes, clear separation. The shared infrastructure is already
DOF-agnostic.

### 14.3 Option C: Dimension-generic `TrussModel<const N: usize>`

Use const generics for DOF count per node. `TrussDof` becomes a
`TrussDof<const N: usize>` with `ALL: [TrussDof<N>; N]`.

**Problems:**
- Const generics on enums are unstable and ergonomically poor
- Element stiffness size depends on N (4×4 for N=2, 6×6 for N=3) — can't
  express `[[f64; 2*N]; 2*N]` in stable Rust
- Rigid-body mode count depends on N (3 for 2D, 6 for 3D) — runtime logic
  needed
- Massive complexity for 2 cases

**Verdict: REJECTED.** Over-engineering. The complexity is not justified
by having only two dimensions.

### 14.4 Recommendation

**Option B: separate 3D Truss types.** The 2D Truss module is 946 lines,
self-contained, and the 3D version would be similar in size. The shared
solver infrastructure and mechanism diagnostics are already
DOF-agnostic. The duplication is small and mechanical.

---

## 15. 3D Frame Strategy

### 15.1 Why 3D Frame is fundamentally different

A 3D frame element is not a "3D version of the 2D element" — it is a
different element with different physics:

| Aspect | 2D Frame | 3D Frame |
|---|---|---|
| DOF per node | 3 (ux, uy, rz) | 6 (ux, uy, uz, rx, ry, rz) |
| Stiffness size | 6×6 | 12×12 |
| Section properties | A, I | A, Iy, Iz, J (+ G for torsion) |
| Bending | single plane | biaxial (xz + yz planes) |
| Torsion | none | GJ/L term |
| Transformation | 2D rotation (c, s) | 3D rotation matrix (3×3) |
| End releases | 1 rotation per end | 3 rotations per end |
| Loads | qx, qy, mz | qx, qy, qz, mx, my, mz |
| Section forces | N, V, M | N, Vy, Vz, My, Mz, T |
| Rigid-body modes | 3 (Tx, Ty, Rz) | 6 (Tx, Ty, Tz, Rx, Ry, Rz) |
| Equilibrium equations | 3 | 6 |

### 15.2 Options

**Same type:** Impossible. Every method signature, return type, and field
would change. This is a complete API replacement, not an extension.

**Separate type (`FrameModel3D`, `BeamElement3D`, etc.):** No breaking
changes to 2D. Large code duplication for the element formulation, but the
solver infrastructure, mechanism diagnostics, and load combination
architecture are shared. The 3D frame module would be a new
`frame3d` (or `frame_3d`) module with its own types.

**Generic:** Extremely complex. The stiffness matrix size, section property
count, and load component count all change. Not worth the complexity.

**Defer:** 3D frame is a major undertaking (new element formulation, new
section properties, new load types, new result types, new support
vocabulary, new equilibrium checks). The 2D frame is mature and
well-tested. 3D frame should be a deliberate future project, not a
side-effect of this audit.

### 15.3 Recommendation

**Defer 3D Frame.** When undertaken, use separate types in a new module
(`frame3d`), sharing only `SparseMatrix`, `LinearSolver`,
`diagnose_reduced`, `FemError`, and the `LoadCombination` architecture.
The 2D frame API is locked in and should not change.

---

## 16. Prohibited Refactors (not performed)

The following were considered and explicitly rejected:

1. **No `Dof` trait / `Dof<N>` generic.** The DOF enum is a small, concrete
   type. A generic abstraction adds complexity for no benefit.

2. **No `Dimension` enum / const generic.** Two cases (2D, 3D) do not
   justify a dimension parameter.

3. **No `Result` trait.** `FrameAnalysisResult`, `BeamAnalysisResult`, and
   `TrussAnalysisResult` have different APIs tailored to their element
   types. A common trait would be either too narrow (no useful methods) or
   too broad (leaky abstraction).

4. **No `Element` trait.** `BeamElement` and `TrussElement` have different
   stiffness shapes, DOF counts, and result types. A common trait would
   not unify anything useful.

5. **No `Constraint` abstraction.** BCs are stored as raw `(node, dof,
   value)` tuples. A constraint hierarchy adds indirection without
   benefit.

6. **No crate split.** `structural-analysis` is a single crate with
   `publish = false`. 3D types would be additional modules in the same
   crate.

7. **No rewrite of 2D code.** The 2D implementation is mature, tested, and
   correct. 3D types will be additive, not replacement.

---

## 17. Findings

| Severity | Count | Description |
|---|---|---|
| P0 | 0 | — |
| P1 | 0 | — |
| P2 | 0 | — |

No production code changes. No issues found. The audit is purely
informational — it establishes the API lock-in contract for future 3D work.

---

## 18. Final Classification

```
P0/P1/P2:           0 / 0 / 0
3D Truss:            READY WITH LIMITED REFACTOR
                    (Option B: separate types, shared infrastructure,
                     ~100 lines of mechanical duplication, no breaking
                     changes to 2D API)
3D Frame:            NOT READY
                    (fundamental rewrite: 12×12 stiffness, biaxial
                     bending + torsion, 6 DOF/node, new section
                     properties; defer to a dedicated future phase)
Architecture:        KEEP
                    (no refactor needed for 2D; 3D should be separate
                     modules in the same crate)
Crate split:         NO
                    (single crate, separate modules for 3D)
API lock-in:         SAFE
                    (dimension-agnostic types survive 3D unchanged;
                     2D-specific types are locked in and will not
                     change; 3D types will be additive)
Implementation:      NONE
                    (audit-only phase, no production code changes)
Next phase:          TBD
                    (Phase 114: 3D Truss implementation if desired,
                     or feature development based on user needs)
```

---

## 19. API Lock-in Contract

The following **contract** binds future development:

1. **Dimension-agnostic types** (§13.1, ~20 types) shall not change their
   public API for 3D reasons. They are shared between 2D and 3D.

2. **2D-specific types** (§13.1, ~35 types) are frozen. 3D will introduce
   **new** types (`TrussModel3D`, `FrameModel3D`, etc.) in **new modules**
   (`truss3d`, `frame3d`). No existing 2D type shall be modified to
   accommodate 3D.

3. **Shared infrastructure** (`SparseMatrix`, `LinearSolver`,
   `SolverRegistry`, `diagnose_reduced`, `StructuralDiagnostic`,
   `FemError`, `LoadCombination` architecture) shall remain
   DOF-agnostic. No dimension-specific logic shall be added to these.

4. **No generic abstractions** (`Dof<N>`, `Element` trait, `Result` trait,
   `Constraint` trait, `Dimension` enum) shall be introduced. 3D types
   will be concrete, separate, and self-contained.

5. **`section-properties` v0.4.0** public API is frozen (published to
   crates.io). No changes for 3D reasons.

---

## 20. Verification

```
cargo fmt --all -- --check     ✓ pass
cargo check --workspace        ✓ pass (3 pre-existing pardiso warnings)
cargo test -p structural-analysis   ✓ (run separately)
cargo test --doc --workspace       ✓ (run separately)
cargo test --examples --workspace  ✓ (run separately)
cargo clippy --workspace --all-targets --all-features  ✓ (existing lints)
```

No production code was modified. All verification is against the unchanged
baseline `304a03f`.
