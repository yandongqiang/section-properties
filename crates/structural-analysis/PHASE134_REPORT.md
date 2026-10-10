# Phase 134 — 3D Frame Global Model, Assembly, and Solver

**Status**: Complete  
**Baseline**: `cd4f9ba` (Phase 133)  
**Date**: 2026-10-10  

## Summary

Implemented the full end-to-end linear-elastic 3D frame analysis pipeline on top
of the Phase 133 element foundation. The new code adds a global model
(`FrameModel3D`), a solver (`FrameSolver3D`) that assembles the global stiffness
matrix, applies boundary conditions by static condensation, and delegates the
linear solve to a `section-properties` backend, and an owned result snapshot
(`FrameAnalysisResult3D`) with displacement, reaction, and member-end-force
recovery.

No new crate, no public Element trait, and no changes to `section-properties`
public API.

## New public API

All types live in `crate::frame3d` and are re-exported from the crate root.

| Type | Purpose |
|------|---------|
| `Dof3D` | Enum: `Ux, Uy, Uz, Rx, Ry, Rz` with `index()`, `name()`, `ALL`. |
| `FrameNode3D` | Node with `id`, `(x, y, z)` coordinates. |
| `FrameModel3D` | Model: nodes, members, nodal forces, prescribed DOFs. |
| `FrameSolver3D` | Solver: assembles global K, applies BCs, solves. |
| `FrameAnalysisResult3D` | Owned snapshot: displacements, reactions, end forces. |

### `FrameModel3D` methods

| Method | Description |
|--------|-------------|
| `new()` / `default()` | Empty model. |
| `add_node(x, y, z)` | Add node, return index. |
| `add_member(i, j, section, ref_vec)` | Add validated `FrameElement3D`. |
| `add_nodal_load(node, fx, fy, fz, mx, my, mz)` | Accumulate global load. |
| `fix_dof(node, dof, value)` | Prescribe a single DOF. |
| `fix_node(node)` | Fix all 6 DOFs to zero (built-in). |
| `pin_node(node)` | Fix 3 translations to zero (pin). |
| `solve()` | Convenience: `FrameSolver3D::from_model(self)?.solve()`. |

### `FrameSolver3D` methods

| Method | Description |
|--------|-------------|
| `from_model(&model)` | Assemble K, F, apply BCs. |
| `set_solver(SolverSelection)` | Configure backend. |
| `solve()` | Solve and return owned `FrameAnalysisResult3D`. |
| `model()` | Borrow the underlying model. |

### `FrameAnalysisResult3D` methods

| Method | Description |
|--------|-------------|
| `displacement(node, Dof3D)` | Single DOF displacement. |
| `reaction(node, Dof3D)` | Single DOF reaction. |
| `member_end_forces(member)` | 12 local end forces `[fx_i,…,mz_j]`. |

## Mathematical formulation

### DOF mapping

6 DOF per node: `[Ux, Uy, Uz, Rx, Ry, Rz]`.  
Global index: `dof(node, d) = 6·node + d`.

### Global stiffness assembly

For each member, compute `K_global = Tᵀ · K_local · T` (12×12) and scatter into
the global `SparseMatrix` using the DOF map
`[6·i, 6·i+1, …, 6·i+5, 6·j, 6·j+1, …, 6·j+5]`.  
Shared-node contributions accumulate.

### Boundary conditions — static condensation

Partition DOFs into free (`f`) and constrained (`c`):

```
K_ff · u_f = F_f − K_fc · u_c
```

Non-zero prescribed displacements are accounted for in the right-hand side.
The reduced system is solved by `SolverRegistry` with `SolverSelection::Auto`
( configurable via `set_solver`).

### Reaction recovery

```
R = K_original · u − F_global
```

Reactions at free DOFs are zero by construction (equilibrium check).

### Member end forces

```
f_local = K_local · T · u_global
```

12 values per member in local coordinates. These are the forces that nodes
exert **on** the member (element end actions).

## Tests

**47 total** (30 Phase 133 + 17 Phase 134), all passing.

### Phase 134 analytical benchmarks (4)

| Test | Formula | Tolerance |
|------|---------|-----------|
| `phase134_axial_bar_displacement` | δ = PL/(EA) | 1e-10 rel |
| `phase134_torsion_displacement` | θ = TL/(GJ) | 1e-10 rel |
| `phase134_cantilever_bending_xy` | δ = PL³/(3EIz), θ = PL²/(2EIz) | 1e-10 rel |
| `phase134_cantilever_bending_xz` | δ = PL³/(3EIy), θ = −PL²/(2EIy) | 1e-10 rel |

Section: E=200 GPa, G=80 GPa, A=1e-3 m², Iy=2e-6 m⁴, Iz=1e-6 m⁴, J=3e-6 m⁴.  
Length: L=1 m. Loads: P=1 kN, T=100 N·m.

Each benchmark also verifies support reactions (force + moment).

### Phase 134 integration tests (13)

| Test | Description |
|------|-------------|
| `phase134_oblique_member_axial` | 45° member, axial load, displacement components. |
| `phase134_two_member_L_frame` | L-shaped frame, force + moment equilibrium. |
| `phase134_prescribed_nonzero_displacement` | Non-zero prescribed disp → rigid translation, zero reactions. |
| `phase134_fully_constrained` | All DOFs fixed, no loads → zero everything. |
| `phase134_mechanism_singular` | Unconstrained model → `Err`. |
| `phase134_no_load_zero_displacement` | Stable, no load → zero everything. |
| `phase134_solve_twice` | Repeated `solve()` gives identical results. |
| `phase134_member_end_forces_axial` | End forces: `[−P, …, +P]`. |
| `phase134_member_end_forces_bending` | End forces: `[−P, …, −PL, …, +P, …, 0]`. |
| `phase134_pin_node` | Pin support (translations only), mid-span load. |
| `phase134_solver_selection` | `Auto` vs `Dense` give same result. |
| `phase134_model_validation_errors` | Empty model, OOB node, OOB load, OOB fix. |
| `phase134_result_access_errors` | OOB displacement, reaction, member_end_forces. |

## Validation

| Check | Result |
|-------|--------|
| `cargo fmt --check` | Clean |
| `cargo check` | Clean, 0 warnings |
| `cargo test --test frame3d --release` | 47 passed, 0 failed |
| `cargo doc --no-deps` | 3 pre-existing warnings (none from `frame3d`) |
| `cargo clippy --no-deps` (frame3d.rs) | 0 warnings |

## Files changed

| File | Change |
|------|--------|
| `src/frame3d.rs` | +712 lines: `Dof3D`, `FrameNode3D`, `FrameModel3D`, `FrameSolver3D`, `FrameAnalysisResult3D` |
| `src/lib.rs` | +5 exports: `Dof3D`, `FrameNode3D`, `FrameModel3D`, `FrameSolver3D`, `FrameAnalysisResult3D` |
| `tests/frame3d.rs` | +500 lines: 17 new tests |
| `PHASE134_REPORT.md` | This report. |

## Out of scope (per spec)

- Distributed member loads
- Load cases / combinations
- End releases (hinges)
- Springs
- Nonlinear / buckling / dynamic analysis
- Post-processing: diagrams, envelopes, batch solve
- Shear deformation (Timoshenko)
