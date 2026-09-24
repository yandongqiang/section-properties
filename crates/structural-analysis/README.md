# structural-analysis

2D structural analysis: beam and frame finite element analysis, solver
selection, and mechanism diagnostics.

> **Workspace-only crate.** This crate is part of the `section-properties`
> workspace and has `publish = false`. It is not published to crates.io.
> Use it via the workspace, not as an independent dependency.

## Purpose

Provides linear static analysis of 2D Euler–Bernoulli beam and frame
structures. Built on top of
[`section-properties`](../section-properties) for cross-section properties,
materials, and numerical infrastructure (sparse matrix assembly, linear
solvers).

## Current scope

- **Elements**: 2D Euler–Bernoulli beam/frame element (3 DOF per node:
  `ux`, `uy`, `rz`) and 2D pin-jointed truss element (2 DOF per node:
  `ux`, `uy`)
- **Analysis**: linear static
- **Supports**: fixed, pinned, roller (x/y), inclined roller, spring,
  arbitrary DOF restraint
- **End releases**: hinge (rotation release) at either or both member ends
- **Loads**: nodal forces, nodal moments, uniform distributed loads,
  trapezoidal distributed loads, member point loads, applied moments
- **Load cases & combinations**: named load cases with linear combination
  factors
- **Results**: displacements, reactions, member end forces (local & global),
  section forces `N(x)`, `V(x)`, `M(x)`, force diagrams
- **Diagnostics**: mechanism detection, rigid-body mode classification,
  ill-conditioning reporting
- **Solvers**: dense Gaussian, skyline LDLᵀ, sparse LU, CG, ICCG (via
  `section-properties`)

**Not supported**: plate/shell/solid elements, nonlinear
analysis, dynamic analysis, buckling, design code combinations, envelope
generation.

## Relationship with `section-properties`

```text
section-properties          ← cross-section properties, materials, solvers
        ↑
        │
structural-analysis         ← structural model, elements, loads, solve
```

`section-properties` provides:

- section geometry and section properties (area, moments of inertia, etc.)
- material definitions
- FEA numerical infrastructure (`SparseMatrix`, `LinearSolver`, solver
  registry, solver selection)
- stress analysis, warping analysis, fire analysis, section database

`structural-analysis` provides:

- structural model (`BeamModel`, `FrameModel`)
- beam/frame elements and end releases
- structural loads (nodal, distributed, point, moment)
- supports and constraints (fixed, pin, roller, spring, inclined roller)
- load cases and load combinations
- structural solution and results (displacements, reactions, member forces)
- mechanism diagnostics

There is no circular dependency: `structural-analysis` depends on
`section-properties`, never the reverse.

## Installation / workspace usage

This crate is workspace-only. Add it to your workspace `Cargo.toml`:

```toml
[workspace]
members = ["crates/section-properties", "crates/structural-analysis"]
```

Then depend on it in your crate:

```toml
[dependencies]
structural-analysis = { path = "../structural-analysis" }
section-properties = { path = "../section-properties" }
```

## Quick start — cantilever beam

```rust
use structural_analysis::{FrameModel, BeamSection, Dof};
use section_properties::Material;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut frame = FrameModel::new();
    let base = frame.add_node(0.0, 0.0)?;
    let tip = frame.add_node(2.0, 0.0)?;
    frame.add_member(base, tip, Material::new(200e9, 0.3, 7850.0, "Steel"),
                     BeamSection::new(5e-3, 2e-5))?;
    frame.fix(base)?;
    frame.nodal_load(tip, 0.0, -1.0e4)?;

    let result = frame.solve()?;
    let uy = result.displacement(tip, Dof::Uy)?;
    // v = -P L^3 / 3EI
    assert!((uy + 1.0e4 * 2.0f64.powi(3) / (3.0 * 200e9 * 2e-5)).abs() < 1e-12);
    assert!(result.equilibrium().is_balanced());
    Ok(())
}
```

## Frame workflow

```rust
use structural_analysis::{FrameModel, BeamSection, Dof};
use section_properties::{Material, SolverSelection};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0)?;
    let b = frame.add_node(0.0, 3.0)?;
    let c = frame.add_node(4.0, 3.0)?;
    let d = frame.add_node(4.0, 0.0)?;

    let mat = Material::new(200e9, 0.3, 7850.0, "Steel");
    let sec = BeamSection::new(5e-3, 2e-5);
    let col_l = frame.add_member(a, b, mat, sec)?;
    let beam  = frame.add_member(b, c, mat, sec)?;
    let col_r = frame.add_member(c, d, mat, sec)?;

    frame.fix(a)?;
    frame.fix(d)?;
    frame.nodal_load(b, 20e3, 0.0)?;   // global lateral load
    frame.member_udl(beam, 0.0, -8e3)?; // local transverse UDL

    let result = frame.solve_with(SolverSelection::named("sparse_lu"))?;

    println!("sway: {:.4e} m", result.displacement(b, Dof::Ux)?);
    println!("beam end forces (local): {:?}",
             result.member_end_forces(beam)?);
    assert!(result.equilibrium().is_balanced());
    Ok(())
}
```

## Supported loads and supports

| Type | API | Coordinate system |
|------|-----|-------------------|
| Nodal force | `frame.nodal_load(node, fx, fy)` | Global |
| Nodal moment | `frame.nodal_moment(node, mz)` | Global (CCW positive) |
| Uniform distributed load | `frame.member_udl(member, qx, qy)` | Member local |
| Trapezoidal distributed load | `frame.member_trapezoidal(member, qx, qy, qx_end, qy_end)` | Member local |
| Member point load | `frame.member_point_load(member, xi, fx, fy, mz)` | Member local |
| Applied moment | `model.add_applied_moment(node, value)` | Global |

| Support | API | Constrained DOFs |
|---------|-----|------------------|
| Fixed (encastré) | `frame.fix(node)` | ux, uy, rz |
| Pinned | `frame.pin(node)` | ux, uy |
| Roller (free in x) | `frame.roller_x(node)` | uy, rz |
| Roller (free in y) | `frame.roller_y(node)` | ux, rz |
| Arbitrary restraint | `frame.restrain(node, dof, value)` | specified DOF |
| Spring | `frame.spring(node, dof, k)` | none (adds stiffness) |
| Inclined roller | `frame.inclined_roller(node, nx, ny, value)` | along (nx, ny) |

## Load cases and combinations

```rust
use structural_analysis::{FrameModel, LoadCase, LoadCombination, BeamSection};
use section_properties::Material;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0)?;
    let b = frame.add_node(4.0, 0.0)?;
    let m = frame.add_member(a, b, Material::new(200e9, 0.3, 7850.0, "Steel"),
                            BeamSection::new(5e-3, 2e-5))?;
    frame.fix(a)?;
    frame.fix(b)?;

    let mut dead = LoadCase::new("dead");
    dead.nodal_load(b, 0.0, -1000.0)?;
    dead.member_udl(m, 0.0, -500.0)?;

    let mut live = LoadCase::new("live");
    live.nodal_load(b, 0.0, -800.0)?;

    let mut combo = LoadCombination::new("1.4D + 1.6L");
    combo.add_case(&dead, 1.4)?;
    combo.add_case(&live, 1.6)?;

    let result = frame.solve_combination(&combo)?;
    println!("load source: {:?}", result.load_source());
    assert!(result.equilibrium().is_balanced());
    Ok(())
}
```

## Result extraction

```rust
# use structural_analysis::{FrameModel, BeamSection, Dof};
# use section_properties::Material;
# fn main() -> Result<(), Box<dyn std::error::Error>> {
# let mut frame = FrameModel::new();
# let a = frame.add_node(0.0, 0.0)?;
# let b = frame.add_node(2.0, 0.0)?;
# let m = frame.add_member(a, b, Material::new(200e9, 0.3, 7850.0, "Steel"),
#                         BeamSection::new(5e-3, 2e-5))?;
# frame.fix(a)?;
# frame.nodal_load(b, 0.0, -1e4)?;
# let result = frame.solve()?;
// Displacements (global)
let ux = result.displacement(b, Dof::Ux)?;
let uy = result.displacement(b, Dof::Uy)?;
let rz = result.displacement(b, Dof::Rz)?;

// Reactions (global)
let rx = result.reaction(a, Dof::Ux)?;
let ry = result.reaction(a, Dof::Uy)?;
let mz = result.reaction(a, Dof::Rz)?;

// Member end forces (local: [N_i, V_i, M_i, N_j, V_j, M_j])
let forces = result.member_end_forces(m)?;

// Section forces at midspan (local: N, V, M)
let sf = result.section_forces(m, 0.5)?;
println!("N={:.3}  V={:.3}  M={:.3}", sf.axial, sf.shear, sf.moment);

// Global equilibrium check
assert!(result.equilibrium().is_balanced());
# Ok(())
# }
```

## Member end releases (hinges)

A member end release removes the force-transfer between the member end and
the node for a specific DOF (typically rotation). This models hinges, pins,
and pinned connections.

**Key distinction**: an end release does *not* remove or constrain the node's
DOF — the node still has `Ux`, `Uy`, `Rz`. The release only affects how the
*member* contributes to the global stiffness. This is different from
`FrameModel::pin` which constrains a *node*.

```rust
use structural_analysis::{FrameModel, BeamSection, EndRelease, MemberHandle};
use section_properties::Material;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut frame = FrameModel::new();
    let a = frame.add_node(0.0, 0.0)?;
    let b = frame.add_node(4.0, 0.0)?;
    let m: MemberHandle = frame.add_member_with_release(a, b,
        Material::new(200e9, 0.3, 7850.0, "Steel"),
        BeamSection::new(5e-3, 2e-5),
        EndRelease::end_pin())?; // hinge at b
    frame.fix(a)?;
    frame.fix(b)?;
    frame.member_udl(m, 0.0, -1000.0)?;
    let result = frame.solve()?;
    let forces = result.member_end_forces(m)?;
    assert!(forces[5].abs() < 1.0); // M_j ≈ 0 (released)
    Ok(())
}
```

## Conventions

- **Global DOF**: `[ux, uy, rz]` per node; `rz` is counter-clockwise
  positive. Index mapping: `dof(node, d) = 3*node + d`.
- **Nodal loads**: in **global** coordinates.
- **Member loads** (distributed, point): in the member's **local**
  coordinate system (local x from `node_i` to `node_j`, local y transverse).
- **Applied moments**: in **global** coordinates (counter-clockwise positive).
- **Reactions**: `R = K_original · u - f_global` (global).
- **Member end forces**: `f_end = f_equiv - K_e · u_e` (element-on-node,
  local axes by default; global variant available).
- **Section forces**: `N` (tension positive), `V` (dM/dx), `M` (sagging
  positive).
- **Springs**: `reaction = -k · displacement`; stiffness added to the
  global diagonal, DOF remains free.
- **Inclined rollers**: `(nx, ny)` is the constrained direction
  (auto-normalized); the orthogonal direction is free.
- **End releases**: remove the moment transfer between member end and node;
  the node's rotational DOF is **not** removed from the global system.

## Limitations

- Only 2D (planar) analysis.
- Only Euler–Bernoulli beam kinematics (no shear deformation).
- Parallel members between the same node pair are rejected.
- A single connected structural system is required.
- Workspace-only (`publish = false`), not on crates.io.

## Building and testing

Requires Rust 1.85+ (edition 2024).

```bash
cargo build
cargo test -p structural-analysis
cargo doc -p structural-analysis --open
```

## Examples

```bash
cargo run --example beam_cantilever_tip_load
cargo run --example beam_cantilever_udl
cargo run --example beam_rotated_mixed_loading
cargo run --example frame_portal
cargo run --example frame_load_combination
cargo run --example frame_advanced_supports
cargo run --example truss_basic
```
