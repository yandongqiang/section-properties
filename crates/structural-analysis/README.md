# structural-analysis

2D structural analysis: beam and frame finite element analysis, solver
selection, and mechanism diagnostics.

This crate is part of the `section-properties` workspace and depends on
[`section-properties`](../section-properties) for cross-section properties,
materials, and numerical infrastructure (sparse solvers, FEA kernels).

## Features

- **Beam FEM** (`beam_fem`): 2D Euler–Bernoulli beam analysis — nodes and
  elements, distributed/point loads, applied moments, static-condensation
  boundary conditions, element end forces, section resultants
  `N(x)/V(x)/M(x)`, and pluggable `LinearSolver` backends
  (dense, skyline LDLᵀ, sparse LU, CG/ICCG).
- **Frame analysis** (`frame`): multi-member 2D frame analysis with the
  `FrameModel` façade — node/member handles, support vocabulary, **member end
  releases (hinges)**, global equilibrium reporting, and solver selection.
- **Mechanism diagnostics** (`mechanism`): rank-deficiency detection and
  mechanism classification for under-restrained structures.

## Quick start — frame analysis

```rust
use structural_analysis::{FrameModel, BeamSection, Dof};
use section_properties::Material;

# fn main() -> Result<(), Box<dyn std::error::Error>> {
let mut frame = FrameModel::new();
let base = frame.add_node(0.0, 0.0)?;
let tip = frame.add_node(2.0, 0.0)?;
frame.add_member(base, tip, Material::new(200e9, 0.3, 7850.0, "Steel"),
                  BeamSection::new(5e-3, 2e-5))?;
frame.fix(base)?;
frame.nodal_load(tip, 0.0, -1.0e4)?;

let result = frame.solve()?;
let uy = result.displacement(tip, Dof::Uy)?;
assert!((uy + 1.0e4 * 2.0f64.powi(3) / (3.0 * 200e9 * 2e-5)).abs() < 1e-12);
assert!(result.equilibrium().is_balanced());
# Ok(())
# }
```

## Member end releases (hinges)

A member end release removes the force-transfer between the member end and the
node for a specific DOF (typically rotation). This models hinges, pins, and
pinned connections.

**Key distinction**: an end release does *not* remove or constrain the node's
DOF — the node still has `Ux`, `Uy`, `Rz`. The release only affects how the
*member* contributes to the global stiffness. This is different from
`FrameModel::pin` which constrains a *node*.

```rust
use structural_analysis::{FrameModel, BeamSection, EndRelease, MemberHandle};
use section_properties::Material;

# fn main() -> Result<(), Box<dyn std::error::Error>> {
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
# Ok(())
# }
```

## Building and testing

Requires Rust 1.85+ (edition 2024).

```bash
cargo build
cargo test
```

## Documentation

Generate API docs with:

```bash
cargo doc --open
```

Beam FEM conventions (DOF ordering, coordinate systems, sign conventions,
load ownership, boundary conditions, model snapshot semantics, solver
selection and error behaviour) are documented in
[`docs/beam_fem.md`](../../docs/beam_fem.md). End-to-end usage is shown by the
examples in this crate:

```bash
cargo run --example beam_cantilever_tip_load
cargo run --example beam_cantilever_udl
cargo run --example beam_rotated_mixed_loading
cargo run --example frame_portal
```
