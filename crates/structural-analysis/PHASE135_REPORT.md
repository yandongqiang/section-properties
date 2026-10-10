# Phase 135: 3D Frame Member Loads and Equivalent Nodal Loads

## Overview

Phase 135 extends the 3D frame analysis (Phase 134) with distributed member
loads. Five load types are supported: uniform axial, uniform transverse y,
uniform transverse z, linearly varying transverse y, and linearly varying
transverse z. All load intensities are in **local** member coordinates.

The equivalent nodal loads are derived from the Euler–Bernoulli consistent
load formulation (shape-function integration). The global load vector is
augmented with `Tᵀ · f_eq_local` for each loaded member. Member end-force
recovery is corrected to `f_local = K_local · u_local − f_eq_local`.

## API additions

### `FrameMemberLoad` enum

```rust
pub enum FrameMemberLoad {
    UniformAxial { member_idx: usize, q_x: f64 },
    UniformY     { member_idx: usize, q_y: f64 },
    UniformZ     { member_idx: usize, q_z: f64 },
    LinearY      { member_idx: usize, q_i: f64, q_j: f64 },
    LinearZ      { member_idx: usize, q_i: f64, q_j: f64 },
}
```

Methods:
- `member_idx(&self) -> usize`
- `equivalent_nodal_loads_local(&self, L: f64) -> [f64; 12]`

### `FrameModel3D` methods

- `add_uniform_axial(&mut self, member_idx, q_x) -> Result<(), FemError>`
- `add_uniform_y(&mut self, member_idx, q_y) -> Result<(), FemErrorD>`
- `add_uniform_z(&mut self, member_idx, q_z) -> Result<(), FemError>`
- `add_linear_y(&mut self, member_idx, q_i, q_j) -> Result<(), FemError>`
- `add_linear_z(&mut self, member_idx, q_i, q_j) -> Result<(), FemError>`

All validate `member_idx` bounds and intensity finiteness. Multiple calls
accumulate.

### `FrameModel3D` field

- `pub member_loads: Vec<FrameMemberLoad>`

### `FrameSolver3D` internal

- `member_equiv_loads: Vec<[f64; 12]>` — per-member accumulated local
  equivalent loads, used for end-force recovery.

## Mathematical formulation

### Local DOF ordering

```
[u_i, v_i, w_i, rx_i, ry_i, rz_i, u_j, v_j, w_j, rx_j, ry_j, rz_j]
  0    1    2     3     4     5    6    7    8     9    10    11
```

### Sign conventions

- **Bending about z** (local y load): `θ_z = +dv/dx`. Consistent moments:
  `+qL²/12` at i, `−qL²/12` at j.
- **Bending about y** (local z load): `θ_y = −dw/dx`. Consistent moments:
  `−qL²/12` at i, `+qL²/12` at j (signs flipped due to right-hand rule).

### Equivalent nodal load vectors

**Uniform axial** (`q_x`):
```
f[0] = f[6] = q_x · L / 2
```

**Uniform y** (`q_y`):
```
f[1] = f[7] = q_y · L / 2
f[5] = +q_y · L² / 12
f[11] = −q_y · L² / 12
```

**Uniform z** (`q_z`):
```
f[2] = f[8] = q_z · L / 2
f[4] = −q_z · L² / 12
f[10] = +q_z · L² / 12
```

**Linear y** (`q_i` at x=0, `q_j` at x=L):
```
f[1]  = L · (7·q_i + 3·q_j) / 20
f[5]  = L² · (3·q_i + 2·q_j) / 60
f[7]  = L · (3·q_i + 7·q_j) / 20
f[11] = −L² · (2·q_i + 3·q_j) / 60
```

**Linear z** (`q_i` at x=0, `q_j` at x=L): same as Linear y for forces,
moments sign-flipped:
```
f[2]  = L · (7·q_i + 3·q_j) / 20
f[4]  = −L² · (3·q_i + 2·q_j) / 60
f[8]  = L · (3·q_i + 7·q_j) / 20
f[10] = L² · (2·q_i + 3·q_j) / 60
```

### Global assembly

For each member load:
1. Compute `f_eq_local` (12-vector, local coordinates).
2. Accumulate into `member_equiv_loads[mid]` for end-force recovery.
3. Transform to global: `f_eq_global = Tᵀ · f_eq_local`.
4. Scatter into `f_global[dof_map[a]] += f_eq_global[a]`.

### Member end-force recovery

```
f_local = K_local · (T · u_global) − f_eq_local
```

The `f_eq_local` correction accounts for distributed loads. Without member
loads, `f_eq_local = 0` and this reduces to the Phase 134 formula.

### Reaction recovery

Unchanged from Phase 134: `R = K·u − f_global`. Since `f_global` already
includes the equivalent nodal loads, no modification is needed.

## Analytical benchmarks

All benchmarks use E=200 GPa, G=80 GPa, A=1e-3 m², Iy=2e-6 m⁴, Iz=1e-6 m⁴,
J=3e-6 m⁴, L=1.0 m, q=1 kN/m. Member along global x with ref=[0,1,0]
(local y = global y, local z = global z).

### Cantilever (fixed at node 0, single element — FEM exact)

| Load | δ_tip | θ_tip | R_fixed | M_fixed |
|------|-------|-------|---------|---------|
| Uniform q_x | qL²/(2EA) | — | R_x = −qL | — |
| Uniform q_y | qL⁴/(8EIz) | qL³/(6EIz) | R_y = −qL | M_z = −qL²/2 |
| Uniform q_z | qL⁴/(8EIy) | −qL³/(6EIy) | R_z = −qL | M_y = +qL²/2 |
| Linear q_y (q,0) | qL⁴/(30EIz) | qL³/(24EIz) | R_y = −qL/2 | M_z = −qL²/6 |
| Linear q_z (q,0) | qL⁴/(30EIy) | −qL³/(24EIy) | R_z = −qL/2 | M_y = +qL²/6 |

### Fixed-fixed beam (uniform q_y)

- All displacements zero.
- R_y = −qL/2 at each end.
- M_z = −qL²/12 at i, +qL²/12 at j.
- Member end forces = −f_eq (since u = 0).

### Simply supported beam (uniform q_y, pinned both ends)

- θ_i = qL³/(24EIz), θ_j = −qL³/(24EIz).
- R_y = −qL/2 at each end.
- Note: Rx fixed at both ends to prevent torsional rigid-body mode.

### Oblique cantilever (45° inclined, uniform local q_y)

- Local y = global z for this orientation.
- Tip displacement in global z = qL⁴/(8EIz).
- Global reactions: R_x = 0, R_y = 0, R_z = −qL.
- M_x = −qL²/(2√2), M_y = qL²/(2√2), M_z = 0.

## Test summary

24 new tests in `tests/frame3d.rs`, organized by spec section:

### A. Equivalent nodal load vectors (local) — 6 tests
- `phase135_equiv_load_uniform_axial` — f[0]=f[6]=qL/2
- `phase135_equiv_load_uniform_y` — f[1]=f[7]=qL/2, f[5]=+qL²/12, f[11]=−qL²/12
- `phase135_equiv_load_uniform_z` — f[2]=f[8]=qL/2, f[4]=−qL²/12, f[10]=+qL²/12
- `phase135_equiv_load_linear_y` — exact consistent vector for q_i, q_j
- `phase135_equiv_load_linear_z` — same with moment signs flipped
- `phase135_equiv_load_member_idx_and_zero` — member_idx() accessor, zero load

### B. Cantilever benchmarks — 5 tests
- `phase135_cantilever_uniform_axial` — δ_x = qL²/(2EA), R_x = −qL
- `phase135_cantilever_uniform_y` — δ_y, θ_z, R_y, M_z
- `phase135_cantilever_uniform_z` — δ_z, θ_y, R_z, M_y
- `phase135_cantilever_linear_y` — triangular load, δ_y = qL⁴/(30EIz)
- `phase135_cantilever_linear_z` — triangular load, z plane

### C. Fixed-fixed beam — 2 tests
- `phase135_fixed_fixed_uniform_y` — reactions + member end forces
- `phase135_fixed_fixed_uniform_z` — reactions (z plane)

### D. Simply supported beam — 1 test
- `phase135_simply_supported_uniform_y` — exact rotations

### E. Oblique member — 1 test
- `phase135_oblique_cantilever_uniform_y` — 45° member, equilibrium

### F. End-force recovery — 2 tests
- `phase135_end_forces_cantilever_y` — f_local matches reactions
- `phase135_end_forces_cantilever_z` — z plane

### G. Edge cases and regression — 7 tests
- `phase135_zero_member_load_regression` — no member loads → Phase 134 behavior
- `phase135_multiple_additive_loads` — q + 2q = 3q
- `phase135_invalid_member_idx` — out-of-bounds rejected
- `phase135_nan_intensity_rejected` — NaN/infinity rejected
- `phase135_repeated_solve` — deterministic results
- `phase135_mixed_nodal_and_member_load` — superposition
- `phase135_both_planes_and_scale` — q_y + q_z simultaneously, L=2.0

## Quality gates

| Gate | Status |
|------|--------|
| `cargo fmt --all -- --check` | ✅ Clean |
| `cargo check -p structural-analysis` | ✅ 0 warnings |
| `cargo test -p structural-analysis --release --test frame3d` | ✅ 71/71 passed |
| `cargo doc -p structural-analysis --no-deps` | ✅ 3 pre-existing warnings (unrelated) |
| `cargo clippy -p structural-analysis --tests` | ✅ 0 warnings in frame3d.rs/tests |

## Files modified

- `crates/structural-analysis/src/frame3d.rs` — `FrameMemberLoad` enum,
  `member_loads` field, 5 `add_*` methods, `validate_member_load` helper,
  `member_equiv_loads` in solver, `from_model` equivalent load integration,
  `solve()` end-force correction.
- `crates/structural-analysis/src/lib.rs` — export `FrameMemberLoad`.
- `crates/structural-analysis/tests/frame3d.rs` — 24 Phase 135 tests.
- `crates/structural-analysis/PHASE135_REPORT.md` — this report.

## Limitations

- No distributed moments (only distributed forces).
- No member interior point loads.
- No temperature loads or prestress.
- No self-weight automation.
- No nonlinear effects.
- Shear deformation (Timoshenko) not included (Euler–Bernoulli only).
