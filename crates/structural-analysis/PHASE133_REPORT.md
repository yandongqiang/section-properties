# Phase 133 — 3D Frame Element Foundation

**Date:** 2026-10-10  
**Implementer:** GLM-5.2  
**Base HEAD:** `66851b2` (Phase 132)  
**Status:** Complete — implementation + 30 tests, all passing  

## 1. Verified baseline

```
HEAD: 66851b2 Phase 132: audit 3D frame architecture readiness
```

Working tree: clean (only untracked audit docs from prior phases). No production code changes existed at baseline.

## 2. Files changed

| File | Status | Description |
|------|--------|-------------|
| `src/frame3d.rs` | **New** (549 lines) | `FrameSection3D`, `FrameElement3D`, 12×12 local stiffness, 12×12 transformation, global stiffness |
| `src/lib.rs` | Modified (+2 lines) | `pub mod frame3d;` + `pub use crate::frame3d::{FrameElement3D, FrameSection3D};` |
| `tests/frame3d.rs` | **New** (896 lines) | 30 tests: section validation (3), geometry (9), local stiffness (11), transformation (6) |

No existing APIs modified. No changes to `beam_fem.rs`, `frame.rs`, `truss.rs`, `truss3d.rs`, `load.rs`, `mechanism.rs`, or `postprocessing.rs`.

## 3. API and local-axis conventions

### `FrameSection3D`

```rust
pub struct FrameSection3D {
    pub E: f64,      // Young's modulus (Pa)
    pub G: f64,      // Shear modulus (Pa)
    pub area: f64,   // Cross-sectional area A (m²)
    pub iy: f64,     // Second moment about local y (m⁴)
    pub iz: f64,     // Second moment about local z (m⁴)
    pub j: f64,      // Saint-Venant torsional constant (m⁴)
}
```

Constructor `FrameSection3D::new(E, G, area, iy, iz, j)` validates all six parameters are finite and strictly positive. Returns `FemError::InvalidInput` on violation.

**Euler–Bernoulli assumption:** No shear areas (`Ay`, `Az`) — transverse shear deformation excluded. Documented in type doc.

**Torsion constant:** `j` is Saint-Venant torsional constant, *not* polar second moment. Documented explicitly. No silent substitution.

### `FrameElement3D`

```rust
pub struct FrameElement3D {
    pub node_i: usize,
    pub node_j: usize,
    pub section: FrameSection3D,
    pub ref_vec: [f64; 3],  // orientation reference vector
}
```

Constructor validates `node_i != node_j`, `ref_vec` finite and nonzero.

### Local DOF ordering

```text
[u_i, v_i, w_i, rx_i, ry_i, rz_i, u_j, v_j, w_j, rx_j, ry_j, rz_j]
  0    1    2     3     4     5    6    7    8     9    10    11
```

- `u` — axial (local x), `v` — transverse (local y), `w` — transverse (local z)
- `rx` — torsion (about x), `ry` — bending (about y), `rz` — bending (about z)

### Local-axis convention

1. **Local x** = `(p_j − p_i) / L` (from node i to node j).
2. **Local z** = `(x̂ × ref) / |x̂ × ref|`.
3. **Local y** = `ẑ × x̂`.

Right-handed orthonormal triple: `x̂ × ŷ = ẑ`.

**Parallel detection:** Scale-aware — checks `sin(angle) = |x̂ × ref| / |ref|` against tolerance `1e-8` (≈ 5.7×10⁻⁷ rad). Rejects with `FemError::InvalidInput`.

### Transformation convention

- `T` maps **global → local**: `u_local = T · u_global`
- `T` is block-diagonal with four 3×3 direction cosine blocks (translations and rotations transform identically)
- Global stiffness: `K_global = Tᵀ · K_local · T`
- Consistent with existing 2D convention in `beam_fem.rs`

## 4. Stiffness matrix derivation and reference

The 12×12 local stiffness follows the standard 3D Euler–Bernoulli frame element formulation from:

- **Bathe, K.-J.** *Finite Element Procedures*, 2nd ed. (2014), §5.6.
- **Cook, Malkus, Plesha** *Concepts and Applications of FEA*, 4th ed. (2002), §4.3.
- **Logan** *A First Course in the FEM*, 4th ed. (2007), §5.6.

### Structure

| Action | Stiffness | DOFs | Sign notes |
|--------|-----------|------|------------|
| Axial | `EA/L` | 0, 6 | Same as 2D |
| Torsion | `GJ/L` | 3, 9 | Same form as axial |
| Bending about z (x-y plane) | `EIz/L³` | 1, 5, 7, 11 | Same signs as 2D beam (`θz = +dv/dx`) |
| Bending about y (x-z plane) | `EIy/L³` | 2, 4, 8, 10 | **Opposite** coupling signs (`θy = −dw/dx` by right-hand rule) |

### Sign convention for y-bending

The 6·EIy/L² coupling terms have **opposite signs** compared to z-bending. This is because the right-hand rule gives `θy = −dw/dx` (rotation about y moves z toward x), whereas `θz = +dv/dx` (rotation about z moves y toward x). This was verified analytically: all 6 rigid-body modes produce `K·v = 0` to machine precision.

## 5. Numerical validation evidence

### Rigid-body modes (6 modes, K·v ≈ 0)

| Mode | Displacement vector | ‖K·v‖ |
|------|-------------------|--------|
| Translation x | `[1,0,0,0,0,0,1,0,0,0,0,0]` | < 1e-3 |
| Translation y | `[0,1,0,0,0,0,0,1,0,0,0,0]` | < 1e-3 |
| Translation z | `[0,0,1,0,0,0,0,0,1,0,0,0]` | < 1e-3 |
| Rotation x | `[0,0,0,1,0,0,0,0,0,1,0,0]` | < 1e-3 |
| Rotation y | `[0,0,0,0,1,0,0,0,-L,0,1,0]` | < 1e-3 |
| Rotation z | `[0,0,0,0,0,1,0,L,0,0,0,1]` | < 1e-3 |

### Strain energy invariance

For 6 test displacement vectors (including oblique orientation):
`½ u_localᵀ K_local u_local = ½ u_globalᵀ K_global u_global` to relative tolerance 1e-3.

### Transformation orthogonality

`T · Tᵀ = I` and `Tᵀ · T = I` to absolute tolerance 1e-13.

### Scaling verification

| Parameter | Scaling | Verified |
|-----------|---------|----------|
| E | Axial + bending ∝ E, torsion unchanged | ✅ |
| G | Only torsion ∝ G | ✅ |
| L | Axial ∝ 1/L, bending ∝ 1/L³ | ✅ |
| A | Axial ∝ A | ✅ |
| Iy | Bending y ∝ Iy | ✅ |
| Iz | Bending z ∝ Iz | ✅ |
| J | Torsion ∝ J | ✅ |

## 6. Test commands and outcomes

| Command | Result |
|---------|--------|
| `cargo fmt --all -- --check` | ✅ PASS |
| `cargo check -p structural-analysis` | ✅ PASS |
| `cargo test -p structural-analysis --test frame3d` | ✅ 30 passed, 0 failed |
| `cargo test -p structural-analysis` (all tests) | ✅ All passed, 0 failed |
| `cargo doc -p structural-analysis --no-deps` | ✅ 3 pre-existing private link warnings (truss3d.rs, unchanged) |
| `cargo clippy -p structural-analysis --lib` | ✅ 0 warnings from frame3d.rs (1 `type_complexity` suppressed with `#[allow]`) |
| `cargo clippy -p structural-analysis --test frame3d` | ✅ 0 warnings from frame3d tests |

### Existing test regression

All existing test suites pass unchanged:
- `beam_fem`: 22 tests ✅
- `frame_api`: 56 tests ✅
- `frame_correctness`: 19 tests ✅
- `truss`: 111 tests ✅
- `truss3d`: 95 tests ✅
- `end_release`: 25 tests ✅
- All other suites: ✅
- Doctests: 13 passed ✅

### Clippy note

`cargo clippy -p structural-analysis --all-targets -- -D warnings` fails with 133 errors in `section-properties` (dependency). This is **pre-existing lint debt** in the dependency, not introduced by this phase. Zero clippy warnings from `frame3d.rs` or `tests/frame3d.rs`.

## 7. Test inventory (30 tests)

### Section validation (3)
- `valid_parameters` — constructs valid section, checks field values
- `zero_and_negative_values` — rejects 0 and negative for all 6 parameters
- `nan_and_infinity` — rejects NaN, +inf, -inf for all 6 parameters

### Geometry and orientation (9)
- `axis_aligned_member` — member along global X, verifies local axes
- `member_along_each_global_axis` — X, Y (parallel rejection), Z
- `oblique_member` — (1,2,3) direction, verifies x_hat components
- `nontrivial_roll` — different ref_vec produces different y axis
- `zero_length_member` — rejects coincident nodes
- `non_finite_coordinates` — rejects NaN/inf coordinates
- `zero_orientation_vector` — rejects zero ref_vec at construction
- `parallel_orientation_vector` — rejects exact and near-parallel ref
- `orthonormal_right_handed_basis` — unit length, orthogonality, x×y=z

### Local stiffness (11)
- `dimensions_and_finiteness` — all 144 entries finite
- `symmetry` — k[i][j] = k[j][i] to relative tolerance
- `axial_coefficients` — exact EA/L at DOFs 0,6
- `torsion_coefficients` — exact GJ/L at DOFs 3,9
- `bending_z_coefficients_and_signs` — all 10 Iz terms with correct signs
- `bending_y_coefficients_and_signs` — all 10 Iy terms with opposite coupling signs
- `six_rigid_body_modes` — K·v ≈ 0 for all 6 modes
- `positive_non_rigid_stiffness` — uᵀKu > 0 for 7 non-rigid displacements
- `scaling_with_E` — axial/bending ∝ E, torsion unchanged
- `scaling_with_G` — only torsion ∝ G
- `scaling_with_length` — axial ∝ 1/L, bending ∝ 1/L³
- `scaling_with_area_and_inertias` — A, Iy, Iz, J scaling

### Transformation (6)
- `identity_aligned_element` — verifies R block structure for axis-aligned member
- `oblique_element` — verifies block-diagonal structure
- `orthogonality` — T·Tᵀ = I and Tᵀ·T = I
- `round_trip_transformation` — Tᵀ·T·u = u
- `transformed_stiffness_symmetry` — K_global symmetric
- `strain_energy_invariance` — U_local = U_global for 6 displacement vectors

## 8. Known limitations and remaining risks

1. **No global assembly** — element stiffness and transformation only; no model, solver, or result recovery.
2. **No end releases** — all joints rigid (no static condensation of released DOFs).
3. **No shear deformation** — pure Euler–Bernoulli (no Timoshenko shear areas).
4. **No distributed loads** — only element-level mechanics.
5. **No warping torsion** — only Saint-Venant torsion (GJ/L).
6. **Orientation degeneracy** — members parallel to ref_vec are rejected; caller must choose an appropriate ref_vec (e.g., global Z for horizontal members, global Y for vertical members).
7. **`type_complexity` suppressed** — `local_axes` return type `Result<([f64;3],[f64;3],[f64;3]), FemError>` triggers clippy::type_complexity; suppressed with `#[allow]` (consistent with engineering code style).

## 9. Readiness for Phase 134

**The element foundation is READY for Phase 134 global assembly.**

The following are in place:
- ✅ Validated 12×12 local stiffness matrix (symmetric, correct signs, 6 rigid-body modes)
- ✅ Validated 12×12 transformation matrix (orthogonal, strain-energy invariant)
- ✅ `global_stiffness` helper (Tᵀ·K_local·T)
- ✅ Section/material validation
- ✅ Geometry and orientation validation
- ✅ Comprehensive test coverage (30 tests)

Phase 134 scope: `FrameModel3D` (nodes, elements, supports) + `FrameSolver3D` (assembly, static condensation, solve) + analytical benchmarks (axial bar, torsion shaft, cantilever bending).
