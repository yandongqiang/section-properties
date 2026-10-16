# Phase 137 — 3D Frame Numerical Validation & API Audit

## HEAD

- Before: `9c04ef7` Phase 136: harden 3D frame validation
- After:  see commit below

## Audited files

- `crates/structural-analysis/src/frame3d.rs` (1785 lines) — full implementation
- `crates/structural-analysis/tests/frame3d.rs` — 112 tests (106 existing + 6 new)
- `crates/structural-analysis/src/lib.rs` — public exports

## API surface audited

| Type | Constructors | Mutators | Getters | Solver entry |
|------|-------------|----------|---------|-------------|
| `FrameSection3D` | `new()` ✅ | `set_E/G/area/iy/iz/j()` ✅ | `E/G/area/iy/iz/j()` ✅ | `validate()` ✅ |
| `FrameElement3D` | `new()` ✅ | — | `node_i/j/section/ref_vec()` ✅ | `local_stiffness/local_axes/global_stiffness` ✅ |
| `FrameNode3D` | `new()` ⚠️ | — | `id/x/y/z/coords()` ✅ | — |
| `FrameModel3D` | `new()` ✅ | `add_node/member/nodal_load/fix_dof/...` ✅ | `n_nodes/n_members/node/member()` ✅ | `solve()` ✅ |
| `FrameSolver3D` | `from_model()` ✅ | `set_solver()` ✅ | `model()` ✅ | `solve()` ✅ |
| `FrameAnalysisResult3D` | (solver-constructed) | — | `displacement/reaction/member_end_forces()` ✅ | — |

## Findings by severity

### P0 (Critical) — None

### P1 (High) — None

### P2 (Medium)

**P2-1: Missing full 6-direction equilibrium tests** *(fixed)*

The existing `phase134_two_member_L_frame` test only verified ΣFy and ΣMz. No test
checked all 6 global equilibrium equations (ΣFx, ΣFy, ΣFz, ΣMx, ΣMy, ΣMz).

**Fix**: Added 6 Phase 137 tests including `full_equilibrium_nodal_loads`,
`full_equilibrium_member_load`, `full_equilibrium_inclined_combined`, and
`strain_energy_local_equals_global` — all verify complete 6-equation equilibrium
with independently computed external force resultants.

**P2-2: `FrameNode3D::new()` does not validate coordinate finiteness**

`FrameNode3D::new(id, x, y, z)` accepts NaN/inf coordinates without validation,
unlike `FrameSection3D::new()` which validates all parameters. However, since
`FrameModel3D` fields are private (Phase 136), invalid nodes cannot enter a model
through the public API — `add_node()` validates coordinates. This is an API
inconsistency, not a security defect.

**Decision**: Not fixed — fixing would change public API behavior with no actual
safety benefit. Documented as a known limitation.

**P2-3: `FrameAnalysisResult3D` lacks `ref_vec` for member force transformation**

`member_end_forces` are in local coordinates, but the result does not store
`ref_vec` per member. A consumer cannot reconstruct `local_axes` or
`transformation_matrix` to transform end forces to global coordinates.

**Decision**: Not fixed — fixing requires public API expansion (adding
`member_ref_vecs` field). Per spec: "Avoid unnecessary data duplication, broad
public API expansion." Documented as a gap for future 3D viewport integration.

### P3 (Low)

**P3-1**: `FrameAnalysisResult3D` lacks solver diagnostics (condition number,
solver name, iteration count). Not needed for basic visualization.

**P3-2**: `FrameAnalysisResult3D` lacks member section properties (E, G, A, Iy,
Iz, J). Needed for stress visualization but not for displacement visualization.

**P3-3**: `add_member()` does not detect duplicate members (same node pair).
Duplicate members are structurally legal (parallel chords); behavior is correct
but undocumented.

**P3-4**: `add_member()` does not detect zero-length members (different nodes at
same coordinates). Detection is deferred to `from_model()` → `local_axes()`.
Acceptable design — zero length is geometric, not topological.

**P3-5**: `constrained_dofs` stores DOF as `usize` (0–5) rather than `Dof3D`
enum. Consumers must map manually.

## Independent formulation verification

### Local stiffness matrix (12×12)

All four action blocks verified against standard references (Bathe §5.6,
Cook §4.3, Logan §5.6):

| Action | DOFs | Formula | Status |
|--------|------|---------|--------|
| Axial | 0, 6 | `EA/L` | ✅ correct |
| Torsion | 3, 9 | `GJ/L` | ✅ correct |
| Bending z (x-y plane) | 1, 5, 7, 11 | `12EIz/L³, 6EIz/L², 4EIz/L, 2EIz/L` | ✅ correct |
| Bending y (x-z plane) | 2, 4, 8, 10 | Same coefficients, flipped coupling signs | ✅ correct |

The sign flip for bending about y (θy = −dw/dx by right-hand rule) is correctly
implemented: `k[2][4] = -6·EIy/L²` vs `k[1][5] = +6·EIz/L²`.

### Transformation

- `T` is block-diagonal with 4 copies of direction cosine `R` (rows = local axes). ✅
- `u_local = T · u_global` verified. ✅
- `K_global = Tᵀ · K_local · T` verified by `strain_energy_local_equals_global` test. ✅
- Orthogonality verified by existing `transformation::orthogonality` test. ✅

### Local axes

- `x̂ = (p_j − p_i)/L` ✅
- `ẑ = (x̂ × ref)/|x̂ × ref|` ✅
- `ŷ = ẑ × x̂` ✅
- Right-handed: `x̂ × ŷ = ẑ` verified mathematically. ✅

### Equivalent nodal loads

Uniform and linearly varying load formulas independently derived via
shape-function integration:

| Load type | Formula | Verification |
|-----------|---------|-------------|
| Uniform y | `qL/2, qL²/12, qL/2, -qL²/12` | ✅ standard |
| Uniform z | `qL/2, -qL²/12, qL/2, qL²/12` | ✅ sign-flipped |
| Linear y | `L(7qi+3qj)/20, L²(3qi+2qj)/60, ...` | ✅ derived by ∫q(x)·N(x)dx |
| Linear z | Same as LinearY with sign flip | ✅ |

### End-force recovery

`f_local = K_local · T · u_global − f_eq_local` — verified by
`end_force_equilibrium_at_free_node` test. ✅

### Reaction recovery

`R = K_original · u_global − f_global` — verified by 3 equilibrium tests
checking all 6 equations. ✅

## End-to-end benchmarks

Existing tests already cover all 8 scenarios from the spec:

| # | Scenario | Test | Analytical reference |
|---|----------|------|---------------------|
| 1 | Axial tension/compression | `phase134_axial_bar_displacement` | δ = PL/(EA) |
| 2 | Torsion | `phase134_torsion_displacement` | θ = TL/(GJ) |
| 3 | Cantilever bending xy | `phase134_cantilever_bending_xy` | δ = PL³/(3EIz) |
| 3 | Cantilever bending xz | `phase134_cantilever_bending_xz` | δ = PL³/(3EIy) |
| 4 | Fixed-fixed distributed | `phase135_fixed_fixed_uniform_y/z` | δ = qL⁴/(384EI) |
| 5 | Inclined member | `phase134_oblique_member_axial` | Projected axial |
| 6 | Multi-member frame | `phase134_two_member_L_frame` | FEM + equilibrium |
| 7 | Reaction equilibrium | `phase137::full_equilibrium_*` (new) | ΣF=0, ΣM=0 |
| 8 | Nonzero prescribed disp | `phase134_prescribed_nonzero_displacement` | δ = δ₀ + PL/(EA) |

**New tests added (6)**:

| Test | What it verifies |
|------|-----------------|
| `full_equilibrium_nodal_loads` | 6-equation equilibrium with 3D nodal force+moment |
| `full_equilibrium_member_load` | 6-equation equilibrium with distributed member load |
| `full_equilibrium_inclined_combined` | 6-equation equilibrium with inclined member + combined loads |
| `end_force_equilibrium_at_free_node` | Local end forces match applied loads at free node |
| `global_stiffness_symmetry_inclined` | K_global symmetric for oblique member |
| `strain_energy_local_equals_global` | U_local = U_global for arbitrary displacement |

## Constraint and solver behavior

| Scenario | Behavior | Test |
|----------|----------|------|
| Duplicate same-value constraint | Idempotent (no-op) | `phase136::fix_dof_idempotent_same_value` ✅ |
| Duplicate conflicting constraint | `Err(InvalidInput)` | `phase136::fix_dof_conflicting_values` ✅ |
| Out-of-bounds node in fix_dof | `Err(InvalidNode)` | `phase136::fix_dof_rejects_invalid_node` ✅ |
| Non-finite prescribed value | `Err(InvalidInput)` | `phase136::fix_dof_rejects_nan/inf_value` ✅ |
| Unconstrained model (mechanism) | `Err` from solver | `phase134_mechanism_singular` ✅ |
| Non-finite solution | `Err(SolverError)` | Checked in `solve()` ✅ |
| Empty model | `Err(InvalidModel)` | `phase136::solver_rejects_empty_model` ✅ |
| Reactions from K·u−f | Correct | `phase137::full_equilibrium_*` ✅ |

## Result API readiness for visualization

| Capability | Available | Source |
|-----------|-----------|--------|
| Node coordinates | ✅ | `node_coords` |
| Node displacements | ✅ | `displacements`, `displacement()` |
| Reactions | ✅ | `reactions`, `reaction()` |
| Support info | ✅ | `constrained_dofs` |
| Member connectivity | ✅ | `member_nodes` |
| Member end forces (local) | ✅ | `member_end_forces`, `member_end_forces()` |
| Member end forces (global) | ❌ P2-3 | Needs `ref_vec` or pre-computed transformation |
| Member section properties | ❌ P3-2 | Not stored in result |
| Solver diagnostics | ❌ P3-1 | Not stored in result |
| Stable node/member IDs | ✅ | Indices are stable within a solve |

**Verdict**: Ready for basic 3D displacement visualization. Needs P2-3 fix
(member `ref_vec` in result) for global-force visualization, and P3-2 for
stress visualization.

## Test commands and results

| Command | Result |
|---------|--------|
| `cargo fmt --all -- --check` | ✅ pass |
| `cargo check -p structural-analysis` | ✅ pass |
| `cargo test -p structural-analysis --release --test frame3d` | ✅ 112 passed, 0 failed |
| `cargo test -p structural-analysis --release` | ✅ all pass, 0 failed |
| `cargo doc -p structural-analysis --no-deps` | ✅ 3 pre-existing warnings (none from Phase 137) |
| `cargo clippy -p structural-analysis --all-targets -- -D warnings` | ✅ 0 warnings in frame3d.rs (pre-existing debt in other files) |

## Remaining limitations

1. **P2-2**: `FrameNode3D::new()` does not validate coordinates (no safety impact)
2. **P2-3**: `FrameAnalysisResult3D` lacks `ref_vec` for global force transformation
3. **P3-1**: No solver diagnostics in result
4. **P3-2**: No member section properties in result
5. **P3-3**: Duplicate members not documented
6. **P3-5**: `constrained_dofs` uses integer DOF, not `Dof3D` enum

## Readiness verdicts

**3D Frame numerical analysis**: ✅ Ready. All formulations independently verified,
all 6 equilibrium equations tested, no P0/P1 defects found.

**3D viewport integration**: ⚠️ Partially ready. Displacement visualization ready.
Global force visualization needs P2-3 fix. Stress visualization needs P3-2.

## Recommended next phase

Phase 138: Extend `FrameAnalysisResult3D` with `member_ref_vecs` field (P2-3)
to enable global end-force transformation for 3D visualization. Minimal API
expansion — one new `Vec<[f64; 3]>` field populated during `solve()`.
