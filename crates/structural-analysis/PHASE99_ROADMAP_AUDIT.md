# Phase 99 — Structural Analysis Roadmap / API Audit

## 1. Baseline

| Item | Value |
|------|-------|
| HEAD | `a1cfcb6` (Phase 98: fix cross-validation fixture paths) |
| Working tree | 4 staged PHASE*.md (审查文档，未 commit) |
| Workspace | `section-properties` v0.4.0 (crates.io), `structural-analysis` v0.1.0 (`publish = false`) |
| Dependency | `structural-analysis → section-properties` (单向，无循环) |
| `cargo fmt --all -- --check` | PASS |
| `cargo check --workspace --tests --examples` | PASS |
| `cargo test -p structural-analysis` | **493 tests, 0 failures** (34 test binaries) |

### structural-analysis public API

```
Beam:     BeamModel, BeamSolver, BeamElement, BeamNode, BeamSection, Dof, EndRelease, FemError, BeamAnalysis
Frame:    FrameModel, FrameSolver, FrameAnalysisResult, EquilibriumReport, NodeHandle, MemberHandle, StructuralDiagnostic
Load:     LoadCase, LoadCombination
```

### Source files

| File | Lines | Responsibility |
|------|-------|----------------|
| `beam_fem.rs` | 4253 | Core FEM: element, model, solver, assembly, recovery |
| `frame.rs` | 2010 | Frame façade: typed handles, solve, result, equilibrium |
| `load.rs` | 307 | LoadCase, LoadCombination |
| `mechanism.rs` | 578 | Structural diagnostics: rank probe, mechanism classification |
| `lib.rs` | 45 | Module declarations and re-exports |

## 2. Current structural-analysis Functional Boundary

### 2.1 Modeling

| Capability | Status | API |
|------------|--------|-----|
| 2D Euler-Bernoulli beam | ✅ | `BeamElement`, 3 DOF/node [Ux, Uy, Rz] |
| Nodes | ✅ | `FrameModel::add_node` → `NodeHandle` |
| Members | ✅ | `FrameModel::add_member` → `MemberHandle` |
| Material | ✅ | `section_properties::Material` (E, ν, ρ) |
| Cross-section | ✅ | `BeamSection::new(A, I)` |
| Coordinate transformation | ✅ | Local ↔ global via `transformation_matrix` |

### 2.2 Boundary Conditions

| Type | Status | Math Model | Applied At |
|------|--------|------------|------------|
| Fixed DOF | ✅ | Direct elimination (static condensation) | Solver (`from_model`) |
| Roller X/Y | ✅ | Same as Fixed (different DOF) | Solver |
| Pin (Ux+Uy) | ✅ | Two fixed DOFs | Solver |
| Inclined Roller | ✅ | Coordinate rotation → DOF elimination | Solver (`from_model`) |
| Spring Support | ✅ | Add k to K diagonal | Solver (`from_model`) |
| End Release (hinge) | ✅ | Element-level static condensation | Element (`released_global_stiffness`) |
| Prescribed displacement | ✅ | Fixed DOF with non-zero value | Solver |

### 2.3 Loads

| Type | Status | Scope | API |
|------|--------|-------|-----|
| Nodal force (Fx, Fy) | ✅ | Global | `nodal_load` |
| Nodal moment (Mz) | ✅ | Global | `nodal_moment` |
| Uniform distributed load | ✅ | Member-local | `member_udl` |
| Trapezoidal distributed load | ✅ | Member-local | `member_trapezoidal` |
| Point load on member | ✅ | Member-local | `member_point_load` |
| Applied moment at node | ✅ | Global | `nodal_moment` |

### 2.4 Load Cases & Combinations

| Feature | Status | API |
|---------|--------|-----|
| LoadCase | ✅ | `LoadCase::new`, owns loads |
| LoadCombination | ✅ | `LoadCombination::new`, `add_case` |
| Single case solve | ✅ | `solve_case` |
| Combination solve | ✅ | `solve_combination` (merged RHS, single factorize) |
| Load source tracking | ✅ | `result.load_source()` → None / "case:{name}" / "combination:{name}" |

### 2.5 Solver

| Feature | Status | Detail |
|---------|--------|--------|
| Dense Gaussian | ✅ | via `section_properties::fea::solver` |
| Skyline LDLᵀ | ✅ | via `section_properties::fea::solver` |
| Sparse LU | ✅ | via `section_properties::fea::solver` |
| CG | ✅ | via `section_properties::fea::solver` |
| ICCG | ✅ | via `section_properties::fea::solver` |
| Auto selection | ✅ | `SolverSelection::Auto` (capability-based) |
| Named selection | ✅ | `SolverSelection::Named("sparse_lu")` |

### 2.6 Results & Post-processing

| Feature | Status | API |
|---------|--------|-----|
| Displacements | ✅ | `result.displacement(node, Dof)` |
| Reactions | ✅ | `result.reaction(node, Dof)` |
| Member end forces (local) | ✅ | `result.member_end_forces(member)` |
| Member end forces (global) | ✅ | `result.member_end_forces_global(member)` |
| Section forces N/V/M | ✅ | `result.section_forces(member, xi)` |
| Force diagrams | ✅ | `result.element_force_diagram(member, n)` |
| Equilibrium report | ✅ | `result.equilibrium()` → `EquilibriumReport` |
| Solver name | ✅ | `result.solver_name()` |

### 2.7 Diagnostics

| Feature | Status | API |
|---------|--------|-----|
| Mechanism detection | ✅ | `StructuralDiagnostic` enum |
| Stable / RigidBodyMode / Mechanism / IllConditioned / Indeterminate | ✅ | `diagnose_reduced` |
| Automatic on solve failure | ✅ | `FrameSolver` attaches diagnosis |

## 3. Beam/Frame MVP Assessment

### Verdict: **已经足够冻结**

#### 建模完整性

用户可以表达典型 2D frame：nodes, members, fixed/roller/pin supports, springs, inclined rollers, end releases, nodal forces/moments, distributed loads (uniform + trapezoidal), point loads on members, load cases, load combinations. **无 MVP 缺口。**

#### 求解完整性

完整流程已实现：model → assemble K/f → apply BCs → solve → reactions → member forces → section forces → equilibrium → diagnostics. **无 MVP 缺口。**

#### API 完整性

- 一致：所有 fallible API 返回 `Result<T, FemError>`
- 可发现：typed handles (`NodeHandle`, `MemberHandle`), `Dof` enum
- 命名合理：`fix`, `pin`, `roller_x`, `spring`, `inclined_roller`, `member_udl`, `member_trapezoidal`
- 不暴露内部：`BeamSolver` fields are private, access via methods
- **无 MVP 缺口。**

#### 数值边界

| Edge case | Handled | Test |
|-----------|---------|------|
| Singular system | ✅ Mechanism detection | `mechanism_diagnostics` |
| Zero-length member | ✅ Rejected with `FemError` | `api_misuse` |
| Invalid material/section | ✅ Validated at construction | `beam_fem_contract` |
| Non-finite input | ✅ All load/support APIs validate finite | `api_robustness_audit` |
| Extreme stiffness (spring) | ✅ Finite + positive required | `spring_support` |
| Inclined roller zero direction | ✅ Rejected | `inclined_roller` |
| Releases + springs + inclined组合 | ✅ Tested | `spring_support`, `inclined_roller`, `end_release` |

**无 MVP 缺口。**

## 4. Element Architecture Assessment

### Current structure

```
BeamElement — 2D Euler-Bernoulli, 3 DOF/node, 6 DOF/element
    ↓
BeamModel — owns elements, assembles K/f, manages BCs
    ↓
BeamSolver — solves, recovers forces
    ↓
FrameModel — façade around BeamModel
```

### Future extension analysis

| Direction | Needs Element trait? | Current code duplication? |
|-----------|---------------------|------------------------|
| Truss (2 DOF/node) | No — different DOF count, different element | None — would be new code |
| Timoshenko beam | No — same DOF, different stiffness | Minimal — override `local_stiffness` |
| Plate/Shell | No — fundamentally different math | None — different crate scope |
| Solid 3D | No — different dimensionality | None — different crate scope |

**结论：不需要 Element trait 抽象。**

- `BeamElement` 承担的职责（local stiffness, transformation, end release condensation, load integration）是梁单元特有的，不与未来 Truss/Plate 共享。
- `BeamModel` 同时承担 model + assembly + BC management，但这是 2D beam-specific 的组装逻辑。未来 Truss 会有自己的组装（2 DOF/node），不会复用 beam 的 3 DOF 组装。
- 提前抽象 `Element` trait 会增加 indirection，约束未来 element 的 DOF 数量和数学模型，而不减少任何当前重复。
- **如果未来增加 Truss**，正确做法是新增 `TrussElement` + `TrussModel`（或泛化 `FrameModel` 以支持多种 element），而非重构现有 `BeamElement`。

## 5. Load Architecture Assessment

### Current design

```
LoadCase
├── nodal_forces: Vec<(node, dof, value)>
├── distributed_loads: Vec<DistributedLoad>  (trapezoidal: qx, qx_end, qy, qy_end)
├── point_loads: Vec<PointLoad>
└── applied_moments: Vec<AppliedMoment>

LoadCombination
└── terms: Vec<(LoadCase, factor)>
```

### Extension analysis

| Future load type | Extension path | Needs refactor? |
|-----------------|---------------|----------------|
| Thermal load | New collection in LoadCase | No |
| Self weight | Computed from member + material density | No |
| Moving load | Multiple LoadCases at different positions | No |
| Prescribed displacement | Already supported (fixed_dofs with value) | No |

**结论：Load API 自然扩展方向是新增 load collection 类型。** 当前设计无结构性问题，不需要重构。

## 6. Boundary Condition Architecture Assessment

Phase 94 已确认：4 种约束（Fixed/Roller, Inclined Roller, Spring, End Release）各有独立数学模型，无重复代码，不需要抽象层。此结论不变。

| Constraint | Math model | Unique? |
|------------|-----------|---------|
| Fixed/Roller | u_i = v (direct elimination) | Yes |
| Inclined Roller | n·u = v (coordinate rotation + elimination) | Yes |
| Spring | F = -k·u (diagonal addition) | Yes |
| End Release | M = 0 (element condensation) | Yes (member-level, not node-level) |

**结论：保持独立 API。不引入 Constraint trait。**

## 7. Result / Post-processing Assessment

### Current design

```
FrameAnalysisResult
├── beam: BeamSolver (owns K, f, u, model)
├── model: FrameModel (for typed access)
└── load_source: Option<String>
```

All post-processing methods delegate to `BeamSolver`:
- `displacement()` → reads `u_global`
- `reaction()` → computes `K_original · u - f`
- `member_end_forces()` → computes `f_equiv - K_local · u_local`
- `section_forces()` → equilibrium from end forces + load distribution
- `equilibrium()` → global force/moment balance

**无 solver 逻辑泄漏到 result。** Result 是纯读取层。Model 状态在 solve 后不可变。

### Future extension

| Feature | Extension path | Needs refactor? |
|---------|---------------|----------------|
| Envelope (max/min across cases) | New `EnvelopeResult` type wrapping multiple `FrameAnalysisResult` | No |
| Member diagrams (N/V/M plots) | Already partially supported via `element_force_diagram` | No |
| Deflection curve | Already available via `section_forces` + integration | No |

**结论：Result API 设计正确，不需要重构。** Envelope 是自然扩展方向。

## 8. Solver Boundary Assessment

```
section-properties
└── fea::solver (LinearSolver trait, 5 backends, SparseMatrix)
        ↑
        │ used by
        │
structural-analysis
├── beam_fem (assembles K, calls solver)
├── mechanism (probes K_ff rank)
└── frame (selects solver, orchestrates solve)
```

- Solver 独立于 Frame — `LinearSolver` trait 在 section-properties 中，不知道 Beam/Frame
- Future Truss 可复用同一 solver — 只需组装不同 K 矩阵
- Future Plate/Shell 可复用同一 solver — 同上
- **无 solver 耦合问题。**

## 9. section-properties Boundary Assessment

section-properties 导出：
- 截面计算：Section, SectionProperties, Geometry, Polygon, ...
- 材料：Material
- FEM 基础设施：SparseMatrix, LinearSolver, SolverRegistry, SolverSelection, ...
- 截面级 FEM：warping_fem, stress_fem, mesh, ...

**不导出** 任何结构分析类型（Frame, BeamModel, LoadCase 等）。

**结论：边界正确。section-properties 是纯截面库 + 共享数值基础设施。**

## 10. Third-Crate Assessment

Phase 95 已确认：不创建 `numerical-core` 第三 crate。理由：
1. structural-analysis 仍需 `Material` 和 `Point`（来自 section-properties）
2. 创建第三 crate 不改变 structural-analysis → section-properties 依赖
3. 只增加 workspace 复杂度，无实际收益

**此结论不变。**

## 11. Test Audit

### 测试覆盖矩阵

| Category | Test files | Tests | Coverage |
|----------|-----------|-------|----------|
| Core FEM | beam_fem, beam_fem_api_*, beam_fem_contract, beam_fem_convergence | ~150 | Element, assembly, solve, recovery |
| Analytical | beam_analytical_benchmarks, beam_section_forces, beam_force_diagrams | ~40 | Closed-form verification |
| Solver | beam_solver_backends, beam_multisolver_cross_validation, solver_robustness, solver_selection | ~50 | All 5 backends, auto-selection |
| Frame | frame_api, frame_correctness, frame_beam_integration, frame_transformation_contract | ~60 | FrameModel API, correctness |
| End Release | end_release | 19 | Hinges, pins, condensation |
| Spring | spring_support | 9 | Stiffness, equilibrium, combinations |
| Inclined Roller | inclined_roller | 9 | Direction, normalization, equilibrium |
| Loads | load_case, trapezoidal_load | 36 | Cases, combinations, trapezoidal |
| Diagnostics | mechanism_diagnostics | 7 | Stable, mechanism, ill-conditioned |
| API robustness | api_misuse, api_robustness_audit | ~30 | Invalid input, error behavior |
| Numerical audit | beam_fem_numerical_audit | ~50 | Edge cases, convergence |
| **Total** | **32 files** | **493** | **All features covered** |

### 测试质量评估

- ✅ 验证 public API contract（frame_api, beam_fem_api_*）
- ✅ 验证数学结果（beam_analytical_benchmarks, beam_fem_reference）
- ✅ 验证错误行为（api_misuse, api_robustness_audit）
- ✅ 验证多功能组合（spring + load, inclined + end release, trapezoidal + combination）
- ✅ 无 `#[ignore]` 测试
- ✅ 无 tautological 断言

**测试覆盖充分。**

## 12. Future Extension Analysis

| Direction | Code readiness | Prerequisite | Scale | API impact | Crate boundary | Worth now? |
|-----------|---------------|-------------|-------|-----------|---------------|-----------|
| A. API docs/examples | Ready | None | Small | No | No | **Yes** |
| B. Truss element | Moderate | None | Moderate | Add new types | No | **Yes** |
| C. Post-processing (diagrams, envelopes) | Ready | None | Moderate | Add new types | No | Yes |
| D. Timoshenko beam | Moderate | None | Moderate | Add option to BeamElement | No | Deferred |
| E. Plate/Shell | Low | Major refactor or new crate | Large | Large | Maybe | No |
| F. Solid 3D | Low | New crate | Large | Large | Yes | No |
| G. Solver robustness | Ready | None | Small | No | No | Deferred |
| H. Nonlinear/dynamic | Low | Major architecture | Large | Large | No | No |

### Immediate next phase

**A. API documentation & examples** — 最低风险、最高用户价值。当前 4 个 examples 覆盖基本用法，但缺少：
- Load case / combination example
- Spring / inclined roller example
- End release example
- Error handling guide

### Prerequisite phases

**B. Truss element** — 可直接开始，无需重构。新增 `TrussElement`（2 DOF/node）和 `TrussModel`，或扩展 `FrameModel` 支持混合 element。这是对 element 扩展性的第一次真实测试。

### Deferred directions

| Direction | Defer reason |
|-----------|-------------|
| D. Timoshenko beam | 当前 Euler-Bernoulli 足够用；Timoshenko 需要剪切修正系数，增加复杂度但用户需求未明确 |
| E. Plate/Shell | 完全不同的数学（板壳理论），需要新的 DOF、新的组装、新的后处理。应作为独立 crate 或重大架构扩展 |
| F. Solid 3D | 3D 应力分析，完全不同的领域。应作为独立 crate |
| G. Solver robustness | 当前 5 个后端已覆盖所有需求。进一步优化是 section-properties 的责任 |
| H. Nonlinear/dynamic | 需要完全不同的求解架构（增量/时间步）。当前线性静力 MVP 不应为此重构 |

## 13. Recommended Phase Sequence

```
Phase 100: API documentation & examples
    ↓  (无代码变更风险，提升可发现性)
Phase 101: Truss element
    ↓  (测试 element 扩展性，可能揭示需要轻量抽象)
Phase 102: Post-processing (envelopes, diagrams)
    ↓  (用户价值高，基于现有 Result API 自然扩展)
Phase 103: (视 101 结果决定) Element extensibility review
    ↓  (如果 Truss 暴露了真实重复，考虑轻量抽象)
```

**依赖关系说明：**
- Phase 100 → 101：文档先行，让 Truss 设计时能参考清晰的现有 API
- Phase 101 → 103：只有 Truss 实现后才能判断是否需要 Element 抽象
- Phase 102 独立于 101/103：后处理基于现有 Result，不受 element 类型影响

## 14. Risks

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Truss 实现后发现 BeamElement 有过多耦合 | Medium | Medium | Phase 103 审查后再决定重构 |
| 过早抽象 Element trait | Low | High | Phase 99 明确禁止提前抽象 |
| Post-processing API 膨胀 | Low | Low | 保持 on-demand 计算，不存储中间结果 |
| section-properties API 变更影响 structural-analysis | Low | Medium | version pinning (path v0.4) |
| 第三 crate 诱惑 | Low | Medium | Phase 95/99 两次确认不需要 |

## 15. Final Conclusion

### `structural-analysis` Beam/Frame MVP 冻结状态

**可以冻结。** 当前 2D Beam/Frame MVP 在建模、求解、API、数值边界、测试五个维度均无缺口。493 tests 覆盖所有功能，包括组合场景。

### 扩展前是否需要重构

**不需要。** 当前架构：
- Element 层无重复代码（只有一种 element）
- Load 层自然可扩展（新增 collection）
- BC 层独立无重复（4 种各有独立数学模型）
- Result 层纯读取（无 solver 泄漏）
- Solver 层在 section-properties（无耦合）

### 未来演进方向

沿当前 crate 边界继续演进：
1. **Phase 100**：API 文档与示例（最低风险）
2. **Phase 101**：Truss element（测试扩展性）
3. **Phase 102**：后处理（envelopes, diagrams）

**不应做：**
- 提前创建 Element trait
- 创建第三 crate
- 引入 Constraint 抽象层
- 进入 Plate/Shell/Solid/Nonlinear/Dynamic
- 将结构分析 API 移回 section-properties
