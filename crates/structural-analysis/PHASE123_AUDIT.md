# Phase 123 — Cross-Model LoadCase / LoadCombination Contract Audit

## 审计基线

- **HEAD**: `18aee82` (Phase 122: add load cases and combinations to 2D truss)
- **Working tree**: clean (only untracked historical audit docs)
- **Phase 122 diff**: 4 files, +770 lines (load.rs +60, truss.rs +133, tests/truss.rs +418, PHASE122_REPORT.md +159)
- **Date**: 2026-10-09

## 审计 A：跨模型工况 API 对照表

### 模型能力矩阵

| 能力 | 2D Truss | 3D Truss | Beam | Frame |
|------|----------|----------|------|-------|
| LoadCase | ✅ Phase 122 | ✅ Phase 117 | ❌ (用 Frame) | ✅ |
| LoadCombination | ✅ Phase 122 | ✅ Phase 117 | ❌ (用 Frame) | ✅ |
| solve_case | ✅ TrussModel | ✅ TrussModel3D | ❌ | ✅ FrameModel + PreparedFrameAnalysis |
| solve_combination | ✅ TrussModel | ✅ TrussModel3D | ❌ | ✅ FrameModel + PreparedFrameAnalysis |
| solve_cases (批量) | ❌ | ❌ | ❌ | ✅ PreparedFrameAnalysis |
| load_source 字段 | ✅ TrussAnalysisResult | ✅ TrussAnalysisResult3D | ❌ | ✅ FrameAnalysisResult |
| Envelope | ❌ | ✅ Phase 120 | ✅ | ✅ |
| 自重 | ❌ (无 density) | ❌ (无 density) | ❌ | ✅ |

### 荷载类型支持

| 荷载类型 | 2D Truss | 3D Truss | Beam | Frame |
|----------|----------|----------|------|-------|
| 节点力 (Fx, Fy) | ✅ nodal_load_2d | ✅ nodal_load_3d (含 Fz) | ✅ add_nodal_force | ✅ nodal_load (NodeHandle) |
| 节点弯矩 | ❌ | ❌ | ✅ | ✅ nodal_moment |
| 分布荷载 | ❌ | ❌ | ✅ | ✅ member_udl/trapezoidal |
| 集中荷载 | ❌ | ❌ | ✅ | ✅ member_point_load |
| 支座沉降 | ✅ prescribed_displacement_2d | ✅ prescribed_displacement_3d | ❌ | ✅ prescribed_displacement |

### 求解入口语义

| 模型 | solve_case 语义 | solve_combination 语义 | 直接 solve 语义 |
|------|-----------------|----------------------|-----------------|
| 2D Truss | F = F_case (模型荷载忽略) | F = Σ factor_i × F_case_i | F = F_model |
| 3D Truss | F = F_case (模型荷载忽略) | F = Σ factor_i × F_case_i | F = F_model |
| Frame | F = F_case (模型荷载忽略) | F = Σ factor_i × F_case_i | F = F_model |

**结论**: 所有模型的 `solve_case`/`solve_combination` 语义一致：忽略模型直接荷载，仅使用工况/组合荷载。

### 合理的模型差异

1. **Beam 无 LoadCase**: Beam 是底层 API，Frame 是建立在 Beam 之上的高层门面。LoadCase/LoadCombination 工作流通过 Frame 提供。Beam 提供 `assemble_global_load_vector` 作为 `pub(crate)` 辅助函数供 Frame 使用。**合理差异**。

2. **2D/3D Truss 无 solve_cases (批量)**: Frame 的 `PreparedFrameAnalysis` 通过预分解刚度矩阵实现批量求解。Truss 没有等价的 PreparedAnalysis。**合理差异** — 新增批量求解是新功能，非缺陷。

3. **Truss 无分布荷载/弯矩/自重**: Truss 只有轴向刚度，不支持横向荷载。**合理差异**。

4. **2D Truss 无 Envelope**: 3D Truss 和 Frame 有 Envelope，2D Truss 尚未实现。**合理差异** — 可作为未来阶段。

## 审计 B：荷载组装与重复施加

### 荷载隔离验证

| 模型 | 隔离方式 | 验证结果 |
|------|----------|----------|
| 2D Truss | `model.nodal_forces.clear()` + `extend_from_slice(case.nodal_forces())` | ✅ 正确隔离 |
| 3D Truss | 同上 | ✅ 正确隔离 |
| Frame | `inner_with_loads()` 替换所有荷载向量 | ✅ 正确隔离 |

### 组合系数应用

| 模型 | 系数应用方式 | 验证结果 |
|------|-------------|----------|
| 2D Truss | `model.nodal_forces.push((node, dof, factor * value))` | ✅ 系数恰好应用一次 |
| 3D Truss | 同上 | ✅ |
| Frame | 对所有荷载类型 (nodal, distributed, point, moment) 应用系数 | ✅ |

### 状态污染验证

- **2D Truss**: `solve_case` 克隆 `self`，不修改原模型。多次求解不同工况互不污染。✅
- **3D Truss**: 同上。✅
- **Frame**: `inner_with_loads` 克隆 `self.inner`，不修改原模型。✅
- **结果独立性**: 所有模型返回 owned 结果对象，后续求解不会修改前一次结果。✅

**结论**: 无荷载重复施加或状态污染问题。

## 审计 C：约束与指定支座位移

### 约束处理

| 模型 | 固定约束 | 支座沉降 | 组合拒绝沉降 |
|------|----------|----------|-------------|
| 2D Truss | `fixed_dofs` + 静力凝缩 | 覆盖或新增 `fixed_dofs` 条目 | ✅ |
| 3D Truss | 同上 | 同上 | ✅ |
| Frame | `fixed_dofs` + 静力凝缩 | `try_override_dof` + 弹簧/斜辊冲突检查 | ✅ |

### P1 缺陷：2D Truss DOF 索引未验证

**文件**: `crates/structural-analysis/src/truss.rs`
**触发条件**: 创建 `LoadCase` 时使用 `nodal_load_3d` 或 `prescribed_displacement_3d`（或 Frame 的 `prescribed_displacement` with `Dof::Rz`），然后传入 2D Truss 的 `solve_case`/`solve_combination`。
**技术原因**: `LoadCase` 是共享类型，`nodal_load_3d` 存储 `(node, 2, fz)` — DOF 2 在 3D Truss 中是 uz，但在 2D Truss 中越界。2D Truss 的 `solve_case`/`solve_combination` 仅验证节点索引，不验证 DOF 索引。
**影响**:
- **Panic 场景**: 末节点 + dof=2 → `dof_index = node*2+2 = 2*n_nodes`，越界 panic
- **静默错误场景**: 非末节点 + dof=2 → `dof_index` 映射到下一节点的 ux DOF，静默约束/加载错误 DOF

**修复**: 在 `solve_case` 和 `solve_combination` 中添加 `dof >= 2` 验证，返回 `FemError::InvalidInput`。

**3D Truss 不受影响**: 3 DOF/节点，最大 DOF=2 始终有效。
**Frame 不受影响**: 3 DOF/节点，最大 DOF=2 始终有效。

## 审计 D：结果来源与后处理

### LoadSource 设置

| 模型 | solve_case | solve_combination | 直接 solve |
|------|-----------|-------------------|-----------|
| 2D Truss | `LoadSource::LoadCase` | `LoadSource::LoadCombination` | `LoadSource::ModelLoads` |
| 3D Truss | 同上 | 同上 | 同上 |
| Frame | 同上 | 同上 | 同上 |

**结论**: 所有模型的 `load_source` 设置一致且正确。

### 来源标识问题检查

- **名称冲突**: LoadCase 名称由用户提供，不强制唯一。不同工况同名不会导致数值错误，但可能影响后处理可追溯性。**设计决策** — 不强制唯一性，与 Frame 一致。
- **空字符串**: `LoadCase::new("")` 允许空名称。不会导致歧义（`LoadSource::LoadCase { name: "" }` 仍可区分于 `ModelLoads`）。**P3** — 非关键。
- **组合标记为工况**: 不可能 — `solve_case` 设置 `LoadSource::LoadCase`，`solve_combination` 设置 `LoadSource::LoadCombination`，类型不同。✅
- **Envelope 跨模型混合**: 2D Truss 无 Envelope。3D Truss Envelope (`Truss3DEnvelope`) 和 Frame Envelope (`Envelope`) 是不同类型，不会跨模型混合。✅

## 审计 E：线性组合数学正确性

### 线性叠加验证

所有模型在一致边界条件下满足：

`u_combination ≈ Σ(factor_i × u_i)`

| 模型 | 验证方式 | 结果 |
|------|----------|------|
| 2D Truss | Phase 122 Test 3: 1.4×D + 1.6×L 与手动叠加比较 | ✅ 容差 < 1e-9 |
| 3D Truss | Phase 117 Test 24: 组合等价性验证 | ✅ |
| Frame | 既有 multi_load_case 测试 | ✅ |
| Beam | N/A (无 LoadCase) | N/A |

**前提条件**: 一致刚度矩阵、一致边界条件、无支座沉降在组合中。所有模型均拒绝组合中的支座沉降。✅

## 审计 F：公共 API 与兼容性

### Phase 122 新增方法审查

| 方法 | 命名一致性 | 错误处理 | 文档 |
|------|-----------|----------|------|
| `LoadCase::nodal_load_2d` | ✅ 与 `nodal_load_3d` 一致 | ✅ `FemError::InvalidInput` | ✅ |
| `LoadCase::prescribed_displacement_2d` | ✅ 与 `prescribed_displacement_3d` 一致 | ✅ DOF + 有限性验证 | ✅ |
| `TrussModel::solve_case` | ✅ 与 `TrussModel3D::solve_case` 一致 | ✅ | ✅ |
| `TrussModel::solve_combination` | ✅ 与 `TrussModel3D::solve_combination` 一致 | ✅ | ✅ |
| `TrussAnalysisResult::load_source()` | ✅ 与 `TrussAnalysisResult3D::load_source()` 一致 | N/A | ✅ |

### 向后兼容性

- `add_nodal_force` 和既有 `TrussSolver` API 完全保留。✅
- `TrussAnalysisResult` 新增 `load_source` 字段为 private，不影响既有代码。✅
- 无公共方法签名变更。✅

## P0–P3 问题与证据

| 级别 | 问题 | 文件 | 状态 |
|------|------|------|------|
| P0 | 0 | — | — |
| P1 | 2D Truss solve_case/solve_combination 不验证 DOF 索引，跨模型误用可致 panic 或静默错误 | truss.rs:375,439 | **已修复** |
| P2 | 0 | — | — |
| P3 | LoadCase 允许空字符串名称 | load.rs:61 | 设计决策，不修复 |

## 新增测试

| 测试名 | 验证内容 |
|--------|----------|
| p123_solve_case_rejects_3d_dof_in_nodal_load | solve_case 拒绝 nodal_load_3d 的 dof=2，返回 InvalidInput |
| p123_solve_case_rejects_3d_dof_in_prescribed_displacement | solve_case 拒绝 prescribed_displacement_3d 的 dof=2，返回 InvalidInput |
| p123_solve_combination_rejects_3d_dof | solve_combination 拒绝 nodal_load_3d 的 dof=2，返回 InvalidInput |
| p123_solve_combination_rejects_3d_dof_in_second_case | 多工况组合中第二个工况有非法 DOF，返回 InvalidInput |
| p123_valid_dofs_0_and_1_unaffected | 有效 DOF 0 和 1 不受影响，数值结果正确 |

## 验证命令与结果

| 检查 | 结果 |
|------|------|
| `cargo fmt --all -- --check` | ✅ PASS |
| `cargo check --workspace` | ✅ PASS |
| `cargo test -p structural-analysis --test truss` | ✅ 50 passed, 0 failed |
| `cargo test -p structural-analysis` | ✅ ALL passed |
| `cargo test -p structural-analysis --doc` | ✅ 13 doctests passed |
| `cargo clippy -p structural-analysis --all-targets` | ✅ 0 errors (412 既有风格警告, 0 新警告) |
| `cargo test --workspace` | 未执行（含 FEM benchmark 超时，见 memory） |
| `cargo clippy --workspace --all-targets -- -D warnings` | 未执行（既有 lint debt，见 memory） |

## 修改文件及 diff 摘要

| 文件 | 变更 |
|------|------|
| `crates/structural-analysis/src/truss.rs` | +28 -2 (DOF 验证 in solve_case + solve_combination) |
| `crates/structural-analysis/tests/truss.rs` | +74 -1 (5 个 P123 验证测试 + FemError 导入) |
| `crates/structural-analysis/PHASE123_AUDIT.md` | 新建 (审计报告) |
| **总计** | **+102 -3** (代码) + 审计报告 |

## 未解决问题及其风险

- **P3: LoadCase 空名称**: 允许 `LoadCase::new("")`。风险极低 — 不影响数值正确性，仅影响可读性。不修复。
- **2D Truss 无 Envelope**: 2D Truss 尚无 Envelope 支持（3D Truss 和 Frame 有）。不影响本阶段功能，可作为未来阶段。
- **2D/3D Truss 无批量求解**: 无 `solve_cases` 方法。不影响正确性，是功能差距。

## 架构与 API 稳定性评估

- **共享基础设施**: `LoadCase`, `LoadCombination`, `LoadSource`, `load_case_source`, `load_combination_source` 被所有模型正确复用。✅
- **无重复抽象**: 无 Element trait 或通用 LoadCase 子类型化。✅
- **API 一致性**: 2D Truss 的 `solve_case`/`solve_combination` 与 3D Truss 和 Frame 模式一致。✅
- **向后兼容**: P1 修复仅添加验证，不改变既有签名或行为。✅

## 下一阶段建议

- Phase 124: 2D Truss Envelope 实现（与 3D Truss Envelope 和 Frame Envelope 一致）
- 或: 2D/3D Truss 批量求解 (solve_cases) — 预分解刚度矩阵 + 多工况回代
- 或: 跨模型后处理统一框架审计

## 最终报告

```
HEAD:                 18aee82 → Phase 123 commit (DOF validation + audit)
P0: 0
P1: 1 (fixed — 2D Truss DOF validation)
P2: 0
P3: 1 (LoadCase empty name — design decision, not fixed)
Tests:               50 passed (45 existing + 5 new P123)
LoadCase Contract:   Consistent across all models
Constraint Semantics: Consistent; P1 fix adds DOF validation
Result Provenance:   Consistent across all models
API Stability:       Fully backward compatible
Changes:             truss.rs +28/-2, tests/truss.rs +74/-1, PHASE123_AUDIT.md (new)
Commit:              "Phase 123: validate 2D truss load case DOFs"
Push Status:         not pushed
Next Phase:          TBD — awaiting user instruction
```
