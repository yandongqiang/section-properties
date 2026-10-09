# Phase 124 — 2D Truss Envelope Postprocessing

## 初始 HEAD 与工作区状态

- **HEAD**: `292ec0a` (Phase 123: validate 2D truss load case DOFs)
- **Working tree**: clean (only untracked historical audit docs)
- **Date**: 2026-10-09

## Envelope API 设计依据

### 类型选择：独立 `TrussEnvelope`

2D Truss 有 2 DOF/节点 (ux, uy)，与 Frame (3 DOF: ux, uy, rz) 和 3D Truss (3 DOF: ux, uy, uz) 不同。无法复用既有 Envelope 类型：

- **Frame `Envelope`**: 含 `SectionForces` (N, V, M)、`EnvelopeSample` (member sampling)、`rz` 旋转 DOF — 不适用
- **3D Truss `Truss3DEnvelope`**: 含 `uz`/`rz` 分量 — 不适用

新建独立 `TrussEnvelope` 类型，复用共享基础设施 `Extremum` 和 `LoadSource`。

### 新增类型

| 类型 | 字段 | 用途 |
|------|------|------|
| `TrussEnvelope` | `node_displacements`, `support_reactions`, `axial_forces`, `n_results`, `n_nodes`, `n_elements`, `sources` | 包络容器 |
| `TrussNodeDisplacementSample` | `node_index`, `ux: Extremum`, `uy: Extremum` | 节点位移包络 |
| `TrussNodeReactionSample` | `node_index`, `rx: Extremum`, `ry: Extremum` | 节点反力包络 |
| `TrussAxialForceSample` | `element_index`, `axial: Extremum` | 杆件轴力包络 |

### 复用类型

- `Extremum` — 通用 min/max + governing source（model-agnostic）
- `LoadSource` — 工况来源标识（model-agnostic）
- `FemError` — 错误类型

### `TrussAnalysisResult` 补充

添加 `constrained_dofs: Vec<(usize, usize)>` 公共字段，在 `TrussSolver::results()` 中从 `fixed_dofs` 提取 `(i / 2, i % 2)`。与 3D Truss `TrussAnalysisResult3D.constrained_dofs` 一致。

## 与 3D Truss 和现有后处理 API 的差异

| 特性 | `TrussEnvelope` (2D) | `Truss3DEnvelope` (3D) | `Envelope` (Frame) |
|------|----------------------|------------------------|---------------------|
| DOF/节点 | 2 (ux, uy) | 3 (ux, uy, uz) | 3 (ux, uy, rz) |
| 位移分量 | ux, uy | ux, uy, uz | ux, uy, rz |
| 反力分量 | rx, ry | rx, ry, rz | rx, ry, mz |
| 轴力 | ✅ tension+ | ✅ tension+ | N, V, M (section forces) |
| Member sampling | ❌ (truss: axial only) | ❌ | ✅ (n_per_member) |
| **拓扑验证** | **节点坐标 + 连接关系** | 仅计数 | 仅计数 |
| 反力采样 | constrained_dofs | constrained_dofs | support_dofs |
| 来源标识 | LoadSource | LoadSource | LoadSource |
| 并列策略 | 首输入结果 | 首输入结果 | 首输入结果 |
| 非有限值 | InvalidInput | InvalidInput | InvalidInput |

### 关键差异：拓扑验证

`TrussEnvelope` 验证所有输入结果的节点坐标（相对容差 1e-9）和单元连接关系（精确匹配），**强于** 3D Truss 和 Frame Envelope（仅验证计数）。这是因为 `TrussAnalysisResult` 存储 `node_coords` 和 `element_nodes` 快照，数据可用。

## 数学验证过程

### 解析基准 (Test 17)

单杆轴向拉伸：E=200 GPa, A=1e-4 m², L=3 m, F=50 kN

- `ux = FL/EA = 50000 × 3 / (200e9 × 1e-4) = 7.5e-3 m`
- `N = F = 50000 N` (tension)
- `R_x at node 0 = -F = -50000 N` (opposing)

Envelope 值与解析解的相对误差 < 1e-10。

### 反力符号约定 (Test 23)

`R = K·u - f` (global)。对 node 1 施加 +x 方向力，固定端 node 0 反力为负（ opposing）。

### 轴力拉压符号 (Test 4)

- 拉力 (Fx > 0 at free node): N > 0 (tension)
- 压力 (Fx < 0 at free node): N < 0 (compression)
- `max = tension`, `min = compression`

### 线性叠加 (Test 7)

`solve_combination` 结果的位移 = Σ factor_i × case_i 位移，Envelope 正确聚合多个组合结果。

## 拓扑和约束匹配规则

### 拓扑匹配

1. **节点计数**: 所有结果须有相同 `n_nodes`
2. **单元计数**: 所有结果须有相同 `n_elements`
3. **节点坐标**: 所有结果的 `node_coords` 须在相对容差 1e-9 内一致
4. **单元连接**: 所有结果的 `element_nodes` 须精确匹配

违反任一条 → `FemError::InvalidInput`。

### 约束匹配

约束集**不需要**一致。不同工况可能通过 `prescribed_displacement_2d` 添加约束。某 DOF 在部分结果中约束、在部分结果中自由时，仅从约束结果中采样反力。自由 DOF 的反力不填充（`min = +∞`, `max = -∞`, `None` sources）。

## P0–P3 审计结果

| 级别 | 问题 | 状态 |
|------|------|------|
| P0 | 0 | — |
| P1 | 0 | — |
| P2 | 0 | — |
| P3 | 0 | — |

## 测试及质量检查结果

### 新增测试 (23 项)

| # | 测试名 | 验证内容 |
|---|--------|----------|
| 1 | p124_single_result_envelope_matches_original | 单结果 min==max==value |
| 2 | p124_two_results_displacement_extrema | 两结果位移 min/max |
| 3 | p124_two_results_reaction_extrema | 两结果反力 min/max |
| 4 | p124_axial_force_tension_compression | 轴力拉压符号 |
| 5 | p124_governing_source_correct | 极值与 governing source 配对 |
| 6 | p124_multiple_load_cases | 多 LoadCase |
| 7 | p124_multiple_load_combinations | 多 LoadCombination |
| 8 | p124_repeated_source_names | 重复来源名称 |
| 9 | p124_ties_keep_first_source | 并列极值确定性 |
| 10 | p124_empty_input_rejected | 空结果集 |
| 11 | p124_mismatched_node_counts_rejected | 节点数不匹配 |
| 12 | p124_same_count_different_node_coords_rejected | 同计数不同坐标 |
| 13 | p124_different_element_connectivity_rejected | 不同连接关系 |
| 14 | p124_different_constraint_sets | 不同约束集 |
| 15 | p124_non_finite_values_rejected | 非有限数值 |
| 16 | p124_direct_load_result_compatible | 直接荷载结果兼容 |
| 17 | p124_analytical_benchmark | 解析基准 |
| 18 | p124_reorder_preserves_extrema | 重排序保持极值 |
| 19 | p124_free_dofs_not_populated | 自由 DOF 不填充 |
| 20 | p124_constrained_dofs_match_model | constrained_dofs 正确性 |
| 21 | p124_source_index_alignment | 来源索引对齐 |
| 22 | p124_both_displacement_components_independent | ux/uy 独立跟踪 |
| 23 | p124_reaction_sign_convention | 反力符号约定 |

### 验证命令与结果

| 检查 | 结果 |
|------|------|
| `cargo fmt --all -- --check` | ✅ PASS |
| `cargo check --workspace` | ✅ PASS |
| `cargo test -p structural-analysis --test truss` | ✅ 73 passed (50 existing + 23 new) |
| `cargo test -p structural-analysis` | ✅ ALL passed |
| `cargo test -p structural-analysis --doc` | ✅ 13 doctests passed |
| `cargo clippy -p structural-analysis --all-targets` | ✅ 0 errors (412 既有风格警告, 0 新警告) |
| `cargo test --workspace` | ⏱ 超时（含 FEM benchmark，已知问题，非回归） |
| `cargo clippy --workspace --all-targets -- -D warnings` | ❌ 既有 147 errors in section-properties（历史遗留 lint debt，非回归） |

## 文件修改清单和 diff 摘要

| 文件 | 变更 |
|------|------|
| `crates/structural-analysis/src/truss.rs` | +22 (constrained_dofs 字段 + 访问器 + results() 填充) |
| `crates/structural-analysis/src/postprocessing.rs` | +294 (TrussEnvelope 类型 + from_results + 拓扑验证 + 计算) |
| `crates/structural-analysis/src/lib.rs` | +2 -1 (导出新类型) |
| `crates/structural-analysis/tests/truss.rs` | +520 (23 个 P124 测试) |
| `crates/structural-analysis/PHASE124_REPORT.md` | 新建 (本报告) |
| **总计** | **+838 -1** (代码) + 报告 |

## API 稳定性与遗留问题

### 向后兼容性

- `TrussAnalysisResult` 新增 `constrained_dofs` 为公共字段 — 不影响既有代码（字段初始化在 `results()` 内部）
- `constrained_dofs()` 访问器为新增方法 — 不影响既有 API
- `TrussEnvelope` 及相关类型为新增 — 不影响既有 API
- 无公共方法签名变更 ✅

### 性能

- 单次遍历所有结果更新所有极值 — O(n_results × n_nodes) 位移, O(n_results × n_constrained) 反力, O(n_results × n_elements) 轴力
- 无模型克隆 — 从结果快照读取
- 无 O(n²) ✅

### 遗留问题

- **2D/3D Truss 无批量求解**: 无 `solve_cases` 方法。不影响正确性，是功能差距。
- **3D Truss Envelope 拓扑验证较弱**: `Truss3DEnvelope` 仅验证计数，不验证坐标/连接。2D Truss Envelope 已更强。未来可统一升级 3D Truss Envelope。

## 最终报告

```
HEAD:                 292ec0a → Phase 124 commit (2D truss envelope)
P0: 0
P1: 0
P2: 0
P3: 0
Tests:               73 passed (50 existing + 23 new P124)
Envelope API:        TrussEnvelope with 2-DOF samples, topology validation
Topology Validation: node coords (1e-9 rel tol) + element connectivity (exact)
Result Provenance:   LoadSource, source index aligned with input order
API Stability:       Fully backward compatible
Changes:             truss.rs +22, postprocessing.rs +294, lib.rs +2/-1, tests/truss.rs +520
Commit:              "Phase 124: add 2D truss envelope postprocessing"
Push Status:         not pushed
Next Phase:          TBD — awaiting user instruction
```
