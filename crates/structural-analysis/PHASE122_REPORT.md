# Phase 122 — 2D Truss LoadCase / LoadCombination Support

## 初始状态

- **HEAD**: `977cc5b` (Phase 121: 3D Truss Envelope audit)
- **Working tree**: clean (only untracked historical audit docs)
- **Date**: 2026-10-09

## Phase 122A — 审计发现

### 2D Truss 现状

- `TrussModel` (truss.rs): `nodal_forces: Vec<(usize, usize, f64)>`, `fixed_dofs: Vec<(usize, usize, f64)>`
- `add_nodal_force(node_idx, fx, fy)` — 直接添加到模型
- `TrussSolver::from_model` → `solve` / `solve_configured` → `results()`
- `TrussAnalysisResult`: displacements, reactions, axial_forces, solver_name, node_coords, element_nodes, nodal_forces, n_nodes, n_elements
- **无 `load_source` 字段**, **无 `solve_case`**, **无 `solve_combination`**

### Frame LoadCase/LoadCombination

- `LoadCase` (load.rs): `nodal_forces: Vec<(usize, usize, f64)>` — 原始 (node_idx, dof, value)，模型无关
- `LoadCombination` (load.rs): `terms: Vec<(LoadCase, f64)>` — 完全模型无关
- `LoadSource` (load.rs): `ModelLoads` / `LoadCase { name, ... }` / `LoadCombination { name, terms }` — 完全模型无关
- `load_case_source` / `load_combination_source` (frame.rs, `pub(crate)`) — 辅助函数，已共享
- Frame 有 `nodal_load(NodeHandle, fx, fy)` — Frame 专属（使用 NodeHandle）
- 3D Truss 有 `nodal_load_3d(node_idx, fx, fy, fz)` — 3D Truss 专属（原始索引）

### 3D Truss 模式

- `solve_case` / `solve_combination` 定义在 `TrussModel3D` 上（非 solver）
- 模式: clone model → clear nodal_forces → 从 LoadCase/LoadCombination 填充 → 创建 solver → solve → 设置 load_source
- `TrussAnalysisResult3D` 有 `load_source: LoadSource` 字段（private + `load_source()` 访问器）
- `results()` 默认设置 `LoadSource::ModelLoads`

### 设计决策

| 问题 | 决策 | 理由 |
|------|------|------|
| 复用 LoadCase/LoadCombination | 是 | 已是模型无关的原始 (node_idx, dof, value) 存储 |
| 复用 LoadSource | 是 | 已是模型无关的 enum |
| 复用 load_case_source/load_combination_source | 是 | `pub(crate)` 已共享 |
| 添加 nodal_load_2d | 是 | 2D Truss 需要原始索引 + 2 分量接口 |
| 添加 prescribed_displacement_2d | 是 | 2D Truss 需要支座沉降支持 |
| solve_case/solve_combination 位置 | TrussModel | 与 3D Truss 和 Frame 一致 |
| load_source 字段 | TrussAnalysisResult (private + accessor) | 与 3D Truss 一致 |

## 公开 API 变更

### load.rs

- `LoadCase::nodal_load_2d(node_idx: usize, fx: f64, fy: f64) -> Result<(), FemError>` — 2D Truss 节点荷载
- `LoadCase::prescribed_displacement_2d(node_idx: usize, dof: usize, value: f64) -> Result<(), FemError>` — 2D Truss 支座沉降

### truss.rs

- `TrussModel::solve_case(&self, case: &LoadCase) -> Result<TrussAnalysisResult, FemError>` — 求解单个荷载工况
- `TrussModel::solve_combination(&self, combo: &LoadCombination) -> Result<TrussAnalysisResult, FemError>` — 求解荷载组合
- `TrussAnalysisResult::load_source(&self) -> &LoadSource` — 荷载来源访问器
- `TrussAnalysisResult.load_source` 字段 (private) — 荷载来源标识

### lib.rs

- 无变更（`LoadCase`, `LoadCombination`, `LoadSource` 已导出）

## 直接荷载与工况荷载兼容策略

- `add_nodal_force` 和既有 `TrussSolver` API 完全保留
- `solve_case` / `solve_combination` **忽略**模型上直接添加的荷载（`model.nodal_forces.clear()`），仅使用 LoadCase/LoadCombination 的荷载
- 这与 3D Truss 和 Frame 的行为一致
- 直接荷载与工况荷载不会静默重复施加

## 数值验证方法

- **解析解**: 单杆桁架 (E=1, A=1, L=1)，Fx=10 → ux=10
- **线性叠加**: 1.4×D + 1.6×L 与手动叠加比较
- **对称性**: 等腰三角形桁架，Fx 不产生 uy，Fy 不产生 ux
- **平衡**: 三角形桁架竖向荷载，反力对称分布 (Ry0=Ry1=50)
- **支座沉降**: 三杆系统，中间节点位移 = L1/(L1+L2) × δ = 0.005

## 测试结果

20 个新增测试 (p122_*)，全部通过:

| # | 测试名 | 验证内容 |
|---|--------|----------|
| 1 | p122_single_case_matches_direct_load | 工况与直接荷载结果一致 |
| 2 | p122_two_cases_no_pollution | 两工况互不污染 |
| 3 | p122_combination_linear_superposition | 线性叠加正确 |
| 4 | p122_fx_fy_independent | Fx/Fy 独立性 |
| 5 | p122_cancellation_same_dof | 同自由度荷载抵消 |
| 6 | p122_positive_negative_zero_factors | 正/负/零系数 |
| 7 | p122_invalid_node_error | 非法节点错误 |
| 8 | p122_duplicate_case_in_combination | 重复工况累加 |
| 9 | p122_empty_combination | 空组合零结果 |
| 10 | p122_reaction_equilibrium | 反力平衡 |
| 11 | p122_axial_force_sign | 轴力符号正确 |
| 12 | p122_load_source_case | LoadSource::LoadCase |
| 13 | p122_load_source_combination | LoadSource::LoadCombination |
| 14 | p122_load_source_model_loads | LoadSource::ModelLoads |
| 15 | p122_direct_api_regression | 直接 API 回归 |
| 16 | p122_results_not_mutated_by_subsequent_solve | 结果不被后续求解修改 |
| 17 | p122_combination_rejects_prescribed_displacement | 组合拒绝支座沉降 |
| 18 | p122_prescribed_displacement_2d | 支座沉降正确 |
| 19 | p122_prescribed_displacement_2d_invalid_dof | 非法 DOF 错误 |
| 20 | p122_nodal_load_2d_non_finite | 非有限值错误 |

## 质量检查结果

| 检查 | 结果 |
|------|------|
| `cargo fmt --all -- --check` | ✅ PASS |
| `cargo check --workspace` | ✅ PASS |
| `cargo test -p structural-analysis --test truss` | ✅ 45 passed, 0 failed |
| `cargo test -p structural-analysis` | ✅ ALL passed |
| `cargo test -p structural-analysis --doc` | ✅ 13 doctests passed |
| `cargo clippy -p structural-analysis --all-targets` | ✅ 0 errors (5 既有风格警告) |

## 修改文件与 diff 摘要

| 文件 | 变更 |
|------|------|
| `crates/structural-analysis/src/load.rs` | +60 行 (2 个新方法) |
| `crates/structural-analysis/src/truss.rs` | +133 行 (导入, load_source, solve_case, solve_combination) |
| `crates/structural-analysis/tests/truss.rs` | +418 行 (20 个新测试 + 2 个辅助函数) |
| **总计** | **+611 行, 0 删除** |

## 尚未解决的问题

无。所有 P0/P1 缺陷已解决。

## 架构与 API 稳定性评估

- **API 稳定性**: 新增方法不修改任何既有签名，完全向后兼容
- **架构一致性**: 与 3D Truss 和 Frame 的 LoadCase/LoadCombination 模式完全一致
- **共享基础设施**: 复用 `LoadCase`, `LoadCombination`, `LoadSource`, `load_case_source`, `load_combination_source`，无重复代码
- **无新抽象层**: 不引入 Element trait 或新的通用接口
- **无新依赖**: 不引入任何外部依赖

## 下一阶段建议

- Phase 123: 2D Truss Envelope 实现（与 3D Truss Envelope 和 Frame Envelope 一致）
- 或: 2D Truss 自重支持（需添加 density 字段到 TrussElement）
- 或: 跨模型后处理统一（如通用 Envelope 接口）

## 完成标准

```
HEAD: 977cc5b (pre-commit)
P0: 0
P1: 0
P2: 0
P3: 0
Tests: 45 (25 existing + 20 new), all passed
API Changes: +4 public methods, +1 field (private + accessor)
API Stability: Fully backward compatible
Commit: pending
Push Status: not pushed
Next Phase: TBD (await user instruction)
```
