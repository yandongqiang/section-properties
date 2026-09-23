# Phase 92 — Load Case / Load Combination 实现报告

## 概述

Phase 92 为 structural-analysis 引入了荷载工况（LoadCase）和荷载组合（LoadCombination），建立了清晰的荷载语义层次：

```
Load → LoadCase → LoadCombination → Analysis → AnalysisResult
```

**基础提交**: `7db1ea1` (Phase 91: member end releases)
**设计文档**: `PHASE92_DESIGN.md`

## 实现内容

### 新增模块 `load.rs`

- **`LoadCase`** — 命名荷载集合，复用既有 `DistributedLoad`、`PointLoad`、`AppliedMoment` 类型
  - `new(name)`, `name()`, `is_empty()`
  - `nodal_load(node, fx, fy)`, `nodal_moment(node, mz)`
  - `member_udl(member, qx, qy)`, `member_point_load(member, xi, fx, fy, mz)`
  - 全部 `Result`-based，验证 finite

- **`LoadCombination`** — LoadCase 的线性组合 `C = Σ factor_i × LoadCase_i`
  - `new(name)`, `name()`, `n_terms()`, `is_empty()`
  - `add_case(case, factor)` — factor 必须有限（NaN/±∞ 拒绝，负数允许）
  - Owned `Vec<(LoadCase, f64)>`

### 修改 `beam_fem.rs`

- **提取 `assemble_global_load_vector`** — 从 `from_model` 提取荷载组装逻辑为 `pub(crate)` 自由函数。`from_model` 调用它（零行为变化）。这是唯一的荷载组装路径，无重复实现。
- `BeamSolver` 未新增任何 LoadCase/LoadCombination 相关代码。

### 修改 `frame.rs`

- **`FrameModel::solve_case(&LoadCase)`** — 用 LoadCase 的荷载替换 model 的荷载，通过 `from_model` 组装 K 和 f，求解。
- **`FrameModel::solve_combination(&LoadCombination)`** — 合并 RHS（`f = Σ factor_i × f_i`），一次求解。
- **`FrameAnalysisResult::load_source()`** — 返回 `"case:{name}"` / `"combination:{name}"` / `None`。
- **`FrameAnalysisResult` 新增 `load_source: Option<String>` 字段**。
- 私有辅助 `inner_with_loads()` — 克隆 inner BeamModel 并替换荷载集合。

### 修改 `lib.rs`

- 添加 `pub mod load;`
- Re-export `LoadCase`, `LoadCombination`

### 关键设计决策

1. **不修改 BeamModel 结构** — LoadCase 作为独立容器，最小侵入
2. **合并 RHS 方案** — `f = Σ factor_i × f_i`，一次求解（数学等价于 per-case solve + superposition，更高效）
3. **model 带荷载存储** — `FrameAnalysisResult` 存储带荷载的 `FrameModel`，确保 `equilibrium()` 正确
4. **LoadCase 在 `load.rs`** — 不在 `beam_fem.rs`，为未来可复用留空间
5. **复用既有 Load 类型** — 不创建 `CaseDistributedLoad` 等新类型

## 测试

### 新增测试 `tests/load_case.rs` — 33 个测试

| 类别 | 测试数 | 覆盖 |
|------|--------|------|
| §1 单 LoadCase 与直接加载一致 | 5 | nodal/UDL/point/moment/provenance |
| §2 两个独立 LoadCase | 1 | 结果独立 |
| §3 线性叠加 | 3 | 位移/反力/杆端力 |
| §4 Load factors | 4 | 2×/负/零/多系数 |
| §5 混合荷载类型 | 1 | nodal+moment+UDL+point |
| §6 Local/global 变换 | 1 | 45° 斜杆 |
| §7 End release + LoadCase | 2 | case+release, combination+release |
| §8 平衡 | 2 | case/combination |
| §9 Invalid input | 10 | NaN/∞/invalid handle/empty |
| §10 Provenance/metadata | 4 | name/terms/source |

### 回归测试

| 测试集 | 结果 |
|--------|------|
| structural-analysis integration tests | 455 passed, 0 failed |
| structural-analysis doc tests | 9 passed, 0 failed |
| structural-analysis lib tests | 26 passed, 0 failed |
| **总计** | **490 passed, 0 failed** |
| section-properties | 未受影响 (`cargo check` 通过) |

### 代码质量

| 检查 | 结果 |
|------|------|
| `cargo fmt --check` | PASS |
| `cargo clippy` | 无 structural-analysis 警告 |

## 架构审计

| 检查 | 结果 |
|------|------|
| A: Frame 仍只是 façade | ✅ `FrameModel { inner: BeamModel }`，新方法委托 `BeamSolver` |
| B: LoadCase 无复制 equivalent nodal force | ✅ `assemble_global_load_vector` 是唯一组装路径 |
| C: BeamModel 无组合管理逻辑 | ✅ `BeamModel` 未修改 |
| D: LoadCombination 未侵入 section-properties | ✅ 仅在 `load.rs` |
| E: section-properties API 未修改 | ✅ 确认 |
| F: solver 不知道 LoadCase/LoadCombination | ✅ `beam_fem.rs` 无相关引用 |

## 文件变更

| 文件 | 变更 |
|------|------|
| `src/load.rs` | **新增** — LoadCase, LoadCombination |
| `src/beam_fem.rs` | 提取 `assemble_global_load_vector` |
| `src/frame.rs` | 添加 `solve_case`, `solve_combination`, `load_source`, `inner_with_loads` |
| `src/lib.rs` | 添加 `pub mod load` + re-export |
| `tests/load_case.rs` | **新增** — 33 个测试 |
| `PHASE92_DESIGN.md` | **新增** — 设计文档 |
| `PHASE92_REPORT.md` | **新增** — 本报告 |

## 未实现（spec 禁止）

- 设计规范组合（LRFD/ASD 荷载系数）
- 包络分析
- 优化
- 序列化/反序列化
- 动力/模态/非线性/塑性
- Truss/Plate/Shell/Solid/3D
- Generic hierarchy
