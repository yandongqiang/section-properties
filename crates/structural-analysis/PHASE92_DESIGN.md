# Phase 92 — Load Case / Load Combination 设计文档

## 1. 当前荷载架构审查结论

### 1.1 荷载存储

荷载作为 **model state** 直接存储在 `BeamModel` 的四个 public 字段中：

| 字段 | 类型 | 坐标系 | 索引方式 |
|------|------|--------|----------|
| `nodal_forces` | `Vec<(usize, usize, f64)>` | global | (node_idx, dof, value) |
| `distributed_loads` | `Vec<DistributedLoad>` | local | element_idx |
| `point_loads` | `Vec<PointLoad>` | local | element_idx + position |
| `applied_moments` | `Vec<AppliedMoment>` | global | node_idx |

`FrameModel` 是 `BeamModel` 的 façade（`FrameModel { inner: BeamModel }`），所有荷载添加方法委托给 `inner`。

### 1.2 荷载组装流水线

```
FrameModel::solve()
  → FrameSolver::solve()
    → BeamSolver::from_model(&model.inner)   // 组装 K 和 f
    → beam.solve_configured()                 // 解 K u = f
    → FrameAnalysisResult { beam, model }
```

`from_model` 做两件事：
1. **组装全局刚度矩阵 K** — 从 elements + nodes + end releases
2. **组装全局荷载向量 f** — 从 nodal_forces + distributed_loads + point_loads + applied_moments

这两件事在同一个函数中完成，不可分离。`f_global` 是 `BeamSolver` 的 private 字段，`from_model` 之后无法替换。

### 1.3 核心问题

- **solve() 隐含唯一荷载工况** — 一次 solve() 使用 model 中所有荷载，无法区分不同工况
- **无荷载组合** — 无法表达 `1.4D + 1.6L` 这样的线性组合
- **无 provenance** — 结果不记录荷载来源
- **K 和 f 耦合** — `from_model` 同时组装 K 和 f，无法仅组装 K 后注入不同 f

### 1.4 审查问题回答

| 问题 | 回答 |
|------|------|
| A. 荷载在哪里？ | `BeamModel` 的 4 个 pub 字段 |
| B. 荷载是什么 state？ | model state |
| C. solve() 隐含唯一工况？ | 是 |
| D. 生命周期一致？ | 是，全部存在 BeamModel |
| E. FrameModel 是 façade？ | 是，`inner: BeamModel` |
| F. 引入 LoadCase 应移出哪些字段？ | 不移出 — LoadCase 作为独立容器，不修改 BeamModel |
| G. LoadCombination 如何求解？ | 合并 RHS：f = Σ factor_i × f_i，一次 solve |

## 2. 设计目标

建立清晰的荷载语义层次：

```
Load → LoadCase → LoadCombination → Analysis → AnalysisResult
```

- **Load**：既有类型（DistributedLoad, PointLoad, AppliedMoment, nodal force）— 不复制
- **LoadCase**：一个命名的荷载集合（一个工况）
- **LoadCombination**：LoadCase 的线性组合 `C = Σ factor_i × LoadCase_i`
- **Analysis**：求解 `K u = f`，其中 f 来自 LoadCase 或 LoadCombination
- **AnalysisResult**：结果 + provenance（记录来源 case/combination 名称）

## 3. API 设计

### 3.1 LoadCase

```rust
pub struct LoadCase {
    name: String,
    nodal_forces: Vec<(usize, usize, f64)>,
    distributed_loads: Vec<DistributedLoad>,
    point_loads: Vec<PointLoad>,
    applied_moments: Vec<AppliedMoment>,
}
```

方法（全部 `Result`-based，验证 finite）：

| 方法 | 说明 |
|------|------|
| `new(name: &str) -> Self` | 空工况 |
| `name() -> &str` | 工况名称 |
| `is_empty() -> bool` | 是否无荷载 |
| `nodal_load(node: NodeHandle, fx: f64, fy: f64) -> Result<(), FemError>` | 全局节点力 |
| `nodal_moment(node: NodeHandle, mz: f64) -> Result<(), FemError>` | 全局节点弯矩 |
| `member_udl(member: MemberHandle, qx: f64, qy: f64) -> Result<(), FemError>` | 局部均布荷载 |
| `member_point_load(member: MemberHandle, xi: f64, fx: f64, fy: f64, mz: f64) -> Result<(), FemError>` | 局部点荷载 |

**设计决策**：
- LoadCase 复用既有 Load 类型（DistributedLoad, PointLoad, AppliedMoment），不创建新类型
- LoadCase 存储 raw indices（NodeHandle/MemberHandle 的 .0），验证推迟到 solve 时
- LoadCase 不依赖 BeamModel/FrameModel — 独立容器
- LoadCase 在 `load.rs` 模块中，不在 `beam_fem.rs`

### 3.2 LoadCombination

```rust
pub struct LoadCombination {
    name: String,
    terms: Vec<(LoadCase, f64)>,  // (case, factor) — owned
}
```

方法：

| 方法 | 说明 |
|------|------|
| `new(name: &str) -> Self` | 空组合 |
| `name() -> &str` | 组合名称 |
| `is_empty() -> bool` | 是否无项 |
| `add_case(case: &LoadCase, factor: f64) -> Result<(), FemError>` | 添加一项（clone case） |
| `n_terms() -> usize` | 项数 |

**factor 验证**：`factor.is_finite()` — NaN 和 ±∞ 拒绝，负系数允许。

### 3.3 Solve API

FrameModel 新增方法：

```rust
pub fn solve_case(&self, case: &LoadCase) -> Result<FrameAnalysisResult, FemError>
pub fn solve_combination(&self, combo: &LoadCombination) -> Result<FrameAnalysisResult, FemError>
```

**语义**：
- `solve_case`：用 LoadCase 的荷载组装 f，解 `K u = f`
- `solve_combination`：合并 RHS `f = Σ factor_i × f_i`，一次解 `K u = f`
- 两者都忽略 model 中直接添加的荷载（model 仅提供几何/材料/约束）
- 现有 `solve()` 保留不变 — 使用 model 中直接添加的荷载

### 3.4 Provenance

`FrameAnalysisResult` 新增：

```rust
pub fn load_source(&self) -> Option<&str>
```

返回 `"case:dead"` / `"combination:1.4D+1.6L"` / `None`（直接 solve）。

## 4. 实现方案

### 4.1 荷载组装函数提取

从 `from_model` 提取荷载组装逻辑为独立函数：

```rust
fn assemble_global_load_vector(
    model: &BeamModel,
    nodal_forces: &[(usize, usize, f64)],
    distributed_loads: &[DistributedLoad],
    point_loads: &[PointLoad],
    applied_moments: &[AppliedMoment],
) -> Result<Vec<f64>, FemError>
```

- `from_model` 调用此函数（传入 model 自身的荷载）— 零行为变化
- `solve_case` / `solve_combination` 调用此函数（传入 LoadCase 的荷载）

**不重复实现 equivalent nodal force 计算** — 同一函数服务所有路径。

### 4.2 BeamSolver 荷载注入

```rust
impl BeamSolver {
    pub(crate) fn set_load_vector(&mut self, f: Vec<f64>)
}
```

替换 `f_global`。仅在 `f.len() == n_dof` 时有效（debug_assert）。

### 4.3 solve_case 实现流程

```
1. validate model
2. BeamSolver::from_model(&model.inner)  // K + f (f 可能为 0)
3. f = assemble_global_load_vector(model, case 的荷载)
4. solver.set_load_vector(f)
5. solver.solve_configured()
6. FrameAnalysisResult { beam: solver, model: clone, load_source: "case:{name}" }
```

### 4.4 solve_combination 实现流程

```
1. validate model
2. BeamSolver::from_model(&model.inner)  // K + f (f 可能为 0)
3. f = Σ factor_i × assemble_global_load_vector(model, case_i 的荷载)
4. solver.set_load_vector(f)
5. solver.solve_configured()
6. FrameAnalysisResult { beam: solver, model: clone, load_source: "combination:{name}" }
```

**合并 RHS 方案**（非 per-case solve + superposition）：
- 一次 solve，一个 result
- 数学等价（线性性）：`K u = Σ f_i ⟹ u = Σ K⁻¹ f_i`
- 更高效（一次 factorization）
- 不支持获取单个 case 结果（spec 不要求）

### 4.5 模块结构

```
crates/structural-analysis/src/
  load.rs       ← NEW: LoadCase, LoadCombination
  beam_fem.rs   ← 修改: 提取 assemble_global_load_vector, 添加 set_load_vector
  frame.rs      ← 修改: 添加 solve_case, solve_combination, load_source
  lib.rs        ← 修改: re-export LoadCase, LoadCombination
```

## 5. 数学语义

### 5.1 LoadCase

LoadCase 是荷载的集合。给定模型几何 M 和 LoadCase C：

```
f(C, M) = assemble(C.nodal_forces, C.distributed_loads, C.point_loads, C.applied_moments, M)
```

`f` 是全局荷载向量，包含 equivalent nodal forces（均布荷载、点荷载的等效节点力）。

### 5.2 LoadCombination

```
f(combination) = Σ_i factor_i × f(case_i, M)
```

### 5.3 求解

```
K u = f
```

K 仅依赖模型几何/材料/约束/端释放，不依赖荷载。因此：
- 不同 LoadCase 共享同一 K
- LoadCombination 是 f 的线性组合，解也是对应线性组合（精确到浮点误差）

## 6. 被拒绝的替代方案

### 6.1 修改 BeamModel 移出荷载字段

**拒绝原因**：大重构，影响所有现有测试和 API。LoadCase 作为独立容器更最小侵入。

### 6.2 Per-case solve + superposition

**拒绝原因**：多次 factorization 效率低。合并 RHS 数学等价且更简单。spec 不要求单 case 结果。

### 6.3 LoadCase 存储 handle 而非 raw index

**拒绝原因**：NodeHandle/MemberHandle 就是 raw index（`NodeHandle(usize)`），存储 handle 无额外安全性，反而引入不必要的包装。

### 6.4 在 BeamModel 中添加 LoadCase 管理

**拒绝原因**：违反"LoadCase 是分析域对象，不是 BeamModel 专属概念"。BeamModel 不应知道 LoadCase。

### 6.5 创建 CaseDistributedLoad / CasePointLoad 等新类型

**拒绝原因**：spec 明确禁止。复用既有 Load 类型。

### 6.6 LoadCombination 借用 LoadCase

**拒绝原因**：生命周期约束复杂，API 不友好。Owned + clone 更简单，LoadCase 是轻量数据结构。

## 7. 未来扩展边界

Phase 92 **不实现**以下内容：
- 设计规范组合（LRFD/ASD 荷载系数）
- 包络分析（envelope of multiple combinations）
- 优化
- 序列化/反序列化
- 动力/模态/非线性/塑性分析
- Truss/Plate/Shell/Solid/3D
- Generic hierarchy（Element<T>, Load<T>）

LoadCase/LoadCombination 的设计不排斥未来添加这些功能，但 Phase 92 的实现范围严格限定在静态线性分析的荷载工况/组合。

## 8. 测试计划（10 类）

1. **单 LoadCase 与直接加载一致** — 同一荷载通过 LoadCase 和直接 add，结果相同
2. **两个独立 LoadCase 分别 solve** — 结果独立
3. **线性叠加** — `solve(1.0A + 1.0B)` ≈ `solve(A) + solve(B)`（位移/反力/杆端力）
4. **Load factors** — `2.0×A` ≈ `2×solve(A)`
5. **混合荷载类型** — nodal + UDL + point + moment 在同一 case
6. **Local/global 变换** — 斜杆上的 local UDL 正确变换到 global
7. **End release + LoadCase** — 有端释放的 member + LoadCase 荷载
8. **平衡** — 每个 case 和 combination 的 result 都满足平衡
9. **Invalid input** — NaN, infinity, invalid handles, empty case/combination
10. **完整回归** — 既有 234+ 测试零回归
