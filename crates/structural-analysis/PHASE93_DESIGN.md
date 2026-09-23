# Phase 93 — Support & Load Refinement 设计文档

## 1. 当前架构审查结论

### A. 当前支座模型

| API | 约束 DOFs | 映射 |
|-----|----------|------|
| `fix(node)` | Ux=0, Uy=0, Rz=0 | 全固定 |
| `pin(node)` | Ux=0, Uy=0 | 铰支（转动自由） |
| `roller_y(node)` | Uy=0 | 水平辊轴 |
| `roller_x(node)` | Ux=0 | 垂直辊轴 |
| `restrain(node, dof, value)` | 单 DOF = value | 任意单 DOF 约束 |

全部映射到 `BeamModel.fixed_dofs: Vec<(usize, usize, f64)>` — (node_idx, dof, prescribed_value)。

**无 spring support，无 inclined roller，无多 DOF 约束。**

### B. 当前约束实现

**DOF 消除（静态凝聚）**：
1. `from_model` 组装 K_global（单元刚度矩阵叠加）
2. `k_original = k_global.clone()` — 用于 reaction 恢复
3. `apply_boundary_conditions` 将 DOF 分为 free / constrained
4. K_ff = K_global 的 free-free 子矩阵
5. f_reduced = f_f - K_fc * u_c（处理 prescribed displacement）
6. 解 K_ff * u_f = f_reduced
7. Reactions: R = K_original * u - f_global

**关键**：
- 无 penalty，无 Lagrange multiplier
- Prescribed displacement 已支持（`restrain`）
- Mechanism diagnostics 使用 retained K_ff（与 solver factorize 的矩阵相同）
- `k_original` 在 spring 添加前克隆 → spring 不在 k_original 中

### C. 当前 distributed load

`DistributedLoad { element_idx, qx, qy }` — **仅常量均布荷载**。

`consistent_nodal_load` 公式（uniform q）：
- 轴向: f_u_i = qx*L/2, f_u_j = qx*L/2
- 横向: f_v_i = qy*L/2, f_θ_i = qy*L²/12, f_v_j = qy*L/2, f_θ_j = -qy*L²/12

**唯一数学实现**，路径：`DistributedLoad` → `consistent_nodal_load` (local) → `T^T * f_local` (global) → `f_global`。

### D. Phase 92 影响

`assemble_global_load_vector` 是唯一荷载组装路径。`LoadCase` / `LoadCombination` 通过 `inner_with_loads` / merged model 使用它。**无第二组装路径**。Phase 93 必须保持此不变式。

## 2. Spring Support 设计

### 2.1 数学模型

节点某 DOF 具有有限刚度 k：F = k * u。Spring stiffness 加入整体刚度矩阵：

```
K_total = K_structure + K_spring
```

Spring DOF 保持 free（不消除），spring stiffness 参与 K_ff。

### 2.2 实现

1. `BeamModel` 新增 `spring_supports: Vec<(usize, usize, f64)>` — (node_idx, dof, stiffness)
2. `from_model` 中，在 `k_original = k_global.clone()` **之后**，将 spring stiffness 加到 `k_global` 对角线
3. **k_original 不含 spring** → reaction = K_original * u - f = -k * u（spring 恢复力）
4. `reactions()` 无需修改 — spring DOF 的 reaction 自动为 -k * u
5. equilibrium 自动满足：applied + reactions = 0

### 2.3 API

```rust
// FrameModel
pub fn spring(&mut self, node: NodeHandle, dof: Dof, stiffness: f64) -> Result<(), FemError>
```

验证：NaN/∞ 拒绝，负刚度拒绝，零刚度拒绝（无物理意义，用 no-op 代替）。

### 2.4 Reaction 语义

- Fixed DOF: R = constraint force（K_original * u - f）
- Spring DOF: R = -k * u（spring 恢复力，K_original 不含 spring）
- Free DOF: R ≈ 0

## 3. Inclined Roller 设计

### 3.1 数学模型

约束方向单位向量 n = (nx, ny)，约束：nx * Ux + ny * Uy = prescribed。

**实现方法：post-assembly coordinate transformation**

在 K_global 和 f_global 组装后，对有 inclined roller 的节点施加坐标旋转，将 (Ux, Uy) 变换为 (U_n, U_t)：
- U_n = nx * Ux + ny * Uy（约束方向）
- U_t = -ny * Ux + nx * Uy（自由方向）

旋转后，约束变为单 DOF 消除：U_n = prescribed。Solver 架构不变。

### 3.2 变换公式

设 c = nx, s = ny，DOF i = 3*node (Ux), j = 3*node+1 (Uy)：

**K 变换**（对 2×2 子块及耦合项）：
- K'[i,i] = c²K[i,i] + 2csK[i,j] + s²K[j,j]
- K'[i,j] = cs(K[j,j]-K[i,i]) + (c²-s²)K[i,j]
- K'[j,j] = s²K[i,i] - 2csK[i,j] + c²K[j,j]
- ∀ k ≠ i,j: K'[i,k] = cK[i,k] + sK[j,k], K'[j,k] = -sK[i,k] + cK[j,k]
- 对称：K'[k,i] = K'[i,k], K'[k,j] = K'[j,k]

**f 变换**：
- f'[i] = c*f[i] + s*f[j]
- f'[j] = -s*f[i] + c*f[j]

**结果变换**（solve 后）：
- Ux = c*U_n - s*U_t
- Uy = s*U_n + c*U_t
- Rx = c*R_n - s*R_t
- Ry = s*R_n + c*R_t

### 3.3 实现

1. `BeamModel` 新增 `inclined_rollers: Vec<(usize, f64, f64, f64)>` — (node_idx, nx, ny, prescribed)
2. `BeamSolver` 新增 `node_rotations: Vec<Option<(f64, f64)>>` — 每节点 (cos, sin) 或 None
3. `from_model` 中，在 K_global 和 f_global 组装后、k_original 克隆前：
   a. 对每个 inclined roller，对 K_global 和 f_global 施加旋转
   b. k_original = k_global.clone()（含变换）
   c. 标记 U_n DOF 为 constrained
4. `displacement` / `reaction` 恢复时，对有旋转的节点变换回全局坐标

### 3.4 API

```rust
// FrameModel
pub fn inclined_roller(&mut self, node: NodeHandle, nx: f64, ny: f64) -> Result<(), FemError>
pub fn inclined_roller_settlement(&mut self, node: NodeHandle, nx: f64, ny: f64, value: f64) -> Result<(), FemError>
```

n = (nx, ny) 是**被约束方向**（法向）。验证：NaN/∞ 拒绝，零向量拒绝，自动归一化。

### 3.5 Reaction

约束反力沿 n 方向。变换回全局坐标后，Rx/Ry 自动满足方向关系。

## 4. Trapezoidal Load 设计

### 4.1 数学模型

线性变化分布荷载：q(x) = q_start + (q_end - q_start) * x/L

### 4.2 等效节点力推导

对 Hermite 形函数积分：

**轴向**（线性形函数）：
- f_u_i = L*(2*qx_start + qx_end)/6
- f_u_j = L*(qx_start + 2*qx_end)/6

**横向**（Hermite 三次形函数）：
- f_v_i = L*(7*qy_start + 3*qy_end)/20
- f_θ_i = L²*(3*qy_start + 2*qy_end)/60
- f_v_j = L*(3*qy_start + 7*qy_end)/20
- f_θ_j = -L²*(2*qy_start + 3*qy_end)/60

**退化验证**（q_start = q_end = q）：
- f_u_i = q*L/2 ✓, f_θ_i = q*L²/12 ✓ — 与 uniform 一致

### 4.3 截面力公式

```
N(x) = N_i - qx_start*x - (qx_end - qx_start)*x²/(2L)
V(x) = -V_i + qy_start*x + (qy_end - qy_start)*x²/(2L)
M(x) = M_i - x*V_i + qy_start*x²/2 + (qy_end - qy_start)*x³/(6L)
```

### 4.4 实现：扩展 DistributedLoad

```rust
pub struct DistributedLoad {
    pub element_idx: usize,
    pub qx: f64,       // start (at node_i) — 兼容现有字段名
    pub qy: f64,       // start (at node_i)
    pub qx_end: f64,   // end (at node_j)
    pub qy_end: f64,   // end (at node_j)
}
```

- `new(element_idx, qx, qy)` → qx_end=qx, qy_end=qy（uniform，向后兼容）
- 新增 `trapezoidal(element_idx, qx_start, qy_start, qx_end, qy_end)`
- `consistent_nodal_load` 改用 trapezoidal 公式（uniform 是特例）
- `assemble_global_load_vector` 不变（仍调用 `released_consistent_nodal_load`）
- `element_section_forces` 更新截面力公式
- `element_equivalent_nodal_forces` 更新为使用 qx_start/qx_end/qy_start/qy_end

### 4.5 唯一组装路径保持

所有 distributed load（uniform + trapezoidal）通过同一 `consistent_nodal_load` → `assemble_global_load_vector` 路径。**无第二套数学实现**。

## 5. 被拒绝的替代方案

### 5.1 Spring as penalty

**拒绝**：spec 明确 "不要把 spring 实现成'近似固定'"。Spring 是真实刚度，进入 K_global。

### 5.2 Inclined roller as penalty

**拒绝**：不精确。Coordinate transformation 是精确方法。

### 5.3 Inclined roller as Lagrange multiplier

**拒绝**：改变矩阵结构（增加行列），比 coordinate transformation 更侵入。

### 5.4 TrapezoidalLoad 新类型

**拒绝**：spec 要求避免两套 distributed-load 数学实现。扩展 DistributedLoad 保持单一路径。

### 5.5 Trapezoidal via two point loads / UDL 拼接

**拒绝**：spec 明确禁止 "用简单平均值代替" "通过两个 point load 近似"。必须用精确积分。

## 6. Reaction 语义总结

| DOF 类型 | Reaction | 来源 |
|----------|----------|------|
| Fixed | constraint force | K_original * u - f |
| Spring | -k * u (spring restoring force) | K_original (不含 spring) * u - f |
| Inclined roller | constraint force along n | 变换回全局坐标 |
| Free | ≈ 0 | K_original * u - f |
| End release | 不产生 node-level reaction | Phase 91 语义 |

## 7. 数值容差

复用现有 equilibrium tolerance。不引入新 magic tolerance。Spring/inclined roller 的验证使用与现有测试相同的容差（1e-6 ~ 1e-12 视量级而定）。

## 8. 禁止范围

同 spec §十四：不实现 Truss/Plate/Shell/Solid/3D/nonlinear/dynamic/modal/plastic/buckling/design code/envelope/serialization/GUI/units/generic hierarchy。
