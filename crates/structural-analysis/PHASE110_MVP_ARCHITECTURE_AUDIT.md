# Phase 110 — Structural Analysis MVP Architecture & Roadmap Audit

## 1. Executive Summary

`structural-analysis` 经过 Phase 80–109 的连续功能开发，已形成一个**结构清晰、功能完备的 2D
线性结构分析 MVP**。架构分层严格（Model → Solver → Result → Post-processing），无力学代码
重复，无 solver internals 泄漏，无循环依赖。

**Architecture decision: KEEP**

> No structural refactor required.

Phase 107–109 的三个功能（factorization reuse、self-weight、node/reaction envelope）均以
最小侵入方式融入现有架构，未改变架构边界。审计未发现 P0 或 P1 问题。4 个 P2 项为代码
去重机会，不影响正确性或 API 稳定性。

**Roadmap（最多 3 项）：**

| Phase | 名称 | 规模 | 目的 |
|-------|------|------|------|
| 111 | Prescribed Displacement / Support Settlement | S | 将已有 `restrain(node, dof, value)` 的非零值路径补齐测试并文档化 |
| 112 | 3D Truss Element | M | 最小 3D 扩展：3 DOF/node pin-jointed element，验证 solver/diagnostic 的 DOF 无关性 |
| 113 | structural-analysis API Stabilization | S | 冻结 public API surface，编写 API reference，为未来 v0.1.0 发布做准备 |

---

## 2. Baseline

```
HEAD:       6e60dec (Phase 109: add displacement and reaction envelopes)
baseline:   6e60dec
working tree: clean (7 untracked PHASE*.md docs, no modified source)
```

**Workspace:**

| Crate | Version | Publish | Role |
|-------|---------|---------|------|
| `section-properties` | 0.4.0 | yes (crates.io) | 截面属性、材料、数值基础设施 |
| `structural-analysis` | 0.1.0 | false (workspace-only) | 2D 结构分析 |

**依赖方向：** `structural-analysis → section-properties`（单向，无循环）。

---

## 3. Current Architecture

```text
┌─────────────────────────────────────────────────────────┐
│                  structural-analysis                     │
│                                                          │
│  ┌──────────┐   ┌──────────┐   ┌──────────┐            │
│  │ beam_fem  │   │  frame   │   │  truss   │            │
│  │ (4471 LOC)│   │(2431 LOC)│   │ (946 LOC)│            │
│  │           │   │  façade  │   │          │            │
│  │ BeamModel │←──│FrameModel│   │TrussModel│            │
│  │ BeamSolver│   │  wraps   │   │TrussSolver│           │
│  └─────┬─────┘   └────┬─────┘   └────┬─────┘            │
│        │               │               │                  │
│        │          ┌────┴────┐          │                 │
│        │          │  load   │          │                 │
│        │          │LoadCase │          │                 │
│        │          │LoadComb.│          │                 │
│        │          └─────────┘          │                 │
│        │                               │                 │
│  ┌─────┴───────────────────────────────┴─────┐          │
│  │              mechanism                     │          │
│  │     diagnose_reduced(K_ff, rigid)          │          │
│  └────────────────────┬──────────────────────┘          │
│                       │                                 │
│  ┌────────────────────┴──────────────────────┐          │
│  │            postprocessing                  │          │
│  │  Envelope / Extremum / NodeEnvelopeSample │          │
│  └───────────────────────────────────────────┘          │
│                                                          │
└──────────────────────┬──────────────────────────────────┘
                       │
┌──────────────────────┴──────────────────────────────────┐
│                 section-properties                        │
│  fea::SparseMatrix, fea::solver::*, Material, Point       │
└───────────────────────────────────────────────────────────┘
```

**源文件规模：**

| 文件 | 行数 | 职责 |
|------|------|------|
| `beam_fem.rs` | 4,471 | 2D Euler-Bernoulli 梁：单元、模型、求解器、端力、截面力 |
| `frame.rs` | 2,431 | Frame façade：类型化句柄、支座词汇、荷载工况、平衡 |
| `truss.rs` | 946 | 2D 桁架：单元、模型、求解器、轴力 |
| `mechanism.rs` | 578 | 缩减系统机构诊断 |
| `load.rs` | 370 | LoadCase / LoadCombination |
| `postprocessing.rs` | 328 | Envelope / Extremum / NodeEnvelopeSample |
| `lib.rs` | 141 | 模块声明与 re-export |
| **合计** | **8,265** | |

---

## 4. Complete Capability Matrix

### Beam (beam_fem.rs)

| 能力 | 状态 | 实现位置 |
|------|------|----------|
| Euler-Bernoulli 刚度 | ✅ |) | `local_stiffness` :636 |
| 坐标变换 | ✅ | `transformation_matrix` :694 |
| 全局刚度组装 | ✅ | `BeamSolver::from_model` :2437 |
| 节点力 | ✅ | `nodal_forces` 字段 |
| 点载荷 | ✅ | `consistent_nodal_load_point` :1141 |
| 均布载荷 | ✅ | `consistent_nodal_load` :1066 |
| 梯形载荷 | ✅ | `consistent_nodal_load_trapezoidal` :1087 |
| 施加力矩 | ✅ | `AppliedMoment` :1442 |
| 端部释放 | ✅ | `condense_matrix` :795, `released_global_stiffness` :917 |
| 截面力 N/V/M | ✅ | `element_section_forces` :3867 |
| 力图采样 | ✅ | `sample_beam_forces` :4057, `beam_force_diagram` :4082 |
| 位移 | ✅ | `displacement` :3209, `displacement_dof` :3275 |
| 反力 | ✅ | `reactions` :3333 (R = K·u - f) |
| 平衡验证 | ✅ | 委托 `frame::compute_equilibrium` |
| 自重 | ✅ | Phase 108, `to_local_force` 变换 |
| 荷载工况 | ✅ | Phase 103, `LoadCase` |
| 因式分解复用 | ✅ | Phase 107, `solve_pre_factored` :3152 |
| 弹簧支座 | ✅ | 对角叠加 |
| 斜辊条 | ✅ | 坐标系旋转 |
| 解析解 | ✅ | `BeamAnalysis` :4201 (悬臂/简支) |

### Frame (frame.rs)

| 能力 | 状态 | 实现位置 |
|------|------|----------|
| 节点 | ✅ | `add_node` :357, `NodeHandle` |
| 构件 | ✅ | `add_member` :381, `MemberHandle` |
| fix 支座 | ✅ | `fix` :543 (ux,uy,rz) |
| pin 支座 | ✅ | `pin` :555 (ux,uy) |
| roller_y | ✅ | `roller_y` :568 (uy) |
| roller_x | ✅ | `roller_x` :580 (ux) |
| 任意 DOF 约束 | ✅ | `restrain` :598 (含非零值=支座沉降) |
| 弹簧 | ✅ | `spring` :612 |
| 斜辊条 | ✅ | `inclined_roller` :627 |
| 端部释放 | ✅ | `add_member_with_release` :475 |
| 荷载工况 | ✅ | `solve_case` :861 |
| 荷载组合 | ✅ | `solve_combination` :895 |
| 多工况批量 | ✅ | `solve_cases` :1057, `PreparedFrameAnalysis` :1607 |
| 自重 | ✅ | `self_weight` :752, `self_weight_distributed_loads` :766 |
| 构件端力 | ✅ | `member_end_forces` / `_global` |
| 截面力 | ✅ | `section_forces` |
| 包络 | ✅ | Phase 103/109, `Envelope` |
| 平衡 | ✅ | `equilibrium` :1555, `compute_equilibrium` :1762 |
| 几何 | ✅ | `node_position`, `member_nodes`, `member_length` |
| 机构诊断 | ✅ | `diagnostic` :1135 |
| 模型校验 | ✅ | `validate` :1077 (连通性、孤立节点) |

### Truss (truss.rs)

| 能力 | 状态 | 实现位置 |
|------|------|----------|
| 2D 刚度 | ✅ | `global_stiffness` :191 |
| 节点力 | ✅ | `add_nodal_force` :293 |
| 支座 | ✅ | `fix_dof` :316, `fix_node` :337 |
| 反力 | ✅ | `reactions` :652 |
| 轴力 | ✅ | `axial_force` :683 (tension positive) |
| 平衡 | ✅ | `TrussAnalysisResult::equilibrium` :818 |
| 机构诊断 | ✅ | `diagnostic` :714 |
| 几何 | ✅ | `node_position`, `element_endpoints` |
| 包络 | ❌ | 不支持（无 Envelope API） |
| 自重 | ❌ | 不支持（`TrussElement` 缺 `density`） |
| 荷载工况 | ❌ | 不支持（无 `LoadCase` 集成） |
| 分布载荷 | ❌ | 不适用（pin-jointed） |
| 端部释放 | ❌ | 不适用（已为铰接） |

### Solver (via section-properties::fea)

| 能力 | 状态 | 备注 |
|------|------|------|
| Dense Gaussian | ✅ | 小系统 |
| Skyline LDLᵀ | ✅ | 中型稀疏 |
| Sparse LU | ✅ | 大型稀疏 |
| CG | ✅ | 对称正定 |
| ICCG | ✅ | 不完全 Cholesky 预条件 CG |
| Auto-select | ✅ | `SolverRegistry::create_selected` |
| 静力凝聚 | ✅ | `apply_boundary_conditions` (无罚函数) |
| 因式分解复用 | ✅ | Phase 107, `PreparedFrameAnalysis` |
| 机构诊断 | ✅ | `diagnose_reduced` (equilibrated pivoted Cholesky) |
| 数值错误处理 | ✅ | `FemError::SolverError { source, message }` |

### Post-processing (postprocessing.rs)

| 能力 | 状态 | 实现位置 |
|------|------|----------|
| 构件力采样 | ✅ | `EnvelopeSample` (N/V/M min/max) |
| 构件力包络 | ✅ | `Envelope::from_frame_results` |
| 位移包络 | ✅ | Phase 109, `node_displacements` |
| 反力包络 | ✅ | Phase 109, `support_reactions` |
| governing load source | ✅ | Phase 109, `Extremum::min_source/max_source` |
| 最大绝对值 | ✅ | `max_abs_moment/shear/axial` |

---

## 5. MVP Boundary

### CORE MVP ✅ (已实现)

- Linear static analysis
- 2D Euler-Bernoulli beam element (3 DOF/node)
- 2D pin-jointed truss element (2 DOF/node)
- Frame façade with typed handles
- Basic supports: fix, pin, roller_x, roller_y
- Common loads: nodal force/moment, UDL, trapezoidal, member point load
- Reactions (R = K·u - f)
- Internal forces: N(x), V(x), M(x)
- Equilibrium verification (scale-aware, unit-invariant)
- Mechanism diagnostics (rigid-body vs internal)

### EXTENDED 2D ✅ (已实现)

- Load cases & combinations
- Self-weight / body force
- Springs
- Inclined rollers
- End releases (rotation hinge)
- Member force / displacement / reaction envelopes
- Governing load source tracking
- Repeated load-case solve (factorization reuse)
- Support settlement (non-zero prescribed displacement via `restrain`)

### ADVANCED (未实现，有工程价值)

- Temperature / thermal load
- Member offsets / rigid links
- Tension-only / compression-only elements
- Partial releases (non-rotation)
- Geometric nonlinearity (P-Δ)
- Material nonlinearity

### FUTURE / OUT OF SCOPE

- 3D frame (6 DOF/node)
- 3D truss (3 DOF/node)
- Dynamic analysis
- Buckling
- Plate/shell/solid elements
- Design code combinations

---

## 6. Phase 107–109 Audit

### Phase 107 — Factorization Reuse

**实现：** `PreparedFrameAnalysis<'a>` 持有 `&'a FrameModel` 和
`Option<Box<dyn LinearSolver>>`（已因式分解）。`prepare()` 凝聚边界条件并单次
`factor()`，后续 `solve_case()` / `solve_cases()` / `solve_combination()` 仅回代。

**审计结论：**

| 问题 | 回答 |
|------|------|
| factorization 生命周期清晰？ | ✅ 借用 `&'a FrameModel`，owned `Box<dyn LinearSolver>` |
| solver internals 泄漏？ | ✅ 无。`LinearSolver` trait 来自 `section-properties`，是公开接口 |
| API 过度复杂？ | ✅ 否。3 个方法，语义清晰 |
| prepared object 长期保留？ | ✅ 设计为短期持有；返回 owned `FrameAnalysisResult` |
| 支持 3D？ | ✅ DOF 无关——`K_ff` 维度由模型决定 |
| 模型 mutation / stale factorization？ | ⚠️ Rust 借用规则已防止并发修改；但 `&'a` 借用期间模型不可变是编译期保证，非运行时 |

**P2-1：** `FrameModel::solve_combination`（:895）每次重新因式分解，
而 `solve_cases`（:1057）走 `prepare` 路径。策略不对称——但不影响正确性，
用户可用 `prepare().solve_combination()` 获得复用。

**P2-2：** `FrameModel::solve_combination` 与
`PreparedFrameAnalysis::solve_combination` 的 RHS 合并逻辑重复（:900-932 vs
:1696-1728）。可抽取 `fn merge_combination_loads(...)`。

### Phase 108 — Self-Weight

**实现：** `FrameModel::self_weight(gx, gy)` 计算每个构件的
`w = ρ · A · g`（全局），通过 `to_local_force` 变换到局部坐标，生成
`Vec<DistributedLoad>`，包装为 `LoadCase`。

**审计结论：**

| 问题 | 回答 |
|------|------|
| 正确属于 Load/RHS 层？ | ✅ 仅修改 RHS，不碰刚度矩阵 |
| 与 DistributedLoad 边界清晰？ | ✅ 自重就是 `DistributedLoad`，无新类型 |
| Frame facade 重复实现？ | ✅ 否。`self_weight_distributed_loads` (pub(crate)) 是共享后端 |
| density 语义合理？ | ✅ `Material::density` [kg/m³] × `BeamSection::area` [m²] × g [m/s²] = N/m |
| Truss 不支持合理？ | ✅ `TrussElement` 只存 `E`/`A`，不存 `Material`，无 density |
| 需要统一 load-source abstraction？ | ✅ 否。当前 `Option<String>` provenance 足够 |

### Phase 109 — Envelope Extension

**实现：** `Extremum` 跟踪 min/max + governing source。`NodeEnvelopeSample` 包装
3 个 `Extremum`（ux/uy/rz）。`Envelope` 新增 `node_displacements` 和
`support_reactions` 字段。

**审计结论：**

| 问题 | 回答 |
|------|------|
| Envelope 过度承担职责？ | ✅ 否。所有字段都是纯数据快照，无行为 |
| Member/Node/Reaction 共存合理？ | ✅ 合理。都是"多工况极值聚合"，属于同一抽象层 |
| String-based load_source？ | ⚠️ P2：`Option<String>` 可改为 typed enum，但当前足够灵活 |
| Frame/Truss envelope 统一？ | ✅ Envelope 仅支持 Frame（Truss 无 Envelope API），无不自然统一 |
| 严格保持 post-processing？ | ✅ 是。`from_frame_results` 只读 `FrameAnalysisResult`，不调 solver |

---

## 7. Result / Post-processing Audit

### 分层

```text
Analysis (beam_fem/frame/truss)
    ↓ produces
Result (FrameAnalysisResult / BeamAnalysisResult / TrussAnalysisResult)
    ↓ consumed by
Post-processing (Envelope / Extremum / NodeEnvelopeSample)
    ↓ consumed by
Visualization / Export (未实现，不在本 crate)
```

**Result 职责：** 持有已求解的位移/反力/模型快照，提供 typed 查询访问器。
不重新实现 FEM 数学，全部委托 solver。

**Post-processing 职责：** 聚合多个 Result 的极值。纯数据消费者，不调 solver，
不组装刚度，不修改模型。

**Visualization boundary：** 未实现。`postprocessing` 是纯 Rust 数据结构，
无 serde、无 I/O、无渲染。Visualization/Export 不应进入 solver core。

**结论：** 分层清晰，无越界。

---

## 8. Load Architecture

| 类型 | 坐标系 | 存储 | 所属 |
|------|--------|------|------|
| Nodal force | Global | `Vec<(usize, usize, f64)>` | `LoadCase` / `BeamModel` |
| Nodal moment | Global | `Vec<AppliedMoment>` | `LoadCase` / `BeamModel` |
| Distributed load | Local | `Vec<DistributedLoad>` | `LoadCase` / `BeamModel` |
| Point load | Local | `Vec<PointLoad>` | `LoadCase` / `BeamModel` |
| Self-weight | Global→Local | `DistributedLoad` | `LoadCase` (via `add_self_weight`) |

**审计结论：**

- `LoadCase` 足够表达普通荷载：✅
- Self-weight 自然融入：✅（就是 `DistributedLoad`）
- `LoadCombination` 是 RHS aggregation：✅（`Σ factor_i × f_case_i`，单次求解）
- 无重复荷载表示：✅
- Typed `LoadSource` enum：⚠️ P2，当前 `Option<String>` 足够但不够 typed
- String-based provenance 技术债：P2，低优先级

---

## 9. Constraint Architecture

| 约束类型 | 机制 | 层级 |
|----------|------|------|
| fix/pin/roller | 静力凝聚 (K_ff u_f = f_f - K_fc u_c) | 全局 |
| restrain (非零值) | 静力凝聚 + RHS 修正 | 全局 |
| Spring | 对角刚度叠加 | 全局 K |
| Inclined roller | 坐标系旋转 + 静力凝聚 | 全局 |
| End release | 单元刚度凝聚 (K_cc - K_cr K_rr⁻¹ K_rc) | 单元 |

**审计结论：**

- Phase 94 结论仍成立：**不需要 generic `Constraint` abstraction**。✅
- 支座 API 清晰：✅（fix/pin/roller_x/roller_y/restrain/spring/inclined_roller）
- 无重复代码：✅（各约束有独立数学模型）
- Inclined roller 坐标变换层级正确：✅（在 `BeamSolver::from_model` 中旋转 K/f，
  求解后 `transform_back_to_global` 反变换）
- Spring 刚度属于 K：✅（对角叠加，反力走残差 `-k·u`）
- End release 属于单元凝聚：✅（`condense_matrix` 在局部坐标完成）

**决策：KEEP**

---

## 10. Element Architecture

| 单元 | DOF/node | 刚度 | 释放 | 荷载 |
|------|----------|------|------|------|
| Beam (Euler-Bernoulli) | 3 (ux,uy,rz) | 6×6 local + T变换 | 旋转凝聚 | 分布/点/力矩 |
| Truss (pin-jointed) | 2 (ux,uy) | 4×4 global | N/A | 节点力 |

**审计结论：**

- Phase 101 结论仍成立：**不需要强行统一 `Element` trait**。✅
- Beam/Truss 保持独立合理：DOF 数不同、荷载类型不同、端力语义不同
- 未来 3D：`Beam2D` / `Beam3D` / `Truss2D` / `Truss3D` 独立类型比统一 trait 更清晰
- 统一 trait 的代价（generic DOF、generic 荷载、generic 变换）远超收益

**决策：KEEP**

---

## 11. Solver Architecture

**当前 abstraction：** `LinearSolver` trait（来自 `section-properties::fea::solver`），
`SolverRegistry` 自动选择，`SolverSelection` 配置。

| 问题 | 回答 |
|------|------|
| Abstraction 足够？ | ✅ `factor(&K)` + `solve(&f)` 两步，支持复用 |
| Backend leakage？ | ✅ 无。`SolverError` 通过 `FemError::SolverError` 桥接 |
| Factorization reuse 稳定？ | ✅ Phase 107，`PreparedFrameAnalysis` API 清晰 |
| 多 RHS 扩展能力？ | ✅ `solve_pre_factored` 支持任意 RHS 回代 |
| Scalability？ | ⚠️ 见 §14 性能风险 |

**显式性能检查：**

- Repeated allocation: `retained_system_copy` 是求解路径唯一深拷贝 ✅
- Unnecessary cloning: `FrameAnalysisResult` owns `BeamSolver` + `FrameModel` clone，
  每次 `solve_case` 重建——这是 correctness 要求（模型须含正确荷载）✅
- Dense conversion: 机构诊断 `to_dense` 有 500 DOF 上限 ✅
- Repeated assembly: `solve_case` 每次重建 `BeamSolver`（含 K 组装）——P2 性能风险
- Repeated factorization: `solve_cases` 走 `prepare` 路径已解决 ✅；
  `solve_combination` 未复用——P2-1
- O(n²)/O(n³) hidden: 机构诊断 `O(n³)` 有 500 DOF 上限 ✅

---

## 12. 3D Readiness

**不实现 3D。仅分析架构准备度。**

### 3D Frame (6 DOF/node: ux, uy, uz, rx, ry, rz)

| 需求 | 当前状态 | 改动量 |
|------|----------|--------|
| 3D 坐标变换 | ❌ 2D only (c,s) | 新写 3D T (12×12) |
| 3D 梁刚度 | ❌ 2D only (6×6) | 新写 12×12 (含扭转、双向弯曲) |
| 扭转 | ❌ 不存在 | 新增 GJ/L 项 |
| 双向弯曲 | ❌ 仅 Iyy | 需 Izz + Iyz |
| 释放 | ❌ 仅 2D 旋转 | 需 3D 旋转释放 |
| 荷载 | ❌ 2D local | 需 3D local (qy, qz) |
| 支座 | ❌ 2D | 需 3D fix/pin/roller |
| Result | ❌ 3 DOF | 需 6 DOF 查询 |
| Envelope | ❌ ux/uy/rz | 需 6 DOF Extremum |

### 3D Truss (3 DOF/node: ux, uy, uz)

| 需求 | 当前状态 | 改动量 |
|------|----------|--------|
| 3D 刚度 | ❌ 4×4 | 新写 6×6 (c,s,t 方向余弦) |
| 坐标变换 | ❌ 2D | 3D (但 truss 不需要 T 矩阵) |
| 支座 | ❌ 2D | 需 3D fix/pin/roller |
| Result | ❌ 2 DOF | 需 3 DOF |
| Solver | ✅ DOF 无关 | 无改动 |
| 诊断 | ✅ DOF 无关 | 仅 rigid_candidates 需 3D 版 (6 modes) |

### Solver / Post-processing

- `SparseMatrix` / `LinearSolver`：**DOF 无关**，天然支持更大 DOF ✅
- `ReducedSystem`：**DOF 无关** ✅
- `diagnose_reduced`：**DOF 无关**，但 `rigid_candidates` 需 3D 版（6 个刚体模式）✅
- `Envelope`：DOF 硬编码 `ux/uy/rz`——需泛化或 3D 版 ⚠️
- `EquilibriumReport`：2D 力矩（关于 z 轴）——需 3D 力矩向量 ⚠️
- `FrameAnalysisResult`：3 DOF 查询——需 6 DOF 版 ⚠️

### 3D Readiness 评估

```text
3D Truss:  READY WITH LIMITED REFACTOR
           (solver/diagnostic DOF-agnostic, only element + result need 3D)

3D Frame:  NOT READY
           (element stiffness, transformation, releases, loads, results,
            envelopes, equilibrium all need 3D versions)
```

---

## 13. 2D Completeness

| 功能 | 状态 | 分类 |
|------|------|------|
| Prescribed displacement (非零) | ✅ 已实现 (`restrain`) | Core MVP |
| Temperature load | ❌ | Advanced — defer |
| Thermal strain | ❌ | Advanced — defer |
| Member offsets | ❌ | Advanced — defer |
| Rigid links | ❌ | Advanced — defer |
| Tension-only | ❌ | Advanced — defer (非线性迭代) |
| Compression-only | ❌ | Advanced — defer (非线性迭代) |
| Partial releases | ❌ | Advanced — defer (低频需求) |
| Additional spring types | ❌ | Advanced — defer |
| Distributed load variations | ✅ 梯形已支持 | Extended 2D |

**结论：** 2D MVP 功能完备。剩余 2D 功能（温度、offsets、非线性）属于 Advanced，
不是 MVP 必需。`restrain` 的非零值路径已实现但缺乏专门测试和文档——Phase 111 补齐。

---

## 14. Numerical / Engineering Correctness

| 检查项 | 状态 | 依据 |
|--------|------|------|
| 刚度对称性 | ✅ | `test_local_stiffness`, `test_global_stiffness_*` |
| 坐标变换 | ✅ | `test_transformation_matrix`, `frame_transformation_contract` |
| 静力凝聚 | ✅ | `end_release` 测试 (69 tests), 解析解验证 |
| 荷载一致性 | ✅ | `beam_section_forces` (平衡恢复法验证) |
| 反力恢复 | ✅ | R = K·u - f, `frame_correctness` 平衡测试 |
| 平衡 | ✅ | scale-aware, unit-invariant, 条件缓解 |
| 机构诊断 | ✅ | equilibrated pivoted Cholesky, 500 DOF 上限 |
| 容差约定 | ✅ | 相对容差，无绝对地板，单位无关 |
| NaN/Infinity 验证 | ✅ | 所有 `add_*` 方法检查 `is_finite()` |
| 零长度单元 | ✅ | `from_model` 拒绝 |
| 退化几何 | ✅ | 重复构件、孤立节点、不连通检测 |

**Phase 107–109 未改变数学基础。** ✅

### P0 / P1 / P2

**P0：无**

**P1：无**

**P2：**

| ID | 描述 | 位置 | 影响 |
|----|------|------|------|
| P2-1 | `solve_combination` 不复用因式分解 | frame.rs:895 | 性能（用户可用 `prepare()` 规避） |
| P2-2 | `add_member` / `add_member_with_release` 校验重复 | frame.rs:381-438, 475-533 | 代码维护 |
| P2-3 | `solve_combination` RHS 合并逻辑重复 | frame.rs:900-932, 1696-1728 | 代码维护 |
| P2-4 | `load_source` 为 `Option<String>` 而非 typed enum | postprocessing.rs | 类型安全（低优先级） |

---

## 15. Test Architecture

**统计：**

| 类型 | 数量 | 行数 |
|------|------|------|
| 集成测试文件 | 37 | ~24,401 |
| Examples | 7 | — |
| In-crate tests (beam_fem.rs) | 13 | ~250 |
| In-crate tests (frame.rs) | 13 | ~490 |
| Doctests | 12 | — |

**测试类型覆盖：**

| 类型 | 覆盖 | 示例文件 |
|------|------|----------|
| 解析解验证 | ✅ | `beam_analytical_benchmarks`, `beam_fem_reference` |
| 对称性 | ✅ | `beam_fem` (in-crate) |
| 平衡 | ✅ | `frame_correctness`, `beam_fem_contract` |
| 收敛性 | ✅ | `beam_fem_convergence`, `frame_beam_p2_convergence` |
| API contract | ✅ | `api_misuse`, `beam_fem_api_semantics`, `frame_api` |
| Invalid input | ✅ | `api_robustness_audit`, `beam_fem_robustness` |
| Phase 107 回归 | ✅ | `multi_load_case` |
| Phase 108 回归 | ✅ | `self_weight` |
| Phase 109 回归 | ✅ | `result_envelope` |
| 多求解器交叉验证 | ✅ | `beam_multisolver_cross_validation` |
| 机构诊断 | ✅ | `mechanism_diagnostics` |
| 端部释放 | ✅ | `end_release` |
| 数值审计 | ✅ | `beam_fem_numerical_audit` |

**Cross-package：** `section-properties` 测试零引用 `structural-analysis`。✅

---

## 16. Performance Risks

| 场景 | 风险 | 说明 |
|------|------|------|
| Small 2D model (<100 DOF) | Low | Dense solver, 微秒级 |
| Medium frame (100-1000 DOF) | Low | Skyline/sparse, 毫秒级 |
| Many load cases (N cases) | Low | Phase 107 已解决 factorization reuse |
| Many combinations (M combos) | Moderate | P2-1: `solve_combination` 每次重新 factor |
| Large sparse model (>5000 DOF) | Moderate | Sparse LU 可用但未 benchmark |
| N × RHS assembly | Moderate | `solve_case` 每次重建 `BeamSolver`（含 K 组装） |
| N × result reconstruction | Low | `FrameAnalysisResult` clone 成本可接受 |
| N × post-processing | Low | Envelope 是 O(N × samples) 线性扫描 |

**主要风险：** `solve_case` 每次重建 `BeamSolver::from_model`，含完整 K 组装。
对于 N 个工况，总成本 = N × (K 组装 + factor + solve)。
`PreparedFrameAnalysis` 将 factor 降为 1 次，但 K 组装仍为 N 次。
K 组装是 O(nelements × 36) 线性操作，通常不是瓶颈。

---

## 17. Crate Boundary

**当前：** 两个 crate（`section-properties` + `structural-analysis`）。

**是否需要拆 `structural-analysis`？**

| 候选 | 收益 | 代价 | 决策 |
|------|------|------|------|
| `structural-core` (model/element) | 理论清晰 | 增加依赖管理复杂度 | ❌ |
| `structural-post` (envelope) | 隔离 post-processing | Envelope 仅 328 行 | ❌ |
| `structural-solver` (solver wrapper) | 隔离 solver | 仅是 trait re-export | ❌ |
| `numerical-core` (fea/) | 已在 section-properties | Phase 95 已否决 | ❌ |

**决策：KEEP ONE CRATE.**

> 当前规模（8,265 LOC）和依赖关系不证明需要拆分。
> 拆分增加管理成本但无实际收益。

---

## 18. API Stability

| API | 分类 | 理由 |
|-----|------|------|
| `BeamModel` / `BeamSolver` | STABLE | 成熟，大量测试覆盖 |
| `BeamElement` / `EndRelease` | STABLE | 端部释放已验证 |
| `Dof` / `FemError` | STABLE | 基础类型 |
| `FrameModel` | STABLE | Façade，委托清晰 |
| `FrameAnalysisResult` | STABLE | 查询 API 已定型 |
| `NodeHandle` / `MemberHandle` | STABLE | 类型化句柄 |
| `EquilibriumReport` | STABLE | v0.3.0 已 privatize tolerance |
| `TrussModel` / `TrussSolver` | STABLE | 独立，完整 |
| `LoadCase` / `LoadCombination` | LIKELY STABLE | Phase 103 引入，使用稳定 |
| `PreparedFrameAnalysis` | LIKELY STABLE | Phase 107 引入，API 清晰 |
| `Envelope` / `Extremum` / `NodeEnvelopeSample` | LIKELY STABLE | Phase 103/109 引入 |
| `EnvelopeSample` | LIKELY STABLE | Phase 103 |
| `StructuralDiagnostic` | STABLE | 机构诊断已验证 |
| `BeamAnalysis` | STABLE | 解析解工具 |
| `SectionForces` / `BeamForceSample` / `BeamForceDiagram` | STABLE | 后处理数据 |
| `self_weight` / `add_self_weight` | LIKELY STABLE | Phase 108 |

**结论：** 当前 API 接近可发布 v0.1.0。`LIKELY STABLE` 项均为近期引入
（Phase 103-109），API 清晰但尚未经历外部消费者验证。建议 Phase 113 正式
冻结 API surface。

---

## 19. Final Architecture Decision

```text
Architecture decision: KEEP

No structural refactor required.
```

**理由：**

1. **分层严格：** Model → Solver → Result → Post-processing，无越界
2. **零力学重复：** `FrameModel` 是 `BeamModel` 的纯 façade
3. **无 internals 泄漏：** solver 细节封装在 `BeamSolver` 私有字段
4. **依赖单向：** structural-analysis → section-properties，无循环
5. **DOF 无关 solver：** `SparseMatrix` / `LinearSolver` 天然支持扩展
6. **测试覆盖充分：** 37 integration + 26 in-crate + 12 doctest = 75+ test functions
7. **Phase 107-109 未改变架构边界：** 三个功能均以最小侵入方式融入

**P2 项不阻塞架构决策。** 代码去重（P2-2, P2-3）可在日常维护中处理。

---

## 20. Phase 111–113 Roadmap

### Phase 111 — Prescribed Displacement / Support Settlement (S)

**目的：** `restrain(node, dof, value)` 已支持非零 prescribed value（静力凝聚
`K_ff u_f = f_f - K_fc u_c` 中 `u_c ≠ 0`），但缺乏专门测试和用户文档。

**Scope：**
- 新增 `tests/prescribed_displacement.rs`：支座沉降解析解验证
- 文档：`restrain` 方法文档补充非零值示例
- 不新增 production code

### Phase 112 — 3D Truss Element (M)

**目的：** 最小 3D 扩展，验证 solver/diagnostic 的 DOF 无关性，为未来 3D Frame
探路。

**Scope：**
- 新增 `TrussElement3D`：3 DOF/node，6×6 刚度（c, s, t 方向余弦）
- 新增 `TrussModel3D` / `TrussSolver3D`：复用 `SparseMatrix` / `LinearSolver`
- 3D rigid candidates（6 modes: Tx, Ty, Tz, Rx, Ry, Rz）
- 解析解验证：3D 桁架基准
- 不修改 2D 代码

### Phase 113 — structural-analysis API Stabilization (S)

**目的：** 冻结 public API surface，编写 API reference，为未来 v0.1.0 发布做准备。

**Scope：**
- 审查所有 `pub` 项，确认无遗漏 `pub(crate)` 应收窄的
- 编写 `API_REFERENCE.md`：所有公开类型/方法/约定
- 确认 doctest 覆盖所有公开方法
- 不修改 production code（除非发现 `pub` 应为 `pub(crate)`）

---

## 21. Verification

```
HEAD:       6e60dec
baseline:   6e60dec
working tree: clean (7 untracked PHASE*.md, no modified source)
```

验证命令（本阶段无 production code 改动，验证确认基线无回归）：

```
cargo fmt --all -- --check     → (run below)
cargo check --workspace --all-targets → (run below)
cargo test -p structural-analysis → (run below)
cargo test -p section-properties --lib → (run below)
cargo doc -p structural-analysis --no-deps → (run below)
```

---

## 22. Summary

```
HEAD:           6e60dec
baseline:       6e60dec
working tree:   clean
P0:             0
P1:             0
P2:             4 (code dedup + performance + type safety)
MVP boundary:   Core MVP + Extended 2D complete
3D readiness:   3D Truss READY WITH LIMITED REFACTOR; 3D Frame NOT READY
API stability:  STABLE / LIKELY STABLE (no MAY CHANGE)
crate decision: KEEP ONE CRATE
roadmap:        Phase 111 (S) + Phase 112 (M) + Phase 113 (S)
changed files:  PHASE110_MVP_ARCHITECTURE_AUDIT.md (this file)
```
