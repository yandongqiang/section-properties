# Phase 90 — structural-analysis 功能完整性与领域模型 Gap Audit

**日期**: 2026-09-23
**Baseline**: HEAD = `f7db808` (Phase 89)
**工具链**: rustc 1.97.1 / cargo 1.97.1
**性质**: AUDIT ONLY — 不修改生产代码

---

## 1. Baseline

| 项目 | 值 |
|------|-----|
| HEAD | `f7db8081a7e3e26099b6d4eda8fb9bd1d75d1702` |
| 工作树 | clean |
| section-properties | v0.4.0 已发布至 crates.io (tag `section-properties-v0.4.0` → `e6264bb`) |
| structural-analysis | v0.1.0 workspace-only，未发布 |
| Rust | 1.97.1 (2026-07-14) |
| 测试文件 | 26 个，18,529 行 |
| Examples | 4 个 |

---

## 2. Current Capability Map

| 能力 | 状态 | 实现位置 | 备注 |
|------|------|----------|------|
| 2D 梁单元 (Euler-Bernoulli) | ✅ | `beam_fem.rs` | 6 DOF, 一致质量矩阵 |
| 2D 框架分析 | ✅ | `frame.rs` | FrameModel 为 BeamModel façade |
| 线性静力分析 | ✅ | `FrameModel::solve()` | 静态凝聚法 |
| 多求解器后端 | ✅ | `SolverRegistry` | dense / skyline_ldlt / sparse_lu / cg / iccg |
| 求解器选择 API | ✅ | `solve_with(selection)` | Auto / Named |
| 机构检测 | ✅ | `mechanism.rs` | 5 变体诊断 |
| 内力 (N, V, M) | ✅ | `BeamAnalysisResult::section_forces` | Beam 层 |
| 反力 | ✅ | `reactions()` | |
| 全局平衡检查 | ✅ | `EquilibriumReport` | Frame 层 |
| 力图采样 | ✅ | `sample_forces` / `diagram` | Beam 层 |
| 均布荷载 | ✅ | `DistributedLoad` | 局部坐标 |
| 点荷载 | ✅ | `PointLoad` | 局部坐标, 含弯矩 |
| 节点弯矩 | ✅ | `AppliedMoment` | 全局坐标 |
| 节点力 | ✅ | `nodal_forces` | 全局坐标 |
| 固定支座 | ✅ | `fix()` | ux=uy=rz=0 |
| 铰支座 | ✅ | `pin()` | ux=uy=0 |
| 滚动支座 (y) | ✅ | `roller_y()` | uy=0 |
| 滚动支座 (x) | ✅ | `roller_x()` | ux=0 |
| 任意 DOF 约束 | ✅ | `restrain()` | 预定位移 |
| 矩形/圆形截面 | ✅ | `BeamSection` | 含空心圆 |
| 从 SectionProperties 转换 | ✅ | `from_section_properties_x/y` | 显式轴选择 |
| 端部释放/铰接 | ❌ | — | P1 gap |
| 荷载工况/组合 | ❌ | — | P1 gap |
| Frame 层 section_forces | ❌ | — | P2 gap |
| 弹簧支座 | ❌ | — | P2 gap |
| 梯形/三角形分布荷载 | ❌ | — | P2 gap |
| 斜向 roller | ❌ | — | P2 gap |
| 非线性分析 | ❌ | — | Future |
| 屈曲分析 | ❌ | — | Future |
| 模态/动力分析 | ❌ | — | Future |
| P-Delta / 二阶分析 | ❌ | — | Future |
| 3D 框架 | ❌ | — | Future |

---

## 3. Current Domain Model

```
                    ┌──────────────────────────────────┐
                    │         section-properties        │
                    │  (v0.4.0 @ crates.io)            │
                    │  SectionProperties, Material,     │
                    │  fea::solver::{SolverRegistry,    │
                    │    SolverSelection, LinearSolver} │
                    └──────────────┬───────────────────┘
                                   │ depends on
                                   ▼
┌─────────────────────────────────────────────────────────────┐
│                    structural-analysis                       │
│                    (v0.1.0 workspace-only)                  │
│                                                              │
│  ┌─────────────────────────────────────────────────────┐    │
│  │                    frame.rs (façade)                  │    │
│  │  FrameModel { inner: BeamModel }                     │    │
│  │  ├── NodeHandle / MemberHandle (newtype<usize>)      │    │
│  │  ├── add_node / add_member / add_member_with_length  │    │
│  │  ├── fix / pin / roller_x / roller_y / restrain      │    │
│  │  ├── add_load / add_distributed_load / add_moment    │    │
│  │  ├── solve / solve_with / check_equilibrium          │    │
│  │  └── FrameAnalysisResult → delegates to BeamSolver   │    │
│  └────────────────────┬────────────────────────────────┘    │
│                       │ owns                                 │
│                       ▼                                      │
│  ┌─────────────────────────────────────────────────────┐    │
│  │                beam_fem.rs (core mechanics)           │    │
│  │  BeamModel                                           │    │
│  │  ├── BeamNode { id, x, y }                           │    │
│  │  ├── BeamElement { node_i, node_j, material, section }│   │
│  │  ├── BeamSection { area, second_moment }             │    │
│  │  ├── Dof { Ux, Uy, Rz }                              │    │
│  │  ├── Loads: DistributedLoad / PointLoad /            │    │
│  │  │         AppliedMoment / nodal_forces              │    │
│  │  ├── consistent_nodal_load (均布)                     │    │
│  │  ├── consistent_nodal_load_point (点)                 │    │
│  │  ├── solve / solve_with (静态凝聚)                    │    │
│  │  ├── section_forces → SectionForces { N, V, M }      │    │
│  │  ├── reactions / sample_forces / diagram             │    │
│  │  └── BeamAnalysisResult<'a> (borrows BeamSolver)     │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                              │
│  ┌─────────────────────────────────────────────────────┐    │
│  │              mechanism.rs (diagnostics)               │    │
│  │  StructuralDiagnostic:                              │    │
│  │    Stable / RigidBodyMode / Mechanism /              │    │
│  │    IllConditioned / Indeterminate                    │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                              │
│  FemError (11 variants, thiserror, From<SolverError>)       │
└─────────────────────────────────────────────────────────────┘
```

**关键架构特征**: `FrameModel` 是 `BeamModel` 的纯 façade (`inner: BeamModel`)，零力学重复。Frame 层增加类型化 handle、连接验证、支座词汇、平衡报告、机构诊断。Beam 层独有 `section_forces` 和力图采样。

---

## 4. Architecture Assessment

### 4.1 模块边界 — **合理**
- `beam_fem.rs` (核心力学) / `frame.rs` (用户 API façade) / `mechanism.rs` (诊断) 三模块职责清晰
- 无循环依赖，无跨层泄漏

### 4.2 Frame ↔ Beam 关系 — **合理**
- FrameModel 纯 façade，零力学重复
- Frame 层增加工程词汇 (handle, 支座类型, 平衡报告) 而非重复力学
- 唯一不足: FrameAnalysisResult 不暴露 section_forces (P2)

### 4.3 求解器集成 — **合理**
- 通过 `section-properties::fea::solver` 复用已发布的求解器基础设施
- `SolverSelection` (Auto/Named) 作为公共 API 参数
- 错误传播结构化 (`From<SolverError> for FemError`)
- 无自动重试/回退 — 当前可接受，未来可加

### 4.4 错误模型 — **合理**
- `FemError` 11 变体 + `thiserror` 派生
- `SolverError { source, message }` 保留结构化信息，不 flatten
- `solver_error()` accessor 可提取底层 solver 错误

### 4.5 材料/截面模型 — **合理**
- `BeamSection` 仅需 `area` + `second_moment`，最小化
- 故意不提供 `From<SectionProperties>` — 必须显式 `_x` 或 `_y` 避免轴选择错误
- `Material` 仅用 `youngs_modulus`；密度/泊松比未用于静力分析 (正确)

### 4.6 荷载/边界条件模型 — **基本合理，有扩展空间**
- 荷载: 均布 + 点 + 弯矩 + 节点力 — 覆盖常见工况
- BC: fix/pin/roller/restrain — 覆盖常见支座
- 缺失: 荷载工况、梯形荷载、弹簧支座、斜向 roller — 均为已知 gap

---

## 5. Functional Gaps

### P0 — 阻塞发布 (0 项)
无。

### P1 — 核心功能缺失 (2 项)

| # | Gap | 影响 | 代码证据 |
|---|-----|------|----------|
| 1 | **端部释放/铰接** | 无法建模铰接框架、连续梁铰、半刚性连接 | `frame.rs:35` 明确声明 "Rigid offsets / releases are not modelled." |
| 2 | **荷载工况/组合** | 无法组织多工况分析 (恒载/活载/风载)，无法做 1.4D+1.6L 组合 | 无 LoadCase/LoadCombination 类型，荷载直接存于 BeamModel |

### P2 — 完善性缺失 (4 项)

| # | Gap | 影响 |
|---|-----|------|
| 3 | Frame 层缺 `section_forces` | 用户需降级到 Beam 层获取内力 |
| 4 | 弹簧支座 | 无法建模弹性地基、柔性支座 |
| 5 | 梯形/三角形分布荷载 | 无法建模线性变化分布荷载 |
| 6 | 斜向 roller | 无法建模斜向滚动支座 (罕见但偶尔需要) |

### Future — 超出 MVP 范围

- 3D 框架 (需全新单元、坐标系、自由度)
- 非线性分析 (材料/几何非线性)
- 屈曲分析 (特征值屈曲)
- 模态/动力分析 (质量矩阵、特征值)
- P-Delta / 二阶分析

### Out of Scope — 明确不做

- PARDISO / MKL / oneAPI 求解器
- 第三个 FEA crate
- 过度泛化的 FEM 抽象 (泛型单元/材料/荷载 trait 层叠)
- Timoshenko 梁 (当前 Euler-Bernoulli 足够用)

---

## 6. Test Capability Review

### 测试分类 (26 文件, 18,529 行)

| 类别 | 文件数 | 行数 | 覆盖内容 |
|------|--------|------|----------|
| API 契约 | 7 | ~2,159 | smoke, contract, misuse, robustness, semantics, ergonomics |
| 解析验证 | 5 | ~4,431 | 闭式解对比, 参考解, 收敛率, P2 收敛 |
| 数值/单元 | 5 | ~5,927 | 核心单元, section_forces, 力图, 求解器后端, 交叉验证 |
| Frame 集成 | 4 | ~3,088 | frame API, correctness, 变换, frame-beam 集成 |
| 求解器选择 | 2 | ~1,302 | selection, robustness |
| 机构诊断 | 1 | 611 | 5 种诊断变体 |
| 分析结果 | 1 | 489 | BeamAnalysisResult |
| FEM 鲁棒性 | 1 | 709 | 边界情况, 退化输入 |

### 测试覆盖评估

**已覆盖**:
- ✅ API 契约 (输入验证、错误路径、类型安全)
- ✅ 解析验证 (闭式解、收敛率)
- ✅ 数值精度 (多后端交叉验证)
- ✅ 机构检测 (5 种诊断变体)
- ✅ 求解器选择 (Auto/Named, 回退行为)
- ✅ Frame-Beam 集成 (façade 一致性)

**缺失 (与功能缺失对齐)**:
- ❌ 端部释放测试 (功能不存在)
- ❌ 荷载组合测试 (功能不存在)
- ❌ 弹簧支座测试 (功能不存在)
- ❌ 非线性/动力测试 (超出范围)

**结论**: 测试覆盖与已实现功能高度对齐，无遗漏测试。缺失测试均对应未实现功能。

---

## 7. MVP Definition

**当前状态已达到 2D Frame/Beam 线性静力分析 MVP (级别 B)**。

MVP 范围:
- 2D Euler-Bernoulli 梁单元
- 线性静力分析 (静态凝聚)
- 均布/点/弯矩/节点荷载
- fix/pin/roller/restrain 支座
- 多求解器后端 (5 种)
- 机构检测
- 内力/反力/平衡检查
- 力图采样

**MVP 已完成**。后续为功能扩展。

---

## 8. Post-MVP Roadmap (Phase 91+ 建议)

### Phase 91 — 端部释放/铰接 (P1)
- 实现 `add_member_with_release(node_i_release, node_j_release)`
- 释放 DOF 通过静力凝聚处理 (修改单元刚度矩阵)
- 支持铰接 (释放 Rz)、滑动铰 (释放 Ux 或 Uy)、全释放
- 测试: 铰接悬臂梁、连续梁铰、铰接门架

### Phase 92 — 荷载工况/组合 (P1)
- 引入 `LoadCase { name, loads }` 和 `LoadCombination { factors }`
- `FrameModel::solve_load_cases(&[LoadCase])` → 多工况批量分析
- `LoadCombination::combine(&results)` → 结果组合
- 测试: 1.4D+1.6L 组合、多工况包络

### Phase 93 — Frame 层 section_forces + 完善性 (P2)
- `FrameAnalysisResult::section_forces(member, position)` 委托 Beam 层
- 弹簧支座 (在 DOF 级别添加弹簧刚度)
- 梯形分布荷载 (线性插值)
- 斜向 roller (坐标变换)

### Phase 94+ — Future (按需)
- 3D 框架
- 屈曲分析
- 模态分析
- 非线性

---

## 9. "Do Not Build Yet" 列表

以下功能 **明确不在当前阶段实现**，避免过度工程:

1. **3D 框架** — 需全新单元/坐标系/自由度，工作量巨大，当前 2D 足够用
2. **非线性分析** — 需迭代求解器、切线刚度、收敛控制，架构冲击大
3. **模态/动力分析** — 需质量矩阵、特征值求解器，与静力分析架构差异大
4. **泛型 FEM 抽象** — trait 层叠 (单元/材料/荷载泛型化) 会增加复杂度而无实际收益
5. **Timoshenko 梁** — Euler-Bernoulli 对当前截面尺寸/跨度比足够
6. **PARDISO/MKL** — 已明确排除，保持纯 Rust + 开源求解器
7. **第三个 FEA crate** — 已明确排除

---

## 10. Final Conclusion

| 维度 | 评估 |
|------|------|
| 架构成熟度 | **A** — 模块边界清晰，façade 模式正确，零力学重复 |
| 功能完整性 | **B** — 线性静力 MVP 已完成，缺铰接和荷载组合 |
| 测试覆盖 | **A** — 26 文件 18.5K 行，与已实现功能高度对齐 |
| API 设计 | **A** — 类型化 handle，显式轴选择，结构化错误 |
| 求解器集成 | **A** — 复用 section-properties，5 后端，结构化错误传播 |
| 发布就绪 | **B** — workspace-only 符合定位，缺 P1 功能后可考虑发布 |

**总体: B+ — 可用的 2D Frame/Beam MVP，架构扎实，有明确的扩展路线图。**

**下一步**: Phase 91 实现端部释放/铰接 (P1 gap #1)。
