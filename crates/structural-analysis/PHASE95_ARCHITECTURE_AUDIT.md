# Phase 95 — Section Properties / Structural Analysis 最终架构边界审查

## 1. 当前实际架构

### 1.1 Workspace 结构

```
section-properties/                    (workspace root)
├── Cargo.toml                         [workspace] members = [section-properties, structural-analysis]
├── crates/
│   ├── section-properties/            v0.4.0, 已发布 crates.io
│   │   ├── Cargo.toml                 依赖: serde, serde_json, csv, thiserror
│   │   ├── src/
│   │   │   ├── lib.rs                 119 行, 模块声明 + re-exports
│   │   │   ├── geometry.rs + geometry/
│   │   │   ├── section.rs + section/
│   │   │   ├── section_properties.rs
│   │   │   ├── composite.rs
│   │   │   ├── material.rs + material/
│   │   │   ├── fea.rs + fea/          (solver 基础设施)
│   │   │   │   ├── solver.rs + solver/impls/  (LinearSolver, SolverRegistry, 5 backends)
│   │   │   │   └── solvers.rs                  (SparseLu, Ic0Factor 等 legacy)
│   │   │   ├── mesh.rs + mesh/
│   │   │   ├── plastic.rs + plastic/  (含 warping_fem.rs)
│   │   │   ├── stress.rs + stress_fem.rs
│   │   │   ├── cold_formed_analysis.rs + cold_formed_analysis/
│   │   │   ├── fire.rs + fire/
│   │   │   ├── database.rs + database/
│   │   │   ├── section_library.rs + section_library/
│   │   │   ├── io.rs + io/
│   │   │   └── post.rs + post/
│   │   └── tests/                     31 个测试文件
│   │
│   └── structural-analysis/           v0.1.0, workspace-only (未发布)
│       ├── Cargo.toml                 依赖: section-properties (path, v0.4), thiserror
│       ├── src/
│       │   ├── lib.rs                 45 行, 模块声明 + re-exports
│       │   ├── beam_fem.rs            4253 行 (BeamModel, BeamSolver, BeamElement, EndRelease, ...)
│       │   ├── frame.rs               2010 行 (FrameModel, FrameSolver, FrameAnalysisResult, ...)
│       │   ├── load.rs                307 行 (LoadCase, LoadCombination)
│       │   └── mechanism.rs           578 行 (StructuralDiagnostic, diagnose_reduced)
│       ├── tests/                     32 个测试文件
│       └── examples/                  4 个示例
```

### 1.2 实际依赖关系图

```
structural-analysis (v0.1.0, workspace-only)
        │
        ↓  section-properties (path v0.4)
        │
section-properties (v0.4.0, crates.io)
```

**单向依赖，无反向依赖，无循环依赖。**

### 1.3 structural-analysis → section-properties 具体依赖

| 导入项 | 用途 | 性质 |
|--------|------|------|
| `fea::SparseMatrix` | 全局刚度矩阵存储 | 数值基础设施 |
| `fea::solver::LinearSolver` | 求解器 trait | 数值基础设施 |
| `fea::solver::SolverError` | 求解器错误类型 | 数值基础设施 |
| `fea::solver::SolverRegistry` | 求解器选择/创建 | 数值基础设施 |
| `fea::solver::SolverSelection` | 求解器选择枚举 | 数值基础设施 |
| `fea::PIVOT_TOL_BASE` | Cholesky pivot 容差常数 | 数值基础设施 |
| `geometry::Point` | 2D 点 | 几何基础类型 |
| `material::Material` | 弹性模量、泊松比、密度 | 材料属性 |

### 1.4 section-properties → structural-analysis 反向依赖

**无。** `grep "use crate::(beam_fem|frame|load|mechanism)"` 在 section-properties/src 中返回 0 匹配。

## 2. 模块归属表

| 模块 | 当前归属 | 建议归属 | 原因 |
|------|----------|----------|------|
| geometry (Point, Polygon) | section-properties | section-properties | 截面几何基础 |
| section (Section, FrameProperties) | section-properties | section-properties | 截面定义 |
| section_properties (Area, I, J, W) | section-properties | section-properties | 截面属性计算 |
| composite | section-properties | section-properties | 组合截面 |
| material (Material) | section-properties | section-properties | 材料属性（被两个 crate 共用） |
| fea (SparseMatrix, solver) | section-properties | section-properties | 数值基础设施（warping FEM 也依赖） |
| fea/solvers (SparseLu, Ic0Factor) | section-properties | section-properties | warping FEM 直接依赖 |
| mesh | section-properties | section-properties | 截面网格划分 |
| plastic (warping_fem) | section-properties | section-properties | 截面级 FEM |
| stress / stress_fem | section-properties | section-properties | 截面应力分析 |
| cold_formed_analysis | section-properties | section-properties | 冷弯分析 |
| fire | section-properties | section-properties | 防火分析 |
| database / section_library | section-properties | section-properties | 标准截面库 |
| io / post | section-properties | section-properties | 截面 I/O 与后处理 |
| beam_fem (BeamModel, BeamSolver) | structural-analysis | structural-analysis | 杆系结构分析 |
| frame (FrameModel, FrameSolver) | structural-analysis | structural-analysis | 框架结构分析 |
| load (LoadCase, LoadCombination) | structural-analysis | structural-analysis | 结构荷载 |
| mechanism (StructuralDiagnostic) | structural-analysis | structural-analysis | 结构机构诊断 |

**结论：当前归属与建议归属完全一致。无需迁移。**

## 3. Public API 污染

### 3.1 section-properties 公共导出

| 类别 | 导出项 | 是否属于结构分析 |
|------|--------|-----------------|
| 几何 | Point, Polygon, Axis, CompoundGeometry, ... | 否 |
| 截面 | Section, FrameProperties, TransformedFrameProperties | 否 |
| 截面属性 | SectionProperties, GeometricProperties, PrincipalProperties, ... | 否 |
| 组合 | CompositeComponent, ElasticComposite, ... | 否 |
| 材料 | Material | 否 |
| FEM 基础 | FemError, solver, SolverSelection, SolverRegistry, SolverError, SolverCapabilities, SelectionReason | 否（数值基础设施） |
| 网格 | Mesh, MeshParams, FemWarpingProperties, ... | 否 |
| 塑性 | PlasticAnalysis, WarpingProperties, TorsionAnalysis, ... | 否 |
| 应力 | StressAnalysis, SectionLoads, ... | 否 |
| 冷弯 | ColdFormedSection, EffectiveSectionProperties, ... | 否 |
| 防火 | FireAnalysis, FireResistanceResult, ... | 否 |
| 数据库 | SectionDatabase, SectionEntry, ... | 否 |
| I/O | SectionExporter, to_csv, to_json, to_dxf, ... | 否 |

### 3.2 section-properties 中不存在以下类型

```
Frame, FrameModel, FrameSolver, FrameAnalysisResult
BeamModel, BeamSolver, BeamElement, BeamNode
LoadCase, LoadCombination, DistributedLoad, PointLoad
EndRelease, Dof, NodeHandle, MemberHandle
EquilibriumReport, StructuralDiagnostic
```

**结论：section-properties 公共 API 无结构分析污染。**

### 3.3 FemError 同名碰撞

| crate | 类型路径 | 变体数 | 服务领域 |
|-------|---------|--------|---------|
| section-properties | `section_properties::fea::FemError` | 6 | 截面级 FEM（Tri6/warping/stress） |
| structural-analysis | `structural_analysis::beam_fem::FemError` | 11+ | 梁/框架 FEM |

不同 crate、不同模块路径、不同变体集，Rust 类型系统完全区分。**无碰撞问题。**

## 4. FEM / Solver 边界

### 4.1 Section FEM（留在 section-properties）

| 模块 | 用途 | 依赖 |
|------|------|------|
| `fea.rs` (Tri6) | 六节点三角形截面单元 | geometry, material |
| `fea/solver.rs` | 统一求解器抽象 (LinearSolver trait) | fea::SparseMatrix |
| `fea/solver/impls/` | 5 个后端: Dense, SkylineLDLT, SparseLU, CG, ICCG | fea::solver |
| `fea/solvers.rs` | Legacy 求解器 (SparseLu, Ic0Factor) | fea::solver |
| `plastic/warping_fem.rs` | 扭转/翘曲 FEM | fea::solvers, mesh |
| `stress_fem.rs` | 截面应力 FEM | fea, mesh |

### 4.2 Structural FEM（已在 structural-analysis）

| 模块 | 用途 | 依赖 |
|------|------|------|
| `beam_fem.rs` | 2D Euler-Bernoulli 梁单元 | section-properties::fea, geometry, material |
| `frame.rs` | Frame façade | beam_fem, load, mechanism |
| `mechanism.rs` | 机构诊断 | section-properties::fea |

### 4.3 Solver 共享分析

structural-analysis 通过 `section_properties::fea::solver` 使用：
- `LinearSolver` trait — 求解器接口
- `SolverRegistry` — 后端选择
- `SolverSelection` — 选择枚举
- `SparseMatrix` — 稀疏矩阵
- 5 个后端实现（Dense, SkylineLDLT, SparseLU, CG, ICCG）

section-properties 内部也使用同一套求解器基础设施（warping FEM、stress FEM）。

**关键判断：求解器是否应独立为第三个 crate？**

- `fea/solvers.rs` 依赖 `fea::solver::SolverError`
- `warping_fem` 依赖 `fea::solvers`（SparseLu, Ic0Factor）
- 若将 `fea::solver` 移到第三 crate，section-properties 和 structural-analysis 都依赖它
- 但 structural-analysis 同时还需要 `Material` 和 `Point`（来自 section-properties）
- 因此 structural-analysis 无论如何都要依赖 section-properties
- 创建第三 crate 不改变 structural-analysis 的依赖集，只增加 workspace 复杂度

**结论：不创建第三 crate。求解器留在 section-properties。**

## 5. 候选架构比较

### Option A — 单 crate（历史状态，已不存在）

```
section-properties
├── geometry, section, plastic, ...
├── fea (solver)
└── beam_fem, frame, load, mechanism
```

| 维度 | 评估 |
|------|------|
| 依赖方向 | 无（单 crate） |
| API 清晰度 | 差 — 截面用户被迫安装结构分析 |
| crates.io 用户体验 | 差 — 一个 crate 做两件事 |
| 编译时间 | 长 — 全量编译 |
| 版本发布 | 耦合 — 截面和结构分析同步发版 |

### Option B — 两个 crate（当前状态）

```
section-properties (v0.4.0, crates.io)
├── geometry, section, plastic, ...
└── fea (solver)

structural-analysis (v0.1.0, workspace-only)
├── beam_fem, frame, load, mechanism
└── depends on section-properties
```

| 维度 | 评估 |
|------|------|
| 依赖方向 | structural-analysis → section-properties（单向） |
| API 清晰度 | 好 — 截面用户不安装结构分析 |
| crates.io 用户体验 | 好 — 截面用户 `cargo add section-properties` 即可 |
| 编译时间 | 可独立编译各 crate |
| 版本发布 | 解耦 — 截面已发 v0.4.0，结构分析按需发版 |
| 测试边界 | 独立 — 31 + 32 测试文件，无交叉引用 |
| 代码迁移量 | 已完成（Phase 86-88） |

### Option C — 三层 crate

```
numerical-core (SparseMatrix, LinearSolver, ...)
    ↑           ↑
    │           │
section-properties   structural-analysis
```

| 维度 | 评估 |
|------|------|
| 依赖方向 | 两个上层 crate 都依赖 numerical-core |
| API 清晰度 | 理论上更清晰，但实际无收益 |
| crates.io 用户体验 | 差 — 用户须安装 3 个 crate |
| 编译时间 | 略好（numerical-core 可独立编译） |
| 版本发布 | 复杂 — 3 个 crate 协调发版 |
| 代码迁移量 | 大 — 需拆 fea/ 出 section-properties |
| 循环依赖风险 | 无，但增加 workspace 复杂度 |
| 实际收益 | 低 — structural-analysis 仍需依赖 section-properties（Material, Point） |

**关键事实：Option C 不消除 structural-analysis → section-properties 依赖。** structural-analysis 需要 `Material` 和 `Point`，这些是截面级类型，不属于数值基础设施。因此即使创建 numerical-core，依赖图仍是：

```
numerical-core
    ↑           ↑
    │           │
section-properties   structural-analysis → section-properties
```

structural-analysis 仍依赖 section-properties，Option C 只增加了一个中间 crate 而未改变上层依赖关系。

## 6. 最终架构建议

**当前架构（Option B）已是正确最终状态。**

```
section-properties (v0.4.0, crates.io)
    ↑
    │
structural-analysis (v0.1.0, workspace-only)
```

理由：
1. 拆分已完成，代码已在正确位置
2. 依赖方向正确（单向，无循环）
3. 公共 API 无污染
4. 测试边界独立
5. 第三 crate 无实际收益
6. section-properties 已发布，用户可独立使用
7. structural-analysis 保留 workspace-only 是正确决策（API 尚未稳定，无外部消费者需求）

## 7. 迁移计划

**无需迁移。** 拆分已在 Phase 86-88 完成。

如未来需要发布 structural-analysis 到 crates.io：
1. 确认 API 稳定性（当前 493 tests 通过，Phase 94 审计完成）
2. 确认有明确外部消费者需求
3. 执行 `cargo publish -p section-properties`（如需更新依赖版本）
4. 执行 `cargo publish -p structural-analysis`
5. 创建 tag

## 8. 风险

| 风险 | 状态 | 说明 |
|------|------|------|
| Breaking API | 无 | 拆分已完成，无待执行变更 |
| 依赖 | 无风险 | 单向依赖，无循环 |
| 测试 | 无风险 | 31 + 32 独立测试，无交叉引用 |
| 文档 | 无风险 | 各 crate 独立 rustdoc |
| CI | 无风险 | workspace 级 CI 已运行 |
| crates.io | 无风险 | section-properties v0.4.0 已发布 |
| Solver reuse | 无风险 | 共享 fea::solver，无重复实现 |

## 9. 测试边界

| crate | 测试文件数 | 引用对方 crate | 独立性 |
|-------|-----------|--------------|--------|
| section-properties | 31 | 0 个引用 structural-analysis | 完全独立 |
| structural-analysis | 32 | 引用 section-properties (Material, SolverSelection, SparseMatrix, ...) | 依赖 section-properties（预期行为） |

```
cargo test -p section-properties     → 31 文件, 截面级测试
cargo test -p structural-analysis    → 32 文件, 结构分析测试
```

**无隐藏耦合。** section-properties 测试不访问 structural-analysis 的任何 API。

## 10. 当前 Phase 结论

### 现在是否应该开始拆？

**不需要。拆分已完成。**

当前架构（Option B）是正确最终状态：
- workspace 已有两个 crate
- 依赖方向正确（structural-analysis → section-properties，单向）
- 公共 API 无污染
- 测试边界独立
- section-properties v0.4.0 已发布 crates.io
- structural-analysis v0.1.0 workspace-only 是正确决策

**无下一 Phase 需要执行。** 如未来需要发布 structural-analysis，按 §7 迁移计划执行即可。
