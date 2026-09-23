# Phase 91 — Member End Releases / Hinges

**日期**: 2026-09-23
**Baseline**: HEAD = `f7db808` (Phase 89)

---

## 1. Baseline

| 项目 | 值 |
|------|-----|
| HEAD before | `f7db808` (Phase 89) |
| HEAD after | (本 Phase 提交后) |
| 工作树 | clean (提交前) |
| section-properties | v0.4.0 @ crates.io (不修改) |
| structural-analysis | v0.1.0 workspace-only |

---

## 2. Design

### EndRelease 表示

```rust
pub struct EndRelease {
    pub start_rotation: bool,  // 释放 node_i 端旋转: M_i = 0
    pub end_rotation: bool,    // 释放 node_j 端旋转: M_j = 0
}
```

预设: `none()`, `start_pin()`, `end_pin()`, `both_pins()`。

### 刚度实现

**静态凝聚** (static condensation) 在局部坐标系中进行：

1. 将 6×6 局部刚度矩阵 K 分块为保留 (c) 和释放 (r) DOF
2. 凝聚: `K_condensed = K_cc − K_cr · K_rr⁻¹ · K_rc`
3. 散射回 6×6 (释放行列 = 0)
4. 变换到全局: `K_global = Tᵀ · K_condensed · T`

K_rr 最多 2×2 (释放 θ_i 和 θ_j)，直接求逆。

### 荷载实现

等效节点荷载同样凝聚：
```
f_condensed = f_c − K_cr · K_rr⁻¹ · f_r
```

在 `from_model` 中使用 `released_consistent_nodal_load` 和 `released_consistent_nodal_load_point`。

### 端力恢复

在 `element_on_node_end_forces_local` 中：

1. 从全局解获取 `u_local = T · u_global` (6 维，含节点旋转)
2. 恢复释放 DOF 的实际构件端部位移: `u_r = K_rr⁻¹ · (f_r − K_rc · u_c)`
3. 构造完整 `u_local_full = [u_c, u_r]`
4. 使用**原始** K_local 计算: `f_end = f_equiv − K_local · u_local_full`
5. 释放端力自动为 0: `f_end_r = f_r − K_rc·u_c − K_rr·K_rr⁻¹·(f_r − K_rc·u_c) = 0`

---

## 3. API changes

### 新增 public API

| 类型/方法 | 位置 | 说明 |
|-----------|------|------|
| `EndRelease` struct | `beam_fem.rs` | 端部释放规格 |
| `EndRelease::none/start_pin/end_pin/both_pins` | `beam_fem.rs` | 预设构造 |
| `EndRelease::is_empty` | `beam_fem.rs` | 查询 |
| `BeamElement::with_end_release` | `beam_fem.rs` | 带释放的构造函数 |
| `BeamElement.end_release` field | `beam_fem.rs` | 公开字段 |
| `FrameModel::add_member_with_release` | `frame.rs` | Frame 层 API |
| `FrameAnalysisResult::section_forces` | `frame.rs` | Frame 层内力 (P2 gap 修复) |

### 修改的 internal API

| 方法 | 修改 |
|------|------|
| `BeamElement::new` | 添加 `end_release: EndRelease::none()` |
| `from_model` | 使用 `released_global_stiffness` 和 `released_consistent_nodal_load*` |
| `element_on_node_end_forces_local` | 使用 `released_local_displacement` 恢复释放 DOF |

### 既有 API 保持不变

`local_stiffness`, `global_stiffness`, `consistent_nodal_load`, `consistent_nodal_load_point` 返回**原始** (未凝聚) 值，不影响直接调用这些方法的测试。

---

## 4. Mathematical correctness

### 为什么 release 后 M_end = 0

释放 DOF r 的凝聚条件: `u_r = K_rr⁻¹ · (f_r − K_rc · u_c)`

端力: `f_end_r = f_r − K_rc · u_c − K_rr · u_r`
`= f_r − K_rc · u_c − K_rr · K_rr⁻¹ · (f_r − K_rc · u_c)`
`= f_r − K_rc · u_c − f_r + K_rc · u_c`
`= 0` ✓

### 为什么 Node Rz 仍然存在

- 全局 DOF 数 = 3 × n_nodes (不变)
- 凝聚在**局部**坐标系进行，散射回 6×6 后释放行列 = 0
- 释放的局部 DOF 不映射到全局 DOF — 该构件不向节点的 Rz 贡献刚度
- 但节点的 Rz 仍然存在于全局系统中，可被其他构件或边界条件约束
- 如果没有其他刚度来源，节点 Rz 是自由无刚度的 DOF → 机构 (正确物理行为)

---

## 5. Tests

**文件**: `tests/end_release.rs` — 19 个测试

| 类别 | 测试 | 数量 |
|------|------|------|
| 回归 | `test_no_release_matches_rigid_baseline`, `test_no_release_identical_to_plain_member` | 2 |
| 解析验证 | `test_cantilever_correct_result`, `test_rigid_pinned_end_moment_zero`, `test_pinned_rigid_start_moment_zero`, `test_pinned_pinned_both_moments_zero` | 4 |
| 节点 vs 构件 | `test_node_rz_exists_after_member_release` | 1 |
| 变换 | `test_angled_member_with_release` | 1 |
| 分布荷载 | `test_distributed_load_with_release_equilibrium` | 1 |
| 点荷载 | `test_point_load_with_release` | 1 |
| 弯矩 | `test_applied_moment_with_release` | 1 |
| 多构件 | `test_multi_member_frame_with_release`, `test_three_member_frame_with_release` | 2 |
| 机构 | `test_release_creates_mechanism`, `test_release_stable_structure` | 2 |
| 求解器 | `test_release_with_default_solver`, `test_release_with_sparse_lu_solver` | 2 |
| section_forces | `test_section_forces_at_released_end` | 1 |
| API | `test_end_release_api` | 1 |

---

## 6. Existing regression

Phase 89/90 已有测试全部保持通过:

| 测试文件 | 通过数 |
|----------|--------|
| lib (26) | 26 ✓ |
| doctest (7) | 7 ✓ |
| frame_api (20) | 20 ✓ |
| frame_correctness (25) | 25 ✓ |
| beam_fem (56) | 56 ✓ |
| beam_section_forces (19) | 19 ✓ |
| mechanism_diagnostics (11) | 11 ✓ |
| solver_selection (9) | 9 ✓ |
| frame_beam_integration (8) | 8 ✓ |
| frame_transformation_contract (5) | 5 ✓ |
| beam_fem_api_semantics (17) | 17 ✓ |
| beam_fem_robustness (11) | 11 ✓ |
| beam_force_diagrams (13) | 13 ✓ |

**零回归。**

---

## 7. Findings

```
P0: 0
P1: 0
P2: 0
H: 0
```

无问题发现。实现满足 spec 所有要求。

---

## 8. Architecture impact

| 问题 | 回答 |
|------|------|
| 是否需要修改 FrameModel？ | 是 — 添加 `add_member_with_release` |
| 是否需要修改 BeamModel？ | 否 — `BeamModel` 结构不变 |
| 是否需要修改 BeamElement？ | 是 — 添加 `end_release` 字段 |
| 是否改变 Node DOF 模型？ | **否** — 节点仍有 3 DOF (Ux, Uy, Rz) |
| 是否改变 solver boundary？ | 否 — 求解器接口不变 |
| 是否改变 section-properties boundary？ | **否** — 不修改 section-properties |

---

## 9. Next step

Phase 91 正确完成。下一目标:

```
Phase 92 — Load Case / Load Combination
```

---

## 验证结果

| 检查 | 结果 |
|------|------|
| `cargo fmt --all -- --check` | PASS |
| `cargo check --workspace --all-targets` | PASS |
| `cargo test -p structural-analysis --lib` | 26 passed |
| `cargo test -p structural-analysis --doc` | 7 passed |
| `cargo test -p structural-analysis --test end_release` | 19 passed |
| `cargo clippy -p structural-analysis --all-targets` | 0 errors |
| `cargo package -p structural-analysis` | 44 files, 927.4KiB |
| `cargo publish --dry-run -p structural-analysis` | PASS |
