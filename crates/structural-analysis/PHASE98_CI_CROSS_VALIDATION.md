# Phase 98 — Python Cross-Validation CI Environment Audit & Fix

## 1. Baseline

| Item | Value |
|------|-------|
| HEAD | `68f42c5` (Phase 97: harden workspace release configuration) |
| Runner | Windows (local), Ubuntu (CI) |
| Rust | stable (1.85+) |
| Python | N/A — test does not call Python |

## 2. Failure Root Cause

**The `NotFound` error is NOT a missing Python executable.** It is a missing JSON input file.

### Dependency chain

```
global_matrix_cross_validation (test)
    ↓
test_section("Channel_200x75")
    ↓
run_fem_on_python_tri6_mesh("python_global_Channel_200x75.json", ...)
    ↓
File::open("python_global_Channel_200x75.json")
    ↓
Os { code: 2, kind: NotFound }
```

### Why the file is not found

| Fact | Detail |
|------|--------|
| JSON files location | Workspace root (`D:\code\section-properties\python_global_*.json`) |
| Git tracked | Yes — all 4 files tracked |
| Test CWD | Crate directory (`crates/section-properties/`) |
| Test path | Relative: `python_global_{}.json` |
| Resolved path | `crates/section-properties/python_global_{}.json` — **does not exist** |

`cargo test` sets the working directory to the crate's directory (`CARGO_MANIFEST_DIR`), not the workspace root. The test used a relative path that assumed CWD = workspace root.

### Same issue in `rust_on_python_mesh.rs`

`rust_on_python_mesh.rs` had the identical problem with `python_mesh_*.json` files.

## 3. Fix

**Changed 2 test files to use `env!("CARGO_MANIFEST_DIR")` for path resolution.**

### `global_matrix_cross_validation.rs`

Added `workspace_root()` helper:
```rust
fn workspace_root() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf()
}
```

Changed `test_section()` to use `workspace_root().join(...)` for JSON file paths.

### `rust_on_python_mesh.rs`

Same `workspace_root()` helper. Changed `run_fem_on_python_meshes()` to use it for `python_mesh_*.json` paths.

### What was NOT changed

- FEM math: **unchanged**
- Test expected values: **unchanged**
- Public API: **unchanged**
- Crate version: **unchanged**
- CI workflow: **unchanged** (no CI changes needed — fix is in test code)
- No test deleted, ignored, or weakened

## 4. Verification

| Check | Result |
|-------|--------|
| `cargo fmt --check` | **PASS** |
| `cargo check --workspace` | **PASS** |
| `cargo test -p structural-analysis` | **PASS** — 493 tests, 0 failures |
| `cargo test --release -p section-properties --test global_matrix_cross_validation` | **PASS** — 1 test, 0 failures (1.66s) |
| `cargo test --release -p section-properties --test rust_on_python_mesh` | **PASS** — 1 test, 0 failures (0.03s) |

## 5. CI Result (predicted)

With this fix, the CI release job (`cargo test --release --all -- --skip warping_domain_regression`) will now find the JSON files via `CARGO_MANIFEST_DIR` regardless of the runner OS or working directory.

```
global_matrix_cross_validation = PASS (predicted)
rust_on_python_meshes = PASS (predicted)
```

**Note:** CI confirmation requires pushing and observing the CI run. The fix is portable (uses `CARGO_MANIFEST_DIR`, no hardcoded paths, works on both Windows and Linux).

## 6. Files Modified

| File | Change |
|------|--------|
| `crates/section-properties/tests/global_matrix_cross_validation.rs` | +13/-2: `workspace_root()` helper + path fix |
| `crates/section-properties/tests/rust_on_python_mesh.rs` | +12/-2: same pattern |

Total: 2 files, +25/-4 lines.

## 7. Remaining Issues

| # | Issue | Severity | Status |
|---|-------|----------|--------|
| 1 | None P0 | — | — |
| 2 | None P1 | — | — |
| 3 | `python_warping_parity` crashes with STACK_BUFFER_OVERRUN in sandbox | P2 | Pre-existing, unrelated to this fix |
| 4 | `warping_domain_regression` skipped in CI | P2 | Pre-existing (mesh generation out-of-bounds, Phase 24 frozen) |

## 8. Conclusion

**Root cause:** Test used relative path `python_global_{}.json` which resolved to the crate directory, but the JSON files are in the workspace root. Not a Python issue — the test does not call Python.

**Fix:** Use `env!("CARGO_MANIFEST_DIR")` to locate the workspace root and construct absolute paths. Portable across Windows/Linux, no hardcoded paths, no CI workflow changes needed.

**Not committed** — awaiting confirmation per user instruction.
