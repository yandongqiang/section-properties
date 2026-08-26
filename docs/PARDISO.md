# PARDISO backend (Intel MKL)

The `pardiso` feature exposes Intel MKL's PARDISO direct solver for the
augmented Lagrangian warping system.

## Requirements

* Intel oneAPI MKL (or pip `mkl` / `mkl-devel` wheels) providing
  `mkl_rt.2.dll` / `mkl_rt.3.dll` plus companion DLLs (`mkl_core.*`,
  `mkl_def.*`, threading layer, `tbb12.dll` / `libiomp5md.dll`) on PATH or
  beside the executable.
* A complete, matching runtime set. Mixing partial pip-extracted components
  can cause heap corruption (observed with MKL 2026.1 wheels).

## Build

```powershell
cargo build --release --features pardiso
```

The binding uses raw-dylib against `mkl_rt.3`; adjust the version in
`src/fea/solvers.rs` if your MKL ships a different name.

## Status & known issue

The FFI itself is verified: `pardisoinit` returns err=0 against MKL 2025.2 /
2026.1 runtimes obtained from pip wheels. However, in ad-hoc environments
built from partial wheel extractions, the process can crash (access
violation) inside `pardisoinit` or at thread-layer load because the
versioned DLL set (`mkl_rt.3` expecting `.2` companions) is incomplete.

For that reason:

* `DirectLagrangeSolver::new` always uses the **skyline LDL^T** backend.
* To use PARDISO, call `DirectLagrangeSolver::with_kernel(
  LagrangeKernel::Pardiso, ...)` explicitly inside a full oneAPI environment
  (`setvars.bat`) where all companion DLLs are provisioned.

## API

```rust
use section_properties::fea::{
    DirectLagrangeSolver, LagrangeKernel, solvers::pardiso::PardisoSolver,
};

// Default (skyline):
let mut s = DirectLagrangeSolver::new(&k, &c)?;
let u = s.solve(&f)?;

// Explicit PARDISO:
let mut s = DirectLagrangeSolver::with_kernel(LagrangeKernel::Pardiso, &k, &c)?;
let u = s.solve(&f)?;
// or standalone:
let mut p = PardisoSolver::new(&k, &c)?;
let u = p.solve_direct_lagrange(&f)?;   // multiplier check |u[-1]|/max|u| <= 1e-7
```
