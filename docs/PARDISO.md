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

## API

```rust
use section_properties::fea::solvers::pardiso::PardisoSolver;

let mut s = PardisoSolver::new(&k, &c)?;
let u = s.solve_direct_lagrange(&f)?;   // multiplier check |u[-1]|/max|u| <= 1e-7
```
