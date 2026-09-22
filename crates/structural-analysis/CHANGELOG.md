# Changelog

## Unreleased

## 0.1.0 - 2026-09-22

### Initial release

Extracted from `section-properties` 0.4.0. This crate contains the structural
analysis modules that were previously part of `section-properties`:

- `beam_fem` — 2D Euler–Bernoulli beam FEM
- `frame` — 2D frame analysis with `FrameModel` façade
- `mechanism` — mechanism diagnostics and rank-deficiency detection

Depends on `section-properties` for cross-section properties, materials, and
numerical infrastructure (FEA solvers, sparse matrices).
