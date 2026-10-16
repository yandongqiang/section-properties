# Changelog

## Unreleased

### 3-D frame member end releases (pins)

- Added `EndReleases3D`: rotational end releases (`rx`/`ry`/`rz` at each end)
  for the 3-D frame element, applied by static condensation of the local
  stiffness — a released rotation transmits no moment at that end.
- `FrameElement3D::with_releases` / `releases()` / `set_releases()`, and
  `FrameModel3D::set_member_releases` / `member_releases`.
- End releases condense the consistent member-load vector as well, and
  released DOFs report exactly zero end force while the joint's remaining
  members carry the equilibrium.
- Breaking-free: an element with no releases behaves bit-for-bit as before.

### Result / post-processing consistency

- Added typed `LoadSource` with model-load, load-case and combination variants;
  combination results now retain their scaled terms.
- `Envelope` stores the input sources and attributes extrema by source index.
  Member-force samples now report per-component governing sources.
- Reaction envelopes exclude free-DOF round-off residuals and read each
  result's reaction vector once.
- `FrameAnalysisResult` owns a reaction snapshot; `reactions()` returns a slice
  and scalar reaction/equilibrium access is O(1).
- **Breaking:** `BeamAnalysisResult::reaction` now returns the same
  `K·u - f` value as the solver at every DOF. Free DOFs report the round-off
  residual and spring DOFs report `-k·u`, rather than being forced to zero.
- **Breaking:** `FrameAnalysisResult::load_source()` returns `&LoadSource`
  instead of `Option<&str>`.
- Prescribed displacements that overlap a spring or inclined roller are
  rejected, and overlapping model-level supports fail validation.

## 0.1.0 - 2026-09-22

### Initial release

Extracted from `section-properties` 0.4.0. This crate contains the structural
analysis modules that were previously part of `section-properties`:

- `beam_fem` — 2D Euler–Bernoulli beam FEM
- `frame` — 2D frame analysis with `FrameModel` façade
- `mechanism` — mechanism diagnostics and rank-deficiency detection

Depends on `section-properties` for cross-section properties, materials, and
numerical infrastructure (FEA solvers, sparse matrices).
