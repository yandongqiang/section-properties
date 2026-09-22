# Python / Rust cross-validation reference data

Deterministic reference values produced by the Python `sectionproperties`
library, used by `tests/python_rust_cross_validation.rs` for differential
validation of the Rust implementation.

## Regenerating

```bash
python generate_cross_validation_reference.py
```

Regeneration overwrites this directory. The recorded environment is stored in
every case file, so a reference set can never be silently compared against a
different implementation.

Recorded environment for the current data set:

```text
Python:             3.14.2
sectionproperties:  3.10.2
numpy:              2.5.1
scipy:              1.18.0
shapely:            2.1.2
```

## Case file layout

```text
index.json                 environment + list of cases
<case>.json                geometry + reference properties for one case
```

Each case file contains:

| Field | Meaning |
| --- | --- |
| `case` | unique case id |
| `description` | human-readable geometry description |
| `environment` | Python / package versions used for generation |
| `mesh_size` | the `create_mesh(mesh_sizes=[...])` target used by Python |
| `geometry.outer` | outer polygon ring, exported from Python (closing point removed) |
| `geometry.holes` | hole rings |
| `properties.*` | computed reference quantities |
| `properties.warping` | `null` if Python could not compute warping |
| `properties.warping_error` | Python-side failure text, if any |

## Comparison contract

### Geometry

* The **exact polygon coordinates** used by Python are exported, so Rust
  rebuilds an identical geometry rather than relying on a second
  parameterisation of the same section. This makes the test a true differential
  test of the property pipelines.
* Polygon vertex ordering is **not** compared — it is not part of the public
  contract, and `Section::new` normalises the outer boundary to CCW and holes to
  CW on the Rust side.
* Units are consistent (mm-based magnitudes); coordinates are in the section's
  own frame as produced by the Python library (origin at a corner).
* Holes are expressed as interior rings.

### Quantities compared

Area, centroid `(cx, cy)`, second moments `Ixx, Iyy, Ixy`, principal moments
`I11, I22`, principal-axis angle `phi`, radii of gyration `rx, ry`, and section
moduli `Zxx+`, `Zyy+`.

### Convention mappings (NOT defects)

**Principal-axis angle `phi`**

| | Units | Sense |
| --- | --- | --- |
| Rust `SectionProperties::principal.phi` | radians | `phi = ½·atan2(2·Ixy, Ixx − Iyy)`, CCW positive |
| Python `Section.get_phi()` | degrees | opposite rotational sense: `phi_py ≡ −phi_rust (mod 180°)` |

Verified: Rust's `phi` reproduces `½·atan2(2·Ixy, Ixx − Iyy)` computed from
Python's own inertia values to machine precision; Python's value is the negative
of that, expressed in degrees. Both pair with identical `I11/I22`, so the
physical principal axes agree. The test applies this mapping; production code is
**not** changed.

**Shear centre**

| | Meaning |
| --- | --- |
| Rust `FemWarpingSolution::shear_center` | offset from the centroid |
| Python `Section.get_sc()` | absolute coordinates |

The test compares `centroid + rust_offset` against Python's absolute value.

### Tolerance policy

Every quantity is normalised by a characteristic magnitude **of the same
physical kind** — never an absolute-only tolerance, and never a blind
`max(1.0, scale)`:

* lengths and centroids → normalised by `sqrt(area)`;
* second moments → normalised by `area · sqrt(area)²`;
* `Ixy` → normalised by `max(|Ixx|, |Iyy|)` (it is ~0 for symmetric sections,
  where a relative test against itself is meaningless);
* `phi` → absolute radians after the mapping above, mod π;
* section moduli → normalised by the modulus magnitude.

Geometry-level quantities are polygon-exact in Rust and mesh-converged in
Python, so a relative tolerance of `1e-8` is used and asserted.

Warping (`J`, `Iw`) is a FEM quantity and the two codes triangulate
differently, so it uses documented looser tolerances (`J` 5%, `Iw` 10%) and
every out-of-tolerance row is **classified as `MESH_DIFFERENCE`**, not silently
passed. The warping test is `#[ignore]`d because it costs ~200 s per case:

```bash
cargo test --test python_rust_cross_validation -- --ignored --nocapture
CV_CASES=rect_100x50 cargo test --test python_rust_cross_validation -- --ignored
```

Use `CV_MESH=coarse|normal|fine|veryfine|py` to drive the Rust mesh control for
convergence studies (default `py` = `Custom(<python mesh size>)`).

## Known findings

* `rect_scale_p3`, `i_scale_p3` (α = 1e3): Python's warping solver aborts with
  `MemoryError` inside SuperLU `gssv`. Recorded as
  `REFERENCE_EXECUTION_FAILURE`; geometric properties are unaffected.
* `i_300x150`: Rust reports `J_raw < 0` and `used_analytical_fallback = true`,
  giving `J = 4.66e4` against Python `2.165e5` (78% low) and `Iw` 2.6× high.
  Mesh density does not change the outcome (the minimum-edge constraint from
  the 8 mm web dominates), so this is **not** explained by mesh coarseness.
  Recorded as a candidate production defect pending targeted investigation —
  deliberately **not** fixed in the validation phase.
