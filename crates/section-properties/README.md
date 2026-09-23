# section-properties

A Rust library for computing structural **cross-section properties** of
two-dimensional sections — geometric and principal moments of area, section
moduli, radii of gyration, torsion and warping properties, plastic capacity,
and cold-formed / fire analysis.

It reimplements the functionality of the Python
[`sectionproperties`](https://github.com/robbievanleeuwen/section-properties)
library in a single, dependency-light Rust crate.

This crate is part of the `section-properties` workspace. Structural analysis
(beam/frame FEM, mechanism diagnostics) lives in the companion crate
[`structural-analysis`](../structural-analysis).

## Features

- **Geometric properties** (`SectionProperties`): area, centroid, second moments
  of area (`Ix`, `Iy`, `Ixy`), principal moments and orientation (`i11`, `i22`,
  `phi`), section moduli (`Z`), radii of gyration, perimeter, and first moments.
  Supports compound (multi-region) sections, holes, and affine transforms.
- **Geometry toolkit** (`geometry`): polygons, boolean operations
  (union / intersection / difference via Greiner–Hormann), offsets, and
  compound-geometry validation.
- **Finite-element warping analysis** (`mesh`, `fea`, `plastic::warping_fem`):
  TRI3/TRI6 meshing, sparse direct and iterative solvers (sparse LU, skyline
  LDLᵀ, CG, ICCG, optional Intel MKL PARDISO), and warping/torsion constants.
- **Plastic analysis** (`plastic`): section classification (EN 1993-1-1 /
  AISC 360), plastic section moduli, interaction diagrams, and torsion analysis.
- **Cold-formed steel** (`cold_formed_analysis`): effective width method,
  direct strength method, and distortional buckling.
- **Fire engineering** (`fire`): section factors, temperature profiles,
  protection, and fire resistance.
- **Section library** (`section_library`): parametric steel (IPE/HE/etc.),
  cold-formed, concrete, timber, and composite sections.
- **Database** (`database`): in-memory, filterable/searchable section database.
- **I/O** (`io`): JSON, CSV, DXF, SVG, VTK and Nastran export/import.

## Quick start

```rust
use section_properties::{ParametricSection, SectionProperties, section_library::steel::ISection};

let isection = ISection::from_designation("IPE300").unwrap();
let section = isection.build();

let props = SectionProperties::from_section(&section);
println!("Area = {:.6} m²", props.area);
println!("Ix   = {:.6e} m⁴", props.ix);
println!("{}", props.format_results());
```

For invalid input that must not panic, use the fallible constructors:

```rust
use section_properties::{Point, Polygon, Section, SectionProperties};

let outer = Polygon::new(vec![
    Point::new(0.0, 0.0),
    Point::new(10.0, 0.0),
    Point::new(10.0, 5.0),
    Point::new(0.0, 5.0),
]);
let section = Section::new(outer, Vec::new());

let result: Result<SectionProperties, String> =
    SectionProperties::try_from_section(&section);
```

## Building and testing

Requires Rust 1.85+ (edition 2024).

```bash
cargo build
cargo test
```

### PARDISO backend (optional)

To use the Intel MKL PARDISO direct solver, enable the feature and run within a
full oneAPI/MKL environment:

```bash
cargo build --release --features pardiso
```

See [`docs/PARDISO.md`](../../docs/PARDISO.md) for details and known limitations.

## Documentation

Generate API docs with:

```bash
cargo doc --open
```

Beam FEM conventions (DOF ordering, coordinate systems, sign conventions,
load ownership, boundary conditions, model snapshot semantics, solver
selection and error behaviour) are documented in
[`docs/beam_fem.md`](../../docs/beam_fem.md). End-to-end usage is shown by the
`beam_*` and `frame_*` examples in the `structural-analysis` crate.