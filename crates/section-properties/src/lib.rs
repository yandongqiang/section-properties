//! # section-properties
//!
//! Rust crate for structural cross-section analysis: geometric properties,
//! plastic classification, warping/torsion constants, stress analysis,
//! and section-level FEM.
//!
//! For 2D beam/frame structural analysis, see the `structural-analysis` crate.
//!
//! # Quick start — section properties
//!
//! ```rust
//! use section_properties::{Point, Polygon, Section, SectionProperties};
//!
//! // Build a rectangle 10 × 5
//! let outer = Polygon::new(vec![
//!     Point::new(0.0, 0.0),
//!     Point::new(10.0, 0.0),
//!     Point::new(10.0, 5.0),
//!     Point::new(0.0, 5.0),
//! ]);
//! let section = Section::new(outer, Vec::new());
//! let props = SectionProperties::from_section(&section);
//!
//! assert!((props.area - 50.0).abs() < 1e-10);
//! assert!((props.centroid.x - 5.0).abs() < 1e-10);
//! assert!((props.centroid.y - 2.5).abs() < 1e-10);
//! ```
//!
//! # Module overview
//!
//! | Module                  | Purpose                                              |
//! |-------------------------|------------------------------------------------------|
//! | [`geometry`]            | Polygons, boolean ops, compound geometry             |
//! | [`section`]             | Section (outer + holes), frame properties            |
//! | [`section_properties`]  | Area, moments, principal axes, section moduli        |
//! | [`composite`]           | Multi-material elastic transformed-section analysis  |
//! | [`fea`]                 | FEM core: elements, solvers, numerical infrastructure|
//! | [`plastic`]             | Plastic section analysis, warping, classification    |
//! | [`material`]            | Isotropic linear-elastic material                    |
//! | [`stress`]              | Cross-section stress analysis                        |
//! | [`section_library`]     | Standard section shapes (I, channel, tube, …)        |
//!
//! # Error handling conventions
//!
//! * **Fallible constructors** use `try_new` / `new_validated` returning
//!   `Result<Self, E>`.
//! * **Fallible computations** use `try_*` returning `Result<T, E>` or
//!   `Option<T>`.
//! * **Panicking APIs** delegate to the `try_*` variant and `expect()` —
//!   they document the panic condition in a `# Panics` section.

pub mod cold_formed_analysis;
pub mod composite;
pub mod database;
pub mod fea;
pub mod fire;
pub mod geometry;
pub mod io;
pub mod material;
pub mod mesh;
pub mod plastic;
pub mod post;
pub mod section;
pub mod section_library;
pub mod section_properties;
pub mod stress;
pub mod stress_fem;

pub use crate::cold_formed_analysis::{
    BucklingCurve, ColdFormedElement, ColdFormedSection, EdgeSupport, EffectiveSectionProperties,
    EffectiveWidthParams, ElementReduction, Stiffener, StiffenerType,
};
pub use crate::database::{
    SearchFilter, SearchResult, SectionDatabase, SectionEntry, build_standard_database,
};
pub use crate::fire::{
    FireAnalysis, FireExposure, FireProtection, FireResistanceResult, MaterialPropertiesAtTemp,
    ProtectionType, SectionFactor, TemperatureProfile, composite as fire_composite,
};
pub use crate::geometry::{
    Axis, BoundaryExtrema, CompoundError, CompoundGeometry, Geometry, JoinStyle, Point, Polygon,
    Transform,
};
pub use crate::io::{
    CsvExportOptions, DxfColor, DxfExportOptions, ExportFormat, JsonMaterial, JsonSection,
    SectionExporter, SectionImporter, SvgExportOptions, export_section_library, from_csv,
    from_json, plot_centroids, section_from_composite, section_from_parametric, to_csv, to_dxf,
    to_json, to_nastran, to_svg, to_vtk,
};

pub use crate::composite::{
    ComponentAnalysis, CompositeAnalysisResult, CompositeComponent, CompositeError,
    ElasticComposite,
};
pub use crate::fea::{
    FemError, LagrangeKernel, SkylineLdlt, solver, solver::FactoredSolver, solver::LinearSolver,
    solver::SelectionReason, solver::SolverBackend, solver::SolverCapabilities,
    solver::SolverError, solver::SolverRegistry, solver::SolverSelection,
    solver::SolverSelectionInfo, solvers,
};
/// Alias for the interactive HTML viewer export.
pub use crate::io::to_interactive_html as to_html;
pub use crate::material::Material;
pub use crate::mesh::{
    FemCompositeAnalysis, FemGeometricProperties, FemSectionAnalysis, FemWarpingProperties, Mesh,
    MeshParams, PropertyComparison, StressPlotData, StressPost,
};
pub use crate::plastic::warping_fem::warping_svg;
pub use crate::plastic::{
    CapacityCheck, ClassLimit, InteractionDiagram, InteractionPoint, LoadCase3D, PlasticAnalysis,
    PlasticProperties, PlasticSection, SectionClass, SectionClassification, StressDistribution,
    TorsionAnalysis, WarpingProperties, aisc360, classify_section, effective, en1993,
};
pub use crate::section::{FrameProperties, MaterialError, Section, TransformedFrameProperties};
pub use crate::section_library::{CompositeSection, ParametricSection};
pub use crate::section_properties::{
    GeometricProperties, GyrationProperties, PrincipalProperties, SectionProperties,
};
pub use crate::stress::{
    SectionLoads, StressAnalysis, StressAnalysisResult, StressAtPoint, YieldCheckResult,
};
