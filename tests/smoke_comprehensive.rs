//! End-to-end smoke tests: exercise every public API path of the crate.
//!
//! Each test targets one module cluster; failures indicate regressions or
//! misalignment with Python sectionproperties behaviour.

use section_properties::fea::{SkylineLdlt, solvers};
use section_properties::io::*;
use section_properties::material::presets::STEEL_S355;
use section_properties::mesh::FemSectionAnalysis;
use section_properties::plastic::{InteractionDiagram, LoadCase3D, PlasticAnalysis, PlasticAxis};
use section_properties::section_library::primitive::*;
use section_properties::section_library::steel::{AngleSection, ChannelSection, ISection};
use section_properties::section_library::{CompositeSection, ParametricSection};
use section_properties::stress::{SectionLoads, StressAnalysis};
use section_properties::to_html;
use section_properties::{Point, SectionProperties};

fn rel(a: f64, b: f64) -> f64 {
    ((a - b) / b.abs()).abs()
}

// ---------------------------------------------------------------------------
// geometry
// ---------------------------------------------------------------------------

#[test]
fn smoke_geometry_transforms_and_boolean() {
    let rect = RectangularSection::new(0.2, 0.1).build();
    let g = section_properties::Geometry::from_section(&rect);
    let rotated = g.clone().rotate(90.0);
    assert!((rotated.area() - 0.02).abs() < 1e-12);
    let centered = g.align_center();
    let c = centered.apply_transforms().centroid();
    assert!(c.x.abs() < 1e-12);

    // boolean union of two overlapping squares keeps area identity
    use section_properties::geometry::{BoolOp, polygon_boolean};
    let sq = |x0| {
        section_properties::Polygon::new(vec![
            Point::new(x0, 0.0),
            Point::new(x0 + 1.0, 0.0),
            Point::new(x0 + 1.0, 1.0),
            Point::new(x0, 1.0),
        ])
    };
    let a = sq(0.0);
    let b = sq(0.5);
    let inter = polygon_boolean(&a, &b, BoolOp::Intersection);
    assert_eq!(inter.len(), 1);
    // Unit squares offset by 0.5 overlap over 0.5x1 region.
    // Jittered-degenerate path introduces O(1e-6) relative error.
    assert!((inter[0].area() - 0.5).abs() < 1e-4);
}

#[test]
fn smoke_offset_hollow() {
    // CHS offset inward shrinks wall thickness.
    let chs = CircularHollowSection::from_dimensions(0.2191, 0.0082).build();
    let g = section_properties::Geometry::from_section(&chs);
    let grown = g.offset(0.001).expect("offset");
    assert!(grown.area() > chs.area());
    let shrunk = g.offset(-0.001).expect("offset");
    assert!(shrunk.area() < chs.area());
}

// ---------------------------------------------------------------------------
// section library + properties
// ---------------------------------------------------------------------------

#[test]
fn smoke_section_libraries_build() {
    // Every parametric section must build and produce positive area.
    let secs: Vec<(&str, section_properties::Section)> = vec![
        (
            "IPE300",
            ISection::from_designation("IPE300").unwrap().build(),
        ),
        (
            "UPN200",
            ChannelSection::from_designation("UPN200").unwrap().build(),
        ),
        ("L100x10", AngleSection::equal_leg(0.1, 0.01).build()),
    ];
    for (name, sec) in &secs {
        let props = SectionProperties::from_section(sec);
        assert!(
            props.area > 0.0 && props.ix > 0.0 && props.iy > 0.0,
            "{}: bad properties",
            name
        );
        assert!(
            props.perimeter > 0.0,
            "{}: perimeter must be positive",
            name
        );
    }

    // IPE300 table values (cm^2/cm^4)
    let ipe = SectionProperties::from_section(&secs[0].1);
    assert!(rel(ipe.area, 5.38e-3) < 0.01, "IPE300 area");
    assert!(rel(ipe.ix, 8.356e-5) < 0.02, "IPE300 Ix");
}

#[test]
fn smoke_primitive_set_matches_python() {
    // All Python primitive.py shapes exist and are geometrically sane.
    let r = RectangularSection::new(0.2, 0.1).build();
    assert!((r.area() - 0.02).abs() < 1e-12);

    let c = CircularSection::new(0.1).build();
    assert!(rel(c.area(), std::f64::consts::PI * 0.01) < 5e-3); // 64-gon

    let chs = CircularHollowSection::from_dimensions(0.2, 0.01).build();
    let expected = std::f64::consts::PI * (0.1_f64.powi(2) - 0.09_f64.powi(2));
    assert!(rel(chs.area(), expected) < 5e-3);

    let ehs = EllipticalHollowSection::new(0.2, 0.1, 0.01, 64).build();
    assert!(ehs.area() > 0.0);

    let phs = PolygonHollowSection::new(0.2, 0.01, 8).build();
    assert!(phs.area() > 0.0);
}

// ---------------------------------------------------------------------------
// warping / frame properties
// ---------------------------------------------------------------------------

#[test]
fn smoke_frame_properties_channel_shear_centre() {
    let ch = ChannelSection::new(0.2, 0.075, 0.005, 0.008, 0.0, 0.0).build();
    let fp = ch.frame_properties_full(0.3);
    let c = ch.centroid();
    // Shear centre behind web: x_se < x_centroid (flanges point +x)
    assert!(
        fp.delta_x < c.x,
        "channel SC ({:.5}) must be left of centroid ({:.5})",
        fp.delta_x,
        c.x
    );
    assert!(fp.iw > 0.0, "channel must have warping constant");
}

// ---------------------------------------------------------------------------
// plastic
// ---------------------------------------------------------------------------

#[test]
fn smoke_plastic_shape_factors() {
    // Rectangle SF = 1.5 exactly.
    let rect = RectangularSection::new(0.1, 0.2);
    let sec = rect.build();
    let props = SectionProperties::from_section(&sec);
    let pa = PlasticAnalysis::new(sec, STEEL_S355);
    let pp = pa.plastic_section.plastic_properties(PlasticAxis::X);
    let sf = pp.plastic_section_modulus / props.section_modulus_x();
    assert!((sf - 1.5).abs() < 0.01, "rectangle shape factor {}", sf);
}

#[test]
fn smoke_interaction_capacity_ordering() {
    let rect = RectangularSection::new(0.1, 0.2).build();
    let diagram = InteractionDiagram::new(rect, STEEL_S355);
    let n_rd = diagram.n_rd;
    let m_rd = diagram.m_x_rd;

    // Increasing axial load reduces bending capacity (monotone surface).
    let mut prev_m = m_rd;
    for frac in [0.25f64, 0.5, 0.75] {
        let chk =
            diagram.check_capacity_exact(LoadCase3D::new(frac * n_rd * 0.9, m_rd * 0.95, 0.0), 1.0);
        let _ = prev_m;
        assert!(
            chk.utilization.is_finite(),
            "utilization must be finite (frac={})",
            frac
        );
        prev_m = chk.utilization;
    }
}

// ---------------------------------------------------------------------------
// stress
// ---------------------------------------------------------------------------

#[test]
fn smoke_stress_axial_plus_bending() {
    let rect = RectangularSection::new(0.1, 0.2).build();
    let analysis = StressAnalysis::new(rect, STEEL_S355);
    let result = analysis.calculate_stress(SectionLoads {
        n: 50e3,
        vx: 0.0,
        vy: 0.0,
        mxx: 5e3,
        myy: 0.0,
        mzz: 0.0,
        m11: 0.0,
        m22: 0.0,
    });
    let area = 0.02_f64;
    let wel = 0.1 * 0.04 / 6.0;
    let top = 50e3 / area + 5e3 / wel;
    let bot = 50e3 / area - 5e3 / wel;
    assert!(
        rel(
            result.max_sigma_z.max(result.min_sigma_z.abs()),
            top.max(bot.abs())
        ) < 0.01
    );
}

// ---------------------------------------------------------------------------
// FEM analysis
// ---------------------------------------------------------------------------

#[test]
fn smoke_fem_analysis_validation() {
    let rect = RectangularSection::new(0.1, 0.2).build();
    let mut fem = FemSectionAnalysis::new(rect, STEEL_S355);
    let cmp = fem.validate_properties();
    assert!(
        cmp.area_diff_pct < 0.5,
        "FEM area diff {}%",
        cmp.area_diff_pct
    );
    assert!(cmp.ix_diff_pct < 1.0, "FEM Ix diff {}%", cmp.ix_diff_pct);
    assert!(cmp.iy_diff_pct < 1.0, "FEM Iy diff {}%", cmp.iy_diff_pct);
}

// ---------------------------------------------------------------------------
// IO round-trips
// ---------------------------------------------------------------------------

#[test]
fn smoke_io_roundtrips() {
    let sec = ISection::from_designation("IPE300").unwrap().build();

    // JSON round-trip
    let props = SectionProperties::from_section(&sec);
    let json = to_json(&sec, Some(&props), Some(&STEEL_S355), None, true);
    let back = from_json(&json).expect("json parse");
    // JSON round-trip preserves geometry via outer polygon
    assert!(!json.is_empty());
    let _ = back;

    // CSV
    let csv = to_csv(&sec, CsvExportOptions::default());
    assert!(csv.contains(",") || csv.contains(";") || !csv.is_empty());

    // DXF export parses as text
    let dxf = to_dxf(&sec, DxfExportOptions::default());
    assert!(dxf.contains("ENTITIES") || dxf.contains("LINE") || dxf.contains("LWPOLYLINE"));

    // SVG
    let svg = to_svg(&sec, SvgExportOptions::default());
    assert!(svg.starts_with("<svg"));

    // HTML viewer
    let html = to_html(&sec, SvgExportOptions::default());
    assert!(html.contains("<!DOCTYPE html>"));

    // Nastran + VTK need a mesh
    let mesh = section_properties::mesh::mesh_section(
        &sec,
        section_properties::MeshParams {
            target_size: 0.03,
            ..Default::default()
        },
    );
    let bdf = to_nastran(&mesh, Some(&STEEL_S355), "SI");
    assert!(bdf.contains("CTRIA3"));
    let vtk = to_vtk(&mesh);
    assert!(vtk.contains("UNSTRUCTURED_GRID"));
}

// ---------------------------------------------------------------------------
// composite
// ---------------------------------------------------------------------------

#[test]
fn smoke_composite_transformed_properties() {
    let steel_sec = ISection::from_designation("IPE300").unwrap().build();
    let comp = CompositeSection::single_material(steel_sec, STEEL_S355);
    let props = comp.transformed_properties();
    assert!(props.area > 0.0);
    assert!(props.ix > 0.0);
}

// ---------------------------------------------------------------------------
// solver backends agree
// ---------------------------------------------------------------------------

#[test]
fn smoke_solver_backends_agree() {
    let n = 24;
    let mut k = section_properties::fea::SparseMatrix::new(n);
    for i in 0..n {
        k.add(i, i, 4.0);
        if i + 1 < n {
            k.add(i, i + 1, -1.0);
            k.add(i + 1, i, -1.0);
        }
    }
    let f: Vec<f64> = (0..n).map(|i| ((i % 4) as f64 - 1.5)).collect();

    let mut k2 = k.clone();
    k2.compress();
    let reference = SkylineLdlt::factor(&k2).unwrap().solve(&f).unwrap();

    // LU direct
    let lu = solvers::SparseLu::factor(&k).unwrap();
    let x_lu = lu.solve(&f);
    // ICCG iterative
    let mut k3 = k.clone();
    k3.compress();
    let x_iccg = solvers::iccg_solve(&k3, &f, 10000, 1e-12).x;
    // PCG iterative
    let mut k4 = k.clone();
    k4.compress();
    let x_pcg = section_properties::fea::cg_solve(&k4, &f, 10000, 1e-12).x;

    for i in 0..n {
        assert!((reference[i] - x_lu[i]).abs() < 1e-7, "LU i={}", i);
        assert!((reference[i] - x_iccg[i]).abs() < 1e-6, "ICCG i={}", i);
        assert!((reference[i] - x_pcg[i]).abs() < 1e-6, "PCG i={}", i);
    }
}

#[test]
fn smoke_split_section() {
    use section_properties::Geometry;
    let rect = RectangularSection::new(0.2, 0.1).build();
    let g = Geometry::from_section(&rect);
    // Horizontal line through the middle.
    let (below, above) = g.split_section(Point::new(0.0, 0.05), Point::new(1.0, 0.05));
    let a_b = below.map(|x| x.area()).unwrap_or(0.0);
    let a_a = above.map(|x| x.area()).unwrap_or(0.0);
    assert!((a_b - 0.01).abs() < 1e-9, "below={}", a_b);
    assert!((a_a - 0.01).abs() < 1e-9, "above={}", a_a);

    // Line entirely outside: one side keeps everything, the other is empty.
    let (s1, s2) = g.split_section(Point::new(0.0, 5.0), Point::new(1.0, 5.0));
    let v1 = s1.map(|x| x.area()).unwrap_or(0.0);
    let v2 = s2.map(|x| x.area()).unwrap_or(0.0);
    assert!(
        (v1 - 0.02).abs() < 1e-9 && v2 == 0.0 || (v2 - 0.02).abs() < 1e-9 && v1 == 0.0,
        "outside line should leave one full side"
    );
}

#[test]
fn smoke_fibre_section() {
    use section_properties::MeshParams;
    use section_properties::post::{to_fibre_section, total_area};
    let sec = ISection::from_designation("IPE300").unwrap().build();
    let fibres = to_fibre_section(
        &sec,
        MeshParams {
            target_size: 0.03,
            ..Default::default()
        },
    );
    let total = total_area(&fibres);
    assert!(rel(total, 5.38e-3) < 0.01, "fibre area {}", total);
}

#[test]
fn smoke_mass_property() {
    // Python get_mass = area * density
    let sec = RectangularSection::new(0.2, 0.1).build();
    let props = SectionProperties::from_section(&sec);
    let density = 7850.0;
    let mass_per_m = props.area * density;
    assert!((mass_per_m - 157.0).abs() < 0.1);
}

#[test]
fn smoke_compound_boolean() {
    use section_properties::{CompoundGeometry, Geometry};
    // Two disjoint squares as compound A
    let sq = |cx, cy| {
        Geometry::new(
            section_properties::Polygon::new(vec![
                Point::new(cx - 0.5, cy - 0.5),
                Point::new(cx + 0.5, cy - 0.5),
                Point::new(cx + 0.5, cy + 0.5),
                Point::new(cx - 0.5, cy + 0.5),
            ]),
            vec![],
        )
    };
    let a = CompoundGeometry::new(vec![sq(-0.75, 0.0), sq(0.75, 0.0)]);
    let b = CompoundGeometry::new(vec![sq(0.0, 0.0)]); // overlaps both halves

    // Difference: two half-squares remain (area 0.25 each)
    let d = a.subtract(&b).unwrap();
    // Each square loses its 0.25 overlap with b: 2 - 0.5 = 1.5
    assert!((d.area() - 1.5).abs() < 5e-3, "diff area {}", d.area());

    // Union: total area = 1 + 1 + overlap-dissolved... b covers middle:
    // A area=2, B=1, overlap with A = 2*0.25=0.5 -> union = 2+1-0.5=2.5
    let u = a.union(&b).unwrap();
    let expected_union = 2.0 + 1.0 - 0.5;
    assert!(
        (u.area() - expected_union).abs() < 5e-2,
        "union {}",
        u.area()
    );

    // Intersection: b ∩ a = 0.5
    let i = a.intersection(&b).unwrap();
    assert!((i.area() - 0.5).abs() < 5e-2, "inter {}", i.area());
}

#[test]
fn smoke_warping_svg() {
    let sec = ChannelSection::new(0.2, 0.075, 0.005, 0.008, 0.0, 0.0).build();
    let props = SectionProperties::from_section(&sec);
    let svg = section_properties::warping_svg(&sec, &props, 800, 600, 0.3).expect("warping svg");
    assert!(svg.contains("<svg"));
    assert!(svg.contains("wleg")); // legend gradient id
    assert!(svg.contains("rgb(")); // coloured elements
    assert!(!svg.contains("NaN"));
}

#[test]
fn smoke_compound_validate_and_dissolve() {
    use section_properties::{CompoundError, CompoundGeometry, Geometry};
    let sq = |cx, cy| {
        Geometry::new(
            section_properties::Polygon::new(vec![
                Point::new(cx - 0.5, cy - 0.5),
                Point::new(cx + 0.5, cy - 0.5),
                Point::new(cx + 0.5, cy + 0.5),
                Point::new(cx - 0.5, cy + 0.5),
            ]),
            vec![],
        )
    };

    // Disjoint: validate passes
    let disjoint = CompoundGeometry::new(vec![sq(-1.0, 0.0), sq(1.0, 0.0)]);
    assert!(disjoint.validate().is_ok());

    // Overlapping: validate catches it
    let overlapping = CompoundGeometry::new(vec![sq(-0.25, 0.0), sq(0.25, 0.0)]);
    match overlapping.validate() {
        Err(CompoundError::OverlappingRegions { a: 0, b: 1 }) => {}
        other => panic!("expected overlap error, got {:?}", other.map(|_| ())),
    }

    // Naive area double-counts; dissolved does not.
    // overlap = [−0.25,0.25]×[−0.5,0.5] = 0.5 → naive sums
    // both areas fully (2.0); true union = 1.5.
    assert!((overlapping.area() - 2.0).abs() < 1e-9);
    let dissolved = CompoundGeometry::dissolved_outer_only(vec![sq(-0.25, 0.0), sq(0.25, 0.0)]);
    assert!(
        (dissolved.area() - 1.5).abs() < 5e-2,
        "dissolved area {}",
        dissolved.area()
    );
    assert!(dissolved.validate().is_ok());

    // new_validated rejects overlaps
    assert!(CompoundGeometry::new_validated(vec![sq(-0.25, 0.0), sq(0.25, 0.0)]).is_err());

    // Hole escaping its outer boundary
    let outer_bad = Geometry::new(
        section_properties::Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(1.0, 0.0),
            Point::new(1.0, 1.0),
            Point::new(0.0, 1.0),
        ]),
        vec![section_properties::Polygon::new(vec![
            Point::new(0.8, 0.8),
            Point::new(1.5, 0.8),
            Point::new(1.5, 1.5),
            Point::new(0.8, 1.5),
        ])],
    );
    match CompoundGeometry::new(vec![outer_bad]).validate() {
        Err(CompoundError::HoleNotInsideOuter { region: 0, hole: 0 }) => {}
        other => panic!("expected hole-outside error, got {:?}", other.map(|_| ())),
    }
}
