use section_properties::{
    Point, Polygon, Section, SectionProperties,
    plastic::warping::WarpingProperties,
    section_library::ParametricSection,
    section_library::steel::ISection,
    section_library::steel::{
        IGirder, IGirderType, NastranBar, NastranBox, NastranChan, NastranCross, NastranI,
        NastranTee, NastranTube, NastranZed, SuperTGirder, SuperTType, UGirder,
    },
};

#[test]
fn rectangle_section() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 5.0),
        Point::new(0.0, 5.0),
    ]);
    let sec = Section::new(outer, Vec::new());

    // Area
    assert!((sec.area() - 50.0).abs() < 1e-10);

    // Centroid
    let c = sec.centroid();
    assert!((c.x - 5.0).abs() < 1e-10);
    assert!((c.y - 2.5).abs() < 1e-10);

    // Section properties
    let props = SectionProperties::from_section(&sec);
    assert!((props.area - 50.0).abs() < 1e-10);
    assert!((props.centroid.x - 5.0).abs() < 1e-10);
    assert!((props.centroid.y - 2.5).abs() < 1e-10);
    assert!((props.ix - 104.16666666666667).abs() < 1e-10);
    assert!((props.iy - 416.6666666666667).abs() < 1e-10);
    assert!(props.ixy.abs() < 1e-10);
}

#[test]
fn rectangle_with_hole() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 5.0),
        Point::new(0.0, 5.0),
    ]);
    let hole = Polygon::new(vec![
        Point::new(3.0, 1.0),
        Point::new(7.0, 1.0),
        Point::new(7.0, 4.0),
        Point::new(3.0, 4.0),
    ]); // CW hole
    let sec = Section::new(outer, vec![hole]);

    // Area = 50 - 12 = 38
    assert!((sec.area() - 38.0).abs() < 1e-9);

    // Centroid should remain near (5, 2.5) due to symmetry
    let c = sec.centroid();
    assert!((c.x - 5.0).abs() < 0.2);
    assert!((c.y - 2.5).abs() < 0.2);

    // Section properties
    let props = SectionProperties::from_section(&sec);
    assert!((props.area - 38.0).abs() < 1e-9);
    // For a symmetric hole, centroid stays at (5,2.5)
    assert!((props.centroid.x - 5.0).abs() < 0.2);
    assert!((props.centroid.y - 2.5).abs() < 0.2);
    // Moments of inertia: outer - hole (about same centroid)
    let outer_props = SectionProperties::from_section(&Section::new(
        Polygon::new(vec![
            Point::new(0.0, 0.0),
            Point::new(10.0, 0.0),
            Point::new(10.0, 5.0),
            Point::new(0.0, 5.0),
        ]),
        Vec::new(),
    ));
    let hole_props = SectionProperties::from_section(&Section::new(
        Polygon::new(vec![
            Point::new(3.0, 1.0),
            Point::new(7.0, 1.0),
            Point::new(7.0, 4.0),
            Point::new(3.0, 4.0),
        ]),
        Vec::new(),
    ));
    // Approximate check: props ≈ outer - hole (since centroids align)
    assert!((props.ix - (outer_props.ix - hole_props.ix)).abs() < 1.0);
    assert!((props.iy - (outer_props.iy - hole_props.iy)).abs() < 1.0);
}

#[test]
fn principal_moments_and_gyration() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 5.0),
        Point::new(0.0, 5.0),
    ]);
    let sec = Section::new(outer, Vec::new());
    let props = SectionProperties::from_section(&sec);

    // Principal moments for rectangle aligned with axes
    let (i1, i2, theta) = props.principal_moments();
    // Ix = 104.166..., Iy = 416.666..., so I1 should be Iy, I2 = Ix
    assert!((i1 - 416.6666666666667).abs() < 1e-10);
    assert!((i2 - 104.16666666666667).abs() < 1e-10);
    // Angle should be 0 or pi/2? Since Ix < Iy, principal axis rotates 90 deg?
    // Actually formula gives theta = 0.5 * atan2(2*Ixy, Ix - Iy). Ixy=0 => atan2(0, negative) = pi or -pi?
    // atan2(0, -x) = pi (or -pi). Then theta = pi/2 or -pi/2. But we expect axes aligned with x/y, so theta should be 0 or pi/2.
    // For rectangle, Ix and Iy are already principal, so theta should be 0 or pi/2. Due to sign, we expect near pi/2? Let's just check that cos(2*theta) ~ (Ix-Iy)/(Ix+Iy)
    // Instead, verify that rotating by theta gives diagonal form.
    // For simplicity, check that theta is near 0 or pi/2 within tolerance.
    let theta_abs = theta.abs();
    assert!(
        theta_abs < 1e-10 || (theta_abs - std::f64::consts::PI / 2.0).abs() < 1e-10,
        "theta = {}",
        theta
    );

    // Radius of gyration
    let (rx, ry, rho) = props.radius_of_gyration();
    let rx_exp = (props.ix / props.area).sqrt();
    let ry_exp = (props.iy / props.area).sqrt();
    let rho_exp = ((props.ix + props.iy) / props.area).sqrt();
    assert!((rx - rx_exp).abs() < 1e-10);
    assert!((ry - ry_exp).abs() < 1e-10);
    assert!((rho - rho_exp).abs() < 1e-10);

    // Max fiber distances must be measured from the centroid to the boundary
    assert!((props.max_fiber_distance_y - 2.5).abs() < 1e-10);
    assert!((props.max_fiber_distance_x - 5.0).abs() < 1e-10);
}
#[test]
fn principal_properties_invariants() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(8.0, 6.0),
        Point::new(2.0, 5.0),
    ]);

    let section = Section::new(outer, Vec::new());
    let props = SectionProperties::from_section(&section);

    let principal = props.principal_properties();

    // Invariant 1:
    // I1 + I2 = Ix + Iy
    assert!((principal.i11 + principal.i22 - props.ix - props.iy).abs() < 1e-10);

    // Invariant 2:
    // I1 * I2 = Ix * Iy - Ixy²
    assert!(
        (principal.i11 * principal.i22 - (props.ix * props.iy - props.ixy.powi(2))).abs() < 1e-10
    );

    // Principal moments are ordered.
    assert!(principal.i11 >= principal.i22);

    // Principal moments must be non-negative for a valid area.
    assert!(principal.i11 >= 0.0);
    assert!(principal.i22 >= 0.0);
}
#[test]
fn gyration_properties() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 5.0),
        Point::new(0.0, 5.0),
    ]);

    let section = Section::new(outer, Vec::new());
    let props = SectionProperties::from_section(&section);

    let gyration = props.gyration_properties();

    assert!((gyration.rx - (props.ix / props.area).sqrt()).abs() < 1e-10);

    assert!((gyration.ry - (props.iy / props.area).sqrt()).abs() < 1e-10);

    assert!((gyration.polar - ((props.ix + props.iy) / props.area).sqrt()).abs() < 1e-10);

    // Polar radius identity:
    // rp² = rx² + ry²
    assert!((gyration.polar.powi(2) - gyration.rx.powi(2) - gyration.ry.powi(2)).abs() < 1e-10);
}

#[test]
fn asymmetric_section_has_rotated_principal_axes() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 10.0),
        Point::new(0.0, 10.0),
    ]);

    // Remove the upper-right 7 x 7 square,
    // leaving an L-shaped section.
    let hole = Polygon::new(vec![
        Point::new(3.0, 3.0),
        Point::new(3.0, 10.0),
        Point::new(10.0, 10.0),
        Point::new(10.0, 3.0),
    ]);

    let section = Section::new(outer, vec![hole]);
    let props = SectionProperties::from_section(&section);

    // The section is asymmetric, so the product of inertia
    // should not vanish.
    assert!(props.ixy.abs() > 1e-10);

    let principal = props.principal_properties();

    // Principal moments must be ordered.
    assert!(principal.i11 >= principal.i22);

    // Principal-axis transformation must preserve the
    // first invariant.
    assert!((principal.i11 + principal.i22 - props.ix - props.iy).abs() < 1e-10);

    // And preserve the determinant.
    assert!(
        (principal.i11 * principal.i22 - (props.ix * props.iy - props.ixy.powi(2))).abs() < 1e-10
    );

    // Since Ixy != 0, this section should not have its
    // principal axes coincident with the original x/y axes.
    assert!(
        principal.phi.abs() > 1e-10
            && (principal.phi.abs() - std::f64::consts::PI / 2.0).abs() > 1e-10,
        "principal angle = {}",
        principal.phi
    );
}

#[test]
fn frame_properties_rectangle() {
    let outer = Polygon::new(vec![
        Point::new(0.0, 0.0),
        Point::new(10.0, 0.0),
        Point::new(10.0, 5.0),
        Point::new(0.0, 5.0),
    ]);
    let sec = Section::new(outer, Vec::new());

    let (area, ixx, iyy, ixy, j, phi) = sec.frame_properties();

    assert!((area - 50.0).abs() < 1e-10);
    assert!((ixx - 104.16666666666667).abs() < 1e-10);
    assert!((iyy - 416.6666666666667).abs() < 1e-10);
    assert!(ixy.abs() < 1e-10);
    // For rectangle: J = beta * a * b^3 where a >= b
    // a=10, b=5, ratio=2, beta≈0.229
    let expected_j = 0.229 * 10.0 * 5.0_f64.powi(3);
    assert!((j - expected_j).abs() / expected_j < 0.05);
    // For a symmetric section, principal angle is 0 or π/2
    assert!(phi.abs() < 1e-10 || (phi.abs() - std::f64::consts::FRAC_PI_2).abs() < 1e-10);
}

#[test]
fn frame_properties_i_section() {
    let i = ISection::new(0.3, 0.15, 0.007, 0.01, 0.012);
    let section = i.build();

    let (area, ixx, iyy, ixy, j, phi) = section.frame_properties();

    assert!(area > 0.0);
    assert!(ixx > 0.0);
    assert!(iyy > 0.0);
    assert!(ixy.abs() < 1e-10); // Doubly symmetric
    assert!(j > 0.0);
    assert!(phi.abs() < 1e-10); // Symmetric about both axes

    // Verify J matches WarpingProperties
    let wp = WarpingProperties::from_section(&section);
    assert!((j - wp.j).abs() < 1e-12);
}

#[test]
fn nastran_bar_section() {
    let bar = NastranBar::new(25.0); // 25mm diameter
    let sec = bar.build();
    let props = SectionProperties::from_section(&sec);
    let expected_area = std::f64::consts::PI * (0.0125_f64).powi(2);
    assert!((props.area - expected_area).abs() / expected_area < 0.02);
}

#[test]
fn nastran_box_section() {
    let box_sec = NastranBox::new(100.0, 50.0, 5.0); // 100x50x5mm
    let sec = box_sec.build();
    let props = SectionProperties::from_section(&sec);
    let outer_area = 0.1 * 0.05;
    let inner_area = 0.09 * 0.04;
    let expected = outer_area - inner_area;
    assert!((props.area - expected).abs() / expected < 0.02);
}

#[test]
fn nastran_chan_section() {
    let chan = NastranChan::standard_c10x15_3();
    let sec = chan.build();
    let props = SectionProperties::from_section(&sec);
    assert!(props.area > 0.0);
    assert!(props.ix > 0.0);
    assert!(props.iy > 0.0);
}

#[test]
fn nastran_cross_section() {
    let cross = NastranCross::new(100.0, 100.0, 10.0); // 100x100x10mm
    let sec = cross.build();
    let props = SectionProperties::from_section(&sec);
    assert!(props.area > 0.0);
    // Cross should be symmetric
    assert!(props.ixy.abs() < 1e-10);
}

#[test]
fn nastran_i_section() {
    let i = NastranI::standard_w12x26();
    let sec = i.build();
    let props = SectionProperties::from_section(&sec);
    assert!(props.area > 0.0);
    assert!(props.ix > props.iy); // Strong axis > weak axis
    assert!(props.ixy.abs() < 1e-10);
}

#[test]
fn nastran_tee_section() {
    let tee = NastranTee::new(100.0, 100.0, 6.0, 10.0);
    let sec = tee.build();
    let props = SectionProperties::from_section(&sec);
    assert!(props.area > 0.0);
}

#[test]
fn nastran_tube_section() {
    let tube = NastranTube::standard_51x5(); // 51mm OD x 5mm WT
    let sec = tube.build();
    let props = SectionProperties::from_section(&sec);
    let ro: f64 = 0.051 / 2.0;
    let ri: f64 = ro - 0.005;
    let expected = std::f64::consts::PI * (ro.powi(2) - ri.powi(2));
    assert!((props.area - expected).abs() / expected < 0.02);
}

#[test]
fn nastran_zed_section() {
    let zed = NastranZed::standard_z200();
    let sec = zed.build();
    let props = SectionProperties::from_section(&sec);
    assert!(props.area > 0.0);
}

#[test]
fn super_t_girder() {
    let girder = SuperTGirder::new(SuperTType::Type1400);
    let sec = girder.build();
    let props = SectionProperties::from_section(&sec);
    assert!(props.area > 0.0);
    assert!(props.ix > 0.0);
    assert!(props.iy > 0.0);
    // Depth should be ~1.4m
    assert!((sec.height() - 1.4).abs() < 0.05);
}

#[test]
fn i_girder_aashto() {
    let girder = IGirder::new(IGirderType::TypeIV);
    let sec = girder.build();
    let props = SectionProperties::from_section(&sec);
    assert!(props.area > 0.0);
    assert!(props.ix > 0.0);
    assert!(props.iy > 0.0);
    // Depth should be ~1.37m for Type IV
    assert!((sec.height() - 1.37).abs() < 0.05);
}

#[test]
fn u_girder() {
    let girder = UGirder::new(2000.0, 2000.0, 1500.0, 300.0, 200.0);
    let sec = girder.build();
    let props = SectionProperties::from_section(&sec);
    assert!(props.area > 0.0);
    assert!(props.ix > 0.0);
    assert!((sec.height() - 2.0).abs() < 0.05);
}
