use section_properties::{Point, Polygon, Section, SectionProperties};

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
    let rho_exp = ((props.ix + props.iy) / (2.0 * props.area)).sqrt();
    assert!((rx - rx_exp).abs() < 1e-10);
    assert!((ry - ry_exp).abs() < 1e-10);
    assert!((rho - rho_exp).abs() < 1e-10);
}
