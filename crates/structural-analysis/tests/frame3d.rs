//! Phase 133: 3D frame element foundation tests.
//!
//! Covers section validation, geometry/orientation, local stiffness matrix
//! properties, and coordinate transformation — with independent analytical
//! reference calculations.

#![allow(non_snake_case)]
#![allow(clippy::needless_range_loop)]

use structural_analysis::{Dof3D, FemError, FrameElement3D, FrameModel3D, FrameSection3D};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn steel_section() -> FrameSection3D {
    FrameSection3D::new(200e9, 80e9, 1e-3, 1e-6, 2e-6, 3e-6).unwrap()
}

fn default_ref() -> [f64; 3] {
    [0.0, 0.0, 1.0]
}

fn mat_vec(k: &[[f64; 12]; 12], v: &[f64; 12]) -> [f64; 12] {
    let mut r = [0.0; 12];
    for i in 0..12 {
        for j in 0..12 {
            r[i] += k[i][j] * v[j];
        }
    }
    r
}

fn dot(a: &[f64; 12], b: &[f64; 12]) -> f64 {
    let mut s = 0.0;
    for i in 0..12 {
        s += a[i] * b[i];
    }
    s
}

fn mat_mat_trans(k: &[[f64; 12]; 12]) -> [[f64; 12]; 12] {
    let mut r = [[0.0; 12]; 12];
    for i in 0..12 {
        for j in 0..12 {
            r[i][j] = k[j][i];
        }
    }
    r
}

fn mat_mul(a: &[[f64; 12]; 12], b: &[[f64; 12]; 12]) -> [[f64; 12]; 12] {
    let mut r = [[0.0; 12]; 12];
    for i in 0..12 {
        for j in 0..12 {
            for k in 0..12 {
                r[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    r
}

// ---------------------------------------------------------------------------
// Section validation
// ---------------------------------------------------------------------------

mod section_validation {
    use super::*;

    #[test]
    fn valid_parameters() {
        let s = FrameSection3D::new(200e9, 80e9, 1e-3, 1e-6, 2e-6, 3e-6);
        assert!(s.is_ok());
        let s = s.unwrap();
        assert!((s.E - 200e9).abs() < 1e-3);
        assert!((s.G - 80e9).abs() < 1e-3);
        assert!((s.area - 1e-3).abs() < 1e-15);
        assert!((s.iy - 1e-6).abs() < 1e-18);
        assert!((s.iz - 2e-6).abs() < 1e-18);
        assert!((s.j - 3e-6).abs() < 1e-18);
    }

    #[test]
    fn zero_and_negative_values() {
        assert!(matches!(
            FrameSection3D::new(0.0, 80e9, 1e-3, 1e-6, 2e-6, 3e-6),
            Err(FemError::InvalidInput(_))
        ));
        assert!(matches!(
            FrameSection3D::new(-1.0, 80e9, 1e-3, 1e-6, 2e-6, 3e-6),
            Err(FemError::InvalidInput(_))
        ));
        assert!(matches!(
            FrameSection3D::new(200e9, 0.0, 1e-3, 1e-6, 2e-6, 3e-6),
            Err(FemError::InvalidInput(_))
        ));
        assert!(matches!(
            FrameSection3D::new(200e9, -1.0, 1e-3, 1e-6, 2e-6, 3e-6),
            Err(FemError::InvalidInput(_))
        ));
        assert!(matches!(
            FrameSection3D::new(200e9, 80e9, 0.0, 1e-6, 2e-6, 3e-6),
            Err(FemError::InvalidInput(_))
        ));
        assert!(matches!(
            FrameSection3D::new(200e9, 80e9, -1.0, 1e-6, 2e-6, 3e-6),
            Err(FemError::InvalidInput(_))
        ));
        assert!(matches!(
            FrameSection3D::new(200e9, 80e9, 1e-3, 0.0, 2e-6, 3e-6),
            Err(FemError::InvalidInput(_))
        ));
        assert!(matches!(
            FrameSection3D::new(200e9, 80e9, 1e-3, -1.0, 2e-6, 3e-6),
            Err(FemError::InvalidInput(_))
        ));
        assert!(matches!(
            FrameSection3D::new(200e9, 80e9, 1e-3, 1e-6, 0.0, 3e-6),
            Err(FemError::InvalidInput(_))
        ));
        assert!(matches!(
            FrameSection3D::new(200e9, 80e9, 1e-3, 1e-6, -1.0, 3e-6),
            Err(FemError::InvalidInput(_))
        ));
        assert!(matches!(
            FrameSection3D::new(200e9, 80e9, 1e-3, 1e-6, 2e-6, 0.0),
            Err(FemError::InvalidInput(_))
        ));
        assert!(matches!(
            FrameSection3D::new(200e9, 80e9, 1e-3, 1e-6, 2e-6, -1.0),
            Err(FemError::InvalidInput(_))
        ));
    }

    #[test]
    fn nan_and_infinity() {
        let nan = f64::NAN;
        let pinf = f64::INFINITY;
        let ninf = f64::NEG_INFINITY;

        // E: NaN, +inf, -inf
        assert!(FrameSection3D::new(nan, 80e9, 1e-3, 1e-6, 2e-6, 3e-6).is_err());
        assert!(FrameSection3D::new(pinf, 80e9, 1e-3, 1e-6, 2e-6, 3e-6).is_err());
        assert!(FrameSection3D::new(ninf, 80e9, 1e-3, 1e-6, 2e-6, 3e-6).is_err());

        // G
        assert!(FrameSection3D::new(200e9, nan, 1e-3, 1e-6, 2e-6, 3e-6).is_err());
        assert!(FrameSection3D::new(200e9, pinf, 1e-3, 1e-6, 2e-6, 3e-6).is_err());
        assert!(FrameSection3D::new(200e9, ninf, 1e-3, 1e-6, 2e-6, 3e-6).is_err());

        // area
        assert!(FrameSection3D::new(200e9, 80e9, nan, 1e-6, 2e-6, 3e-6).is_err());
        assert!(FrameSection3D::new(200e9, 80e9, pinf, 1e-6, 2e-6, 3e-6).is_err());
        assert!(FrameSection3D::new(200e9, 80e9, ninf, 1e-6, 2e-6, 3e-6).is_err());

        // Iy
        assert!(FrameSection3D::new(200e9, 80e9, 1e-3, nan, 2e-6, 3e-6).is_err());
        assert!(FrameSection3D::new(200e9, 80e9, 1e-3, pinf, 2e-6, 3e-6).is_err());
        assert!(FrameSection3D::new(200e9, 80e9, 1e-3, ninf, 2e-6, 3e-6).is_err());

        // Iz
        assert!(FrameSection3D::new(200e9, 80e9, 1e-3, 1e-6, nan, 3e-6).is_err());
        assert!(FrameSection3D::new(200e9, 80e9, 1e-3, 1e-6, pinf, 3e-6).is_err());
        assert!(FrameSection3D::new(200e9, 80e9, 1e-3, 1e-6, ninf, 3e-6).is_err());

        // J
        assert!(FrameSection3D::new(200e9, 80e9, 1e-3, 1e-6, 2e-6, nan).is_err());
        assert!(FrameSection3D::new(200e9, 80e9, 1e-3, 1e-6, 2e-6, pinf).is_err());
        assert!(FrameSection3D::new(200e9, 80e9, 1e-3, 1e-6, 2e-6, ninf).is_err());
    }
}

// ---------------------------------------------------------------------------
// Geometry and orientation
// ---------------------------------------------------------------------------

mod geometry {
    use super::*;

    #[test]
    fn axis_aligned_member() {
        let e = FrameElement3D::new(0, 1, steel_section(), default_ref()).unwrap();
        let pi = (0.0, 0.0, 0.0);
        let pj = (1.0, 0.0, 0.0);
        let (x, y, z) = e.local_axes(pi, pj).unwrap();
        // x along global X
        assert!((x[0] - 1.0).abs() < 1e-14);
        assert!(x[1].abs() < 1e-14);
        assert!(x[2].abs() < 1e-14);
        // ref = global Z, x = global X → z = x × ref = X × Z = -Y, then y = z × x = (-Y) × X = Z
        // Wait: z = (x × ref)/|...| = (X × Z)/|X × Z| = -Y
        // y = z × x = (-Y) × X = Z
        // So y = global Z, z = -global Y
        assert!(
            (y[2] - 1.0).abs() < 1e-14,
            "y should be global Z, got {:?}",
            y
        );
        assert!(
            (z[1] + 1.0).abs() < 1e-14,
            "z should be -global Y, got {:?}",
            z
        );
    }

    #[test]
    fn member_along_each_global_axis() {
        let e = FrameElement3D::new(0, 1, steel_section(), [0.0, 1.0, 0.0]).unwrap();

        // Along X
        let (x, _, _) = e.local_axes((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)).unwrap();
        assert!((x[0] - 1.0).abs() < 1e-14);

        // Along Y — ref = global Y is parallel, should fail
        let result = e.local_axes((0.0, 0.0, 0.0), (0.0, 1.0, 0.0));
        assert!(result.is_err(), "ref parallel to member should fail");

        // Along Z — ref = global Y, x = global Z → z = Z × Y = -X, y = z × x = (-X) × Z = Y
        let (x, y, z) = e.local_axes((0.0, 0.0, 0.0), (0.0, 0.0, 1.0)).unwrap();
        assert!((x[2] - 1.0).abs() < 1e-14);
        assert!((y[1] - 1.0).abs() < 1e-14);
        assert!((z[0] + 1.0).abs() < 1e-14);
    }

    #[test]
    fn oblique_member() {
        let e = FrameElement3D::new(0, 1, steel_section(), default_ref()).unwrap();
        let pi = (0.0, 0.0, 0.0);
        let pj = (1.0, 2.0, 3.0);
        let (x, y, z) = e.local_axes(pi, pj).unwrap();
        let L = (1.0 + 4.0 + 9.0f64).sqrt();
        assert!((x[0] - 1.0 / L).abs() < 1e-14);
        assert!((x[1] - 2.0 / L).abs() < 1e-14);
        assert!((x[2] - 3.0 / L).abs() < 1e-14);

        // Orthonormality
        for v in [x, y, z] {
            let norm = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
            assert!((norm - 1.0).abs() < 1e-13, "axis not unit: {:?}", v);
        }
    }

    #[test]
    fn nontrivial_roll() {
        let pi = (0.0, 0.0, 0.0);
        let pj = (1.0, 0.0, 0.0);

        // ref = global Z
        let e1 = FrameElement3D::new(0, 1, steel_section(), [0.0, 0.0, 1.0]).unwrap();
        let (_, y1, _) = e1.local_axes(pi, pj).unwrap();

        // ref = (0, 1, 1) — 45° roll
        let e2 = FrameElement3D::new(0, 1, steel_section(), [0.0, 1.0, 1.0]).unwrap();
        let (_, y2, _) = e2.local_axes(pi, pj).unwrap();

        // y axes should differ
        let diff = (y1[0] - y2[0]).abs() + (y1[1] - y2[1]).abs() + (y1[2] - y2[2]).abs();
        assert!(diff > 0.1, "different roll should produce different y axes");
    }

    #[test]
    fn zero_length_member() {
        let e = FrameElement3D::new(0, 1, steel_section(), default_ref()).unwrap();
        let result = e.local_axes((1.0, 2.0, 3.0), (1.0, 2.0, 3.0));
        assert!(matches!(result, Err(FemError::ZeroLengthMember(_))));

        let result2 = e.local_stiffness((1.0, 2.0, 3.0), (1.0, 2.0, 3.0));
        assert!(matches!(result2, Err(FemError::ZeroLengthMember(_))));
    }

    #[test]
    fn non_finite_coordinates() {
        let e = FrameElement3D::new(0, 1, steel_section(), default_ref()).unwrap();
        assert!(e.local_axes((f64::NAN, 0.0, 0.0), (1.0, 0.0, 0.0)).is_err());
        assert!(
            e.local_axes((0.0, f64::INFINITY, 0.0), (1.0, 0.0, 0.0))
                .is_err()
        );
        assert!(
            e.local_axes((0.0, 0.0, 0.0), (f64::NEG_INFINITY, 0.0, 0.0))
                .is_err()
        );
    }

    #[test]
    fn zero_orientation_vector() {
        let result = FrameElement3D::new(0, 1, steel_section(), [0.0, 0.0, 0.0]);
        assert!(matches!(result, Err(FemError::InvalidInput(_))));
    }

    #[test]
    fn parallel_orientation_vector() {
        // Member along X, ref along X — exactly parallel
        let e = FrameElement3D::new(0, 1, steel_section(), [1.0, 0.0, 0.0]).unwrap();
        let result = e.local_axes((0.0, 0.0, 0.0), (1.0, 0.0, 0.0));
        assert!(matches!(result, Err(FemError::InvalidInput(_))));

        // Near-parallel: ref = (1, 1e-10, 0)
        let e2 = FrameElement3D::new(0, 1, steel_section(), [1.0, 1e-10, 0.0]).unwrap();
        let result2 = e2.local_axes((0.0, 0.0, 0.0), (1.0, 0.0, 0.0));
        assert!(matches!(result2, Err(FemError::InvalidInput(_))));
    }

    #[test]
    fn orthonormal_right_handed_basis() {
        let e = FrameElement3D::new(0, 1, steel_section(), [0.3, 0.7, 1.2]).unwrap();
        let pi = (1.0, -2.0, 0.5);
        let pj = (3.0, 1.0, 4.0);
        let (x, y, z) = e.local_axes(pi, pj).unwrap();

        // Unit length
        for (name, v) in [("x", x), ("y", y), ("z", z)] {
            let norm2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
            assert!((norm2 - 1.0).abs() < 1e-13, "{name} not unit");
        }

        // Orthogonality
        let xy = x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
        let xz = x[0] * z[0] + x[1] * z[1] + x[2] * z[2];
        let yz = y[0] * z[0] + y[1] * z[1] + y[2] * z[2];
        assert!(xy.abs() < 1e-13, "x·y != 0: {xy}");
        assert!(xz.abs() < 1e-13, "x·z != 0: {xz}");
        assert!(yz.abs() < 1e-13, "y·z != 0: {yz}");

        // Right-handed: x × y = z
        let cross = [
            x[1] * y[2] - x[2] * y[1],
            x[2] * y[0] - x[0] * y[2],
            x[0] * y[1] - x[1] * y[0],
        ];
        assert!((cross[0] - z[0]).abs() < 1e-13, "x×y != z");
        assert!((cross[1] - z[1]).abs() < 1e-13, "x×y != z");
        assert!((cross[2] - z[2]).abs() < 1e-13, "x×y != z");
    }
}

// ---------------------------------------------------------------------------
// Local stiffness
// ---------------------------------------------------------------------------

mod local_stiffness {
    use super::*;

    fn make_element() -> FrameElement3D {
        FrameElement3D::new(0, 1, steel_section(), default_ref()).unwrap()
    }

    #[test]
    fn dimensions_and_finiteness() {
        let e = make_element();
        let k = e.local_stiffness((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)).unwrap();
        for i in 0..12 {
            for j in 0..12 {
                assert!(k[i][j].is_finite(), "k[{i}][{j}] not finite: {}", k[i][j]);
            }
        }
    }

    #[test]
    fn symmetry() {
        let e = make_element();
        let k = e.local_stiffness((0.0, 0.0, 0.0), (2.5, 0.0, 0.0)).unwrap();
        for i in 0..12 {
            for j in 0..12 {
                assert!(
                    (k[i][j] - k[j][i]).abs() < 1e-6 * (1.0 + k[i][j].abs()),
                    "asymmetry at ({i},{j}): {} vs {}",
                    k[i][j],
                    k[j][i]
                );
            }
        }
    }

    #[test]
    fn axial_coefficients() {
        let e = make_element();
        let L = 2.0;
        let k = e.local_stiffness((0.0, 0.0, 0.0), (L, 0.0, 0.0)).unwrap();
        let s = steel_section();
        let EA_L = s.E * s.area / L;

        assert!(
            (k[0][0] - EA_L).abs() < 1e-3,
            "k[0][0] = {}, expected {}",
            k[0][0],
            EA_L
        );
        assert!(
            (k[0][6] + EA_L).abs() < 1e-3,
            "k[0][6] = {}, expected {}",
            k[0][6],
            -EA_L
        );
        assert!(
            (k[6][0] + EA_L).abs() < 1e-3,
            "k[6][0] = {}, expected {}",
            k[6][0],
            -EA_L
        );
        assert!(
            (k[6][6] - EA_L).abs() < 1e-3,
            "k[6][6] = {}, expected {}",
            k[6][6],
            EA_L
        );
    }

    #[test]
    fn torsion_coefficients() {
        let e = make_element();
        let L = 2.0;
        let k = e.local_stiffness((0.0, 0.0, 0.0), (L, 0.0, 0.0)).unwrap();
        let s = steel_section();
        let GJ_L = s.G * s.j / L;

        assert!(
            (k[3][3] - GJ_L).abs() < 1e-3,
            "k[3][3] = {}, expected {}",
            k[3][3],
            GJ_L
        );
        assert!(
            (k[3][9] + GJ_L).abs() < 1e-3,
            "k[3][9] = {}, expected {}",
            k[3][9],
            -GJ_L
        );
        assert!(
            (k[9][3] + GJ_L).abs() < 1e-3,
            "k[9][3] = {}, expected {}",
            k[9][3],
            -GJ_L
        );
        assert!(
            (k[9][9] - GJ_L).abs() < 1e-3,
            "k[9][9] = {}, expected {}",
            k[9][9],
            GJ_L
        );
    }

    #[test]
    fn bending_z_coefficients_and_signs() {
        // Bending about z (x-y plane): DOFs 1, 5, 7, 11 (v, rz)
        let e = make_element();
        let L = 2.0;
        let k = e.local_stiffness((0.0, 0.0, 0.0), (L, 0.0, 0.0)).unwrap();
        let s = steel_section();
        let EIz_L3 = s.E * s.iz / L.powi(3);
        let EIz_L2 = s.E * s.iz / L.powi(2);
        let EIz_L = s.E * s.iz / L;

        // Independent reference: standard 2D beam stiffness
        assert!((k[1][1] - 12.0 * EIz_L3).abs() < 1e-3);
        assert!((k[1][5] - 6.0 * EIz_L2).abs() < 1e-3);
        assert!((k[1][7] + 12.0 * EIz_L3).abs() < 1e-3);
        assert!((k[1][11] - 6.0 * EIz_L2).abs() < 1e-3);

        assert!((k[5][5] - 4.0 * EIz_L).abs() < 1e-3);
        assert!((k[5][7] + 6.0 * EIz_L2).abs() < 1e-3);
        assert!((k[5][11] - 2.0 * EIz_L).abs() < 1e-3);

        assert!((k[7][7] - 12.0 * EIz_L3).abs() < 1e-3);
        assert!((k[7][11] + 6.0 * EIz_L2).abs() < 1e-3);

        assert!((k[11][11] - 4.0 * EIz_L).abs() < 1e-3);
    }

    #[test]
    fn bending_y_coefficients_and_signs() {
        // Bending about y (x-z plane): DOFs 2, 4, 8, 10 (w, ry)
        // Coupling signs are OPPOSITE to z-bending
        let e = make_element();
        let L = 2.0;
        let k = e.local_stiffness((0.0, 0.0, 0.0), (L, 0.0, 0.0)).unwrap();
        let s = steel_section();
        let EIy_L3 = s.E * s.iy / L.powi(3);
        let EIy_L2 = s.E * s.iy / L.powi(2);
        let EIy_L = s.E * s.iy / L;

        // Independent reference: x-z plane bending with right-hand rule
        assert!((k[2][2] - 12.0 * EIy_L3).abs() < 1e-3);
        assert!(
            (k[2][4] + 6.0 * EIy_L2).abs() < 1e-3,
            "k[2][4] = {}, expected {}",
            k[2][4],
            -6.0 * EIy_L2
        );
        assert!((k[2][8] + 12.0 * EIy_L3).abs() < 1e-3);
        assert!(
            (k[2][10] + 6.0 * EIy_L2).abs() < 1e-3,
            "k[2][10] = {}, expected {}",
            k[2][10],
            -6.0 * EIy_L2
        );

        assert!((k[4][4] - 4.0 * EIy_L).abs() < 1e-3);
        assert!(
            (k[4][8] - 6.0 * EIy_L2).abs() < 1e-3,
            "k[4][8] = {}, expected {}",
            k[4][8],
            6.0 * EIy_L2
        );
        assert!((k[4][10] - 2.0 * EIy_L).abs() < 1e-3);

        assert!((k[8][8] - 12.0 * EIy_L3).abs() < 1e-3);
        assert!(
            (k[8][10] - 6.0 * EIy_L2).abs() < 1e-3,
            "k[8][10] = {}, expected {}",
            k[8][10],
            6.0 * EIy_L2
        );

        assert!((k[10][10] - 4.0 * EIy_L).abs() < 1e-3);
    }

    #[test]
    fn six_rigid_body_modes() {
        let e = make_element();
        let L = 3.0;
        let k = e.local_stiffness((0.0, 0.0, 0.0), (L, 0.0, 0.0)).unwrap();

        let modes = [
            // Translation in x
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            // Translation in y
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            // Translation in z
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            // Rotation about x (torsion)
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            // Rotation about y (through node i): w_j = -L
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -L, 0.0, 1.0, 0.0],
            // Rotation about z (through node i): v_j = L
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, L, 0.0, 0.0, 0.0, 1.0],
        ];

        for (idx, mode) in modes.iter().enumerate() {
            let kv = mat_vec(&k, mode);
            let residual = dot(&kv, &kv).sqrt();
            assert!(
                residual < 1e-3,
                "rigid-body mode {idx} not in null space: |K·v| = {residual:.3e}"
            );
        }
    }

    #[test]
    fn positive_non_rigid_stiffness() {
        let e = make_element();
        let L = 2.0;
        let k = e.local_stiffness((0.0, 0.0, 0.0), (L, 0.0, 0.0)).unwrap();

        // Non-rigid displacements: u^T K u > 0
        let test_displacements = [
            // Axial stretch: u_i = 0, u_j = 1
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            // Transverse v at j
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            // Transverse w at j
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            // Torsion rx at j
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            // Rotation ry at j
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            // Rotation rz at j
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            // Bending mode: v_i = 1, rz_i = 0, v_j = 0, rz_j = 0
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ];

        for (idx, u) in test_displacements.iter().enumerate() {
            let ku = mat_vec(&k, u);
            let energy = 0.5 * dot(u, &ku);
            assert!(
                energy > 0.0,
                "test displacement {idx} has non-positive strain energy: {energy}"
            );
        }
    }

    #[test]
    fn scaling_with_E() {
        let s = steel_section();
        let e1 = FrameElement3D::new(0, 1, s, default_ref()).unwrap();
        let e2 = FrameElement3D::new(
            0,
            1,
            FrameSection3D::new(s.E * 10.0, s.G, s.area, s.iy, s.iz, s.j).unwrap(),
            default_ref(),
        )
        .unwrap();

        let pi = (0.0, 0.0, 0.0);
        let pj = (2.0, 0.0, 0.0);
        let k1 = e1.local_stiffness(pi, pj).unwrap();
        let k2 = e2.local_stiffness(pi, pj).unwrap();

        // Axial and bending terms scale with E; torsion does not (G unchanged)
        assert!(
            (k2[0][0] / k1[0][0] - 10.0).abs() < 1e-12,
            "axial should scale with E"
        );
        assert!(
            (k2[1][1] / k1[1][1] - 10.0).abs() < 1e-12,
            "bending z should scale with E"
        );
        assert!(
            (k2[2][2] / k1[2][2] - 10.0).abs() < 1e-12,
            "bending y should scale with E"
        );
        // Torsion unchanged (G same)
        assert!(
            (k2[3][3] - k1[3][3]).abs() < 1e-3,
            "torsion should not change"
        );
    }

    #[test]
    fn scaling_with_G() {
        let s = steel_section();
        let e1 = FrameElement3D::new(0, 1, s, default_ref()).unwrap();
        let e2 = FrameElement3D::new(
            0,
            1,
            FrameSection3D::new(s.E, s.G * 5.0, s.area, s.iy, s.iz, s.j).unwrap(),
            default_ref(),
        )
        .unwrap();

        let pi = (0.0, 0.0, 0.0);
        let pj = (2.0, 0.0, 0.0);
        let k1 = e1.local_stiffness(pi, pj).unwrap();
        let k2 = e2.local_stiffness(pi, pj).unwrap();

        // Only torsion scales with G
        assert!(
            (k2[3][3] / k1[3][3] - 5.0).abs() < 1e-12,
            "torsion should scale with G"
        );
        assert!(
            (k2[0][0] - k1[0][0]).abs() < 1e-3,
            "axial should not change"
        );
        assert!(
            (k2[1][1] - k1[1][1]).abs() < 1e-3,
            "bending should not change"
        );
    }

    #[test]
    fn scaling_with_length() {
        let e = make_element();
        let k1 = e.local_stiffness((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)).unwrap();
        let k2 = e.local_stiffness((0.0, 0.0, 0.0), (2.0, 0.0, 0.0)).unwrap();

        let s = steel_section();
        // Axial: EA/L → halving when L doubles
        let EA_L1 = s.E * s.area / 1.0;
        let EA_L2 = s.E * s.area / 2.0;
        assert!((k1[0][0] - EA_L1).abs() < 1e-3);
        assert!((k2[0][0] - EA_L2).abs() < 1e-3);

        // Bending: 12*EI/L³ → 1/8 when L doubles
        let ratio = k2[1][1] / k1[1][1];
        assert!(
            (ratio - 0.125).abs() < 1e-12,
            "bending should scale as 1/L³, got ratio {ratio}"
        );
    }

    #[test]
    fn scaling_with_area_and_inertias() {
        let s = steel_section();
        let e1 = FrameElement3D::new(0, 1, s, default_ref()).unwrap();
        let e2 = FrameElement3D::new(
            0,
            1,
            FrameSection3D::new(s.E, s.G, s.area * 3.0, s.iy * 5.0, s.iz * 7.0, s.j * 2.0).unwrap(),
            default_ref(),
        )
        .unwrap();

        let pi = (0.0, 0.0, 0.0);
        let pj = (2.0, 0.0, 0.0);
        let k1 = e1.local_stiffness(pi, pj).unwrap();
        let k2 = e2.local_stiffness(pi, pj).unwrap();

        assert!(
            (k2[0][0] / k1[0][0] - 3.0).abs() < 1e-12,
            "axial scales with A"
        );
        assert!(
            (k2[2][2] / k1[2][2] - 5.0).abs() < 1e-12,
            "bending y scales with Iy"
        );
        assert!(
            (k2[1][1] / k1[1][1] - 7.0).abs() < 1e-12,
            "bending z scales with Iz"
        );
        assert!(
            (k2[3][3] / k1[3][3] - 2.0).abs() < 1e-12,
            "torsion scales with J"
        );
    }
}

// ---------------------------------------------------------------------------
// Transformation
// ---------------------------------------------------------------------------

mod transformation {
    use super::*;

    #[test]
    fn identity_aligned_element() {
        // Member along global X, ref = global Z
        // x = X, z = X × Z / |...| = -Y, y = z × x = (-Y) × X = Z
        // So R = [1 0 0; 0 0 1; 0 -1 0]
        let e = FrameElement3D::new(0, 1, steel_section(), default_ref()).unwrap();
        let T = e
            .transformation_matrix((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
            .unwrap();

        // Check block structure: 4 copies of R
        for block in 0..4 {
            let off = block * 3;
            // R = [1 0 0; 0 0 1; 0 -1 0]
            assert!((T[off][off] - 1.0).abs() < 1e-14, "block {block} R[0][0]");
            assert!(T[off][off + 1].abs() < 1e-14, "block {block} R[0][1]");
            assert!(T[off][off + 2].abs() < 1e-14, "block {block} R[0][2]");
            assert!(T[off + 1][off].abs() < 1e-14, "block {block} R[1][0]");
            assert!(T[off + 1][off + 1].abs() < 1e-14, "block {block} R[1][1]");
            assert!(
                (T[off + 1][off + 2] - 1.0).abs() < 1e-14,
                "block {block} R[1][2]"
            );
            assert!(T[off + 2][off].abs() < 1e-14, "block {block} R[2][0]");
            assert!(
                (T[off + 2][off + 1] + 1.0).abs() < 1e-14,
                "block {block} R[2][1]"
            );
            assert!(T[off + 2][off + 2].abs() < 1e-14, "block {block} R[2][2]");
        }
    }

    #[test]
    fn oblique_element() {
        let e = FrameElement3D::new(0, 1, steel_section(), [0.0, 0.0, 1.0]).unwrap();
        let pi = (0.0, 0.0, 0.0);
        let pj = (1.0, 2.0, 3.0);
        let T = e.transformation_matrix(pi, pj).unwrap();

        // Check that off-diagonal blocks are zero
        for block_i in 0..4 {
            for block_j in 0..4 {
                if block_i == block_j {
                    continue;
                }
                let off_i = block_i * 3;
                let off_j = block_j * 3;
                for i in 0..3 {
                    for j in 0..3 {
                        assert!(
                            T[off_i + i][off_j + j].abs() < 1e-14,
                            "off-diagonal block ({block_i},{block_j}) not zero"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn orthogonality() {
        let e = FrameElement3D::new(0, 1, steel_section(), [0.3, 0.7, 1.2]).unwrap();
        let pi = (1.0, -2.0, 0.5);
        let pj = (3.0, 1.0, 4.0);
        let T = e.transformation_matrix(pi, pj).unwrap();

        // T · T^T = I
        let Tt = mat_mat_trans(&T);
        let TTt = mat_mul(&T, &Tt);
        for i in 0..12 {
            for j in 0..12 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (TTt[i][j] - expected).abs() < 1e-13,
                    "T·Tᵀ[{i}][{j}] = {}, expected {expected}",
                    TTt[i][j]
                );
            }
        }

        // T^T · T = I (since T is orthogonal, T^T = T^{-1})
        let TtT = mat_mul(&Tt, &T);
        for i in 0..12 {
            for j in 0..12 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (TtT[i][j] - expected).abs() < 1e-13,
                    "Tᵀ·T[{i}][{j}] = {}, expected {expected}",
                    TtT[i][j]
                );
            }
        }
    }

    #[test]
    fn round_trip_transformation() {
        // T^T · T · u = u for any u (since T is orthogonal)
        let e = FrameElement3D::new(0, 1, steel_section(), [0.5, 1.0, 0.8]).unwrap();
        let pi = (2.0, -1.0, 3.0);
        let pj = (5.0, 2.0, 7.0);
        let T = e.transformation_matrix(pi, pj).unwrap();
        let Tt = mat_mat_trans(&T);

        let u = [
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0,
        ];
        let Tu = mat_vec(&T, &u);
        let TtTu = mat_vec(&Tt, &Tu);

        for i in 0..12 {
            assert!(
                (TtTu[i] - u[i]).abs() < 1e-12,
                "round-trip failed at {i}: {} vs {}",
                TtTu[i],
                u[i]
            );
        }
    }

    #[test]
    fn transformed_stiffness_symmetry() {
        let e = FrameElement3D::new(0, 1, steel_section(), [0.3, 0.7, 1.2]).unwrap();
        let pi = (1.0, -2.0, 0.5);
        let pj = (3.0, 1.0, 4.0);
        let k_global = e.global_stiffness(pi, pj).unwrap();

        for i in 0..12 {
            for j in 0..12 {
                assert!(
                    (k_global[i][j] - k_global[j][i]).abs() < 1e-3 * (1.0 + k_global[i][j].abs()),
                    "global stiffness asymmetric at ({i},{j}): {} vs {}",
                    k_global[i][j],
                    k_global[j][i]
                );
            }
        }
    }

    #[test]
    fn strain_energy_invariance() {
        // U = 0.5 * u_local^T * K_local * u_local
        //   = 0.5 * u_global^T * K_global * u_global
        // where u_local = T * u_global
        let e = FrameElement3D::new(0, 1, steel_section(), [0.3, 0.7, 1.2]).unwrap();
        let pi = (1.0, -2.0, 0.5);
        let pj = (3.0, 1.0, 4.0);

        let k_local = e.local_stiffness(pi, pj).unwrap();
        let T = e.transformation_matrix(pi, pj).unwrap();
        let k_global = e.global_stiffness(pi, pj).unwrap();

        // Test with several global displacement vectors
        let test_vectors = [
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            [
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0,
            ],
            [
                0.1, -0.5, 0.3, 2.0, -1.0, 0.7, -0.3, 1.5, 0.2, -2.0, 0.8, 0.4,
            ],
        ];

        for (idx, &u_global) in test_vectors.iter().enumerate() {
            let u_local = mat_vec(&T, &u_global);

            let ku_local = mat_vec(&k_local, &u_local);
            let energy_local = 0.5 * dot(&u_local, &ku_local);

            let ku_global = mat_vec(&k_global, &u_global);
            let energy_global = 0.5 * dot(&u_global, &ku_global);

            assert!(
                (energy_local - energy_global).abs() < 1e-3 * (1.0 + energy_local.abs()),
                "strain energy mismatch for vector {idx}: local={energy_local:.6e}, global={energy_global:.6e}"
            );
        }
    }
}

// ===========================================================================
// Phase 134: Global model, solver, and analysis result tests
// ===========================================================================

/// Phase 134 section: E=200 GPa, G=80 GPa, A=1e-3, Iy=2e-6, Iz=1e-6, J=3e-6.
fn phase134_section() -> FrameSection3D {
    FrameSection3D::new(200e9, 80e9, 1e-3, 2e-6, 1e-6, 3e-6).unwrap()
}

const E134: f64 = 200e9;
const G134: f64 = 80e9;
const A134: f64 = 1e-3;
const IY134: f64 = 2e-6;
const IZ134: f64 = 1e-6;
const J134: f64 = 3e-6;
const L134: f64 = 1.0;

// ---------------------------------------------------------------------------
// Analytical benchmarks
// ---------------------------------------------------------------------------

#[test]
fn phase134_axial_bar_displacement() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();

    let p = 1e3_f64;
    model
        .add_nodal_load(n1, p, 0.0, 0.0, 0.0, 0.0, 0.0)
        .unwrap();

    let result = model.solve().unwrap();

    let delta = p * L134 / (E134 * A134);
    let ux = result.displacement(n1, Dof3D::Ux).unwrap();
    assert!(
        (ux - delta).abs() < 1e-10 * (1.0 + delta.abs()),
        "axial displacement: got {ux:.6e}, expected {delta:.6e}"
    );

    let rx = result.reaction(n0, Dof3D::Ux).unwrap();
    assert!(
        (rx + p).abs() < 1e-6 * p,
        "axial reaction: got {rx:.6e}, expected {}",
        -p
    );
}

#[test]
fn phase134_torsion_displacement() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();

    let t = 1e2_f64;
    model
        .add_nodal_load(n1, 0.0, 0.0, 0.0, t, 0.0, 0.0)
        .unwrap();

    let result = model.solve().unwrap();

    let theta = t * L134 / (G134 * J134);
    let rx = result.displacement(n1, Dof3D::Rx).unwrap();
    assert!(
        (rx - theta).abs() < 1e-10 * (1.0 + theta.abs()),
        "torsion rotation: got {rx:.6e}, expected {theta:.6e}"
    );

    let mrx = result.reaction(n0, Dof3D::Rx).unwrap();
    assert!(
        (mrx + t).abs() < 1e-6 * t,
        "torsion reaction: got {mrx:.6e}, expected {}",
        -t
    );
}

#[test]
fn phase134_cantilever_bending_xy() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();

    let p = 1e3_f64;
    model
        .add_nodal_load(n1, 0.0, p, 0.0, 0.0, 0.0, 0.0)
        .unwrap();

    let result = model.solve().unwrap();

    let delta = p * L134.powi(3) / (3.0 * E134 * IZ134);
    let theta = p * L134.powi(2) / (2.0 * E134 * IZ134);

    let uy = result.displacement(n1, Dof3D::Uy).unwrap();
    assert!(
        (uy - delta).abs() < 1e-10 * (1.0 + delta.abs()),
        "bending XY displacement: got {uy:.6e}, expected {delta:.6e}"
    );

    let rz = result.displacement(n1, Dof3D::Rz).unwrap();
    assert!(
        (rz - theta).abs() < 1e-10 * (1.0 + theta.abs()),
        "bending XY rotation: got {rz:.6e}, expected {theta:.6e}"
    );

    let ry = result.reaction(n0, Dof3D::Uy).unwrap();
    assert!(
        (ry + p).abs() < 1e-6 * p,
        "bending XY reaction force: got {ry:.6e}, expected {}",
        -p
    );

    let mz = result.reaction(n0, Dof3D::Rz).unwrap();
    let expected_mz = -p * L134;
    assert!(
        (mz - expected_mz).abs() < 1e-6 * p * L134,
        "bending XY reaction moment: got {mz:.6e}, expected {expected_mz:.6e}"
    );
}

#[test]
fn phase134_cantilever_bending_xz() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();

    let p = 1e3_f64;
    model
        .add_nodal_load(n1, 0.0, 0.0, p, 0.0, 0.0, 0.0)
        .unwrap();

    let result = model.solve().unwrap();

    let delta = p * L134.powi(3) / (3.0 * E134 * IY134);
    let theta = -p * L134.powi(2) / (2.0 * E134 * IY134);

    let uz = result.displacement(n1, Dof3D::Uz).unwrap();
    assert!(
        (uz - delta).abs() < 1e-10 * (1.0 + delta.abs()),
        "bending XZ displacement: got {uz:.6e}, expected {delta:.6e}"
    );

    let ry = result.displacement(n1, Dof3D::Ry).unwrap();
    assert!(
        (ry - theta).abs() < 1e-10 * (1.0 + theta.abs()),
        "bending XZ rotation: got {ry:.6e}, expected {theta:.6e}"
    );

    let rz = result.reaction(n0, Dof3D::Uz).unwrap();
    assert!(
        (rz + p).abs() < 1e-6 * p,
        "bending XZ reaction force: got {rz:.6e}, expected {}",
        -p
    );

    let my = result.reaction(n0, Dof3D::Ry).unwrap();
    let expected_my = p * L134;
    assert!(
        (my - expected_my).abs() < 1e-6 * p * L134,
        "bending XZ reaction moment: got {my:.6e}, expected {expected_my:.6e}"
    );
}

// ---------------------------------------------------------------------------
// Integration tests
// ---------------------------------------------------------------------------

#[test]
fn phase134_oblique_member_axial() {
    let s = L134 / 2f64.sqrt();

    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(s, s, 0.0).unwrap();
    model
        .add_member(n0, n1, phase134_section(), [0.0, 0.0, 1.0])
        .unwrap();
    model.fix_node(n0).unwrap();

    let p = 1e3_f64;
    let px = p / 2f64.sqrt();
    model
        .add_nodal_load(n1, px, px, 0.0, 0.0, 0.0, 0.0)
        .unwrap();

    let result = model.solve().unwrap();

    let delta = p * L134 / (E134 * A134);
    let expected = delta / 2f64.sqrt();

    let ux = result.displacement(n1, Dof3D::Ux).unwrap();
    let uy = result.displacement(n1, Dof3D::Uy).unwrap();
    let uz = result.displacement(n1, Dof3D::Uz).unwrap();

    assert!(
        (ux - expected).abs() < 1e-10 * (1.0 + expected.abs()),
        "oblique ux: got {ux:.6e}, expected {expected:.6e}"
    );
    assert!(
        (uy - expected).abs() < 1e-10 * (1.0 + expected.abs()),
        "oblique uy: got {uy:.6e}, expected {expected:.6e}"
    );
    assert!(uz.abs() < 1e-12, "oblique uz should be zero, got {uz:.6e}");
}

#[test]
fn phase134_two_member_L_frame() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(1.0, 0.0, 0.0).unwrap();
    let n2 = model.add_node(1.0, 0.0, 1.0).unwrap();

    model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model
        .add_member(n1, n2, phase134_section(), [1.0, 0.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();

    let p = 1e3_f64;
    model
        .add_nodal_load(n2, 0.0, p, 0.0, 0.0, 0.0, 0.0)
        .unwrap();

    let result = model.solve().unwrap();

    let uy = result.displacement(n2, Dof3D::Uy).unwrap();
    assert!(
        uy > 0.0,
        "tip displacement should be positive, got {uy:.6e}"
    );

    let mut sum_fy = 0.0;
    let mut sum_mz = 0.0;
    for i in 0..result.n_nodes {
        sum_fy += result.reactions[i * 6 + 1];
        let (x, _y, z) = result.node_coords[i];
        let fy = result.reactions[i * 6 + 1];
        let mz = result.reactions[i * 6 + 5];
        sum_mz += mz + x * fy;
        let _ = z;
    }
    sum_fy += p;
    sum_mz += 1.0 * p;

    assert!(
        sum_fy.abs() < 1e-6 * p,
        "force equilibrium: sum_fy = {sum_fy:.6e}"
    );
    assert!(
        sum_mz.abs() < 1e-6 * p,
        "moment equilibrium: sum_mz = {sum_mz:.6e}"
    );
}

#[test]
fn phase134_prescribed_nonzero_displacement() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();

    let delta0 = 5e-4_f64;
    model.fix_dof(n0, Dof3D::Ux, delta0).unwrap();
    model.fix_dof(n0, Dof3D::Uy, 0.0).unwrap();
    model.fix_dof(n0, Dof3D::Uz, 0.0).unwrap();
    model.fix_dof(n0, Dof3D::Rx, 0.0).unwrap();
    model.fix_dof(n0, Dof3D::Ry, 0.0).unwrap();
    model.fix_dof(n0, Dof3D::Rz, 0.0).unwrap();

    let result = model.solve().unwrap();

    let ux0 = result.displacement(n0, Dof3D::Ux).unwrap();
    let ux1 = result.displacement(n1, Dof3D::Ux).unwrap();
    assert!(
        (ux0 - delta0).abs() < 1e-15,
        "prescribed ux0: got {ux0:.6e}"
    );
    assert!(
        (ux1 - delta0).abs() < 1e-12,
        "rigid translation ux1: got {ux1:.6e}, expected {delta0:.6e}"
    );

    for dof in Dof3D::ALL {
        let d = result.displacement(n1, dof).unwrap();
        if dof == Dof3D::Ux {
            continue;
        }
        assert!(
            d.abs() < 1e-12,
            "DOF {:?} at n1 should be zero, got {d:.6e}",
            dof
        );
    }

    for i in 0..result.reactions.len() {
        assert!(
            result.reactions[i].abs() < 1e-6,
            "reaction {i} should be zero, got {}",
            result.reactions[i]
        );
    }
}

#[test]
fn phase134_fully_constrained() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();
    model.fix_node(n1).unwrap();

    let result = model.solve().unwrap();

    for i in 0..result.displacements.len() {
        assert!(
            result.displacements[i].abs() < 1e-15,
            "displacement {i} should be zero"
        );
    }
    for i in 0..result.reactions.len() {
        assert!(
            result.reactions[i].abs() < 1e-15,
            "reaction {i} should be zero"
        );
    }
}

#[test]
fn phase134_mechanism_singular() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();

    model
        .add_nodal_load(n1, 1e3, 0.0, 0.0, 0.0, 0.0, 0.0)
        .unwrap();

    let result = model.solve();
    assert!(result.is_err(), "unconstrained model should fail");
}

#[test]
fn phase134_no_load_zero_displacement() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();

    let result = model.solve().unwrap();

    for i in 0..result.displacements.len() {
        assert!(
            result.displacements[i].abs() < 1e-15,
            "displacement {i} should be zero"
        );
    }
    for i in 0..result.reactions.len() {
        assert!(
            result.reactions[i].abs() < 1e-15,
            "reaction {i} should be zero"
        );
    }
}

#[test]
fn phase134_solve_twice() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();
    model
        .add_nodal_load(n1, 0.0, 1e3, 0.0, 0.0, 0.0, 0.0)
        .unwrap();

    let r1 = model.solve().unwrap();
    let r2 = model.solve().unwrap();

    for i in 0..r1.displacements.len() {
        assert!(
            (r1.displacements[i] - r2.displacements[i]).abs() < 1e-15,
            "displacement mismatch at DOF {i}"
        );
    }
    for i in 0..r1.reactions.len() {
        assert!(
            (r1.reactions[i] - r2.reactions[i]).abs() < 1e-15,
            "reaction mismatch at DOF {i}"
        );
    }
}

#[test]
fn phase134_member_end_forces_axial() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();

    let p = 1e3_f64;
    model
        .add_nodal_load(n1, p, 0.0, 0.0, 0.0, 0.0, 0.0)
        .unwrap();

    let result = model.solve().unwrap();
    let f = result.member_end_forces(0).unwrap();

    assert!(
        (f[0] + p).abs() < 1e-6 * p,
        "axial end force at i: got {:.6e}, expected {}",
        f[0],
        -p
    );
    assert!(
        (f[6] - p).abs() < 1e-6 * p,
        "axial end force at j: got {:.6e}, expected {}",
        f[6],
        p
    );
    for idx in [1, 2, 3, 4, 5, 7, 8, 9, 10, 11] {
        assert!(
            f[idx].abs() < 1e-6 * p,
            "non-axial end force {idx} should be zero, got {:.6e}",
            f[idx]
        );
    }
}

#[test]
fn phase134_member_end_forces_bending() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();

    let p = 1e3_f64;
    model
        .add_nodal_load(n1, 0.0, p, 0.0, 0.0, 0.0, 0.0)
        .unwrap();

    let result = model.solve().unwrap();
    let f = result.member_end_forces(0).unwrap();

    assert!(
        (f[1] + p).abs() < 1e-6 * p,
        "bending end force at i (fy): got {:.6e}, expected {}",
        f[1],
        -p
    );
    assert!(
        (f[7] - p).abs() < 1e-6 * p,
        "bending end force at j (fy): got {:.6e}, expected {}",
        f[7],
        p
    );
    let expected_mz_i = -p * L134;
    assert!(
        (f[5] - expected_mz_i).abs() < 1e-6 * p * L134,
        "bending end moment at i (mz): got {:.6e}, expected {expected_mz_i:.6e}",
        f[5]
    );
    assert!(
        f[11].abs() < 1e-6 * p * L134,
        "bending end moment at j (mz) should be zero, got {:.6e}",
        f[11]
    );
}

#[test]
fn phase134_pin_node() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    let n2 = model.add_node(2.0 * L134, 0.0, 0.0).unwrap();

    model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model
        .add_member(n1, n2, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();
    model.pin_node(n2).unwrap();

    let p = 1e3_f64;
    model
        .add_nodal_load(n1, 0.0, p, 0.0, 0.0, 0.0, 0.0)
        .unwrap();

    let result = model.solve().unwrap();

    let uy = result.displacement(n1, Dof3D::Uy).unwrap();
    assert!(
        uy > 0.0,
        "mid-span displacement should be positive, got {uy:.6e}"
    );

    let uy2 = result.displacement(n2, Dof3D::Uy).unwrap();
    assert!(
        uy2.abs() < 1e-10,
        "pinned node y-disp should be zero, got {uy2:.6e}"
    );
}

#[test]
fn phase134_solver_selection() {
    use section_properties::fea::solver::SolverSelection;

    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();
    model
        .add_nodal_load(n1, 1e3, 0.0, 0.0, 0.0, 0.0, 0.0)
        .unwrap();

    let solver = structural_analysis::FrameSolver3D::from_model(&model).unwrap();
    let result_auto = solver.solve().unwrap();

    let mut solver2 = structural_analysis::FrameSolver3D::from_model(&model).unwrap();
    solver2.set_solver(SolverSelection::dense());
    let result_dense = solver2.solve().unwrap();

    let ux_auto = result_auto.displacement(n1, Dof3D::Ux).unwrap();
    let ux_dense = result_dense.displacement(n1, Dof3D::Ux).unwrap();
    assert!(
        (ux_auto - ux_dense).abs() < 1e-10 * (1.0 + ux_auto.abs()),
        "solver selection mismatch: auto={ux_auto:.6e}, dense={ux_dense:.6e}"
    );
}

#[test]
fn phase134_model_validation_errors() {
    let empty = FrameModel3D::new();
    assert!(empty.solve().is_err(), "empty model should fail");

    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let _n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    assert!(
        model
            .add_member(n0, 99, phase134_section(), [0.0, 1.0, 0.0])
            .is_err(),
        "out-of-bounds node_j should fail"
    );

    assert!(
        model
            .add_nodal_load(99, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            .is_err(),
        "out-of-bounds nodal load should fail"
    );

    assert!(
        model.fix_dof(99, Dof3D::Ux, 0.0).is_err(),
        "out-of-bounds fix_dof should fail"
    );
}

#[test]
fn phase134_result_access_errors() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();
    model
        .add_nodal_load(n1, 1e3, 0.0, 0.0, 0.0, 0.0, 0.0)
        .unwrap();

    let result = model.solve().unwrap();

    assert!(result.displacement(99, Dof3D::Ux).is_err());
    assert!(result.reaction(99, Dof3D::Ux).is_err());
    assert!(result.member_end_forces(99).is_err());
}
