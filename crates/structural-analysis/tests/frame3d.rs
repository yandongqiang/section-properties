//! Phase 133: 3D frame element foundation tests.
//!
//! Covers section validation, geometry/orientation, local stiffness matrix
//! properties, and coordinate transformation — with independent analytical
//! reference calculations.

#![allow(non_snake_case)]
#![allow(clippy::needless_range_loop)]

use structural_analysis::{
    Dof3D, FemError, FrameElement3D, FrameMemberLoad, FrameModel3D, FrameSection3D,
};

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
        assert!((s.E() - 200e9).abs() < 1e-3);
        assert!((s.G() - 80e9).abs() < 1e-3);
        assert!((s.area() - 1e-3).abs() < 1e-15);
        assert!((s.iy() - 1e-6).abs() < 1e-18);
        assert!((s.iz() - 2e-6).abs() < 1e-18);
        assert!((s.j() - 3e-6).abs() < 1e-18);
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
        let EA_L = s.E() * s.area() / L;

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
        let GJ_L = s.G() * s.j() / L;

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
        let EIz_L3 = s.E() * s.iz() / L.powi(3);
        let EIz_L2 = s.E() * s.iz() / L.powi(2);
        let EIz_L = s.E() * s.iz() / L;

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
        let EIy_L3 = s.E() * s.iy() / L.powi(3);
        let EIy_L2 = s.E() * s.iy() / L.powi(2);
        let EIy_L = s.E() * s.iy() / L;

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
            FrameSection3D::new(s.E() * 10.0, s.G(), s.area(), s.iy(), s.iz(), s.j()).unwrap(),
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
            FrameSection3D::new(s.E(), s.G() * 5.0, s.area(), s.iy(), s.iz(), s.j()).unwrap(),
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
        let EA_L1 = s.E() * s.area() / 1.0;
        let EA_L2 = s.E() * s.area() / 2.0;
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
            FrameSection3D::new(
                s.E(),
                s.G(),
                s.area() * 3.0,
                s.iy() * 5.0,
                s.iz() * 7.0,
                s.j() * 2.0,
            )
            .unwrap(),
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

// ===========================================================================
// Phase 135: Member loads and equivalent nodal loads tests
// ===========================================================================

const Q135: f64 = 1e3;

// ---------------------------------------------------------------------------
// A. Equivalent nodal load vectors (local)
// ---------------------------------------------------------------------------

#[test]
fn phase135_equiv_load_uniform_axial() {
    let L = L134;
    let q = Q135;
    let load = FrameMemberLoad::UniformAxial {
        member_idx: 0,
        q_x: q,
    };
    let f = load.equivalent_nodal_loads_local(L);
    let half = q * L / 2.0;
    assert!(
        (f[0] - half).abs() < 1e-12,
        "f[0]: got {}, expected {}",
        f[0],
        half
    );
    assert!(
        (f[6] - half).abs() < 1e-12,
        "f[6]: got {}, expected {}",
        f[6],
        half
    );
    for i in [1, 2, 3, 4, 5, 7, 8, 9, 10, 11] {
        assert!(f[i].abs() < 1e-15, "f[{i}] should be zero, got {}", f[i]);
    }
}

#[test]
fn phase135_equiv_load_uniform_y() {
    let L = L134;
    let q = Q135;
    let load = FrameMemberLoad::UniformY {
        member_idx: 0,
        q_y: q,
    };
    let f = load.equivalent_nodal_loads_local(L);
    let half = q * L / 2.0;
    let mom = q * L * L / 12.0;
    assert!(
        (f[1] - half).abs() < 1e-12,
        "f[1]: got {}, expected {}",
        f[1],
        half
    );
    assert!(
        (f[5] - mom).abs() < 1e-12,
        "f[5]: got {}, expected {}",
        f[5],
        mom
    );
    assert!(
        (f[7] - half).abs() < 1e-12,
        "f[7]: got {}, expected {}",
        f[7],
        half
    );
    assert!(
        (f[11] + mom).abs() < 1e-12,
        "f[11]: got {}, expected {}",
        f[11],
        -mom
    );
    for i in [0, 2, 3, 4, 6, 8, 9, 10] {
        assert!(f[i].abs() < 1e-15, "f[{i}] should be zero, got {}", f[i]);
    }
}

#[test]
fn phase135_equiv_load_uniform_z() {
    let L = L134;
    let q = Q135;
    let load = FrameMemberLoad::UniformZ {
        member_idx: 0,
        q_z: q,
    };
    let f = load.equivalent_nodal_loads_local(L);
    let half = q * L / 2.0;
    let mom = q * L * L / 12.0;
    assert!(
        (f[2] - half).abs() < 1e-12,
        "f[2]: got {}, expected {}",
        f[2],
        half
    );
    assert!(
        (f[4] + mom).abs() < 1e-12,
        "f[4]: got {}, expected {}",
        f[4],
        -mom
    );
    assert!(
        (f[8] - half).abs() < 1e-12,
        "f[8]: got {}, expected {}",
        f[8],
        half
    );
    assert!(
        (f[10] - mom).abs() < 1e-12,
        "f[10]: got {}, expected {}",
        f[10],
        mom
    );
    for i in [0, 1, 3, 5, 6, 7, 9, 11] {
        assert!(f[i].abs() < 1e-15, "f[{i}] should be zero, got {}", f[i]);
    }
}

#[test]
fn phase135_equiv_load_linear_y() {
    let L = L134;
    let qi = Q135;
    let qj = 0.5 * Q135;
    let load = FrameMemberLoad::LinearY {
        member_idx: 0,
        q_i: qi,
        q_j: qj,
    };
    let f = load.equivalent_nodal_loads_local(L);
    let fi = L * (7.0 * qi + 3.0 * qj) / 20.0;
    let mi = L * L * (3.0 * qi + 2.0 * qj) / 60.0;
    let fj = L * (3.0 * qi + 7.0 * qj) / 20.0;
    let mj = L * L * (2.0 * qi + 3.0 * qj) / 60.0;
    assert!(
        (f[1] - fi).abs() < 1e-12,
        "f[1]: got {}, expected {}",
        f[1],
        fi
    );
    assert!(
        (f[5] - mi).abs() < 1e-12,
        "f[5]: got {}, expected {}",
        f[5],
        mi
    );
    assert!(
        (f[7] - fj).abs() < 1e-12,
        "f[7]: got {}, expected {}",
        f[7],
        fj
    );
    assert!(
        (f[11] + mj).abs() < 1e-12,
        "f[11]: got {}, expected {}",
        f[11],
        -mj
    );
    for i in [0, 2, 3, 4, 6, 8, 9, 10] {
        assert!(f[i].abs() < 1e-15, "f[{i}] should be zero, got {}", f[i]);
    }
}

#[test]
fn phase135_equiv_load_linear_z() {
    let L = L134;
    let qi = Q135;
    let qj = 0.5 * Q135;
    let load = FrameMemberLoad::LinearZ {
        member_idx: 0,
        q_i: qi,
        q_j: qj,
    };
    let f = load.equivalent_nodal_loads_local(L);
    let fi = L * (7.0 * qi + 3.0 * qj) / 20.0;
    let mi = L * L * (3.0 * qi + 2.0 * qj) / 60.0;
    let fj = L * (3.0 * qi + 7.0 * qj) / 20.0;
    let mj = L * L * (2.0 * qi + 3.0 * qj) / 60.0;
    assert!(
        (f[2] - fi).abs() < 1e-12,
        "f[2]: got {}, expected {}",
        f[2],
        fi
    );
    assert!(
        (f[4] + mi).abs() < 1e-12,
        "f[4]: got {}, expected {}",
        f[4],
        -mi
    );
    assert!(
        (f[8] - fj).abs() < 1e-12,
        "f[8]: got {}, expected {}",
        f[8],
        fj
    );
    assert!(
        (f[10] - mj).abs() < 1e-12,
        "f[10]: got {}, expected {}",
        f[10],
        mj
    );
    for i in [0, 1, 3, 5, 6, 7, 9, 11] {
        assert!(f[i].abs() < 1e-15, "f[{i}] should be zero, got {}", f[i]);
    }
}

#[test]
fn phase135_equiv_load_member_idx_and_zero() {
    let load = FrameMemberLoad::UniformAxial {
        member_idx: 3,
        q_x: 0.0,
    };
    assert_eq!(load.member_idx(), 3);
    let f = load.equivalent_nodal_loads_local(L134);
    for i in 0..12 {
        assert!(
            f[i].abs() < 1e-15,
            "zero load f[{i}] should be zero, got {}",
            f[i]
        );
    }
}

// ---------------------------------------------------------------------------
// B. Cantilever benchmarks (exact for single Euler-Bernoulli element)
// ---------------------------------------------------------------------------

#[test]
fn phase135_cantilever_uniform_axial() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    let m0 = model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();
    model.add_uniform_axial(m0, Q135).unwrap();

    let result = model.solve().unwrap();

    let delta = Q135 * L134 * L134 / (2.0 * E134 * A134);
    let ux = result.displacement(n1, Dof3D::Ux).unwrap();
    assert!(
        (ux - delta).abs() < 1e-10 * (1.0 + delta.abs()),
        "axial tip disp: got {ux:.6e}, expected {delta:.6e}"
    );

    let rx = result.reaction(n0, Dof3D::Ux).unwrap();
    let expected_rx = -Q135 * L134;
    assert!(
        (rx - expected_rx).abs() < 1e-6 * Q135,
        "axial reaction: got {rx:.6e}, expected {expected_rx:.6e}"
    );
}

#[test]
fn phase135_cantilever_uniform_y() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    let m0 = model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();
    model.add_uniform_y(m0, Q135).unwrap();

    let result = model.solve().unwrap();

    let delta = Q135 * L134.powi(4) / (8.0 * E134 * IZ134);
    let theta = Q135 * L134.powi(3) / (6.0 * E134 * IZ134);

    let uy = result.displacement(n1, Dof3D::Uy).unwrap();
    assert!(
        (uy - delta).abs() < 1e-10 * (1.0 + delta.abs()),
        "cantilever uy: got {uy:.6e}, expected {delta:.6e}"
    );

    let rz = result.displacement(n1, Dof3D::Rz).unwrap();
    assert!(
        (rz - theta).abs() < 1e-10 * (1.0 + theta.abs()),
        "cantilever rz: got {rz:.6e}, expected {theta:.6e}"
    );

    let ry = result.reaction(n0, Dof3D::Uy).unwrap();
    assert!(
        (ry + Q135 * L134).abs() < 1e-6 * Q135,
        "reaction Ry: got {ry:.6e}, expected {}",
        -Q135 * L134
    );

    let mz = result.reaction(n0, Dof3D::Rz).unwrap();
    let expected_mz = -Q135 * L134 * L134 / 2.0;
    assert!(
        (mz - expected_mz).abs() < 1e-6 * Q135 * L134,
        "reaction Mz: got {mz:.6e}, expected {expected_mz:.6e}"
    );
}

#[test]
fn phase135_cantilever_uniform_z() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    let m0 = model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();
    model.add_uniform_z(m0, Q135).unwrap();

    let result = model.solve().unwrap();

    let delta = Q135 * L134.powi(4) / (8.0 * E134 * IY134);
    let theta = -Q135 * L134.powi(3) / (6.0 * E134 * IY134);

    let uz = result.displacement(n1, Dof3D::Uz).unwrap();
    assert!(
        (uz - delta).abs() < 1e-10 * (1.0 + delta.abs()),
        "cantilever uz: got {uz:.6e}, expected {delta:.6e}"
    );

    let ry = result.displacement(n1, Dof3D::Ry).unwrap();
    assert!(
        (ry - theta).abs() < 1e-10 * (1.0 + theta.abs()),
        "cantilever ry: got {ry:.6e}, expected {theta:.6e}"
    );

    let rz = result.reaction(n0, Dof3D::Uz).unwrap();
    assert!(
        (rz + Q135 * L134).abs() < 1e-6 * Q135,
        "reaction Rz: got {rz:.6e}, expected {}",
        -Q135 * L134
    );

    let my = result.reaction(n0, Dof3D::Ry).unwrap();
    let expected_my = Q135 * L134 * L134 / 2.0;
    assert!(
        (my - expected_my).abs() < 1e-6 * Q135 * L134,
        "reaction My: got {my:.6e}, expected {expected_my:.6e}"
    );
}

#[test]
fn phase135_cantilever_linear_y() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    let m0 = model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();
    model.add_linear_y(m0, Q135, 0.0).unwrap();

    let result = model.solve().unwrap();

    let delta = Q135 * L134.powi(4) / (30.0 * E134 * IZ134);
    let theta = Q135 * L134.powi(3) / (24.0 * E134 * IZ134);

    let uy = result.displacement(n1, Dof3D::Uy).unwrap();
    assert!(
        (uy - delta).abs() < 1e-10 * (1.0 + delta.abs()),
        "linear cantilever uy: got {uy:.6e}, expected {delta:.6e}"
    );

    let rz = result.displacement(n1, Dof3D::Rz).unwrap();
    assert!(
        (rz - theta).abs() < 1e-10 * (1.0 + theta.abs()),
        "linear cantilever rz: got {rz:.6e}, expected {theta:.6e}"
    );

    let ry = result.reaction(n0, Dof3D::Uy).unwrap();
    assert!(
        (ry + Q135 * L134 / 2.0).abs() < 1e-6 * Q135,
        "reaction Ry: got {ry:.6e}, expected {}",
        -Q135 * L134 / 2.0
    );

    let mz = result.reaction(n0, Dof3D::Rz).unwrap();
    let expected_mz = -Q135 * L134 * L134 / 6.0;
    assert!(
        (mz - expected_mz).abs() < 1e-6 * Q135 * L134,
        "reaction Mz: got {mz:.6e}, expected {expected_mz:.6e}"
    );
}

#[test]
fn phase135_cantilever_linear_z() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    let m0 = model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();
    model.add_linear_z(m0, Q135, 0.0).unwrap();

    let result = model.solve().unwrap();

    let delta = Q135 * L134.powi(4) / (30.0 * E134 * IY134);
    let theta = -Q135 * L134.powi(3) / (24.0 * E134 * IY134);

    let uz = result.displacement(n1, Dof3D::Uz).unwrap();
    assert!(
        (uz - delta).abs() < 1e-10 * (1.0 + delta.abs()),
        "linear cantilever uz: got {uz:.6e}, expected {delta:.6e}"
    );

    let ry = result.displacement(n1, Dof3D::Ry).unwrap();
    assert!(
        (ry - theta).abs() < 1e-10 * (1.0 + theta.abs()),
        "linear cantilever ry: got {ry:.6e}, expected {theta:.6e}"
    );

    let rz = result.reaction(n0, Dof3D::Uz).unwrap();
    assert!(
        (rz + Q135 * L134 / 2.0).abs() < 1e-6 * Q135,
        "reaction Rz: got {rz:.6e}, expected {}",
        -Q135 * L134 / 2.0
    );

    let my = result.reaction(n0, Dof3D::Ry).unwrap();
    let expected_my = Q135 * L134 * L134 / 6.0;
    assert!(
        (my - expected_my).abs() < 1e-6 * Q135 * L134,
        "reaction My: got {my:.6e}, expected {expected_my:.6e}"
    );
}

// ---------------------------------------------------------------------------
// C. Fixed-fixed beam
// ---------------------------------------------------------------------------

#[test]
fn phase135_fixed_fixed_uniform_y() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    let m0 = model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();
    model.fix_node(n1).unwrap();
    model.add_uniform_y(m0, Q135).unwrap();

    let result = model.solve().unwrap();

    for dof in Dof3D::ALL {
        let d0 = result.displacement(n0, dof).unwrap();
        let d1 = result.displacement(n1, dof).unwrap();
        assert!(d0.abs() < 1e-12, "disp at n0 {:?}: got {d0:.6e}", dof);
        assert!(d1.abs() < 1e-12, "disp at n1 {:?}: got {d1:.6e}", dof);
    }

    let ry0 = result.reaction(n0, Dof3D::Uy).unwrap();
    let ry1 = result.reaction(n1, Dof3D::Uy).unwrap();
    let expected_ry = -Q135 * L134 / 2.0;
    assert!(
        (ry0 - expected_ry).abs() < 1e-6 * Q135,
        "Ry0: got {ry0:.6e}, expected {expected_ry:.6e}"
    );
    assert!(
        (ry1 - expected_ry).abs() < 1e-6 * Q135,
        "Ry1: got {ry1:.6e}, expected {expected_ry:.6e}"
    );

    let mz0 = result.reaction(n0, Dof3D::Rz).unwrap();
    let mz1 = result.reaction(n1, Dof3D::Rz).unwrap();
    let expected_mz0 = -Q135 * L134 * L134 / 12.0;
    let expected_mz1 = Q135 * L134 * L134 / 12.0;
    assert!(
        (mz0 - expected_mz0).abs() < 1e-6 * Q135 * L134,
        "Mz0: got {mz0:.6e}, expected {expected_mz0:.6e}"
    );
    assert!(
        (mz1 - expected_mz1).abs() < 1e-6 * Q135 * L134,
        "Mz1: got {mz1:.6e}, expected {expected_mz1:.6e}"
    );

    let f = result.member_end_forces(m0).unwrap();
    assert!(
        (f[1] - expected_ry).abs() < 1e-6 * Q135,
        "f[1]: got {}, expected {expected_ry}",
        f[1]
    );
    assert!(
        (f[5] - expected_mz0).abs() < 1e-6 * Q135 * L134,
        "f[5]: got {}, expected {expected_mz0}",
        f[5]
    );
    assert!(
        (f[7] - expected_ry).abs() < 1e-6 * Q135,
        "f[7]: got {}, expected {expected_ry}",
        f[7]
    );
    assert!(
        (f[11] - expected_mz1).abs() < 1e-6 * Q135 * L134,
        "f[11]: got {}, expected {expected_mz1}",
        f[11]
    );
}

#[test]
fn phase135_fixed_fixed_uniform_z() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    let m0 = model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();
    model.fix_node(n1).unwrap();
    model.add_uniform_z(m0, Q135).unwrap();

    let result = model.solve().unwrap();

    let rz0 = result.reaction(n0, Dof3D::Uz).unwrap();
    let rz1 = result.reaction(n1, Dof3D::Uz).unwrap();
    let expected_rz = -Q135 * L134 / 2.0;
    assert!(
        (rz0 - expected_rz).abs() < 1e-6 * Q135,
        "Rz0: got {rz0:.6e}, expected {expected_rz:.6e}"
    );
    assert!(
        (rz1 - expected_rz).abs() < 1e-6 * Q135,
        "Rz1: got {rz1:.6e}, expected {expected_rz:.6e}"
    );

    let my0 = result.reaction(n0, Dof3D::Ry).unwrap();
    let my1 = result.reaction(n1, Dof3D::Ry).unwrap();
    let expected_my0 = Q135 * L134 * L134 / 12.0;
    let expected_my1 = -Q135 * L134 * L134 / 12.0;
    assert!(
        (my0 - expected_my0).abs() < 1e-6 * Q135 * L134,
        "My0: got {my0:.6e}, expected {expected_my0:.6e}"
    );
    assert!(
        (my1 - expected_my1).abs() < 1e-6 * Q135 * L134,
        "My1: got {my1:.6e}, expected {expected_my1:.6e}"
    );
}

// ---------------------------------------------------------------------------
// D. Simply supported beam (rotations exact for single element)
// ---------------------------------------------------------------------------

#[test]
fn phase135_simply_supported_uniform_y() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    let m0 = model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.pin_node(n0).unwrap();
    model.pin_node(n1).unwrap();
    model.fix_dof(n0, Dof3D::Rx, 0.0).unwrap();
    model.fix_dof(n1, Dof3D::Rx, 0.0).unwrap();
    model.add_uniform_y(m0, Q135).unwrap();

    let result = model.solve().unwrap();

    let theta_i = Q135 * L134.powi(3) / (24.0 * E134 * IZ134);
    let theta_j = -theta_i;

    let rz0 = result.displacement(n0, Dof3D::Rz).unwrap();
    let rz1 = result.displacement(n1, Dof3D::Rz).unwrap();
    assert!(
        (rz0 - theta_i).abs() < 1e-10 * (1.0 + theta_i.abs()),
        "rotation at i: got {rz0:.6e}, expected {theta_i:.6e}"
    );
    assert!(
        (rz1 - theta_j).abs() < 1e-10 * (1.0 + theta_j.abs()),
        "rotation at j: got {rz1:.6e}, expected {theta_j:.6e}"
    );

    let ry0 = result.reaction(n0, Dof3D::Uy).unwrap();
    let ry1 = result.reaction(n1, Dof3D::Uy).unwrap();
    let expected_ry = -Q135 * L134 / 2.0;
    assert!(
        (ry0 - expected_ry).abs() < 1e-6 * Q135,
        "Ry0: got {ry0:.6e}, expected {expected_ry:.6e}"
    );
    assert!(
        (ry1 - expected_ry).abs() < 1e-6 * Q135,
        "Ry1: got {ry1:.6e}, expected {expected_ry:.6e}"
    );
}

// ---------------------------------------------------------------------------
// E. Oblique member — local-to-global transformation
// ---------------------------------------------------------------------------

#[test]
fn phase135_oblique_cantilever_uniform_y() {
    let s2 = 2.0_f64.sqrt();
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134 / s2, L134 / s2, 0.0).unwrap();
    let m0 = model
        .add_member(n0, n1, phase134_section(), [0.0, 0.0, 1.0])
        .unwrap();
    model.fix_node(n0).unwrap();
    model.add_uniform_y(m0, Q135).unwrap();

    let result = model.solve().unwrap();

    let delta_z = Q135 * L134.powi(4) / (8.0 * E134 * IZ134);
    let uz = result.displacement(n1, Dof3D::Uz).unwrap();
    assert!(
        (uz - delta_z).abs() < 1e-10 * (1.0 + delta_z.abs()),
        "oblique tip uz: got {uz:.6e}, expected {delta_z:.6e}"
    );

    let rx = result.reaction(n0, Dof3D::Ux).unwrap();
    let ry = result.reaction(n0, Dof3D::Uy).unwrap();
    let rz = result.reaction(n0, Dof3D::Uz).unwrap();
    assert!(rx.abs() < 1e-6 * Q135, "Rx should be zero, got {rx:.6e}");
    assert!(ry.abs() < 1e-6 * Q135, "Ry should be zero, got {ry:.6e}");
    assert!(
        (rz + Q135 * L134).abs() < 1e-6 * Q135,
        "Rz: got {rz:.6e}, expected {}",
        -Q135 * L134
    );

    let mx = result.reaction(n0, Dof3D::Rx).unwrap();
    let my = result.reaction(n0, Dof3D::Ry).unwrap();
    let mz = result.reaction(n0, Dof3D::Rz).unwrap();
    let expected_mx = -Q135 * L134 * L134 / (2.0 * s2);
    let expected_my = Q135 * L134 * L134 / (2.0 * s2);
    assert!(
        (mx - expected_mx).abs() < 1e-6 * Q135 * L134,
        "Mx: got {mx:.6e}, expected {expected_mx:.6e}"
    );
    assert!(
        (my - expected_my).abs() < 1e-6 * Q135 * L134,
        "My: got {my:.6e}, expected {expected_my:.6e}"
    );
    assert!(
        mz.abs() < 1e-6 * Q135 * L134,
        "Mz should be zero, got {mz:.6e}"
    );
}

// ---------------------------------------------------------------------------
// F. End-force recovery
// ---------------------------------------------------------------------------

#[test]
fn phase135_end_forces_cantilever_y() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    let m0 = model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();
    model.add_uniform_y(m0, Q135).unwrap();

    let result = model.solve().unwrap();
    let f = result.member_end_forces(m0).unwrap();

    assert!(
        (f[1] + Q135 * L134).abs() < 1e-6 * Q135,
        "f[1] (shear at i): got {}, expected {}",
        f[1],
        -Q135 * L134
    );
    assert!(
        (f[5] + Q135 * L134 * L134 / 2.0).abs() < 1e-6 * Q135 * L134,
        "f[5] (moment at i): got {}, expected {}",
        f[5],
        -Q135 * L134 * L134 / 2.0
    );
    assert!(
        f[7].abs() < 1e-6 * Q135,
        "f[7] (shear at j): got {}, expected 0",
        f[7]
    );
    assert!(
        f[11].abs() < 1e-6 * Q135 * L134,
        "f[11] (moment at j): got {}, expected 0",
        f[11]
    );

    let ry = result.reaction(n0, Dof3D::Uy).unwrap();
    let mz = result.reaction(n0, Dof3D::Rz).unwrap();
    assert!((f[1] - ry).abs() < 1e-10, "f[1] should match Ry reaction");
    assert!((f[5] - mz).abs() < 1e-10, "f[5] should match Mz reaction");
}

#[test]
fn phase135_end_forces_cantilever_z() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    let m0 = model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();
    model.add_uniform_z(m0, Q135).unwrap();

    let result = model.solve().unwrap();
    let f = result.member_end_forces(m0).unwrap();

    assert!(
        (f[2] + Q135 * L134).abs() < 1e-6 * Q135,
        "f[2] (shear at i): got {}, expected {}",
        f[2],
        -Q135 * L134
    );
    assert!(
        (f[4] - Q135 * L134 * L134 / 2.0).abs() < 1e-6 * Q135 * L134,
        "f[4] (moment at i): got {}, expected {}",
        f[4],
        Q135 * L134 * L134 / 2.0
    );
    assert!(
        f[8].abs() < 1e-6 * Q135,
        "f[8] (shear at j): got {}, expected 0",
        f[8]
    );
    assert!(
        f[10].abs() < 1e-6 * Q135 * L134,
        "f[10] (moment at j): got {}, expected 0",
        f[10]
    );

    let rz = result.reaction(n0, Dof3D::Uz).unwrap();
    let my = result.reaction(n0, Dof3D::Ry).unwrap();
    assert!((f[2] - rz).abs() < 1e-10, "f[2] should match Rz reaction");
    assert!((f[4] - my).abs() < 1e-10, "f[4] should match My reaction");
}

// ---------------------------------------------------------------------------
// G. Edge cases and regression
// ---------------------------------------------------------------------------

#[test]
fn phase135_zero_member_load_regression() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    let m0 = model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();
    let p = 1e3_f64;
    model
        .add_nodal_load(n1, 0.0, p, 0.0, 0.0, 0.0, 0.0)
        .unwrap();

    let result = model.solve().unwrap();

    let delta = p * L134.powi(3) / (3.0 * E134 * IZ134);
    let uy = result.displacement(n1, Dof3D::Uy).unwrap();
    assert!(
        (uy - delta).abs() < 1e-10 * (1.0 + delta.abs()),
        "regression uy: got {uy:.6e}, expected {delta:.6e}"
    );

    let f = result.member_end_forces(m0).unwrap();
    assert!(
        (f[1] + p).abs() < 1e-6 * p,
        "f[1]: got {}, expected {}",
        f[1],
        -p
    );
    assert!(
        (f[5] + p * L134).abs() < 1e-6 * p,
        "f[5]: got {}, expected {}",
        f[5],
        -p * L134
    );
    assert!(
        (f[7] - p).abs() < 1e-6 * p,
        "f[7]: got {}, expected {p}",
        f[7]
    );
    assert!(f[11].abs() < 1e-6 * p, "f[11]: got {}, expected 0", f[11]);
}

#[test]
fn phase135_multiple_additive_loads() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    let m0 = model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();
    model.fix_node(n1).unwrap();
    model.add_uniform_y(m0, Q135).unwrap();
    model.add_uniform_y(m0, 2.0 * Q135).unwrap();

    let result = model.solve().unwrap();

    let q_total = 3.0 * Q135;
    let ry0 = result.reaction(n0, Dof3D::Uy).unwrap();
    let expected_ry = -q_total * L134 / 2.0;
    assert!(
        (ry0 - expected_ry).abs() < 1e-6 * q_total,
        "additive Ry0: got {ry0:.6e}, expected {expected_ry:.6e}"
    );

    let mz0 = result.reaction(n0, Dof3D::Rz).unwrap();
    let expected_mz = -q_total * L134 * L134 / 12.0;
    assert!(
        (mz0 - expected_mz).abs() < 1e-6 * q_total * L134,
        "additive Mz0: got {mz0:.6e}, expected {expected_mz:.6e}"
    );
}

#[test]
fn phase135_invalid_member_idx() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();

    assert!(model.add_uniform_axial(99, Q135).is_err());
    assert!(model.add_uniform_y(99, Q135).is_err());
    assert!(model.add_uniform_z(99, Q135).is_err());
    assert!(model.add_linear_y(99, Q135, 0.0).is_err());
    assert!(model.add_linear_z(99, Q135, 0.0).is_err());
}

#[test]
fn phase135_nan_intensity_rejected() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    let m0 = model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();

    assert!(model.add_uniform_axial(m0, f64::NAN).is_err());
    assert!(model.add_uniform_y(m0, f64::NAN).is_err());
    assert!(model.add_uniform_z(m0, f64::NAN).is_err());
    assert!(model.add_linear_y(m0, f64::NAN, 0.0).is_err());
    assert!(model.add_linear_y(m0, 0.0, f64::NAN).is_err());
    assert!(model.add_linear_z(m0, f64::INFINITY, 0.0).is_err());
}

#[test]
fn phase135_repeated_solve() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    let m0 = model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();
    model.add_uniform_y(m0, Q135).unwrap();

    let solver = structural_analysis::FrameSolver3D::from_model(&model).unwrap();
    let r1 = solver.solve().unwrap();
    let r2 = solver.solve().unwrap();

    let uy1 = r1.displacement(n1, Dof3D::Uy).unwrap();
    let uy2 = r2.displacement(n1, Dof3D::Uy).unwrap();
    assert!(
        (uy1 - uy2).abs() < 1e-15,
        "repeated solve mismatch: {uy1:.6e} vs {uy2:.6e}"
    );

    let f1 = r1.member_end_forces(m0).unwrap();
    let f2 = r2.member_end_forces(m0).unwrap();
    for i in 0..12 {
        assert!((f1[i] - f2[i]).abs() < 1e-15, "end force {i} mismatch");
    }
}

#[test]
fn phase135_mixed_nodal_and_member_load() {
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L134, 0.0, 0.0).unwrap();
    let m0 = model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();

    let p = 500.0_f64;
    model
        .add_nodal_load(n1, 0.0, p, 0.0, 0.0, 0.0, 0.0)
        .unwrap();
    model.add_uniform_y(m0, Q135).unwrap();

    let result = model.solve().unwrap();

    let delta_p = p * L134.powi(3) / (3.0 * E134 * IZ134);
    let delta_q = Q135 * L134.powi(4) / (8.0 * E134 * IZ134);
    let delta = delta_p + delta_q;

    let uy = result.displacement(n1, Dof3D::Uy).unwrap();
    assert!(
        (uy - delta).abs() < 1e-10 * (1.0 + delta.abs()),
        "mixed uy: got {uy:.6e}, expected {delta:.6e}"
    );

    let ry = result.reaction(n0, Dof3D::Uy).unwrap();
    let expected_ry = -(p + Q135 * L134);
    assert!(
        (ry - expected_ry).abs() < 1e-6 * (p + Q135),
        "mixed Ry: got {ry:.6e}, expected {expected_ry:.6e}"
    );
}

#[test]
fn phase135_both_planes_and_scale() {
    let L = 2.0_f64;
    let mut model = FrameModel3D::new();
    let n0 = model.add_node(0.0, 0.0, 0.0).unwrap();
    let n1 = model.add_node(L, 0.0, 0.0).unwrap();
    let m0 = model
        .add_member(n0, n1, phase134_section(), [0.0, 1.0, 0.0])
        .unwrap();
    model.fix_node(n0).unwrap();
    model.add_uniform_y(m0, Q135).unwrap();
    model.add_uniform_z(m0, Q135).unwrap();

    let result = model.solve().unwrap();

    let delta_y = Q135 * L.powi(4) / (8.0 * E134 * IZ134);
    let theta_z = Q135 * L.powi(3) / (6.0 * E134 * IZ134);

    let uy = result.displacement(n1, Dof3D::Uy).unwrap();
    assert!(
        (uy - delta_y).abs() < 1e-10 * (1.0 + delta_y.abs()),
        "both planes uy: got {uy:.6e}, expected {delta_y:.6e}"
    );

    let rz = result.displacement(n1, Dof3D::Rz).unwrap();
    assert!(
        (rz - theta_z).abs() < 1e-10 * (1.0 + theta_z.abs()),
        "both planes rz: got {rz:.6e}, expected {theta_z:.6e}"
    );

    let delta_z = Q135 * L.powi(4) / (8.0 * E134 * IY134);
    let theta_y = -Q135 * L.powi(3) / (6.0 * E134 * IY134);

    let uz = result.displacement(n1, Dof3D::Uz).unwrap();
    assert!(
        (uz - delta_z).abs() < 1e-10 * (1.0 + delta_z.abs()),
        "both planes uz: got {uz:.6e}, expected {delta_z:.6e}"
    );

    let ry = result.displacement(n1, Dof3D::Ry).unwrap();
    assert!(
        (ry - theta_y).abs() < 1e-10 * (1.0 + theta_y.abs()),
        "both planes ry: got {ry:.6e}, expected {theta_y:.6e}"
    );

    let r_y = result.reaction(n0, Dof3D::Uy).unwrap();
    let r_z = result.reaction(n0, Dof3D::Uz).unwrap();
    let m_y = result.reaction(n0, Dof3D::Ry).unwrap();
    let m_z = result.reaction(n0, Dof3D::Rz).unwrap();
    assert!((r_y + Q135 * L).abs() < 1e-6 * Q135, "Ry: got {r_y:.6e}");
    assert!((r_z + Q135 * L).abs() < 1e-6 * Q135, "Rz: got {r_z:.6e}");
    assert!(
        (m_y - Q135 * L * L / 2.0).abs() < 1e-6 * Q135 * L,
        "My: got {m_y:.6e}"
    );
    assert!(
        (m_z + Q135 * L * L / 2.0).abs() < 1e-6 * Q135 * L,
        "Mz: got {m_z:.6e}"
    );
}

// ---------------------------------------------------------------------------
// Phase 136: API invariants and numerical robustness
// ---------------------------------------------------------------------------

mod phase136 {
    use super::*;

    // --- local_stiffness: NaN / infinity coordinate rejection ---

    #[test]
    fn local_stiffness_rejects_nan_pi() {
        let e = FrameElement3D::new(0, 1, steel_section(), default_ref()).unwrap();
        let err = e.local_stiffness((f64::NAN, 0.0, 0.0), (1.0, 0.0, 0.0));
        assert!(matches!(err, Err(FemError::InvalidInput(_))));
    }

    #[test]
    fn local_stiffness_rejects_nan_pj() {
        let e = FrameElement3D::new(0, 1, steel_section(), default_ref()).unwrap();
        let err = e.local_stiffness((0.0, 0.0, 0.0), (1.0, f64::NAN, 0.0));
        assert!(matches!(err, Err(FemError::InvalidInput(_))));
    }

    #[test]
    fn local_stiffness_rejects_inf_pi() {
        let e = FrameElement3D::new(0, 1, steel_section(), default_ref()).unwrap();
        let err = e.local_stiffness((f64::INFINITY, 0.0, 0.0), (1.0, 0.0, 0.0));
        assert!(matches!(err, Err(FemError::InvalidInput(_))));
    }

    #[test]
    fn local_stiffness_rejects_inf_pj() {
        let e = FrameElement3D::new(0, 1, steel_section(), default_ref()).unwrap();
        let err = e.local_stiffness((0.0, 0.0, 0.0), (1.0, 0.0, f64::INFINITY));
        assert!(matches!(err, Err(FemError::InvalidInput(_))));
    }

    #[test]
    fn local_stiffness_rejects_neg_inf() {
        let e = FrameElement3D::new(0, 1, steel_section(), default_ref()).unwrap();
        let err = e.local_stiffness((f64::NEG_INFINITY, 0.0, 0.0), (1.0, 0.0, 0.0));
        assert!(matches!(err, Err(FemError::InvalidInput(_))));
    }

    // --- local_stiffness: zero / near-zero length ---

    #[test]
    fn local_stiffness_rejects_zero_length() {
        let e = FrameElement3D::new(0, 1, steel_section(), default_ref()).unwrap();
        let err = e.local_stiffness((1.0, 2.0, 3.0), (1.0, 2.0, 3.0));
        assert!(matches!(err, Err(FemError::ZeroLengthMember(_))));
    }

    #[test]
    fn local_stiffness_rejects_near_zero_length_overflow() {
        let e = FrameElement3D::new(0, 1, steel_section(), default_ref()).unwrap();
        let err = e.local_stiffness((0.0, 0.0, 0.0), (1e-200, 0.0, 0.0));
        assert!(
            err.is_err(),
            "near-zero length causing overflow must be rejected"
        );
    }

    // --- local_stiffness: extreme but finite coordinates ---

    #[test]
    fn local_stiffness_extreme_coordinates_finite() {
        let e = FrameElement3D::new(0, 1, steel_section(), default_ref()).unwrap();
        let k = e.local_stiffness((0.0, 0.0, 0.0), (1e150, 0.0, 0.0));
        assert!(k.is_ok(), "extreme finite coordinates should not fail");
        let k = k.unwrap();
        for i in 0..12 {
            for j in 0..12 {
                assert!(k[i][j].is_finite(), "entry [{i}][{j}] not finite");
            }
        }
    }

    #[test]
    fn local_stiffness_all_entries_finite() {
        let e = FrameElement3D::new(0, 1, steel_section(), default_ref()).unwrap();
        let k = e.local_stiffness((0.0, 0.0, 0.0), (3.0, 4.0, 0.0)).unwrap();
        for i in 0..12 {
            for j in 0..12 {
                assert!(k[i][j].is_finite(), "entry [{i}][{j}] not finite");
            }
        }
    }

    // --- local_axes: NaN / infinity rejection ---

    #[test]
    fn local_axes_rejects_nan() {
        let e = FrameElement3D::new(0, 1, steel_section(), default_ref()).unwrap();
        let err = e.local_axes((f64::NAN, 0.0, 0.0), (1.0, 0.0, 0.0));
        assert!(err.is_err());
    }

    #[test]
    fn local_axes_rejects_inf() {
        let e = FrameElement3D::new(0, 1, steel_section(), default_ref()).unwrap();
        let err = e.local_axes((0.0, 0.0, 0.0), (1.0, f64::INFINITY, 0.0));
        assert!(err.is_err());
    }

    // --- FrameSection3D setters: reject invalid values ---

    #[test]
    fn section_setter_rejects_nan() {
        let mut s = steel_section();
        assert!(s.set_E(f64::NAN).is_err());
        assert!(s.set_G(f64::NAN).is_err());
        assert!(s.set_area(f64::NAN).is_err());
        assert!(s.set_iy(f64::NAN).is_err());
        assert!(s.set_iz(f64::NAN).is_err());
        assert!(s.set_j(f64::NAN).is_err());
    }

    #[test]
    fn section_setter_rejects_zero() {
        let mut s = steel_section();
        assert!(s.set_E(0.0).is_err());
        assert!(s.set_G(0.0).is_err());
        assert!(s.set_area(0.0).is_err());
        assert!(s.set_iy(0.0).is_err());
        assert!(s.set_iz(0.0).is_err());
        assert!(s.set_j(0.0).is_err());
    }

    #[test]
    fn section_setter_rejects_negative() {
        let mut s = steel_section();
        assert!(s.set_E(-1.0).is_err());
        assert!(s.set_G(-1.0).is_err());
        assert!(s.set_area(-1.0).is_err());
        assert!(s.set_iy(-1.0).is_err());
        assert!(s.set_iz(-1.0).is_err());
        assert!(s.set_j(-1.0).is_err());
    }

    #[test]
    fn section_setter_rejects_inf() {
        let mut s = steel_section();
        assert!(s.set_E(f64::INFINITY).is_err());
        assert!(s.set_G(f64::INFINITY).is_err());
        assert!(s.set_area(f64::INFINITY).is_err());
        assert!(s.set_iy(f64::INFINITY).is_err());
        assert!(s.set_iz(f64::INFINITY).is_err());
        assert!(s.set_j(f64::INFINITY).is_err());
    }

    #[test]
    fn section_setter_updates_correctly() {
        let mut s = steel_section();
        s.set_E(210e9).unwrap();
        assert!((s.E() - 210e9).abs() < 1e-3);
        s.set_area(2e-3).unwrap();
        assert!((s.area() - 2e-3).abs() < 1e-15);
        s.set_j(5e-6).unwrap();
        assert!((s.j() - 5e-6).abs() < 1e-18);
    }

    #[test]
    fn section_setter_rejects_neg_inf() {
        let mut s = steel_section();
        assert!(s.set_E(f64::NEG_INFINITY).is_err());
    }

    // --- FrameModel3D::add_node: NaN / infinity rejection ---

    #[test]
    fn add_node_rejects_nan() {
        let mut m = FrameModel3D::new();
        assert!(m.add_node(f64::NAN, 0.0, 0.0).is_err());
        assert!(m.add_node(0.0, f64::NAN, 0.0).is_err());
        assert!(m.add_node(0.0, 0.0, f64::NAN).is_err());
    }

    #[test]
    fn add_node_rejects_inf() {
        let mut m = FrameModel3D::new();
        assert!(m.add_node(f64::INFINITY, 0.0, 0.0).is_err());
        assert!(m.add_node(0.0, f64::NEG_INFINITY, 0.0).is_err());
    }

    // --- FrameModel3D::add_nodal_load: invalid index / non-finite ---

    #[test]
    fn add_nodal_load_rejects_invalid_node() {
        let mut m = FrameModel3D::new();
        m.add_node(0.0, 0.0, 0.0).unwrap();
        let err = m.add_nodal_load(99, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert!(matches!(err, Err(FemError::InvalidNode(_))));
    }

    #[test]
    fn add_nodal_load_rejects_nan_force() {
        let mut m = FrameModel3D::new();
        m.add_node(0.0, 0.0, 0.0).unwrap();
        let err = m.add_nodal_load(0, f64::NAN, 0.0, 0.0, 0.0, 0.0, 0.0);
        assert!(matches!(err, Err(FemError::InvalidInput(_))));
        let err = m.add_nodal_load(0, 0.0, 0.0, 0.0, 0.0, 0.0, f64::INFINITY);
        assert!(matches!(err, Err(FemError::InvalidInput(_))));
    }

    // --- FrameModel3D member loads: invalid index / non-finite ---

    #[test]
    fn add_member_load_rejects_invalid_member() {
        let mut m = FrameModel3D::new();
        m.add_node(0.0, 0.0, 0.0).unwrap();
        m.add_node(1.0, 0.0, 0.0).unwrap();
        m.add_member(0, 1, steel_section(), default_ref()).unwrap();
        let err = m.add_uniform_y(99, 1.0);
        assert!(matches!(err, Err(FemError::InvalidMember(_))));
    }

    #[test]
    fn add_member_load_rejects_nan_intensity() {
        let mut m = FrameModel3D::new();
        m.add_node(0.0, 0.0, 0.0).unwrap();
        m.add_node(1.0, 0.0, 0.0).unwrap();
        m.add_member(0, 1, steel_section(), default_ref()).unwrap();
        assert!(m.add_uniform_y(0, f64::NAN).is_err());
        assert!(m.add_uniform_axial(0, f64::INFINITY).is_err());
        assert!(m.add_linear_y(0, f64::NAN, 1.0).is_err());
        assert!(m.add_linear_z(0, 1.0, f64::NEG_INFINITY).is_err());
    }

    // --- fix_dof: constraint semantics ---

    #[test]
    fn fix_dof_idempotent_same_value() {
        let mut m = FrameModel3D::new();
        m.add_node(0.0, 0.0, 0.0).unwrap();
        m.fix_dof(0, Dof3D::Ux, 0.0).unwrap();
        m.fix_dof(0, Dof3D::Ux, 0.0).unwrap();
        m.fix_dof(0, Dof3D::Uy, 1.5).unwrap();
        m.fix_dof(0, Dof3D::Uy, 1.5).unwrap();
    }

    #[test]
    fn fix_dof_conflicting_values() {
        let mut m = FrameModel3D::new();
        m.add_node(0.0, 0.0, 0.0).unwrap();
        m.fix_dof(0, Dof3D::Ux, 0.0).unwrap();
        let err = m.fix_dof(0, Dof3D::Ux, 1.0);
        assert!(matches!(err, Err(FemError::InvalidInput(_))));
    }

    #[test]
    fn fix_dof_rejects_nan_value() {
        let mut m = FrameModel3D::new();
        m.add_node(0.0, 0.0, 0.0).unwrap();
        let err = m.fix_dof(0, Dof3D::Ux, f64::NAN);
        assert!(matches!(err, Err(FemError::InvalidInput(_))));
    }

    #[test]
    fn fix_dof_rejects_inf_value() {
        let mut m = FrameModel3D::new();
        m.add_node(0.0, 0.0, 0.0).unwrap();
        let err = m.fix_dof(0, Dof3D::Ux, f64::INFINITY);
        assert!(matches!(err, Err(FemError::InvalidInput(_))));
        let err = m.fix_dof(0, Dof3D::Ux, f64::NEG_INFINITY);
        assert!(matches!(err, Err(FemError::InvalidInput(_))));
    }

    #[test]
    fn fix_dof_rejects_invalid_node() {
        let mut m = FrameModel3D::new();
        m.add_node(0.0, 0.0, 0.0).unwrap();
        let err = m.fix_dof(99, Dof3D::Ux, 0.0);
        assert!(matches!(err, Err(FemError::InvalidNode(_))));
    }

    #[test]
    fn fix_dof_different_dofs_same_node_ok() {
        let mut m = FrameModel3D::new();
        m.add_node(0.0, 0.0, 0.0).unwrap();
        m.fix_dof(0, Dof3D::Ux, 0.0).unwrap();
        m.fix_dof(0, Dof3D::Uy, 0.0).unwrap();
        m.fix_dof(0, Dof3D::Uz, 0.0).unwrap();
        m.fix_dof(0, Dof3D::Rx, 0.0).unwrap();
        m.fix_dof(0, Dof3D::Ry, 0.0).unwrap();
        m.fix_dof(0, Dof3D::Rz, 0.0).unwrap();
    }

    // --- Solver: empty model rejection ---

    #[test]
    fn solver_rejects_empty_model() {
        let m = FrameModel3D::new();
        assert!(m.solve().is_err());
    }

    // --- Solver: well-posed model produces finite solution ---

    #[test]
    fn solver_produces_finite_solution() {
        let mut m = FrameModel3D::new();
        let n0 = m.add_node(0.0, 0.0, 0.0).unwrap();
        let n1 = m.add_node(2.0, 0.0, 0.0).unwrap();
        m.add_member(n0, n1, steel_section(), default_ref())
            .unwrap();
        m.fix_node(n0).unwrap();
        m.fix_dof(n1, Dof3D::Uy, 0.0).unwrap();
        m.fix_dof(n1, Dof3D::Uz, 0.0).unwrap();
        m.fix_dof(n1, Dof3D::Rx, 0.0).unwrap();
        m.fix_dof(n1, Dof3D::Ry, 0.0).unwrap();
        m.fix_dof(n1, Dof3D::Rz, 0.0).unwrap();
        m.add_nodal_load(n1, 1e3, 0.0, 0.0, 0.0, 0.0, 0.0).unwrap();
        let result = m.solve().unwrap();
        for i in 0..2 {
            for &dof in Dof3D::ALL.iter() {
                let d = result.displacement(i, dof).unwrap();
                assert!(
                    d.is_finite(),
                    "displacement node {i} dof {:?} not finite",
                    dof
                );
            }
        }
    }

    // --- FrameSection3D getters: verify encapsulation ---

    #[test]
    fn section_getters_match_constructor() {
        let s = steel_section();
        assert!(s.E() > 0.0 && s.E().is_finite());
        assert!(s.G() > 0.0 && s.G().is_finite());
        assert!(s.area() > 0.0 && s.area().is_finite());
        assert!(s.iy() > 0.0 && s.iy().is_finite());
        assert!(s.iz() > 0.0 && s.iz().is_finite());
        assert!(s.j() > 0.0 && s.j().is_finite());
    }

    // --- FrameElement3D getters ---

    #[test]
    fn element_getters_match_constructor() {
        let s = steel_section();
        let e = FrameElement3D::new(2, 5, s, [0.0, 1.0, 0.0]).unwrap();
        assert_eq!(e.node_i(), 2);
        assert_eq!(e.node_j(), 5);
        assert_eq!(e.ref_vec(), [0.0, 1.0, 0.0]);
    }

    // --- FrameModel3D accessors ---

    #[test]
    fn model_accessors() {
        let mut m = FrameModel3D::new();
        assert_eq!(m.n_nodes(), 0);
        assert_eq!(m.n_members(), 0);
        let n0 = m.add_node(1.0, 2.0, 3.0).unwrap();
        let n1 = m.add_node(4.0, 5.0, 6.0).unwrap();
        assert_eq!(m.n_nodes(), 2);
        m.add_member(n0, n1, steel_section(), default_ref())
            .unwrap();
        assert_eq!(m.n_members(), 1);
        let node = m.node(n0).unwrap();
        assert!((node.x() - 1.0).abs() < 1e-15);
        assert!((node.y() - 2.0).abs() < 1e-15);
        assert!((node.z() - 3.0).abs() < 1e-15);
        assert_eq!(node.id(), 0);
    }

    // --- length() uses hypot for overflow resistance ---

    #[test]
    fn length_hypot_overflow_resistance() {
        let e = FrameElement3D::new(0, 1, steel_section(), default_ref()).unwrap();
        let big = 1e300;
        let l = e.length((0.0, 0.0, 0.0), (big, big, 0.0));
        assert!(l.is_finite(), "hypot should avoid overflow");
        assert!((l - big * 2.0_f64.sqrt()).abs() / l < 1e-10);
    }
}
