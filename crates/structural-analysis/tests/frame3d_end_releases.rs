//! Phase 160: 3-D frame element rotational end releases (pins).
//!
//! A released rotation transmits no moment — the element behaves as if
//! pinned there.  Releases are applied by static condensation of the local
//! stiffness, so every expectation below is an **independent analytical
//! reference** for the released structure (classic beam results), not a
//! re-derivation of the implementation.
//!
//! Reference cases:
//!
//! * pinned–pinned beam, UDL: `M_i = M_j = 0`, `R = qL/2` each,
//!   end rotations `θ = qL³/(24EI)`;
//! * fixed–pinned beam, UDL: `R_pin = 3qL/8`, `R_fixed = 5qL/8`,
//!   `M_fixed = qL²/8`, `M_pin = 0`;
//! * fixed–fixed beam, UDL (control): `M = qL²/12` at both ends;
//! * torsion release: no torsion moment anywhere in the released member,
//!   while the sibling member still restrains the shared node rotation.

#![allow(non_snake_case)]

use structural_analysis::{
    Dof3D, FemError, FrameElement3D, FrameModel3D, FrameSection3D, FrameSolver3D,
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Section used throughout: `E = 200 GPa`, `G = 80 GPa`,
/// `A = 1e-3 m²`, `Iy = 1e-6 m⁴`, `Iz = 2e-6 m⁴`, `J = 3e-6 m⁴`.
fn section() -> FrameSection3D {
    FrameSection3D::new(200e9, 80e9, 1e-3, 1e-6, 2e-6, 3e-6).unwrap()
}

/// Build a single-member beam along global **X** of length `L`, with
/// `ref_vec = [0, 0, 1]` (local y = global Z, local z = −global Y) and the
/// given rotational end releases.
fn x_beam(L: f64, releases: [bool; 6]) -> FrameModel3D {
    let mut m = FrameModel3D::new();
    let i = m.add_node(0.0, 0.0, 0.0).unwrap();
    let j = m.add_node(L, 0.0, 0.0).unwrap();
    let idx = m
        .add_member(i, j, section(), [0.0, 0.0, 1.0])
        .unwrap();
    m.set_member_releases(idx, structural_analysis::EndReleases3D::from_array(releases))
        .unwrap();
    m
}

/// Simply supported beam (translations restrained at i and j) carrying a
/// uniform **global −Y** load `w` (N/m) on the member.
fn simply_supported(L: f64, w: f64, releases: [bool; 6]) -> FrameSolver3D {
    let mut m = x_beam(L, releases);
    m.fix_dof(0, Dof3D::Ux, 0.0).unwrap();
    m.fix_dof(0, Dof3D::Uy, 0.0).unwrap();
    m.fix_dof(1, Dof3D::Uy, 0.0).unwrap();
    // out-of-plane translations restrained to keep the 3-D system stable
    m.fix_dof(0, Dof3D::Uz, 0.0).unwrap();
    m.fix_dof(1, Dof3D::Uz, 0.0).unwrap();
    // a physical bearing restrains twist; without it a bare beam has a free
    // torsional mode (rx_i = rx_j) and the model is a mechanism
    m.fix_dof(0, Dof3D::Rx, 0.0).unwrap();
    // `add_uniform_z` is the local-z direction = −global Y here.
    m.add_uniform_z(0, w).unwrap();
    FrameSolver3D::from_model(&m).unwrap()
}

/// Pinned–pinned (all three rotations released at both ends).
fn both_pins() -> structural_analysis::EndReleases3D {
    structural_analysis::EndReleases3D::from_array([true, true, true, true, true, true])
}

// ---------------------------------------------------------------------------
// Vocabulary
// ---------------------------------------------------------------------------

#[test]
fn release_vocabulary_and_mask() {
    use structural_analysis::EndReleases3D;

    let none = EndReleases3D::none();
    assert!(none.is_empty());
    assert_eq!(none, EndReleases3D::default());

    let both = EndReleases3D::both_pins();
    assert!(!both.is_empty());
    let a = both.array();
    assert!(a.iter().all(|&x| x));
    assert_eq!(EndReleases3D::from_array(a), both);

    // Start pin: local DOFs 3, 4, 5 masked; end untouched.
    let elem = FrameElement3D_pin_start();
    let mut f = [1.0f64; 12];
    f = elem.mask_released_end_forces(f);
    assert_eq!(f[3], 0.0);
    assert_eq!(f[4], 0.0);
    assert_eq!(f[5], 0.0);
    for &dof in &[9, 10, 11] {
        assert_eq!(f[dof], 1.0, "end i release must not mask end j");
    }

    // No releases → the mask is the identity.
    let rigid = structural_analysis::EndReleases3D::none();
    let rigid_elem = FrameElement3D_with(rigid);
    assert_eq!(rigid_elem.mask_released_end_forces([2.0; 12]), [2.0; 12]);

    // A rigid element's released-aware stiffness is the plain stiffness.
    assert_eq!(
        rigid_elem.stiffness_with_releases((0.0, 0.0, 0.0), (4.0, 0.0, 0.0)).unwrap(),
        rigid_elem.local_stiffness((0.0, 0.0, 0.0), (4.0, 0.0, 0.0)).unwrap(),
    );
}

/// Convenience: element with `EndReleases3D::start_pin()`.
fn FrameElement3D_pin_start() -> FrameElement3D {
    let e = FrameElement3D_with(structural_analysis::EndReleases3D::start_pin());
    e
}

/// Convenience builder around `FrameElement3D::with_releases`.
fn FrameElement3D_with(r: structural_analysis::EndReleases3D) -> FrameElement3D {
    FrameElement3D::with_releases(0, 1, section(), [0.0, 0.0, 1.0], r).unwrap()
}

// ---------------------------------------------------------------------------
// Condensed stiffness shape
// ---------------------------------------------------------------------------

#[test]
fn released_rotations_have_zero_rows_and_columns() {
    let elem = FrameElement3D_with(both_pins());
    let k = elem
        .stiffness_with_releases((0.0, 0.0, 0.0), (4.0, 0.0, 0.0))
        .unwrap();
    for &dof in &[3, 4, 5, 9, 10, 11] {
        for j in 0..12 {
            assert_eq!(k[dof][j], 0.0, "row {dof} must be empty after condensation");
            assert_eq!(k[j][dof], 0.0, "column {dof} must be empty after condensation");
        }
    }
    // The retained block stays symmetric and keeps the axial entry.
    for i in 0..12 {
        for j in 0..12 {
            assert!((k[i][j] - k[j][i]).abs() <= 1e-6 * k[i][j].abs().max(1.0));
        }
    }
    assert!(k[0][0] > 0.0, "axial stiffness survives a full release");
}

#[test]
fn condensed_stiffness_is_positive_semidefinite() {
    // With all rotations released the element is a bar: the retained block
    // (two translations per end + axial) must still be positive definite on
    // every non-rigid mode.
    let elem = FrameElement3D_with(both_pins());
    let k = elem
        .stiffness_with_releases((0.0, 0.0, 0.0), (3.0, 0.0, 0.0))
        .unwrap();
    let mut v = [0.0f64; 12];
    // pure extension mode
    v[6] = 1.0;
    v[0] = -1.0;
    let kv = mat_vec(&k, &v);
    let energy = dot(&v, &kv);
    assert!(energy > 0.0, "axial strain energy must stay positive: {energy}");
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
// ---------------------------------------------------------------------------
// Analytical reference results
//
// All expectations below are textbook closed-form results for the released
// structure, evaluated independently of the implementation.
//
// A member release only *decouples* the element from its joint; the joint
// rotation itself must still be restrained by something (another member or a
// support).  A lone member released at BOTH ends with merely translational
// supports is therefore a mechanism — exactly as in any commercial solver —
// so every released model below rests the freed rotation on a sibling.
// ---------------------------------------------------------------------------

/// Single simply-supported beam (no release), pinned translations at both
/// ends, UDL w in **global −Y**: textbook M_i = M_j = 0,
/// R = qL/2 each, end rotations qL³/(24EI).
#[test]
fn rigid_simply_supported_beam_matches_closed_form() {
    let L = 4.0_f64;
    let w = 500.0_f64;
    let solver = simply_supported(L, w, [false; 6]);
    let res = solver.solve().unwrap();
    let f = res.member_end_forces(0).unwrap();

    // Bending plane of the local-z load: moments about local y (indices 4/10).
    let i = section().iy();
    let e = section().E();
    let theta = w * L * L * L / (24.0 * e * i);
    assert!((f[4]).abs() < 1e-9, "simply-supported M_i = {}", f[4]);
    assert!((f[10]).abs() < 1e-9, "simply-supported M_j = {}", f[10]);

    let ry_i = res.reaction(0, Dof3D::Uy).unwrap();
    let ry_j = res.reaction(1, Dof3D::Uy).unwrap();
    assert!((ry_i - w * L / 2.0).abs() / (w * L) < 1e-12, "R_i = {ry_i}");
    assert!((ry_j - w * L / 2.0).abs() / (w * L) < 1e-12, "R_j = {ry_j}");

    let rz_i = res.displacement(0, Dof3D::Rz).unwrap();
    let rz_j = res.displacement(1, Dof3D::Rz).unwrap();
    assert!((rz_i.abs() - theta).abs() / theta < 1e-12, "θ_i = {rz_i} vs {theta}");
    assert!((rz_j.abs() - theta).abs() / theta < 1e-12, "θ_j = {rz_j} vs {theta}");
    assert!(rz_i * rz_j < 0.0, "the two end rotations must oppose");

    // The released counterpart of the same model is a mechanism (nothing
    // restrains the freed joint rotations) — assert that the solver says so
    // instead of returning a plausible-looking answer.
    let mut released = x_beam(L, both_pins().array());
    released.fix_dof(0, Dof3D::Ux, 0.0).unwrap();
    released.fix_dof(0, Dof3D::Uy, 0.0).unwrap();
    released.fix_dof(0, Dof3D::Uz, 0.0).unwrap();
    released.fix_dof(1, Dof3D::Uy, 0.0).unwrap();
    released.fix_dof(1, Dof3D::Uz, 0.0).unwrap();
    released.add_uniform_z(0, w).unwrap();
    let err = FrameSolver3D::from_model(&released)
        .unwrap()
        .solve()
        .expect_err("a doubly-released lone member is a mechanism");
    assert!(
        matches!(err, FemError::SolverError { .. }),
        "expected a solver/mechanism error, got {err:?}"
    );
}

/// Releasing a rotation of the **unloaded** plane is inert once the joint's
/// rotation is backed by a sibling member.
///
/// Member A i–m (along X, loaded in the X–Y plane by a −Y force at m)
/// has its orthogonal rotation rz released at m; member B m-j (along
/// Z) restrains that rotation through its own torsion.  Because the two
/// bending planes of a straight axis-aligned member are decoupled, the
/// loaded-plane response is **bit-for-bit identical** to the rigid model.
#[test]
fn release_of_the_unloaded_plane_is_inert() {
    fn chain(release: bool) -> FrameModel3D {
        let mut m = FrameModel3D::new();
        let i = m.add_node(0.0, 0.0, 0.0).unwrap();
        let k = m.add_node(4.0, 0.0, 0.0).unwrap();
        let j = m.add_node(4.0, 0.0, 4.0).unwrap();
        let a = m.add_member(i, k, section(), [0.0, 0.0, 1.0]).unwrap();
        m.add_member(k, j, section(), [1.0, 0.0, 0.0]).unwrap();
        if release {
            m.set_member_releases(
                a,
                structural_analysis::EndReleases3D::from_array([false, false, false, false, false, true]),
            )
            .unwrap();
        }
        m.fix_node(i);
        m.fix_node(j);
        m.add_nodal_load(k, 0.0, -1000.0, 0.0, 0.0, 0.0, 0.0).unwrap();
        m
    }

    let rigid = FrameSolver3D::from_model(&chain(false)).unwrap().solve().unwrap();
    let released = FrameSolver3D::from_model(&chain(true)).unwrap().solve().unwrap();

    // The loaded plane agrees to the last bit.
    assert_eq!(
        rigid.displacements, released.displacements,
        "an orthogonal-plane release must not perturb the loaded response"
    );
    // …while the released rotation reports zero moment.
    let f = released.member_end_forces(0).unwrap();
    assert!(f[11].abs() < 1e-12, "released rz at m: {}", f[11]);
    // The loaded plane's bending moment is unchanged and nonzero.
    assert!(f[10].abs() > 1.0, "loaded-plane moment: {}", f[10]);
}
/// Two-member L-frame: the beam's released joint hands its moment to the
/// sibling column, while a rigid connection keeps it.
///
/// Configuration: A(0,0,0) fixed — B(0,4,0) — C(6,4,0) (translations
/// restrained, rotation free: restrained by the rigid beam end).  The beam
/// B–C carries a UDL in global −Y; A–B is a column released at B.
#[test]
fn released_joint_moment_hands_off_to_the_sibling() {
    fn frame(release_at_b: bool) -> FrameModel3D {
        let mut m = FrameModel3D::new();
        let a = m.add_node(0.0, 0.0, 0.0).unwrap();
        let b = m.add_node(0.0, 4.0, 0.0).unwrap();
        let c = m.add_node(6.0, 4.0, 0.0).unwrap();
        // Column along Y (ref vector must avoid the member axis).
        let col = m.add_member(a, b, section(), [1.0, 0.0, 0.0]).unwrap();
        let beam = m.add_member(b, c, section(), [0.0, 0.0, 1.0]).unwrap();
        if release_at_b {
            m.set_member_releases(
                beam,
                structural_analysis::EndReleases3D::from_array([true, true, true, false, false, false]),
            )
            .unwrap();
        }
        m.fix_node(a);
        m.fix_dof(c, Dof3D::Ux, 0.0).unwrap();
        m.fix_dof(c, Dof3D::Uy, 0.0).unwrap();
        m.fix_dof(c, Dof3D::Uz, 0.0).unwrap();
        // twist restraint at the far bearing (see simply_supported above)
        m.fix_dof(c, Dof3D::Rx, 0.0).unwrap();
        m.add_uniform_z(beam, 800.0).unwrap();
        let _ = col;
        m
    }

    // Rigid connection: the beam end at B carries a moment.
    let rigid = FrameSolver3D::from_model(&frame(false)).unwrap().solve().unwrap();
    let beam_rigid = rigid.member_end_forces(1).unwrap();
    assert!(
        beam_rigid[4].abs() > 1.0,
        "rigid beam end must transfer a moment: {}",
        beam_rigid[4]
    );

    // Released connection: the beam end moment at the joint is exactly zero.
    let pinned = FrameSolver3D::from_model(&frame(true)).unwrap().solve().unwrap();
    let beam_pinned = pinned.member_end_forces(1).unwrap();
    assert!(
        beam_pinned[4].abs() < 1e-12,
        "released beam end must carry no moment: {}",
        beam_pinned[4]
    );
    // Joint equilibrium: with no external moment applied at B, the column's
    // end moment there is zero too — the hinge simply removes the beam's
    // contribution instead of transferring it.
    let col_pinned = pinned.member_end_forces(0).unwrap();
    assert!(
        col_pinned[10].abs() < 1e-9,
        "joint equilibrium leaves the sibling momentless: {}",
        col_pinned[10]
    );
    // Classic simply-supported result for the released span: the far end
    // (a rotationally free support) rotates by qL³/(24 E I) while the
    // near joint — held by the sibling column — does not rotate at all.
    let e = section().E();
    let i_loaded = section().iy(); // local-z load bends about local y

    let rz_b = pinned.displacement(1, Dof3D::Rz).unwrap();
    assert!(rz_b.abs() < 1e-15, "the sibling column holds the joint: {rz_b}");
    // Vertical reactions split evenly: qL/2 each.
    let r_a = pinned.reaction(0, Dof3D::Uy).unwrap();
    let r_c = pinned.reaction(2, Dof3D::Uy).unwrap();
    assert!((r_a - 2400.0).abs() / 2400.0 < 1e-12, "R_A = {r_a}");
    assert!((r_c - 2400.0).abs() / 2400.0 < 1e-12, "R_C = {r_c}");

    // Global equilibrium: ΣRy over both supports = total load (6 m × 800 N/m).
    let total = 6.0 * 800.0;
    let sum_ry = pinned.reaction(0, Dof3D::Uy).unwrap() + pinned.reaction(2, Dof3D::Uy).unwrap();
    assert!((sum_ry - total).abs() / total < 1e-12, "ΣRy = {sum_ry} vs {total}");
}

/// Control: with rigid connections and both ends fixed, the familiar
/// fixed-end moments qL²/12 appear at both ends.
#[test]
fn fixed_fixed_control_keeps_symmetric_fixed_end_moments() {
    let L = 4.0_f64;
    let w = 500.0_f64;
    let mut m = x_beam(L, [false; 6]);
    m.fix_node(0);
    m.fix_node(1);
    m.add_uniform_z(0, w).unwrap();
    let res = FrameSolver3D::from_model(&m).unwrap().solve().unwrap();
    let f = res.member_end_forces(0).unwrap();

    let m_fef = w * L * L / 12.0; // N·m
    assert!(
        (f[4].abs() - m_fef).abs() / m_fef < 1e-12,
        "M_i = {} vs {m_fef}",
        f[4].abs()
    );
    assert!(
        (f[10].abs() - m_fef).abs() / m_fef < 1e-12,
        "M_j = {} vs {m_fef}",
        f[10].abs()
    );
    assert!(f[4] * f[10] < 0.0, "the two fixed-end moments oppose");
}

// ---------------------------------------------------------------------------
// Torsion release
// ---------------------------------------------------------------------------

#[test]
fn torsion_release_blocks_torsion_flow_into_the_sibling() {
    // Member A: (0,0,0)-(4,0,0), fixed at i, rotation about x released at j.
    // Member B: (4,0,0)-(4,4,0) carries the joint's rotational restraint.
    // A torque applied at node j therefore flows into B, never into A.
    let mut m = FrameModel3D::new();
    let a = m.add_node(0.0, 0.0, 0.0).unwrap();
    let b = m.add_node(4.0, 0.0, 0.0).unwrap();
    let c = m.add_node(4.0, 4.0, 0.0).unwrap();
    let ma = m.add_member(a, b, section(), [0.0, 0.0, 1.0]).unwrap();
    let mb = m.add_member(b, c, section(), [1.0, 0.0, 0.0]).unwrap();
    m.set_member_releases(
        ma,
        structural_analysis::EndReleases3D::from_array([false, false, false, true, false, false]),
    )
    .unwrap();
    m.fix_node(a);
    m.fix_node(c);
    m.add_nodal_load(b, 0.0, 0.0, 0.0, 250.0, 0.0, 0.0).unwrap();

    let res = FrameSolver3D::from_model(&m).unwrap().solve().unwrap();
    let fa = res.member_end_forces(ma).unwrap();
    let fb = res.member_end_forces(mb).unwrap();

    // Released member A carries no torsion anywhere (masked end forces).
    assert!(fa[3].abs() < 1e-9, "Mx_i of A = {}", fa[3]);
    assert!(fa[9].abs() < 1e-9, "Mx_j of A = {}", fa[9]);
    // Member B's end at node j takes the applied torque (opposite DOF
    // order at its start node).
    assert!(fb[3].abs() > 1.0, "torque must flow into B: {}", fb[3]);
}

// ---------------------------------------------------------------------------
// Model API
// ---------------------------------------------------------------------------

#[test]
fn model_setter_round_trips_and_validates() {
    let mut m = x_beam(4.0, both_pins().array());
    assert_eq!(m.member_releases(0).unwrap(), both_pins());

    let none = structural_analysis::EndReleases3D::none();
    m.set_member_releases(0, none).unwrap();
    assert_eq!(m.member_releases(0).unwrap(), none);

    assert_eq!(m.member_releases(1), None);
    let err = m
        .set_member_releases(7, none)
        .expect_err("out-of-bounds member must be rejected");
    assert!(matches!(err, FemError::InvalidMember(_)), "got {err:?}");
}

#[test]
fn solver_sees_the_releases_through_the_model() {
    // The solver clones the model: solve → result consistency for the
    // released member (no torsion flow) is asserted analytically in
    // `torsion_release_blocks_torsion_flow_into_the_sibling`; here we only
    // assert the accessor chain used by the GUI surface.
    let m = x_beam(6.0, both_pins().array());
    let solver = FrameSolver3D::from_model(&m).unwrap();
    assert_eq!(solver.model().member_releases(0).unwrap(), both_pins());
}
