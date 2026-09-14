//! Phase 13 - 2D frame API: reference cases and validation.

use section_properties::frame::{FrameModel, MemberHandle, NodeHandle};
use section_properties::{BeamSection, Dof, FemError, Material, SolverSelection};

const E: f64 = 200e9;
const A: f64 = 5e-3;
const I: f64 = 2e-5;
const EA: f64 = E * A;
const EI: f64 = E * I;

fn steel() -> Material {
    Material::new(E, 0.3, 7850.0, "Steel")
}
fn sec() -> BeamSection {
    BeamSection::new(A, I)
}

fn assert_close(a: f64, b: f64, abs: f64, rel: f64, label: &str) {
    let bound = abs + rel * a.abs().max(b.abs());
    assert!(
        (a - b).abs() <= bound,
        "{}: {} vs {} (|diff| = {:.3e} > {:.3e})",
        label,
        a,
        b,
        (a - b).abs(),
        bound
    );
}

/// Two-node cantilever along +x with a free tip.
fn cantilever(
    len: f64,
    members: usize,
) -> Result<(FrameModel, NodeHandle, Vec<MemberHandle>), FemError> {
    let mut f = FrameModel::new();
    let base = f.add_node(0.0, 0.0)?;
    let mut handles = Vec::new();
    let mut prev = base;
    let dx = len / members as f64;
    for k in 1..=members {
        let n = f.add_node(k as f64 * dx, 0.0)?;
        handles.push(f.add_member(prev, n, steel(), sec())?);
        prev = n;
    }
    f.fix(base)?;
    Ok((f, prev, handles))
}

// ---------------------------------------------------------------------------
// Test 1 - axial member
// ---------------------------------------------------------------------------

#[test]
fn test1_axial_member() -> Result<(), FemError> {
    let (l, p) = (2.0, 1.0e4);
    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(l, 0.0)?;
    let m = f.add_member(n0, n1, steel(), sec())?;
    f.fix(n0)?;
    f.roller_y(n1)?; // allow axial sliding only
    f.nodal_load(n1, p, 0.0)?;

    let r = f.solve()?;
    assert_close(
        r.displacement(n1, Dof::Ux)?,
        p * l / EA,
        1e-15,
        1e-9,
        "u = PL/EA",
    );
    assert!(
        r.displacement(n1, Dof::Uy)?.abs() < 1e-12,
        "no transverse motion"
    );
    assert_close(r.reaction(n0, Dof::Ux)?, -p, 1e-9, 1e-9, "Rx = -P");

    // Element axial force is +P tension; end forces follow the frozen convention.
    let fe = r.member_end_forces(m)?;
    assert_close(fe[0], p, 1e-6, 1e-9, "N_i = +P");
    assert_close(fe[3], -p, 1e-6, 1e-9, "N_j = -P");

    let e = r.equilibrium();
    assert!(e.is_balanced(), "equilibrium: {e:?}");
    assert_close(e.applied_fx, p, 1e-9, 1e-9, "applied Fx");
    println!(
        "  test1 axial: u={:.6e} Rx={:.6e} residuals=({:.2e},{:.2e},{:.2e})",
        r.displacement(n1, Dof::Ux)?,
        r.reaction(n0, Dof::Ux)?,
        e.fx_residual,
        e.fy_residual,
        e.mz_residual
    );
    Ok(())
}

// ---------------------------------------------------------------------------
// Test 2 - cantilever tip force
// ---------------------------------------------------------------------------

#[test]
fn test2_cantilever_tip_force() -> Result<(), FemError> {
    let (l, p) = (2.0, 1.0e4);
    let (mut f, tip, members) = cantilever(l, 1)?;
    f.nodal_load(tip, 0.0, -p)?;

    let r = f.solve()?;
    assert_close(
        r.displacement(tip, Dof::Uy)?,
        -p * l.powi(3) / (3.0 * EI),
        1e-14,
        1e-9,
        "v = -PL^3/3EI",
    );
    assert_close(
        r.displacement(tip, Dof::Rz)?,
        -p * l.powi(2) / (2.0 * EI),
        1e-14,
        1e-9,
        "theta = -PL^2/2EI",
    );
    assert_close(
        r.reaction(f.node_handle(0)?, Dof::Uy)?,
        p,
        1e-9,
        1e-9,
        "Ry = +P",
    );
    assert_close(
        r.reaction(f.node_handle(0)?, Dof::Rz)?,
        p * l,
        1e-9,
        1e-9,
        "Rz = +PL",
    );

    let fe = r.member_end_forces(members[0])?;
    assert_close(fe[1], -p, 1e-6, 1e-9, "V_i = -P (element-on-node)");
    assert_close(fe[2], -p * l, 1e-6, 1e-9, "M_i = -PL");
    assert!(r.equilibrium().is_balanced(), "{:?}", r.equilibrium());
    println!(
        "  test2 cantilever: v={:.6e}",
        r.displacement(tip, Dof::Uy)?
    );
    Ok(())
}

// ---------------------------------------------------------------------------
// Test 3 - cantilever tip moment
// ---------------------------------------------------------------------------

#[test]
fn test3_cantilever_tip_moment() -> Result<(), FemError> {
    let (l, m0) = (2.0, 8.0e3);
    let (mut f, tip, members) = cantilever(l, 1)?;
    f.nodal_moment(tip, m0)?;

    let r = f.solve()?;
    assert_close(
        r.displacement(tip, Dof::Rz)?,
        m0 * l / EI,
        1e-15,
        1e-9,
        "theta = ML/EI",
    );
    assert_close(
        r.reaction(f.node_handle(0)?, Dof::Rz)?,
        -m0,
        1e-9,
        1e-9,
        "Rz = -M",
    );

    // Frozen convention: the free-end element-on-node moment is M_j = -M while
    // the internal section moment is +M. The frame API must not invert it.
    let fe = r.member_end_forces(members[0])?;
    assert_close(fe[5], -m0, 1e-6, 1e-9, "M_j = -M");
    assert_close(fe[2], m0, 1e-6, 1e-9, "M_i = +M");
    assert!(r.equilibrium().is_balanced());
    println!(
        "  test3 tip moment: theta={:.6e} M_j={:.6e}",
        r.displacement(tip, Dof::Rz)?,
        fe[5]
    );
    Ok(())
}

// ---------------------------------------------------------------------------
// Test 4 - portal frame
// ---------------------------------------------------------------------------

fn portal(
    h: f64,
    w: f64,
) -> Result<(FrameModel, NodeHandle, NodeHandle, Vec<MemberHandle>), FemError> {
    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(0.0, h)?;
    let n2 = f.add_node(w, h)?;
    let n3 = f.add_node(w, 0.0)?;
    let m0 = f.add_member(n0, n1, steel(), sec())?;
    let m1 = f.add_member(n1, n2, steel(), sec())?;
    let m2 = f.add_member(n2, n3, steel(), sec())?;
    f.fix(n0)?;
    f.fix(n3)?;
    Ok((f, n1, n3, vec![m0, m1, m2]))
}

#[test]
fn test4_portal_frame() -> Result<(), FemError> {
    let (h, w, f0) = (2.0, 3.0, 3.0e4);
    let (mut f, top_left, _base_right, _) = portal(h, w)?;
    f.nodal_load(top_left, f0, 0.0)?;

    let r = f.solve()?;
    assert!(
        r.displacement(top_left, Dof::Ux)? > 0.0,
        "must sway with the load"
    );
    // The portal frame is statically indeterminate: the lateral load is shared
    // between the two bases, so only the TOTAL reaction balances the load.
    let rx_base0 = r.reaction(f.node_handle(0)?, Dof::Ux)?;
    let rx_base1 = r.reaction(f.node_handle(3)?, Dof::Ux)?;
    assert_close(rx_base0 + rx_base1, -f0, 1e-6, 1e-9, "total Rx = -F");
    assert!(
        rx_base0.abs() > 0.0 && rx_base1.abs() > 0.0,
        "load shared between bases"
    );
    let e = r.equilibrium();
    assert!(e.is_balanced(), "{e:?}");
    // The right base carries no horizontal load when both columns are identical?
    // It does: the frame is statically indeterminate, so check the total only.
    assert_close(
        e.reaction_fx,
        -f0,
        1e-6,
        1e-9,
        "sum of horizontal reactions = -F",
    );
    println!(
        "  test4 portal: sway={:.6e} Rx(total)={:.6e} residuals=({:.2e},{:.2e},{:.2e})",
        r.displacement(top_left, Dof::Ux)?,
        e.reaction_fx,
        e.fx_residual,
        e.fy_residual,
        e.mz_residual
    );
    Ok(())
}

// ---------------------------------------------------------------------------
// Test 5 - apex frame (horizontal thrust!)
// ---------------------------------------------------------------------------

#[test]
fn test5_apex_frame() -> Result<(), FemError> {
    let p = 25e3;
    let mut f = FrameModel::new();
    let b0 = f.add_node(0.0, 0.0)?;
    let apex = f.add_node(1.0, 1.0)?;
    let b1 = f.add_node(2.0, 0.0)?;
    f.add_member(b0, apex, steel(), sec())?;
    f.add_member(apex, b1, steel(), sec())?;
    f.fix(b0)?;
    f.fix(b1)?;
    f.nodal_load(apex, 0.0, -p)?;

    let r = f.solve()?;
    assert!(
        r.displacement(apex, Dof::Ux)?.abs() < 1e-9,
        "apex must not sway"
    );
    assert_close(
        r.displacement(apex, Dof::Uy)?,
        -3.4526698e-5,
        1e-11,
        1e-6,
        "apex deflection",
    );
    assert_close(
        r.reaction(b0, Dof::Uy)?,
        p / 2.0,
        1e-9,
        1e-9,
        "Ry(base0) = P/2",
    );
    assert_close(
        r.reaction(b1, Dof::Uy)?,
        p / 2.0,
        1e-9,
        1e-9,
        "Ry(base1) = P/2",
    );

    // A vertical load on an inclined frame DOES produce horizontal reactions:
    // an equal-and-opposite base thrust. Zero would be wrong.
    let (rx0, rx1) = (r.reaction(b0, Dof::Ux)?, r.reaction(b1, Dof::Ux)?);
    assert!(
        rx0.abs() > 1.0,
        "an inclined frame develops a thrust, got {rx0}"
    );
    assert_close(rx0 + rx1, 0.0, 1e-6, 1e-9, "base thrusts balance");
    assert!(r.equilibrium().is_balanced(), "{:?}", r.equilibrium());
    println!(
        "  test5 apex: uy={:.6e} thrust={:.6e} (Ry per base={:.6e})",
        r.displacement(apex, Dof::Uy)?,
        rx0,
        r.reaction(b0, Dof::Uy)?
    );
    Ok(())
}

// ---------------------------------------------------------------------------
// Test 6 - prescribed settlement
// ---------------------------------------------------------------------------

#[test]
fn test6_prescribed_settlement() -> Result<(), FemError> {
    let delta = -0.005;
    let (mut f, _, base_right, _) = portal(2.0, 3.0)?;
    f.restrain(base_right, Dof::Uy, delta)?;

    let r = f.solve()?;
    assert_eq!(
        r.displacement(base_right, Dof::Uy)?,
        delta,
        "prescribed value exact"
    );
    let e = r.equilibrium();
    assert!(e.is_balanced(), "{e:?}");
    // No external load: reactions are self-equilibrated but non-zero.
    assert!(
        e.reaction_fx.abs() + e.reaction_fy.abs() > 0.0,
        "settlement induces reactions"
    );
    assert_close(e.applied_fx, 0.0, 1e-12, 1e-12, "no applied load");
    println!(
        "  test6 settlement: Ry={:.6e} residual Fy={:.2e}",
        e.reaction_fy, e.fy_residual
    );
    Ok(())
}

// ---------------------------------------------------------------------------
// Test 7 - multi-member straight beam (subdivision consistency)
// ---------------------------------------------------------------------------

#[test]
fn test7_multi_member_straight_beam() -> Result<(), FemError> {
    let (l, p) = (2.0, 1.0e4);
    let (mut single, tip_s, _) = cantilever(l, 1)?;
    single.nodal_load(tip_s, 0.0, -p)?;
    let r1 = single.solve()?;

    let (mut split, tip_m, members) = cantilever(l, 4)?;
    split.nodal_load(tip_m, 0.0, -p)?;
    let r4 = split.solve()?;

    // Consistent nodal load => the nodal tip value is mesh independent.
    assert_close(
        r4.displacement(tip_m, Dof::Uy)?,
        r1.displacement(tip_s, Dof::Uy)?,
        1e-14,
        1e-9,
        "tip deflection independent of member subdivision",
    );
    assert_close(
        r4.reaction(split.node_handle(0)?, Dof::Rz)?,
        p * l,
        1e-9,
        1e-9,
        "support moment",
    );
    assert_eq!(members.len(), 4);
    assert!(r4.equilibrium().is_balanced(), "{:?}", r4.equilibrium());
    println!(
        "  test7 subdivision: v(1 member)={:.6e} v(4 members)={:.6e}",
        r1.displacement(tip_s, Dof::Uy)?,
        r4.displacement(tip_m, Dof::Uy)?
    );
    Ok(())
}

// ---------------------------------------------------------------------------
// Test 8 - solver cross-validation
// ---------------------------------------------------------------------------

#[test]
fn test8_solver_cross_validation() -> Result<(), FemError> {
    let (mut f, top_left, _, members) = portal(2.0, 3.0)?;
    f.nodal_load(top_left, 2.0e4, -1.0e4)?;
    f.member_udl(members[1], 0.0, -5.0e3)?;

    let mut reference: Option<(Vec<f64>, Vec<f64>)> = None;
    for name in ["dense", "skyline_ldlt", "sparse_lu"] {
        let r = f.solve_with(SolverSelection::named(name))?;
        assert_eq!(r.solver_name(), Some(name));
        let u = r.displacements().to_vec();
        let reac = r.reactions();
        assert!(
            r.equilibrium().is_balanced(),
            "{name}: {:?}",
            r.equilibrium()
        );
        match &reference {
            None => reference = Some((u, reac)),
            Some((u0, r0)) => {
                for i in 0..u.len() {
                    assert_close(u[i], u0[i], 1e-13, 1e-9, &format!("{name}: u[{i}]"));
                }
                for i in 0..reac.len() {
                    assert_close(reac[i], r0[i], 1e-6, 1e-9, &format!("{name}: R[{i}]"));
                }
            }
        }
    }
    println!("  test8 cross-validation: dense == skyline_ldlt == sparse_lu");
    Ok(())
}

// ---------------------------------------------------------------------------
// Validation
// ---------------------------------------------------------------------------

fn simple_frame() -> Result<(FrameModel, NodeHandle, NodeHandle), FemError> {
    let mut f = FrameModel::new();
    let a = f.add_node(0.0, 0.0)?;
    let b = f.add_node(1.0, 0.0)?;
    f.add_member(a, b, steel(), sec())?;
    f.fix(a)?;
    Ok((f, a, b))
}

#[test]
fn validation_invalid_handles() -> Result<(), FemError> {
    let (mut big, _, _) = simple_frame()?;
    let extra = big.add_node(5.0, 5.0)?; // handle index 2, orphan for now
    let mut small = FrameModel::new();
    let only = small.add_node(0.0, 0.0)?;
    let other = small.add_node(1.0, 0.0)?;
    small.add_member(only, other, steel(), sec())?;
    small.fix(only)?;

    // A handle from a different model has an out-of-range index here.
    assert!(matches!(small.fix(extra), Err(FemError::InvalidNode(_))));
    assert!(matches!(
        small.nodal_load(extra, 1.0, 0.0),
        Err(FemError::InvalidNode(_))
    ));
    assert!(matches!(
        small.member_udl(MemberHandle::from_index(7), 0.0, 0.0),
        Err(FemError::InvalidMember(_))
    ));

    // Solved-result accessors validate too.
    let r = small.solve()?;
    assert!(matches!(
        r.displacement(extra, Dof::Ux),
        Err(FemError::InvalidNode(_))
    ));
    assert!(matches!(
        r.reaction(extra, Dof::Ux),
        Err(FemError::InvalidNode(_))
    ));
    assert!(matches!(
        r.member_end_forces(MemberHandle::from_index(3)),
        Err(FemError::InvalidMember(_))
    ));
    println!("  validation: invalid node/member handles rejected with typed errors");
    Ok(())
}

#[test]
fn validation_zero_length_and_duplicate_members() -> Result<(), FemError> {
    let mut f = FrameModel::new();
    let a = f.add_node(0.0, 0.0)?;
    let b = f.add_node(1.0, 0.0)?;
    let same = f.add_node(1.0, 0.0)?; // coincident with b

    assert!(matches!(
        f.add_member(a, a, steel(), sec()),
        Err(FemError::ZeroLengthMember(_))
    ));
    assert!(matches!(
        f.add_member(b, same, steel(), sec()),
        Err(FemError::ZeroLengthMember(_))
    ));

    f.add_member(a, b, steel(), sec())?;
    assert!(matches!(
        f.add_member(a, b, steel(), sec()),
        Err(FemError::DuplicateMember(_))
    ));
    assert!(matches!(
        f.add_member(b, a, steel(), sec()),
        Err(FemError::DuplicateMember(_))
    ));
    println!("  validation: zero-length and duplicate (i,j)/(j,i) members rejected");
    Ok(())
}

#[test]
fn validation_orphan_and_disconnected() -> Result<(), FemError> {
    // (a) orphan node
    let mut f = FrameModel::new();
    let a = f.add_node(0.0, 0.0)?;
    let b = f.add_node(1.0, 0.0)?;
    f.add_member(a, b, steel(), sec())?;
    f.fix(a)?;
    let _orphan = f.add_node(10.0, 10.0)?;
    assert!(matches!(f.solve(), Err(FemError::OrphanNode(_))));

    // (b) disconnected components
    let mut g = FrameModel::new();
    let a = g.add_node(0.0, 0.0)?;
    let b = g.add_node(1.0, 0.0)?;
    let c = g.add_node(10.0, 0.0)?;
    let d = g.add_node(11.0, 0.0)?;
    g.add_member(a, b, steel(), sec())?;
    g.add_member(c, d, steel(), sec())?;
    g.fix(a)?;
    g.fix(c)?;
    assert!(matches!(g.solve(), Err(FemError::DisconnectedStructure(_))));

    // (c) empty model
    let empty = FrameModel::new();
    assert!(matches!(empty.solve(), Err(FemError::InvalidModel(_))));
    println!("  validation: orphan node, disconnected components and empty model diagnosed");
    Ok(())
}

#[test]
fn validation_material_section_coordinates_and_loads() -> Result<(), FemError> {
    let mut f = FrameModel::new();
    let a = f.add_node(0.0, 0.0)?;
    let b = f.add_node(1.0, 0.0)?;

    // invalid section/material
    assert!(matches!(
        f.add_member(a, b, Material::new(0.0, 0.3, 1.0, "S"), sec()),
        Err(FemError::InvalidInput(_))
    ));
    assert!(matches!(
        f.add_member(a, b, steel(), BeamSection::new(-1.0, I)),
        Err(FemError::InvalidInput(_))
    ));
    assert!(matches!(
        f.add_member(a, b, steel(), BeamSection::new(A, f64::NAN)),
        Err(FemError::InvalidInput(_))
    ));

    // non-finite coordinates
    assert!(matches!(
        f.add_node(f64::NAN, 0.0),
        Err(FemError::InvalidInput(_))
    ));
    assert!(matches!(
        f.add_node(0.0, f64::INFINITY),
        Err(FemError::InvalidInput(_))
    ));

    // non-finite loads
    f.add_member(a, b, steel(), sec())?;
    f.fix(a)?;
    let m = MemberHandle::from_index(0);
    assert!(matches!(
        f.nodal_load(b, f64::NAN, 0.0),
        Err(FemError::InvalidInput(_))
    ));
    assert!(matches!(
        f.nodal_moment(b, f64::NAN),
        Err(FemError::InvalidInput(_))
    ));
    assert!(matches!(
        f.member_udl(m, 0.0, f64::NAN),
        Err(FemError::InvalidInput(_))
    ));
    assert!(matches!(
        f.member_point_load(m, 1.5, 0.0, 0.0, 0.0),
        Err(FemError::InvalidInput(_))
    ));
    assert!(matches!(
        f.member_point_load(m, 0.5, 0.0, f64::NAN, 0.0),
        Err(FemError::InvalidInput(_))
    ));
    println!("  validation: material/section, coordinates and loads validated");
    Ok(())
}

#[test]
fn validation_insufficient_restraint_is_a_solver_error() -> Result<(), FemError> {
    // Mechanism detection is deliberately NOT attempted: an unrestrained frame
    // reports the solver's singular-system error, not a targeted diagnostic.
    let mut f = FrameModel::new();
    let a = f.add_node(0.0, 0.0)?;
    let b = f.add_node(2.0, 0.0)?;
    f.add_member(a, b, steel(), sec())?;
    f.nodal_load(b, 0.0, -1.0e3)?;
    let err = f.solve().expect_err("an unsupported frame must not solve");
    assert!(
        matches!(err, FemError::SolverError(_)),
        "expected SolverError, got {err:?}"
    );
    println!("  validation: insufficient restraint -> {err:?} (documented)");
    Ok(())
}

#[test]
fn support_vocabulary_solves() -> Result<(), FemError> {
    // Simply supported beam via pin + roller: a legitimate structural system.
    let (l, q) = (4.0, 2.0e3);
    let mut f = FrameModel::new();
    let a = f.add_node(0.0, 0.0)?;
    let b = f.add_node(l, 0.0)?;
    let m = f.add_member(a, b, steel(), sec())?;
    f.pin(a)?;
    f.roller_y(b)?;
    f.member_udl(m, 0.0, -q)?;

    let r = f.solve()?;
    assert_close(
        r.reaction(a, Dof::Uy)?,
        q * l / 2.0,
        1e-9,
        1e-9,
        "Ry = qL/2",
    );
    assert_close(
        r.reaction(b, Dof::Uy)?,
        q * l / 2.0,
        1e-9,
        1e-9,
        "Ry = qL/2",
    );
    assert!(r.equilibrium().is_balanced(), "{:?}", r.equilibrium());
    // Mid-span deflection of a simply supported beam under UDL: 5qL^4/384EI.
    let mid = {
        let mut f2 = FrameModel::new();
        let a = f2.add_node(0.0, 0.0)?;
        let c = f2.add_node(l / 2.0, 0.0)?;
        let b = f2.add_node(l, 0.0)?;
        let m1 = f2.add_member(a, c, steel(), sec())?;
        let m2 = f2.add_member(c, b, steel(), sec())?;
        f2.pin(a)?;
        f2.roller_y(b)?;
        f2.member_udl(m1, 0.0, -q)?;
        f2.member_udl(m2, 0.0, -q)?;
        f2.solve()?.displacement(c, Dof::Uy)?
    };
    assert_close(
        mid,
        -5.0 * q * l.powi(4) / (384.0 * EI),
        1e-14,
        1e-6,
        "5qL^4/384EI",
    );
    println!("  support vocabulary: pin+roller solves, mid-span v={mid:.6e}");
    Ok(())
}
