//! Frame façade correctness: statics, load recovery, multi-member transfer,
//! API robustness and scale behaviour.
//!
//! Scope: everything here goes through the **public** `FrameModel` API. Where a
//! closed-form answer exists it is *derived from statics / Euler–Bernoulli
//! theory inside this file* (the independent `horizontal_k_u` stiffness product
//! and the hand-written equivalent-nodal-load vectors) - never copied from the
//! implementation - so a sign error cannot be hidden by adjusting an
//! expectation.
//!
//! Frozen conventions (`docs/beam_fem.md`, `docs/frame_analysis.md`):
//!
//! ```text
//! DOFs              [ux, uy, rz] per node (global), rz counter-clockwise positive
//! nodal loads       GLOBAL  (FrameModel::nodal_load / nodal_moment)
//! member loads      LOCAL   (FrameModel::member_udl / member_point_load)
//! member end forces f_end = f_equiv - K_e u_e   (element-on-node, LOCAL axes)
//! reactions         R = K_original u - f_global (global)
//! constraints       static condensation K_ff u_f = f_f - K_fc u_c (no penalties)
//! ```
//!
//! Element end forces are **element-on-node** forces: at a fully fixed,
//! unloaded node `f_end = -R`, at a free node with no applied load
//! `f_end = 0`, and at an interior joint `Σ f_end + f_applied = Σ f_equiv`.
//! These identities are used below to check internal-force transfer without
//! re-deriving the assembly.

use section_properties::frame::{FrameAnalysisResult, FrameModel, MemberHandle, NodeHandle};
use section_properties::{BeamSection, Dof, FemError, Material, SolverSelection};

// ---------------------------------------------------------------------------
// Reference beam properties (identical to `tests/frame_api.rs`)
// ---------------------------------------------------------------------------

const E: f64 = 200e9;
const A: f64 = 5e-3;
const I: f64 = 2e-5;
const EA: f64 = E * A;
const EI: f64 = E * I;

/// Relative tolerance used by [`assert_rel`] and the equilibrium helper.
const REL: f64 = 1e-9;

fn steel() -> Material {
    Material::new(E, 0.3, 7850.0, "Steel")
}

fn sec() -> BeamSection {
    BeamSection::new(A, I)
}

/// `|actual - expected| <= REL · scale`.
///
/// `scale` is the characteristic magnitude of the quantity (e.g. the load
/// magnitude, or the magnitude of a non-zero closed-form value). A **relative**
/// bound is used - there is no absolute epsilon - so the same assertions hold
/// at every scale exercised by the scale tests below.
fn assert_rel(actual: f64, expected: f64, scale: f64, label: &str) {
    assert_rel_with(actual, expected, scale, REL, label);
}

/// [`assert_rel`] with an explicit relative bound (used where the achievable
/// accuracy is conditioning-limited rather than round-off limited).
fn assert_rel_with(actual: f64, expected: f64, scale: f64, rel: f64, label: &str) {
    assert!(actual.is_finite(), "{label}: non-finite value {actual}");
    assert!(
        expected.is_finite(),
        "{label}: non-finite expectation {expected}"
    );
    assert!(
        rel.is_finite() && rel >= 0.0,
        "{label}: invalid bound {rel}"
    );
    let bound = rel * scale;
    assert!(
        (actual - expected).abs() <= bound,
        "{label}: {actual} vs {expected} (|d| = {:.3e} > {:.3e})",
        (actual - expected).abs(),
        bound
    );
}

/// Global equilibrium of a solved frame: `ΣF = 0` and `ΣM = 0` about the global
/// origin, with a **relative** tolerance.
///
/// `l_char` is a characteristic length used to scale the moment residual
/// (moment = force × length). Every load type is counted exactly once: the
/// `EquilibriumReport` already folds nodal loads, applied moments and the true
/// resultants of member distributed/point loads into `applied_*`, and the
/// support reactions into `reaction_*`. Member end forces are internal and are
/// deliberately **not** part of this check.
fn assert_global_equilibrium(r: &FrameAnalysisResult, l_char: f64, label: &str) {
    let e = r.equilibrium();
    let f_raw = e.applied_fx.abs() + e.applied_fy.abs() + e.reaction_fx.abs() + e.reaction_fy.abs();
    let m_raw = e.applied_mz.abs() + e.reaction_mz.abs();
    // A moment contributes an equivalent force scale `M / l_char`, and a force
    // contributes an equivalent moment scale `F · l_char`, so a pure-moment or a
    // pure-force case both have a positive scale on every component.
    let f_scale = f_raw + m_raw / l_char;
    let m_scale = m_raw + f_raw * l_char;

    for (name, v) in [
        ("applied_fx", e.applied_fx),
        ("applied_fy", e.applied_fy),
        ("applied_mz", e.applied_mz),
        ("reaction_fx", e.reaction_fx),
        ("reaction_fy", e.reaction_fy),
        ("reaction_mz", e.reaction_mz),
        ("fx_residual", e.fx_residual),
        ("fy_residual", e.fy_residual),
        ("mz_residual", e.mz_residual),
    ] {
        assert!(v.is_finite(), "{label}: {name} is not finite ({v})");
    }

    assert!(
        f_scale > 0.0 && m_scale > 0.0,
        "{label}: degenerate equilibrium check (no load and no reaction)"
    );
    assert!(
        e.fx_residual.abs() <= REL * f_scale,
        "{label}: ΣFx residual {:.3e} exceeds {:.3e} (scale {:.3e})",
        e.fx_residual,
        REL * f_scale,
        f_scale
    );
    assert!(
        e.fy_residual.abs() <= REL * f_scale,
        "{label}: ΣFy residual {:.3e} exceeds {:.3e} (scale {:.3e})",
        e.fy_residual,
        REL * f_scale,
        f_scale
    );
    assert!(
        e.mz_residual.abs() <= REL * m_scale,
        "{label}: ΣMz residual {:.3e} exceeds {:.3e} (scale {:.3e})",
        e.mz_residual,
        REL * m_scale,
        m_scale
    );
}

// ---------------------------------------------------------------------------
// Independent closed-form helpers (statics / Euler–Bernoulli)
// ---------------------------------------------------------------------------

/// Local DOF vector `[u_i, v_i, θ_i, u_j, v_j, θ_j]` read from the public result.
fn local_u(r: &FrameAnalysisResult, ni: NodeHandle, nj: NodeHandle) -> Result<[f64; 6], FemError> {
    Ok([
        r.displacement(ni, Dof::Ux)?,
        r.displacement(ni, Dof::Uy)?,
        r.displacement(ni, Dof::Rz)?,
        r.displacement(nj, Dof::Ux)?,
        r.displacement(nj, Dof::Uy)?,
        r.displacement(nj, Dof::Rz)?,
    ])
}

/// `K_local · u_local` for a **horizontal** Euler–Bernoulli element, written out
/// from the standard 6×6 beam stiffness matrix. Independent of the crate.
fn horizontal_k_u(ea: f64, ei: f64, l: f64, u: [f64; 6]) -> [f64; 6] {
    let eal = ea / l;
    let a = 12.0 * ei / l.powi(3);
    let b = 6.0 * ei / l.powi(2);
    let c = 4.0 * ei / l;
    let d = 2.0 * ei / l;
    [
        eal * (u[0] - u[3]),
        a * u[1] + b * u[2] - a * u[4] + b * u[5],
        b * u[1] + c * u[2] - b * u[4] + d * u[5],
        eal * (u[3] - u[0]),
        -a * u[1] - b * u[2] + a * u[4] - b * u[5],
        b * u[1] + d * u[2] - b * u[4] + c * u[5],
    ]
}

/// Consistent nodal load vector of a uniform distributed load on a horizontal
/// element, in LOCAL axes: `[0, qyL/2, qyL²/12, 0, qyL/2, -qyL²/12]` for the
/// transverse part and `[qxL/2, 0, 0, qxL/2, 0, 0]` for the axial part.
fn equivalent_udl(qx: f64, qy: f64, l: f64) -> [f64; 6] {
    [
        qx * l / 2.0,
        qy * l / 2.0,
        qy * l * l / 12.0,
        qx * l / 2.0,
        qy * l / 2.0,
        -qy * l * l / 12.0,
    ]
}

/// Build a single-member cantilever `(frame, base, tip, member)`, fixed at the
/// base, free at the tip.
fn single_cantilever(
    l: f64,
) -> Result<(FrameModel, NodeHandle, NodeHandle, MemberHandle), FemError> {
    let mut f = FrameModel::new();
    let base = f.add_node(0.0, 0.0)?;
    let tip = f.add_node(l, 0.0)?;
    let m = f.add_member(base, tip, steel(), sec())?;
    f.fix(base)?;
    Ok((f, base, tip, m))
}

/// Compare the recovered member end forces with `f_end = f_equiv - K_e u_e`,
/// where `f_equiv` and `K_e u_e` are computed independently here.
// The explicit arguments (result, member, both node handles, length, equivalent
// load vector, force scale, label) are each part of the assertion context;
// bundling them into a struct would only add indirection at the call sites.
#[allow(clippy::too_many_arguments)]
fn assert_end_force_principle(
    r: &FrameAnalysisResult,
    m: MemberHandle,
    ni: NodeHandle,
    nj: NodeHandle,
    l: f64,
    f_equiv: [f64; 6],
    force_scale: f64,
    label: &str,
) -> Result<(), FemError> {
    let u = local_u(r, ni, nj)?;
    let ku = horizontal_k_u(EA, EI, l, u);
    let fe = r.member_end_forces(m)?;
    for k in 0..6 {
        assert_rel(
            fe[k],
            f_equiv[k] - ku[k],
            force_scale,
            &format!("{label}: f_end[{k}] = f_equiv - K_e u_e"),
        );
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Step 4.1 - single-element cantilever, every load type
// ---------------------------------------------------------------------------

#[test]
fn cantilever_tip_transverse_force() -> Result<(), FemError> {
    let (l, p) = (2.0_f64, 1.0e4);
    let (mut f, base, tip, m) = single_cantilever(l)?;
    f.nodal_load(tip, 0.0, -p)?;
    let r = f.solve()?;

    // v = -PL^3/3EI, theta = -PL^2/2EI.
    let v = -p * l.powi(3) / (3.0 * EI);
    let th = -p * l.powi(2) / (2.0 * EI);
    assert_rel(r.displacement(tip, Dof::Ux)?, 0.0, l, "ux = 0");
    assert_rel(r.displacement(tip, Dof::Uy)?, v, v.abs(), "v = -PL^3/3EI");
    assert_rel(
        r.displacement(tip, Dof::Rz)?,
        th,
        th.abs(),
        "theta = -PL^2/2EI",
    );

    // Statics: Ry = +P, Rz = +PL about the base.
    assert_rel(r.reaction(base, Dof::Ux)?, 0.0, p, "Rx = 0");
    assert_rel(r.reaction(base, Dof::Uy)?, p, p, "Ry = +P");
    assert_rel(r.reaction(base, Dof::Rz)?, p * l, p * l, "Rz = +PL");

    assert_end_force_principle(&r, m, base, tip, l, [0.0; 6], p * l, "tip force")?;
    // At the loaded tip the element-on-node force balances the applied load:
    // V_j = +P and, since no moment is applied there, M_j = 0.
    let fe = r.member_end_forces(m)?;
    assert_rel(fe[4], p, p, "V_j = +P");
    assert_rel(fe[5], 0.0, p * l, "M_j = 0 (no applied tip moment)");
    // Support reaction is the negative of the element-on-node force.
    let feg = r.member_end_forces_global(m)?;
    assert_rel(feg[1], -(r.reaction(base, Dof::Uy)?), p, "f_end = -R (Ry)");
    assert_rel(
        feg[2],
        -(r.reaction(base, Dof::Rz)?),
        p * l,
        "f_end = -R (Rz)",
    );
    assert_global_equilibrium(&r, l, "tip transverse force");
    Ok(())
}

#[test]
fn cantilever_tip_axial_force() -> Result<(), FemError> {
    let (l, p) = (2.0_f64, 1.0e4);
    let (mut f, base, tip, m) = single_cantilever(l)?;
    f.nodal_load(tip, p, 0.0)?;
    let r = f.solve()?;

    // u = PL/EA (tension positive), no bending.
    let u = p * l / EA;
    assert_rel(r.displacement(tip, Dof::Ux)?, u, u.abs(), "u = PL/EA");
    assert_rel(r.displacement(tip, Dof::Uy)?, 0.0, u, "uy = 0");
    assert_rel(r.displacement(tip, Dof::Rz)?, 0.0, u / l, "rz = 0");
    assert_rel(r.reaction(base, Dof::Ux)?, -p, p, "Rx = -P");

    // N_i = +P (tension on the node); the free tip balances the applied axial
    // load, so N_j = -P.
    let fe = r.member_end_forces(m)?;
    assert_rel(fe[0], p, p, "N_i = +P");
    assert_rel(fe[3], -p, p, "N_j = -P");
    assert_end_force_principle(&r, m, base, tip, l, [0.0; 6], p, "axial force")?;
    assert_global_equilibrium(&r, l, "tip axial force");
    Ok(())
}

/// Step 4.2 - the tip-moment sign convention, checked against statics only.
///
/// For a cantilever fixed at node 0 with an applied CCW moment `M` at the free
/// node 1:
///
/// * moment equilibrium about node 0 gives `Rz + M = 0  =>  Rz = -M`;
/// * free-end nodal equilibrium gives `M_j + M = 0  =>  M_j = -M`;
/// * the fixed-end element-on-node moment is `M_i = -Rz = +M`.
///
/// The expectation is derived from statics, **not** from the implementation.
#[test]
fn cantilever_tip_moment_sign_convention() -> Result<(), FemError> {
    let (l, m0) = (2.0_f64, 200.0);
    let (mut f, base, tip, m) = single_cantilever(l)?;
    f.nodal_moment(tip, m0)?;
    let r = f.solve()?;

    assert_rel(
        r.displacement(tip, Dof::Rz)?,
        m0 * l / EI,
        m0 * l / EI,
        "theta = ML/EI",
    );
    assert_rel(
        r.displacement(tip, Dof::Uy)?,
        m0 * l * l / (2.0 * EI),
        m0 * l * l / (2.0 * EI),
        "v = ML^2/2EI",
    );
    assert_rel(r.reaction(base, Dof::Uy)?, 0.0, m0, "Ry = 0");
    assert_rel(r.reaction(base, Dof::Rz)?, -m0, m0, "Rz = -M");

    let fe = r.member_end_forces(m)?;
    assert_rel(fe[2], m0, m0, "M_i = +M");
    assert_rel(fe[5], -m0, m0, "M_j = -M");
    assert_rel(fe[1], 0.0, m0, "V_i = 0");
    assert_rel(fe[4], 0.0, m0, "V_j = 0");
    assert_end_force_principle(&r, m, base, tip, l, [0.0; 6], m0, "tip moment")?;
    assert_global_equilibrium(&r, l, "tip moment");
    Ok(())
}

#[test]
fn cantilever_uniform_transverse_load() -> Result<(), FemError> {
    let (l, q) = (2.0_f64, 8.0e3);
    let qy = -q; // downward in local +y
    let (mut f, base, tip, m) = single_cantilever(l)?;
    f.member_udl(m, 0.0, qy)?;
    let r = f.solve()?;

    // v = -qL^4/8EI, theta = -qL^3/6EI.
    let v = -q * l.powi(4) / (8.0 * EI);
    let th = -q * l.powi(3) / (6.0 * EI);
    assert_rel(r.displacement(tip, Dof::Uy)?, v, v.abs(), "v = -qL^4/8EI");
    assert_rel(
        r.displacement(tip, Dof::Rz)?,
        th,
        th.abs(),
        "theta = -qL^3/6EI",
    );

    // Resultant qL at mid-span: Ry = qL, Rz = qL^2/2.
    let (ry, rz) = (q * l, q * l * l / 2.0);
    assert_rel(r.reaction(base, Dof::Uy)?, ry, ry, "Ry = qL");
    assert_rel(r.reaction(base, Dof::Rz)?, rz, rz, "Rz = qL^2/2");
    assert_rel(r.reaction(base, Dof::Ux)?, 0.0, ry, "Rx = 0");

    // Fixed end carries the full restraint force; the free end carries nothing.
    let fe = r.member_end_forces(m)?;
    assert_rel(fe[1], -ry, ry, "V_i = -qL");
    assert_rel(fe[2], -rz, rz, "M_i = -qL^2/2");
    assert_rel(fe[4], 0.0, ry, "V_j = 0 at a free end");
    assert_rel(fe[5], 0.0, rz, "M_j = 0 at a free end");

    // Step 4.3: the recovered forces must equal `f_equiv - K_e u_e` with the
    // distributed-load equivalent vector subtracted exactly once.
    assert_end_force_principle(&r, m, base, tip, l, equivalent_udl(0.0, qy, l), ry, "UDL")?;
    assert_global_equilibrium(&r, l, "uniform transverse load");
    Ok(())
}

#[test]
fn cantilever_uniform_axial_load() -> Result<(), FemError> {
    let (l, q) = (2.0_f64, 8.0e3);
    let qx = q; // tensile, towards node_j
    let (mut f, base, tip, m) = single_cantilever(l)?;
    f.member_udl(m, qx, 0.0)?;
    let r = f.solve()?;

    // Total axial load qL; free tip: u = qL^2/2EA.
    let u = q * l * l / (2.0 * EA);
    let rx = -q * l;
    assert_rel(r.displacement(tip, Dof::Ux)?, u, u.abs(), "u = qL^2/2EA");
    assert_rel(r.displacement(tip, Dof::Uy)?, 0.0, u, "uy = 0");
    assert_rel(r.reaction(base, Dof::Ux)?, rx, rx.abs(), "Rx = -qL");

    // N_i = +qL (tension), N_j = 0 at the free end.
    let fe = r.member_end_forces(m)?;
    assert_rel(fe[0], -rx, rx.abs(), "N_i = +qL");
    assert_rel(fe[3], 0.0, rx.abs(), "N_j = 0 at a free end");

    assert_end_force_principle(
        &r,
        m,
        base,
        tip,
        l,
        equivalent_udl(qx, 0.0, l),
        q * l,
        "axial UDL",
    )?;
    assert_global_equilibrium(&r, l, "uniform axial load");
    Ok(())
}

/// Superposition: tip force + tip moment + uniform transverse load.
#[test]
fn cantilever_combined_loading() -> Result<(), FemError> {
    let (l, p, m0, q) = (2.0_f64, 1.0e4, 8.0e3, 8.0e3);
    let qy = -q;
    let (mut f, base, tip, m) = single_cantilever(l)?;
    f.nodal_load(tip, 0.0, -p)?;
    f.nodal_moment(tip, m0)?;
    f.member_udl(m, 0.0, qy)?;
    let r = f.solve()?;

    // Linear superposition of the three closed-form contributions: a tip force
    // gives -PL³/3EI, a uniform load -qL⁴/8EI, a tip moment +ML²/2EI.
    let v = -p * l.powi(3) / (3.0 * EI) - q * l.powi(4) / (8.0 * EI) + m0 * l * l / (2.0 * EI);
    let th = -p * l.powi(2) / (2.0 * EI) - q * l.powi(3) / (6.0 * EI) + m0 * l / EI;
    assert_rel(r.displacement(tip, Dof::Uy)?, v, v.abs(), "v (superposed)");
    assert_rel(
        r.displacement(tip, Dof::Rz)?,
        th,
        th.abs(),
        "theta (superposed)",
    );

    let ry = p + q * l;
    let rz = p * l + q * l * l / 2.0 - m0;
    assert_rel(r.reaction(base, Dof::Uy)?, ry, ry, "Ry (superposed)");
    assert_rel(r.reaction(base, Dof::Rz)?, rz, rz, "Rz (superposed)");

    // f_end = (tip force) + (UDL) + (tip moment); the applied moment only
    // contributes on the rotational DOFs and is never part of f_equiv.
    let fe = r.member_end_forces(m)?;
    let scale = ry * l;
    assert_rel(fe[1], -(p + q * l), scale, "V_i");
    assert_rel(fe[2], -(p * l + q * l * l / 2.0) + m0, scale, "M_i");
    assert_rel(fe[4], p, scale, "V_j");
    assert_rel(fe[5], -m0, scale, "M_j = -M (moment not double-counted)");

    // Independent recovery using the UDL equivalent vector only.
    assert_end_force_principle(
        &r,
        m,
        base,
        tip,
        l,
        equivalent_udl(0.0, qy, l),
        scale,
        "combined",
    )?;
    assert_global_equilibrium(&r, l, "combined loading");
    Ok(())
}

// ---------------------------------------------------------------------------
// Step 4.3 - distributed-load recovery on a multi-element model
// ---------------------------------------------------------------------------

/// With a distributed load present, end forces must subtract the equivalent
/// nodal load exactly once: `f_end = f_equiv - K_e u_e`.
///
/// Checked on two unequal elements so the recovery is not accidentally correct
/// because of symmetry or equal element lengths.
#[test]
fn multi_element_distributed_load_recovery() -> Result<(), FemError> {
    let (l1, l2, q) = (0.8_f64, 1.2, 5.0e3);
    let qy = -q;
    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(l1, 0.0)?;
    let n2 = f.add_node(l1 + l2, 0.0)?;
    let m0 = f.add_member(n0, n1, steel(), sec())?;
    let m1 = f.add_member(n1, n2, steel(), sec())?;
    f.fix(n0)?;
    f.member_udl(m0, 0.0, qy)?; // loaded element only
    let r = f.solve()?;

    // Element 0 recovery from first principles.
    assert_end_force_principle(
        &r,
        m0,
        n0,
        n1,
        l1,
        equivalent_udl(0.0, qy, l1),
        q * l1,
        "elem0",
    )?;
    // Element 1 carries no load: f_end = -K u.
    assert_end_force_principle(&r, m1, n1, n2, l2, [0.0; 6], q * l1, "elem1")?;

    // Interior joint equilibrium: there is no nodal load at node 1, so the two
    // elements' element-on-node forces must cancel there. The distributed load
    // is internal to element 0 and never appears as an external nodal load -
    // exactly what `f_end = f_equiv - K_e u_e` ensures (the equivalent load
    // cancels against the assembled equivalent load).
    let fe0 = r.member_end_forces(m0)?;
    let fe1 = r.member_end_forces(m1)?;
    let scale = q * l1;
    for k in 0..3 {
        assert_rel(
            fe0[3 + k] + fe1[k],
            0.0,
            scale,
            &format!("joint internal transfer [{k}]"),
        );
    }

    // Reactions from the resultant of the applied load (about the base).
    let ry = q * l1;
    let rz = q * l1 * (l1 / 2.0);
    assert_rel(r.reaction(n0, Dof::Uy)?, ry, ry, "Ry = q·L1");
    assert_rel(r.reaction(n0, Dof::Rz)?, rz, rz, "Rz = q·L1·(L1/2)");
    assert_global_equilibrium(&r, l1 + l2, "loaded two-element beam");
    Ok(())
}

// ---------------------------------------------------------------------------
// Step 6 - multi-element frames: continuity and internal force transfer
// ---------------------------------------------------------------------------

/// `node 0 ─ node 1 ─ node 2`, fixed at 0, tip force at 2: shared-node
/// continuity, internal transfer and reactions against beam theory.
#[test]
fn two_member_straight_beam_continuity() -> Result<(), FemError> {
    let (l1, l2, p) = (0.8_f64, 1.2, 1.0e4);
    let l = l1 + l2;
    let mut f = FrameModel::new();
    let n0 = f.add_node(0.0, 0.0)?;
    let n1 = f.add_node(l1, 0.0)?;
    let n2 = f.add_node(l1 + l2, 0.0)?;
    let m0 = f.add_member(n0, n1, steel(), sec())?;
    let m1 = f.add_member(n1, n2, steel(), sec())?;
    f.fix(n0)?;
    f.nodal_load(n2, 0.0, -p)?;
    let r = f.solve()?;

    // Cantilever deflection/rotation at x = l1: v = -Px²(3L-x)/6EI,
    // theta = -Px(2L-x)/2EI. The shared node has a single displacement/rotation,
    // so matching this value proves continuity across the joint.
    let x = l1;
    let v = -p * x * x * (3.0 * l - x) / (6.0 * EI);
    let th = -p * x * (2.0 * l - x) / (2.0 * EI);
    assert_rel(r.displacement(n1, Dof::Uy)?, v, v.abs(), "v(l1)");
    assert_rel(r.displacement(n1, Dof::Rz)?, th, th.abs(), "theta(l1)");
    assert_rel(
        r.displacement(n2, Dof::Uy)?,
        -p * l.powi(3) / (3.0 * EI),
        p * l.powi(3) / (3.0 * EI),
        "v(L)",
    );

    // Interior joint: no load at node 1, so the two elements' end forces are
    // equal and opposite there (internal bending/shear/axial transfer).
    let fe0 = r.member_end_forces(m0)?;
    let fe1 = r.member_end_forces(m1)?;
    let scale = p * l;
    for k in 0..3 {
        assert_rel(fe0[3 + k], -fe1[k], scale, &format!("joint transfer [{k}]"));
    }
    // The internal bending moment is continuous across the joint.
    assert_rel(fe0[5], -fe1[2], scale, "moment continuity");
    // Global end forces transfer identically (horizontal members).
    let g0 = r.member_end_forces_global(m0)?;
    let g1 = r.member_end_forces_global(m1)?;
    for k in 0..3 {
        assert_rel(
            g0[3 + k],
            -g1[k],
            scale,
            &format!("global joint transfer [{k}]"),
        );
    }

    assert_rel(r.reaction(n0, Dof::Uy)?, p, p, "Ry = +P");
    assert_rel(r.reaction(n0, Dof::Rz)?, p * l, p * l, "Rz = +PL");
    assert_global_equilibrium(&r, l, "two-member straight beam");
    Ok(())
}

/// Portal frame: element-to-element transfer at both beam/column joints, and
/// base reactions equal to the negative of the base element-on-node forces.
#[test]
fn portal_frame_joint_transfer() -> Result<(), FemError> {
    let (h, w, fx) = (2.0_f64, 3.0, 3.0e4);
    let mut f = FrameModel::new();
    let base_l = f.add_node(0.0, 0.0)?;
    let top_l = f.add_node(0.0, h)?;
    let top_r = f.add_node(w, h)?;
    let base_r = f.add_node(w, 0.0)?;
    let col_l = f.add_member(base_l, top_l, steel(), sec())?;
    let beam = f.add_member(top_l, top_r, steel(), sec())?;
    let col_r = f.add_member(top_r, base_r, steel(), sec())?;
    f.fix(base_l)?;
    f.fix(base_r)?;
    f.nodal_load(top_l, fx, 0.0)?;
    let r = f.solve()?;

    let scale = fx * (h + w);
    let cl = r.member_end_forces_global(col_l)?;
    let bm = r.member_end_forces_global(beam)?;
    let cr = r.member_end_forces_global(col_r)?;

    // Joint (top-left): column node_j + beam node_i + applied nodal load = 0.
    for k in 0..3 {
        let applied = if k == 0 { fx } else { 0.0 };
        assert_rel(
            cl[3 + k] + bm[k] + applied,
            0.0,
            scale,
            &format!("top-left joint [{k}]"),
        );
    }
    // Joint (top-right): no applied load there.
    for k in 0..3 {
        assert_rel(
            bm[3 + k] + cr[k],
            0.0,
            scale,
            &format!("top-right joint [{k}]"),
        );
    }
    // Fully fixed, unloaded bases: f_end = -R.
    for k in 0..3 {
        let dof = Dof::ALL[k];
        assert_rel(
            cl[k],
            -r.reaction(base_l, dof)?,
            scale,
            &format!("base-left f_end = -R [{k}]"),
        );
        assert_rel(
            cr[3 + k],
            -r.reaction(base_r, dof)?,
            scale,
            &format!("base-right f_end = -R [{k}]"),
        );
    }

    assert_rel(
        r.reaction(base_l, Dof::Ux)? + r.reaction(base_r, Dof::Ux)?,
        -fx,
        fx,
        "ΣRx = -F",
    );
    assert_global_equilibrium(&r, h + w, "portal frame");
    Ok(())
}

// ---------------------------------------------------------------------------
// Step 7 - API robustness
// ---------------------------------------------------------------------------

#[test]
fn robustness_duplicate_node_coordinates_are_distinct_nodes() -> Result<(), FemError> {
    // There is no user-facing "node id": handles are positional, so two nodes
    // at the same coordinates are two distinct nodes with distinct handles.
    let mut f = FrameModel::new();
    let a = f.add_node(1.0, 2.0)?;
    let b = f.add_node(1.0, 2.0)?;
    assert_ne!(a.index(), b.index(), "coincident nodes must be distinct");
    assert_eq!(f.n_nodes(), 2);
    // A member between them is zero-length and is rejected.
    assert!(matches!(
        f.add_member(a, b, steel(), sec()),
        Err(FemError::ZeroLengthMember(_))
    ));
    assert_eq!(f.n_members(), 0, "a rejected member must not be added");
    Ok(())
}

#[test]
fn robustness_member_referencing_unknown_node() -> Result<(), FemError> {
    let mut f = FrameModel::new();
    let a = f.add_node(0.0, 0.0)?;
    let b = f.add_node(1.0, 0.0)?;
    let ghost = NodeHandle::from_index(99);
    assert!(matches!(
        f.add_member(a, ghost, steel(), sec()),
        Err(FemError::InvalidNode(_))
    ));
    assert!(matches!(
        f.add_member(ghost, b, steel(), sec()),
        Err(FemError::InvalidNode(_))
    ));
    assert_eq!(f.n_members(), 0, "a rejected member must not be added");
    Ok(())
}

#[test]
fn robustness_load_and_bc_reference_validation() -> Result<(), FemError> {
    let mut f = FrameModel::new();
    let a = f.add_node(0.0, 0.0)?;
    let b = f.add_node(1.0, 0.0)?;
    f.add_member(a, b, steel(), sec())?;
    f.fix(a)?;
    let ghost_node = NodeHandle::from_index(42);
    let ghost_member = MemberHandle::from_index(42);

    assert!(matches!(
        f.nodal_moment(ghost_node, 1.0),
        Err(FemError::InvalidNode(_))
    ));
    assert!(matches!(
        f.member_udl(ghost_member, 0.0, 0.0),
        Err(FemError::InvalidMember(_))
    ));
    assert!(matches!(
        f.member_point_load(ghost_member, 0.5, 0.0, 0.0, 0.0),
        Err(FemError::InvalidMember(_))
    ));
    assert!(matches!(f.pin(ghost_node), Err(FemError::InvalidNode(_))));
    assert!(matches!(
        f.roller_y(ghost_node),
        Err(FemError::InvalidNode(_))
    ));
    assert!(matches!(
        f.roller_x(ghost_node),
        Err(FemError::InvalidNode(_))
    ));
    // Non-finite prescribed displacement is rejected, not stored.
    assert!(matches!(
        f.restrain(b, Dof::Uy, f64::NAN),
        Err(FemError::InvalidInput(_))
    ));
    assert!(matches!(
        f.restrain(b, Dof::Uy, f64::INFINITY),
        Err(FemError::InvalidInput(_))
    ));
    Ok(())
}

/// A rejected `nodal_load` must not leave a half-applied load behind.
#[test]
fn robustness_nodal_load_is_all_or_nothing() -> Result<(), FemError> {
    let (mut f, base, tip, _m) = single_cantilever(2.0)?;
    // Fy is rejected; Fx must not have been recorded.
    assert!(matches!(
        f.nodal_load(tip, 1.0e4, f64::NAN),
        Err(FemError::InvalidInput(_))
    ));
    let r = f.solve()?;
    let e = r.equilibrium();
    assert_eq!(
        e.applied_fx, 0.0,
        "rejected load must not be partially applied"
    );
    assert_eq!(e.applied_fy, 0.0, "no vertical load was requested");
    assert_rel(r.reaction(base, Dof::Ux)?, 0.0, 1.0, "Rx = 0");
    Ok(())
}

#[test]
fn robustness_unrestrained_and_empty_models_error() -> Result<(), FemError> {
    // No boundary conditions at all: the assembled system is a mechanism and
    // must surface a solver error, never panic or return a silent zero solve.
    let mut f = FrameModel::new();
    let a = f.add_node(0.0, 0.0)?;
    let b = f.add_node(2.0, 0.0)?;
    f.add_member(a, b, steel(), sec())?;
    let err = f.solve().expect_err("an unsupported frame must not solve");
    assert!(matches!(err, FemError::SolverError(_)), "got {err:?}");

    // Nodes but no members.
    let mut g = FrameModel::new();
    g.add_node(0.0, 0.0)?;
    assert!(matches!(g.solve(), Err(FemError::InvalidModel(_))));

    // Completely empty.
    let h = FrameModel::new();
    assert!(matches!(h.solve(), Err(FemError::InvalidModel(_))));
    Ok(())
}

// ---------------------------------------------------------------------------
// Step 8 - scale behaviour (load, stiffness, coordinate)
// ---------------------------------------------------------------------------

const SCALES: [f64; 5] = [1e-12, 1e-6, 1.0, 1e6, 1e12];

/// Scale the **load** by `s`: the solution is exactly linear in the load, so the
/// relative error must be scale-independent.
#[test]
fn scale_load_sweep() -> Result<(), FemError> {
    let l = 2.0_f64;
    let p0 = 1.0e4;
    for s in SCALES {
        let p = p0 * s;
        let (mut f, base, tip, _m) = single_cantilever(l)?;
        f.nodal_load(tip, 0.0, -p)?;
        let r = f.solve()?;

        let v = -p * l.powi(3) / (3.0 * EI);
        let ry = p;
        let rz = p * l;
        assert!(
            r.displacement(tip, Dof::Uy)?.is_finite(),
            "scale {s:e}: non-finite displacement"
        );
        assert_rel(
            r.displacement(tip, Dof::Uy)?,
            v,
            v.abs(),
            &format!("load scale {s:e}: v"),
        );
        assert_rel(
            r.displacement(tip, Dof::Rz)?,
            -p * l * l / (2.0 * EI),
            p * l * l / (2.0 * EI),
            &format!("load scale {s:e}: theta"),
        );
        assert_rel(
            r.reaction(base, Dof::Uy)?,
            ry,
            ry,
            &format!("load scale {s:e}: Ry"),
        );
        assert_rel(
            r.reaction(base, Dof::Rz)?,
            rz,
            rz,
            &format!("load scale {s:e}: Rz"),
        );
        assert_global_equilibrium(&r, l, &format!("load scale {s:e}"));
    }
    Ok(())
}

/// Scale the **stiffness** (`E`) by `s`: the deflection scales as `1/s`, the
/// reactions are unchanged (statics). The global stiffness matrix is multiplied
/// by a constant, so its conditioning is unchanged.
#[test]
fn scale_stiffness_sweep() -> Result<(), FemError> {
    let l = 2.0_f64;
    let p = 1.0e4;
    for s in SCALES {
        let ei = EI * s;
        let mut f = FrameModel::new();
        let base = f.add_node(0.0, 0.0)?;
        let tip = f.add_node(l, 0.0)?;
        let mat = Material::new(E * s, 0.3, 7850.0, "Steel");
        f.add_member(base, tip, mat, sec())?;
        f.fix(base)?;
        f.nodal_load(tip, 0.0, -p)?;
        let r = f.solve()?;

        let v = -p * l.powi(3) / (3.0 * ei);
        assert_rel(
            r.displacement(tip, Dof::Uy)?,
            v,
            v.abs(),
            &format!("stiffness scale {s:e}: v"),
        );
        assert_rel(
            r.reaction(base, Dof::Uy)?,
            p,
            p,
            &format!("stiffness scale {s:e}: Ry"),
        );
        assert_rel(
            r.reaction(base, Dof::Rz)?,
            p * l,
            p * l,
            &format!("stiffness scale {s:e}: Rz"),
        );
        assert_global_equilibrium(&r, l, &format!("stiffness scale {s:e}"));
    }
    Ok(())
}

/// Scale the **coordinates** by `s` with the load and section fixed.
///
/// `v = -PL³/3EI` therefore scales as `s³`. The Euler–Bernoulli element
/// stiffness is *not* invariant under a geometric scale in floating point: after
/// factoring out `EI/L³`, the reduced transverse system is `[[12, -6L],
/// [-6L, 4L²]]`, whose condition number is `max(12/L², 4L²/3)` - it degrades for
/// both very small and very large `L`. (Scaling the section does not help: the
/// `L²` ratio is independent of `E`, `A`, `I`.) That is a documented conditioning
/// boundary of the element formulation, not an absolute tolerance introduced by
/// the frame layer.
///
/// The frame layer must therefore:
/// 1. accept the geometry at every scale - no absolute coordinate/length check
///    may spuriously reject a valid member;
/// 2. never panic and never emit NaN/Inf;
/// 3. either return a result accurate to the **conditioning-limited** relative
///    bound `64 · κ · ε` (a relative bound, computed below), or report a clean
///    `FemError::SolverError` - never a silently wrong answer.
///
/// Oversized/small coordinate *magnitudes* (as opposed to member lengths) are
/// covered separately by [`coordinate_translation_invariance`], which stays
/// well-conditioned at every scale because the stiffness depends only on
/// coordinate *differences*.
#[test]
fn scale_coordinate_sweep() -> Result<(), FemError> {
    let l0 = 2.0_f64;
    let p = 1.0e4;
    for s in SCALES {
        let l = l0 * s;

        // (1) Construction must never be scale-rejected.
        let (mut f, base, tip, _m) = single_cantilever(l)?;
        f.nodal_load(tip, 0.0, -p)?;

        let v = -p * l.powi(3) / (3.0 * EI);
        let th = -p * l * l / (2.0 * EI);

        match f.solve() {
            Ok(r) => {
                // (2) Finite everywhere.
                for (name, val) in [
                    ("ux", r.displacement(tip, Dof::Ux)?),
                    ("uy", r.displacement(tip, Dof::Uy)?),
                    ("rz", r.displacement(tip, Dof::Rz)?),
                    ("Rx", r.reaction(base, Dof::Ux)?),
                    ("Ry", r.reaction(base, Dof::Uy)?),
                    ("Rz", r.reaction(base, Dof::Rz)?),
                ] {
                    assert!(
                        val.is_finite(),
                        "coordinate scale {s:e}: {name} is not finite ({val})"
                    );
                }
                // (3a) Accuracy against the conditioning-limited forward-error
                // bound of a direct solve: |du|/|u| = O(kappa*eps). The factor 64
                // is a safety margin; the bound is still *relative* (there is no
                // absolute epsilon) and stays far tighter than any sign/logic
                // error would produce.
                let kappa = (12.0 / (l * l)).max(4.0 * l * l / 3.0);
                let bound = REL.max(64.0 * kappa * f64::EPSILON);
                assert!(
                    bound.is_finite(),
                    "coordinate scale {s:e}: non-finite accuracy bound"
                );
                assert_rel_with(
                    r.displacement(tip, Dof::Uy)?,
                    v,
                    v.abs(),
                    bound,
                    &format!("coord scale {s:e}: v"),
                );
                assert_rel_with(
                    r.displacement(tip, Dof::Rz)?,
                    th,
                    th.abs(),
                    bound,
                    &format!("coord scale {s:e}: theta"),
                );
                assert_rel_with(
                    r.reaction(base, Dof::Uy)?,
                    p,
                    p,
                    bound,
                    &format!("coord scale {s:e}: Ry"),
                );
                assert_rel_with(
                    r.reaction(base, Dof::Rz)?,
                    p * l,
                    p * l,
                    bound,
                    &format!("coord scale {s:e}: Rz"),
                );
                assert_global_equilibrium(&r, l, &format!("coord scale {s:e}"));
            }
            // (3b) Outside the resolvable window the only acceptable outcome is
            // a clean solver error - the Frame layer must never turn this into a
            // zero/default (the standing error-handling rule).
            Err(e) => {
                assert!(
                    matches!(e, FemError::SolverError(_)),
                    "coordinate scale {s:e}: expected a clean SolverError, got {e:?}"
                );
            }
        }
    }
    Ok(())
}

/// Rigid translation by `(s, -s)`: moving the whole frame in space must not
/// change any structural result (the element stiffness depends only on
/// coordinate differences), at every coordinate magnitude.
///
/// This exercises large coordinate magnitudes - and hence large lever arms in
/// the moment sum about the origin - without changing the member lengths, so it
/// stays well-conditioned where the length-scaling sweep cannot.
#[test]
fn coordinate_translation_invariance() -> Result<(), FemError> {
    let (l, p) = (2.0_f64, 1.0e4);
    let mut reference: Option<(f64, f64, f64, f64)> = None;

    for s in SCALES {
        let (ox, oy) = (s, -s);
        let mut f = FrameModel::new();
        let base = f.add_node(ox, oy)?;
        let tip = f.add_node(ox + l, oy)?;
        f.add_member(base, tip, steel(), sec())?;
        f.fix(base)?;
        f.nodal_load(tip, 0.0, -p)?;
        let r = f.solve()?;

        let current = (
            r.displacement(tip, Dof::Uy)?,
            r.displacement(tip, Dof::Rz)?,
            r.reaction(base, Dof::Uy)?,
            r.reaction(base, Dof::Rz)?,
        );
        for v in [current.0, current.1, current.2, current.3] {
            assert!(v.is_finite(), "translation {s:e}: non-finite result {v}");
        }

        match reference {
            None => reference = Some(current),
            Some((uy0, rz0, ry0, mm0)) => {
                let scale = uy0.abs().max(ry0.abs()) * l;
                assert_rel(current.0, uy0, scale, &format!("translation {s:e}: uy"));
                assert_rel(current.1, rz0, scale, &format!("translation {s:e}: rz"));
                assert_rel(current.2, ry0, ry0.abs(), &format!("translation {s:e}: Ry"));
                assert_rel(current.3, mm0, mm0.abs(), &format!("translation {s:e}: Rz"));
            }
        }
        assert_global_equilibrium(&r, l + 2.0 * s.abs(), &format!("translation {s:e}"));
    }
    Ok(())
}

/// No solver-selection regression across scales: `Auto` and an explicit direct
/// backend must agree at every scale, and both must actually report a backend.
#[test]
fn scale_solver_selection_stable() -> Result<(), FemError> {
    let l = 2.0_f64;
    for s in SCALES {
        let p = 1.0e4 * s;
        let (mut f, _base, tip, _m) = single_cantilever(l)?;
        f.nodal_load(tip, 0.0, -p)?;

        let auto = f.solve()?;
        assert!(
            auto.solver_name().is_some(),
            "scale {s:e}: auto reported no backend"
        );
        let dense = f.solve_with(SolverSelection::named("dense"))?;
        assert_eq!(dense.solver_name(), Some("dense"));

        let ua = auto.displacement(tip, Dof::Uy)?;
        let ud = dense.displacement(tip, Dof::Uy)?;
        assert_rel(ud, ua, ua.abs(), &format!("scale {s:e}: dense vs auto"));
    }
    Ok(())
}
