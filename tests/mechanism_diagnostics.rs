//! Phase 15 - structural mechanism diagnostics for the 2D frame façade.
//!
//! Scope: the **public** diagnostic surface (`FrameModel::diagnostic`,
//! `fea::mechanism`) and the solver error it enriches. Nothing here changes a
//! solver path; the point of this file is to prove the classifier does not
//! mistake a stable structure for a mechanism (and vice versa) and that it does
//! not disturb the existing solve results.
//!
//! Frozen conventions (see `docs/frame_analysis.md`, `docs/beam_fem.md`):
//!
//! ```text
//! DOFs              [ux, uy, rz] per node (global)
//! constraints       static condensation K_ff u_f = f_f - K_fc u_c (no penalties)
//! diagnostic        ranks the REDUCED (free-free) system K_ff, scale-invariantly
//! ```
//!
//! `K_ff` is symmetric positive semi-definite, so `vᵀK_ff v = 0` for `v != 0`
//! means the vector (padded with zeros on the constrained DOFs) is a rigid-body
//! motion of the whole structure. For this element type a frame mechanism is
//! therefore always an under-restraint; the internal `Mechanism` verdict is
//! exercised directly on the probe with a synthetic reduced system.

use section_properties::fea::SparseMatrix;
use section_properties::fea::mechanism::{
    DiagnosticLimit, MAX_DENSE_PROBE_DOF, StructuralDiagnostic, diagnose_reduced,
};
use section_properties::frame::{FrameModel, MemberHandle, NodeHandle};
use section_properties::{BeamSection, Dof, FemError, Material, SolverCapabilities};

// ---------------------------------------------------------------------------
// Reference beam properties (identical to `tests/frame_api.rs`)
// ---------------------------------------------------------------------------

const E: f64 = 200e9;
const A: f64 = 5e-3;
const I: f64 = 2e-5;
const EI: f64 = E * I;

/// Relative tolerance for the analytic closed-form checks.
const REL: f64 = 1e-9;

fn steel() -> Material {
    Material::new(E, 0.3, 7850.0, "Steel")
}

fn sec() -> BeamSection {
    BeamSection::new(A, I)
}

/// `|actual - expected| <= rel * scale` - a purely relative bound, no absolute
/// epsilon, so it holds at every scale used below.
fn assert_rel_with(actual: f64, expected: f64, scale: f64, rel: f64, label: &str) {
    assert!(actual.is_finite(), "{label}: non-finite value {actual}");
    let bound = rel * scale;
    assert!(
        (actual - expected).abs() <= bound,
        "{label}: {actual} vs {expected} (|d| = {:.3e} > {:.3e})",
        (actual - expected).abs(),
        bound
    );
}

fn assert_rel(actual: f64, expected: f64, scale: f64, label: &str) {
    assert_rel_with(actual, expected, scale, REL, label);
}

fn nth(index: usize) -> NodeHandle {
    NodeHandle::from_index(index)
}

/// The single-member reference cantilever used throughout (`base` fixed, tip
/// free) - the "stable" control case.
fn cantilever(len: f64) -> Result<(FrameModel, NodeHandle, NodeHandle), FemError> {
    let mut f = FrameModel::new();
    let base = f.add_node(0.0, 0.0)?;
    let tip = f.add_node(len, 0.0)?;
    f.add_member(base, tip, steel(), sec())?;
    f.fix(base)?;
    Ok((f, base, tip))
}

/// A three-member portal frame (two columns, one beam) with the given supports
/// on the two base nodes - the multi-member structure used for the mechanism
/// and scale tests.
///
/// The `alpha` argument produces a **geometrically similar** structure:
/// coordinates scale by `alpha`, the area by `alpha^2` and the second moment by
/// `alpha^4`. That makes every entry of the stiffness matrix `alpha` times the
/// reference entry, so the equilibrated (unit-diagonal) system is identical at
/// every `alpha` - the strongest possible scale-invariance probe.
fn portal(base_1: &[Dof], base_2: &[Dof], alpha: f64) -> Result<FrameModel, FemError> {
    let mut f = FrameModel::new();
    let b1 = f.add_node(0.0, 0.0)?;
    let t1 = f.add_node(0.0, 3.0 * alpha)?;
    let t2 = f.add_node(4.0 * alpha, 3.0 * alpha)?;
    let b2 = f.add_node(4.0 * alpha, 0.0)?;
    let material = Material::new(E, 0.3, 7850.0, "Steel");
    let section = BeamSection::new(A * alpha * alpha, I * alpha.powi(4));
    f.add_member(b1, t1, material, section)?;
    f.add_member(t1, t2, material, section)?;
    f.add_member(b2, t2, material, section)?;
    for dof in base_1 {
        f.restrain(b1, *dof, 0.0)?;
    }
    for dof in base_2 {
        f.restrain(b2, *dof, 0.0)?;
    }
    Ok(f)
}

// ---------------------------------------------------------------------------
// Test 1 - completely free structure
// ---------------------------------------------------------------------------

#[test]
fn free_structure_is_a_rigid_body_mechanism() -> Result<(), FemError> {
    // node0 -- node1, no boundary conditions at all: the reduced system is the
    // raw 6x6 stiffness with its three rigid-body modes.
    let mut f = FrameModel::new();
    let a = f.add_node(0.0, 0.0)?;
    let b = f.add_node(2.0, 0.0)?;
    f.add_member(a, b, steel(), sec())?;

    assert_eq!(
        f.diagnostic()?,
        StructuralDiagnostic::RigidBodyMode {
            n_free: 6,
            rank: 3,
            rigid_modes: 3,
        },
        "a free beam has exactly the three rigid-body modes and nothing else"
    );

    // Not a valid solution, and no panic / silent zero solve.
    let err = f.solve().expect_err("a free structure must not solve");
    assert!(matches!(err, FemError::SolverError(_)), "got {err:?}");
    // The solver error is no longer opaque: it carries the diagnosis.
    assert!(
        err.to_string().contains("rigid-body"),
        "expected the rigid-body diagnosis in the error, got {err:?}"
    );
    Ok(())
}

// ---------------------------------------------------------------------------
// Test 2 - one rigid-body mode removed, two left
// ---------------------------------------------------------------------------

#[test]
fn under_constrained_structure_leaves_rigid_body_modes() -> Result<(), FemError> {
    // Only `ux` of the base is restrained: translation in y and rotation about
    // the base remain, so the reduced system keeps a 2-dimensional null space.
    let mut f = FrameModel::new();
    let base = f.add_node(0.0, 0.0)?;
    let tip = f.add_node(2.0, 0.0)?;
    f.add_member(base, tip, steel(), sec())?;
    f.restrain(base, Dof::Ux, 0.0)?;

    assert_eq!(
        f.diagnostic()?,
        StructuralDiagnostic::RigidBodyMode {
            n_free: 5,
            rank: 3,
            rigid_modes: 2,
        },
        "restraining one DOF removes one rigid-body mode, not the deficiency"
    );
    assert!(f.solve().is_err(), "the structure is still a mechanism");
    Ok(())
}

// ---------------------------------------------------------------------------
// Test 3 - stable cantilever: stable, solvable, results unchanged
// ---------------------------------------------------------------------------

/// Bit-exact reference values for [`stable_cantilever_is_stable_and_unchanged`],
/// captured from the implementation **before** the diagnostic was added. The
/// diagnostic does not touch the assembly, condensation, factorisation or
/// recovery path, so these must not move by one ULP.
const CANTILEVER_REF: [(Dof, f64); 3] = [
    (Dof::Ux, 0.0),
    (Dof::Uy, -0.006666666666666665),
    (Dof::Rz, -0.005),
];
const CANTILEVER_REACTION_REF: [(Dof, f64); 3] = [
    (Dof::Ux, 0.0),
    (Dof::Uy, 9999.999999999996),
    (Dof::Rz, 19999.999999999996),
];

#[test]
fn stable_cantilever_is_stable_and_unchanged() -> Result<(), FemError> {
    let (l, p) = (2.0, 1.0e4);
    let (mut f, base, tip) = cantilever(l)?;
    f.nodal_load(tip, 0.0, -p)?;

    assert_eq!(
        f.diagnostic()?,
        StructuralDiagnostic::Stable,
        "a fixed-base cantilever is a stable structure"
    );

    let r = f.solve()?;
    assert_eq!(
        r.solver_name(),
        Some("dense"),
        "selection behaviour must not change"
    );

    for (dof, expected) in CANTILEVER_REF {
        assert_eq!(
            r.displacement(tip, dof)?,
            expected,
            "tip {dof:?}: the solve path must be bit-for-bit unchanged"
        );
    }
    for (dof, expected) in CANTILEVER_REACTION_REF {
        assert_eq!(
            r.reaction(base, dof)?,
            expected,
            "base reaction {dof:?}: the solve path must be bit-for-bit unchanged"
        );
    }

    // ... and the frozen values are still the closed-form ones.
    let v = -p * l.powi(3) / (3.0 * EI);
    let theta = -p * l * l / (2.0 * EI);
    assert_rel(r.displacement(tip, Dof::Uy)?, v, v.abs(), "v = -PL^3/3EI");
    assert_rel(r.displacement(tip, Dof::Rz)?, theta, theta.abs(), "rz");
    assert_rel(r.reaction(base, Dof::Uy)?, p, p, "Ry = P");
    assert_rel(r.reaction(base, Dof::Rz)?, p * l, p * l, "Mz = PL");
    assert!(r.equilibrium().is_balanced(), "{:?}", r.equilibrium());
    Ok(())
}

// ---------------------------------------------------------------------------
// Test 4 - stable simply supported beam
// ---------------------------------------------------------------------------

#[test]
fn stable_simply_supported_beam_is_not_a_mechanism() -> Result<(), FemError> {
    // pin - node - roller: a legitimate system that must never be classified as
    // a mechanism (its reduced system mixes translations and rotations and is
    // not diagonal, so it is the natural false-positive trap).
    let (l, p) = (4.0, 1.0e4);
    let mut f = FrameModel::new();
    let a = f.add_node(0.0, 0.0)?;
    let mid = f.add_node(l / 2.0, 0.0)?;
    let b = f.add_node(l, 0.0)?;
    f.add_member(a, mid, steel(), sec())?;
    f.add_member(mid, b, steel(), sec())?;
    f.pin(a)?;
    f.roller_y(b)?;
    f.nodal_load(mid, 0.0, -p)?;

    assert_eq!(f.diagnostic()?, StructuralDiagnostic::Stable);

    let r = f.solve()?;
    let v = -p * l.powi(3) / (48.0 * EI);
    assert_rel(
        r.displacement(mid, Dof::Uy)?,
        v,
        v.abs(),
        "midspan v = -PL^3/48EI",
    );
    assert_rel(r.reaction(a, Dof::Uy)?, p / 2.0, p, "R_A = P/2");
    assert_rel(r.reaction(b, Dof::Uy)?, p / 2.0, p, "R_B = P/2");
    assert_rel(r.reaction(a, Dof::Ux)?, 0.0, p, "no axial reaction");
    assert!(r.equilibrium().is_balanced(), "{:?}", r.equilibrium());
    Ok(())
}

// ---------------------------------------------------------------------------
// Test 5 - multi-member mechanism
// ---------------------------------------------------------------------------

#[test]
fn multi_member_mechanism_is_detected() -> Result<(), FemError> {
    // Portal frame with the horizontal translation of base 1 and the vertical
    // translation of base 2 restrained. Its 10 free DOFs still admit a genuine
    // rigid-body rotation (about a centre off the origin), so the reduced
    // system is singular through a *combination* of the supplied rigid
    // candidates - a deficiency no single member can produce.
    let mechanism = portal(&[Dof::Ux], &[Dof::Uy], 1.0)?;
    let diagnosis = mechanism.diagnostic()?;
    assert_eq!(
        diagnosis,
        StructuralDiagnostic::RigidBodyMode {
            n_free: 10,
            rank: 9,
            rigid_modes: 1,
        },
        "expected the multi-member rigid-body deficiency, got {diagnosis:?}"
    );
    assert!(diagnosis.is_mechanism());
    assert!(mechanism.solve().is_err(), "the portal is a mechanism");

    // Restraining only the two horizontal base translations leaves *two*
    // rigid-body modes (vertical translation and rotation), so the deficiency
    // is not an artefact of a particular support pattern either.
    let translating = portal(&[Dof::Ux], &[Dof::Ux], 1.0)?;
    assert_eq!(
        translating.diagnostic()?,
        StructuralDiagnostic::RigidBodyMode {
            n_free: 10,
            rank: 8,
            rigid_modes: 2,
        }
    );
    assert!(translating.solve().is_err());

    // Control: the same connectivity with a stable support scheme is `Stable`,
    // so the verdict above is about the supports, not about the portal.
    for (supports, label) in [
        ((&[Dof::Ux, Dof::Uy][..], &[Dof::Ux, Dof::Uy][..]), "fixed"),
        ((&[Dof::Ux, Dof::Uy][..], &[Dof::Uy][..]), "pinned"),
    ] {
        let (b1, b2) = supports;
        let stable = portal(b1, b2, 1.0)?;
        assert_eq!(
            stable.diagnostic()?,
            StructuralDiagnostic::Stable,
            "{label} portal must be stable"
        );
        assert!(stable.solve().is_ok(), "{label} portal must solve");
    }

    Ok(())
}

#[test]
fn non_rigid_deficiency_is_reported_as_a_mechanism() -> Result<(), FemError> {
    // The `Mechanism` verdict (a null direction that is *not* a rigid-body
    // motion) is exercised on the probe directly: for a rigid-jointed frame of
    // beam elements every frame-level mechanism is an under-restraint, so this
    // branch is reachable only for a reduced system supplied from elsewhere.
    // K = [[1,0,0],[0,2,2],[0,2,2]] has rank 2 and a null direction
    // (0, 1, -1) that is not a rigid-body motion.
    let mut k = SparseMatrix::new(3);
    for (row, col, value) in [
        (0, 0, 1.0),
        (1, 1, 2.0),
        (2, 2, 2.0),
        (1, 2, 2.0),
        (2, 1, 2.0),
    ] {
        k.add(row, col, value);
    }
    assert_eq!(
        diagnose_reduced(&k, &[]),
        StructuralDiagnostic::Mechanism { n_free: 3, rank: 2 }
    );
    // Supplying a rigid candidate that *does* carry energy does not turn it
    // into an under-restraint.
    let candidate = vec![vec![1.0, 0.0, 0.0]];
    assert_eq!(
        diagnose_reduced(&k, &candidate),
        StructuralDiagnostic::Mechanism { n_free: 3, rank: 2 }
    );
    Ok(())
}

#[test]
fn shallow_deficiency_is_ill_conditioning_not_a_mechanism() -> Result<(), FemError> {
    // A reduced system with a controlled residual stiffness just *above* the
    // representation floor: a unit matrix whose first two DOFs are coupled so
    // that the second pivot is `1 - c^2 = 5e-15`. That is above the project's
    // structural floor `PIVOT_TOL_BASE = 1e-15` but below the probe's numerical
    // rank threshold `n * eps` for `n = 23` - the band in which a structural
    // deficiency cannot be separated from extreme conditioning. The probe must
    // refuse the mechanism verdict and report `IllConditioned`.
    let n = 23;
    let c = (1.0f64 - 5e-15).sqrt();
    let mut k = SparseMatrix::new(n);
    for i in 0..n {
        k.add(i, i, 1.0);
    }
    k.add(0, 1, c);
    k.add(1, 0, c);

    assert_eq!(
        diagnose_reduced(&k, &[]),
        StructuralDiagnostic::IllConditioned {
            n_free: n,
            rank: n - 1
        },
        "a 5e-15 residual is ill-conditioning, not a mechanism"
    );

    // The same system with the coupling removed is stable, so the verdict above
    // is about the coupling and not about the size.
    let mut plain = SparseMatrix::new(n);
    for i in 0..n {
        plain.add(i, i, 1.0);
    }
    assert_eq!(diagnose_reduced(&plain, &[]), StructuralDiagnostic::Stable);
    Ok(())
}

// ---------------------------------------------------------------------------
// Probe limits (no O(n^2) allocation past the documented bound)
// ---------------------------------------------------------------------------

#[test]
fn probe_bound_matches_the_dense_solver_bound() -> Result<(), FemError> {
    assert_eq!(
        MAX_DENSE_PROBE_DOF,
        SolverCapabilities::dense()
            .max_size
            .expect("the dense capability declares a size bound"),
        "the probe bound is the project's own 'dense is affordable' bound"
    );

    // A (sparse) system one DOF past the bound is refused, not allocated.
    let mut big = SparseMatrix::new(MAX_DENSE_PROBE_DOF + 1);
    big.add(0, 0, 1.0);
    assert_eq!(
        diagnose_reduced(&big, &[]),
        StructuralDiagnostic::Indeterminate {
            n_free: MAX_DENSE_PROBE_DOF + 1,
            reason: DiagnosticLimit::SystemTooLarge,
        }
    );

    // A non-symmetric reduced matrix is refused rather than guessed at.
    let mut asymmetric = SparseMatrix::new(2);
    asymmetric.add(0, 0, 1.0);
    asymmetric.add(1, 1, 1.0);
    asymmetric.add(0, 1, 0.5);
    assert_eq!(
        diagnose_reduced(&asymmetric, &[]),
        StructuralDiagnostic::Indeterminate {
            n_free: 2,
            reason: DiagnosticLimit::NotSymmetric,
        }
    );
    Ok(())
}

// ---------------------------------------------------------------------------
// Scale sweep - the verdict must not move with a uniform scaling
// ---------------------------------------------------------------------------

const SCALES: [f64; 5] = [1e-6, 1e-3, 1.0, 1e3, 1e6];

#[test]
fn scale_invariance_of_the_verdict() -> Result<(), FemError> {
    for alpha in SCALES {
        // (a) a geometrically similar stable cantilever stays stable ...
        let mut f = FrameModel::new();
        let base = f.add_node(0.0, 0.0)?;
        let tip = f.add_node(2.0 * alpha, 0.0)?;
        f.add_member(
            base,
            tip,
            steel(),
            BeamSection::new(A * alpha * alpha, I * alpha.powi(4)),
        )?;
        f.fix(base)?;
        assert_eq!(
            f.diagnostic()?,
            StructuralDiagnostic::Stable,
            "stable cantilever at alpha = {alpha:e} must not become a mechanism"
        );
        assert!(
            f.solve().is_ok(),
            "stable cantilever at alpha = {alpha:e} must still solve"
        );

        // (b) ... and a geometrically similar mechanism stays a mechanism with
        //     exactly the same rank and rigid-mode count.
        let mechanism = portal(&[Dof::Ux], &[Dof::Uy], alpha)?;
        assert_eq!(
            mechanism.diagnostic()?,
            StructuralDiagnostic::RigidBodyMode {
                n_free: 10,
                rank: 9,
                rigid_modes: 1,
            },
            "mechanism at alpha = {alpha:e} must not become stable"
        );

        let free = portal(&[], &[], alpha)?;
        assert_eq!(
            free.diagnostic()?,
            StructuralDiagnostic::RigidBodyMode {
                n_free: 12,
                rank: 9,
                rigid_modes: 3,
            },
            "unsupported portal at alpha = {alpha:e}"
        );
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Load independence - the verdict is a property of the structure
// ---------------------------------------------------------------------------

#[test]
fn verdict_is_independent_of_the_loads() -> Result<(), FemError> {
    // The diagnostic never looks at the load vector, so scaling, reversing or
    // removing the loads cannot change it.
    for (label, load) in [
        ("no load", 0.0),
        ("+P", 1.0e3),
        ("-P", -1.0e3),
        ("1e12 P", 1.0e15),
    ] {
        let (mut f, _base, tip) = cantilever(2.0)?;
        if load != 0.0 {
            f.nodal_load(tip, 0.0, load)?;
        }
        assert_eq!(
            f.diagnostic()?,
            StructuralDiagnostic::Stable,
            "stable cantilever, {label}"
        );
    }

    for (label, load) in [("no load", 0.0), ("+P", 1.0e3), ("-P", -1.0e3)] {
        let mut f = portal(&[Dof::Ux], &[Dof::Uy], 1.0)?;
        if load != 0.0 {
            f.nodal_load(nth(2), load, 0.0)?;
            f.member_udl(MemberHandle::from_index(0), 0.0, -load)?;
            f.member_point_load(MemberHandle::from_index(1), 0.5, 0.0, -load, 0.0)?;
        }
        assert_eq!(
            f.diagnostic()?,
            StructuralDiagnostic::RigidBodyMode {
                n_free: 10,
                rank: 9,
                rigid_modes: 1,
            },
            "mechanism, {label}"
        );
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Ill-conditioning - stable structure, singular *numbers*
// ---------------------------------------------------------------------------

#[test]
fn ill_conditioning_is_never_reported_as_a_mechanism() -> Result<(), FemError> {
    // A single-member cantilever whose length is far outside the resolvable
    // window: the structure is fine, the raw reduced system is not. The
    // diagnostic must not claim a mechanism - the honest verdict is
    // `IllConditioned` (full structural rank) and the solver may still fail.
    for (len, expected) in [
        (2.0, StructuralDiagnostic::Stable),
        (
            1.0e8,
            StructuralDiagnostic::IllConditioned { n_free: 3, rank: 3 },
        ),
        (
            1.0e12,
            StructuralDiagnostic::IllConditioned { n_free: 3, rank: 3 },
        ),
    ] {
        let (mut f, _base, tip) = cantilever(len)?;
        f.nodal_load(tip, 0.0, -1.0e4)?;
        let diagnosis = f.diagnostic()?;
        assert_eq!(diagnosis, expected, "cantilever L = {len:e}");
        assert!(
            !diagnosis.is_mechanism(),
            "L = {len:e}: ill-conditioning must never be reported as a mechanism \
             (got {diagnosis:?})"
        );
        // Whatever the conditioning, the outcome is a result or a clean error -
        // never a panic and never a mechanism claim.
        match f.solve() {
            Ok(r) => {
                let uy = r.displacement(tip, Dof::Uy)?;
                assert!(uy.is_finite(), "L = {len:e}: non-finite displacement {uy}");
            }
            Err(e) => assert!(matches!(e, FemError::SolverError(_)), "got {e:?}"),
        }
    }

    // A genuinely soft structure (tiny E, A and I) is *not* a mechanism either:
    // there is no absolute stiffness scale to compare against.
    let mut soft = FrameModel::new();
    let base = soft.add_node(0.0, 0.0)?;
    let tip = soft.add_node(2.0, 0.0)?;
    soft.add_member(
        base,
        tip,
        Material::new(1.0e-3, 0.3, 7850.0, "soft"),
        BeamSection::new(1.0e-6, 1.0e-9),
    )?;
    soft.fix(base)?;
    let diagnosis = soft.diagnostic()?;
    assert!(
        !diagnosis.is_mechanism(),
        "a legitimately soft structure is not a mechanism (got {diagnosis:?})"
    );

    // Sanity: a structure scaled down uniformly by 1e-6 relative to the
    // reference *is* stable, demonstrating the same point with real units.
    let mut small = FrameModel::new();
    let base = small.add_node(0.0, 0.0)?;
    let tip = small.add_node(2.0e-6, 0.0)?;
    small.add_member(base, tip, steel(), BeamSection::new(A * 1e-12, I * 1e-24))?;
    small.fix(base)?;
    assert_eq!(small.diagnostic()?, StructuralDiagnostic::Stable);
    Ok(())
}
