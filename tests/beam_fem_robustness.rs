//! Phase 3 — Beam FEM robustness, pathological-case handling and API hardening.
//!
//! Verifies that invalid / problematic inputs fail explicitly and
//! deterministically (structured `FemError`) instead of panicking, producing
//! NaN/Inf, silently regularizing, or swallowing solver failures.
//!
//! Conventions and sign/load contracts are unchanged; this file only exercises
//! failure behaviour, determinism and physical sanity.

use section_properties::SolverSelection;
use section_properties::beam_fem::{
    BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver, FemError,
};
use section_properties::material::Material;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn steel(e: f64, a: f64, i: f64) -> (Material, BeamSection) {
    (Material::new(e, 0.3, 1.0, "U"), BeamSection::new(a, i))
}

/// Two nodes (0,0)-(1,0) and one element; NOT constrained.
fn bar() -> BeamModel {
    let mut m = BeamModel::new();
    m.add_node(BeamNode::new(0, 0.0, 0.0));
    m.add_node(BeamNode::new(1, 1.0, 0.0));
    let (mat, sec) = steel(1.0, 1.0, 1.0);
    m.add_element(BeamElement::new(0, 1, mat, sec).unwrap());
    m
}

/// Unit cantilever (E = A = I = 1), fixed at node 0.
fn unit_cantilever() -> BeamModel {
    let mut m = bar();
    m.fix_node(0);
    m
}

fn solve_checked(model: &BeamModel) -> Result<BeamSolver, FemError> {
    let mut s = BeamSolver::from_model(model)?;
    s.set_solver(SolverSelection::named("dense"));
    s.solve_configured()?;
    Ok(s)
}

fn assert_all_finite(s: &BeamSolver) {
    for (i, v) in s.displacements().iter().enumerate() {
        assert!(v.is_finite(), "displacement[{}] = {}", i, v);
    }
    for (i, v) in s.reactions().iter().enumerate() {
        assert!(v.is_finite(), "reaction[{}] = {}", i, v);
    }
    for (e, fe) in s.element_end_forces().unwrap().iter().enumerate() {
        for (k, v) in fe.iter().enumerate() {
            assert!(v.is_finite(), "end_force[{}][{}] = {}", e, k, v);
        }
    }
}

fn assert_mixed(a: f64, b: f64, abs: f64, rel: f64, label: &str) {
    let bound = abs + rel * a.abs().max(b.abs());
    assert!(
        (a - b).abs() <= bound,
        "{}: {} vs {} (|diff| {:.3e} > {:.3e})",
        label,
        a,
        b,
        (a - b).abs(),
        bound
    );
}

// ===========================================================================
// 2.1 Zero-length elements
// ===========================================================================

#[test]
fn test_zero_length_element_rejected() {
    // Direct element construction.
    let (mat, sec) = steel(1.0, 1.0, 1.0);
    assert!(matches!(
        BeamElement::new(0, 0, mat, sec),
        Err(FemError::InvalidModel(_))
    ));

    // Model with coincident coordinates.
    let mut m = BeamModel::new();
    m.add_node(BeamNode::new(0, 1.0, 2.0));
    m.add_node(BeamNode::new(1, 1.0, 2.0)); // same position
    m.add_element(BeamElement::new(0, 1, mat, sec).unwrap());
    m.fix_node(0);
    let err = match BeamSolver::from_model(&m) {
        Ok(_) => panic!("zero-length element must be rejected"),
        Err(e) => e,
    };
    assert!(matches!(err, FemError::InvalidModel(_)), "got {:?}", err);

    // And it must never reach the solver: no panic, no NaN.
    println!(
        "[zero-length] rejected at construction/from_model: {:?}",
        err
    );
}

// ===========================================================================
// 2.2 Invalid material / section parameters
// ===========================================================================

#[test]
fn test_invalid_material_and_section_parameters() {
    let bad = |e: f64, a: f64, i: f64| {
        let mut m = BeamModel::new();
        m.add_node(BeamNode::new(0, 0.0, 0.0));
        m.add_node(BeamNode::new(1, 1.0, 0.0));
        let (mat, sec) = steel(e, a, i);
        m.add_element(BeamElement::new(0, 1, mat, sec).unwrap());
        m.fix_node(0);
        BeamSolver::from_model(&m)
    };

    let cases: [(&str, f64, f64, f64); 10] = [
        ("E=0", 0.0, 1.0, 1.0),
        ("E<0", -1.0, 1.0, 1.0),
        ("E=NaN", f64::NAN, 1.0, 1.0),
        ("E=Inf", f64::INFINITY, 1.0, 1.0),
        ("A=0", 1.0, 0.0, 1.0),
        ("A<0", 1.0, -1.0, 1.0),
        ("A=NaN", 1.0, f64::NAN, 1.0),
        ("I=0", 1.0, 1.0, 0.0),
        ("I<0", 1.0, 1.0, -1.0),
        ("I=NaN", 1.0, 1.0, f64::NAN),
    ];
    for (label, e, a, i) in cases {
        match bad(e, a, i) {
            Err(FemError::InvalidModel(_)) => {}
            other => panic!(
                "{}: expected InvalidModel, got {:?}",
                label,
                other.map(|_| "Ok")
            ),
        }
    }
    println!("[material/section] E,A,I must be finite and positive: all rejected");
}

// ===========================================================================
// 2.3 Under-constrained models
// ===========================================================================

#[test]
fn test_underconstrained_models_fail_explicitly() {
    // (a) completely free beam.
    let free = bar();
    // (b) only axial fixed -> transverse/rotational rigid modes remain.
    let mut partial = bar();
    partial.try_fix_dof(0, 0, 0.0).unwrap();
    // (c) translations fixed, rotation free.
    let mut rot_free = bar();
    rot_free.try_fix_dof(0, 0, 0.0).unwrap();
    rot_free.try_fix_dof(0, 1, 0.0).unwrap();

    for (label, m) in [
        ("free-free", free),
        ("ux only", partial),
        ("rotation free", rot_free),
    ] {
        let err = match solve_checked(&m) {
            Ok(_) => panic!("{}: must fail", label),
            Err(e) => e,
        };
        assert!(
            matches!(err, FemError::SolverError(_)),
            "{}: expected SolverError, got {:?}",
            label,
            err
        );
        println!("[under-constrained {}] {:?}", label, err);
    }

    // A fully constrained model succeeds and yields finite results.
    let ok = solve_checked(&unit_cantilever()).unwrap();
    assert_all_finite(&ok);
}

// ===========================================================================
// 2.4 Invalid node / element / DOF references
// ===========================================================================

#[test]
fn test_invalid_node_element_dof_references() {
    let mut m = unit_cantilever();

    // Load APIs with bad references.
    assert!(matches!(
        m.add_distributed_load(9, 0.0, -1.0),
        Err(FemError::InvalidInput(_))
    ));
    assert!(matches!(
        m.add_point_load(9, 0.5, 0.0, -1.0, 0.0),
        Err(FemError::InvalidInput(_))
    ));
    assert!(matches!(
        m.add_applied_moment(9, 1.0),
        Err(FemError::InvalidInput(_))
    ));
    // Point-load position out of range / non-finite.
    assert!(m.add_point_load(0, -0.1, 0.0, -1.0, 0.0).is_err());
    assert!(m.add_point_load(0, 1.1, 0.0, -1.0, 0.0).is_err());
    assert!(m.add_point_load(0, f64::NAN, 0.0, -1.0, 0.0).is_err());

    // Checked BC APIs (no panic).
    assert!(matches!(m.try_fix_node(9), Err(FemError::InvalidInput(_))));
    assert!(matches!(
        m.try_fix_dof(0, 7, 0.0),
        Err(FemError::InvalidInput(_))
    ));
    assert!(matches!(
        m.try_fix_dof(9, 0, 0.0),
        Err(FemError::InvalidInput(_))
    ));
    assert!(matches!(
        m.try_fix_dof(0, 0, f64::NAN),
        Err(FemError::InvalidInput(_))
    ));

    // Malformed connectivity / DOF references reachable through public fields.
    let mut bad_elem = BeamModel::new();
    bad_elem.add_node(BeamNode::new(0, 0.0, 0.0));
    let (mat, sec) = steel(1.0, 1.0, 1.0);
    bad_elem.add_element(BeamElement {
        node_i: 0,
        node_j: 9,
        material: mat,
        section: sec,
    });
    bad_elem.fix_node(0);
    assert!(matches!(
        BeamSolver::from_model(&bad_elem),
        Err(FemError::InvalidModel(_))
    ));

    let mut bad_dof = unit_cantilever();
    bad_dof.fixed_dofs.push((0, 5, 0.0)); // DOF 5 invalid
    assert!(matches!(
        BeamSolver::from_model(&bad_dof),
        Err(FemError::InvalidModel(_))
    ));

    // Checked nodal-force API (no panic).
    assert!(matches!(
        m.try_add_nodal_force(9, 0, 1.0),
        Err(FemError::InvalidInput(_))
    ));
    assert!(matches!(
        m.try_add_nodal_force(0, 3, 1.0),
        Err(FemError::InvalidInput(_))
    ));
    // Bypass via the public field -> validated at from_model.
    let mut bad_nf = unit_cantilever();
    bad_nf.nodal_forces.push((9, 0, 1.0));
    assert!(matches!(
        BeamSolver::from_model(&bad_nf),
        Err(FemError::InvalidModel(_))
    ));

    println!("[invalid refs] node/element/DOF/connectivity errors returned without panic");
}

// ===========================================================================
// 2.5 Invalid point loads / 2.6 distributed / 2.7 applied moments
// ===========================================================================

#[test]
fn test_non_finite_loads_rejected() {
    let mut m = unit_cantilever();

    // Point loads (add-time validation).
    assert!(m.add_point_load(0, 0.5, f64::NAN, 0.0, 0.0).is_err());
    assert!(m.add_point_load(0, 0.5, 0.0, f64::INFINITY, 0.0).is_err());
    assert!(
        m.add_point_load(0, 0.5, 0.0, 0.0, f64::NEG_INFINITY)
            .is_err()
    );
    assert!(m.add_point_moment(0, 0.5, f64::NAN).is_err());

    // Distributed loads.
    assert!(m.add_distributed_load(0, f64::NAN, 0.0).is_err());
    assert!(m.add_distributed_load(0, 0.0, f64::INFINITY).is_err());

    // Applied moments.
    assert!(m.add_applied_moment(0, f64::NAN).is_err());
    assert!(m.add_applied_moment(0, f64::INFINITY).is_err());

    // Bypass via public fields -> caught at from_model.
    let mut bypass = unit_cantilever();
    bypass
        .distributed_loads
        .push(section_properties::beam_fem::DistributedLoad::new(
            0,
            f64::NAN,
            0.0,
        ));
    assert!(matches!(
        BeamSolver::from_model(&bypass),
        Err(FemError::InvalidModel(_))
    ));

    let mut bypass_pl = unit_cantilever();
    bypass_pl
        .point_loads
        .push(section_properties::beam_fem::PointLoad::new(
            0,
            0.5,
            0.0,
            f64::NAN,
            0.0,
        ));
    assert!(matches!(
        BeamSolver::from_model(&bypass_pl),
        Err(FemError::InvalidModel(_))
    ));

    let mut bypass_am = unit_cantilever();
    bypass_am
        .applied_moments
        .push(section_properties::beam_fem::AppliedMoment::new(
            0,
            f64::NAN,
        ));
    assert!(matches!(
        BeamSolver::from_model(&bypass_am),
        Err(FemError::InvalidModel(_))
    ));

    // Nodal force: the checked API rejects non-finite at add-time; a bypass via
    // the public field is caught at from_model.
    let mut nf = unit_cantilever();
    assert!(matches!(
        nf.try_add_nodal_force(1, 1, f64::NAN),
        Err(FemError::InvalidInput(_))
    ));
    nf.nodal_forces.push((1, 1, f64::NAN));
    assert!(matches!(
        BeamSolver::from_model(&nf),
        Err(FemError::InvalidModel(_))
    ));

    println!("[non-finite loads] rejected at add-time or from_model; no NaN propagation");
}

// ===========================================================================
// 2.8 Solver error propagation
// ===========================================================================

#[test]
fn test_solver_error_propagation() {
    // Singular (free-free) system via solve_configured.
    let mut m = bar();
    m.add_nodal_force(1, 1, -1.0);
    let mut s = BeamSolver::from_model(&m).unwrap();
    s.set_solver(SolverSelection::named("dense"));
    let err = s.solve_configured().expect_err("singular solve must fail");
    assert!(matches!(err, FemError::SolverError(_)), "got {:?}", err);
    assert_eq!(s.solver_name(), None);

    // Same via the legacy explicit-solver path.
    let registry = section_properties::SolverRegistry::default();
    let mut lin = registry.create("sparse_lu").unwrap();
    let mut s2 = BeamSolver::from_model(&m).unwrap();
    let err2 = s2.solve(&mut *lin).expect_err("singular solve must fail");
    assert!(matches!(err2, FemError::SolverError(_)), "got {:?}", err2);

    println!("[solver propagation] FemError::SolverError surfaced (no panic/swallow)");
}

// ===========================================================================
// 2.9 Repeated solve / determinism
// ===========================================================================

#[test]
fn test_repeated_solve_determinism() {
    let mut m = unit_cantilever();
    m.add_distributed_load(0, 0.0, -100.0).unwrap();
    m.add_nodal_force(1, 1, -50.0);
    m.add_applied_moment(1, 10.0).unwrap();

    let s1 = solve_checked(&m).unwrap();
    let s2 = solve_checked(&m).unwrap();
    let s3 = solve_checked(&m).unwrap();

    let (u1, r1, f1) = (
        s1.displacements().to_vec(),
        s1.reactions(),
        s1.element_end_forces().unwrap(),
    );
    let (u2, r2, f2) = (
        s2.displacements().to_vec(),
        s2.reactions(),
        s2.element_end_forces().unwrap(),
    );
    let (u3, r3, f3) = (
        s3.displacements().to_vec(),
        s3.reactions(),
        s3.element_end_forces().unwrap(),
    );

    for i in 0..u1.len() {
        assert_eq!(u1[i], u2[i], "u2 not identical at {}", i);
        assert_eq!(u1[i], u3[i], "u3 not identical at {}", i);
    }
    for i in 0..r1.len() {
        assert_eq!(r1[i], r2[i], "R2 not identical at {}", i);
        assert_eq!(r1[i], r3[i], "R3 not identical at {}", i);
    }
    for e in 0..f1.len() {
        for k in 0..6 {
            assert_eq!(f1[e][k], f2[e][k], "end force not identical");
            assert_eq!(f1[e][k], f3[e][k], "end force not identical");
        }
    }

    // Solving through a different backend gives the same physics.
    let mut sd = BeamSolver::from_model(&m).unwrap();
    sd.set_solver(SolverSelection::named("sparse_lu"));
    sd.solve_configured().unwrap();
    for (i, &v) in u1.iter().enumerate() {
        assert_mixed(
            sd.displacements()[i],
            v,
            1e-12,
            1e-9,
            "repeated solve (sparse_lu)",
        );
    }

    // solve -> solve on the same instance does not accumulate loads.
    let mut s4 = BeamSolver::from_model(&m).unwrap();
    s4.set_solver(SolverSelection::named("dense"));
    s4.solve_configured().unwrap();
    let r_first = s4.reactions();
    s4.solve_configured().unwrap();
    let r_second = s4.reactions();
    for i in 0..r_first.len() {
        assert_eq!(
            r_first[i], r_second[i],
            "second solve on same instance changed R[{}]",
            i
        );
    }
    println!(
        "[determinism] repeat solve and cross-backend solve are identical; no load accumulation"
    );
}

// ===========================================================================
// 2.10 solve() vs solve_configured() equivalence
// ===========================================================================

#[test]
fn test_solve_paths_equivalent() {
    // Beam FEM does not expose `solve_many` directly (that is a LinearSolver
    // capability); what it exposes is the explicit-solver `solve` path and the
    // selection-driven `solve_configured` path. They must agree.
    let mut m = unit_cantilever();
    m.add_nodal_force(1, 1, -100.0);

    let registry = section_properties::SolverRegistry::default();
    let mut explicit = BeamSolver::from_model(&m).unwrap();
    let mut lin = registry.create("dense").unwrap();
    explicit.solve(&mut *lin).unwrap();

    let mut configured = BeamSolver::from_model(&m).unwrap();
    configured.set_solver(SolverSelection::named("dense"));
    configured.solve_configured().unwrap();

    for i in 0..explicit.displacements().len() {
        assert_eq!(explicit.displacements()[i], configured.displacements()[i]);
    }
    for i in 0..explicit.reactions().len() {
        assert_eq!(explicit.reactions()[i], configured.reactions()[i]);
    }
    println!("[solve paths] explicit solve == solve_configured; no state corruption");
}

// ===========================================================================
// Step 3 — Numerical scaling
// ===========================================================================

#[test]
fn test_geometry_scaling() {
    let (p, f, m0) = (100.0, 100.0, 100.0);
    let base = 1.0_f64;

    for &alpha in &[0.1_f64, 1.0, 10.0] {
        let mut bend = BeamModel::new();
        bend.add_node(BeamNode::new(0, 0.0, 0.0));
        bend.add_node(BeamNode::new(1, alpha, 0.0));
        let (mat, sec) = steel(1.0, 1.0, 1.0);
        bend.add_element(BeamElement::new(0, 1, mat, sec).unwrap());
        bend.fix_node(0);
        bend.add_nodal_force(1, 1, -p);

        let mut ax = BeamModel::new();
        ax.add_node(BeamNode::new(0, 0.0, 0.0));
        ax.add_node(BeamNode::new(1, alpha, 0.0));
        ax.add_element(BeamElement::new(0, 1, mat, sec).unwrap());
        ax.fix_node(0);
        ax.add_nodal_force(1, 0, f);

        let mut mo = BeamModel::new();
        mo.add_node(BeamNode::new(0, 0.0, 0.0));
        mo.add_node(BeamNode::new(1, alpha, 0.0));
        mo.add_element(BeamElement::new(0, 1, mat, sec).unwrap());
        mo.fix_node(0);
        mo.add_applied_moment(1, m0).unwrap();

        for &name in &["dense", "skyline_ldlt", "sparse_lu"] {
            let sb = {
                let mut s = BeamSolver::from_model(&bend).unwrap();
                s.set_solver(SolverSelection::named(name));
                s.solve_configured().unwrap();
                s
            };
            let sa = {
                let mut s = BeamSolver::from_model(&ax).unwrap();
                s.set_solver(SolverSelection::named(name));
                s.solve_configured().unwrap();
                s
            };
            let sm = {
                let mut s = BeamSolver::from_model(&mo).unwrap();
                s.set_solver(SolverSelection::named(name));
                s.solve_configured().unwrap();
                s
            };
            // Axial u ∝ α ; transverse δ ∝ α³ ; rotation ∝ α² ; tip-moment θ ∝ α.
            assert_mixed(
                sa.displacement(1, 0),
                f * alpha / base,
                1e-9,
                1e-9,
                "axial ∝ α",
            );
            assert_mixed(
                sb.displacement(1, 1),
                -p * alpha.powi(3) / 3.0,
                1e-9,
                1e-9,
                "δ ∝ α³",
            );
            assert_mixed(
                sb.displacement(1, 2),
                -p * alpha.powi(2) / 2.0,
                1e-9,
                1e-9,
                "θ ∝ α²",
            );
            assert_mixed(sm.displacement(1, 2), m0 * alpha, 1e-9, 1e-9, "θ_M ∝ α");
        }
    }
    println!("[geometry scaling] u∝α, δ∝α³, θ∝α², θ_M∝α for α ∈ {{0.1,1,10}} (3 backends)");
}

#[test]
fn test_material_scaling() {
    let (p, f) = (100.0, 100.0);

    for &sc in &[0.5_f64, 1.0, 2.0] {
        // Bending δ ∝ 1/(E I); axial u ∝ 1/(E A).
        let mut bend = BeamModel::new();
        bend.add_node(BeamNode::new(0, 0.0, 0.0));
        bend.add_node(BeamNode::new(1, 1.0, 0.0));
        bend.add_element(
            BeamElement::new(
                0,
                1,
                Material::new(sc, 0.3, 1.0, "U"),
                BeamSection::new(1.0, 1.0),
            )
            .unwrap(),
        );
        bend.fix_node(0);
        bend.add_nodal_force(1, 1, -p);

        let mut ax = BeamModel::new();
        ax.add_node(BeamNode::new(0, 0.0, 0.0));
        ax.add_node(BeamNode::new(1, 1.0, 0.0));
        ax.add_element(
            BeamElement::new(
                0,
                1,
                Material::new(1.0, 0.3, 1.0, "U"),
                BeamSection::new(sc, 1.0),
            )
            .unwrap(),
        );
        ax.fix_node(0);
        ax.add_nodal_force(1, 0, f);

        // I scaling: δ ∝ 1/I.
        let mut ib = BeamModel::new();
        ib.add_node(BeamNode::new(0, 0.0, 0.0));
        ib.add_node(BeamNode::new(1, 1.0, 0.0));
        ib.add_element(
            BeamElement::new(
                0,
                1,
                Material::new(1.0, 0.3, 1.0, "U"),
                BeamSection::new(1.0, sc),
            )
            .unwrap(),
        );
        ib.fix_node(0);
        ib.add_nodal_force(1, 1, -p);

        for &name in &["dense", "skyline_ldlt", "sparse_lu"] {
            let run = |m: &BeamModel| {
                let mut s = BeamSolver::from_model(m).unwrap();
                s.set_solver(SolverSelection::named(name));
                s.solve_configured().unwrap();
                s
            };
            assert_mixed(
                run(&bend).displacement(1, 1),
                -p / (3.0 * sc),
                1e-12,
                1e-9,
                "δ ∝ 1/E",
            );
            assert_mixed(run(&ax).displacement(1, 0), f / sc, 1e-12, 1e-9, "u ∝ 1/A");
            assert_mixed(
                run(&ib).displacement(1, 1),
                -p / (3.0 * sc),
                1e-12,
                1e-9,
                "δ ∝ 1/I",
            );
        }
    }
    println!("[material scaling] δ∝1/E, u∝1/A, δ∝1/I for scale ∈ {{0.5,1,2}} (3 backends)");
}

// ===========================================================================
// Step 4 — Physical sanity invariants
// ===========================================================================

#[test]
fn test_physical_sanity_invariants() {
    // (1) Zero load on a well-constrained model -> zero response.
    let s0 = solve_checked(&unit_cantilever()).unwrap();
    assert_all_finite(&s0);
    for v in s0.displacements() {
        assert_eq!(*v, 0.0, "zero load must give zero displacement");
    }
    for v in s0.reactions() {
        assert_eq!(v, 0.0, "zero load must give zero reaction");
    }

    // (2) Loaded, well-constrained: finite, free-DOF reactions ~0, equilibrium.
    let mut m = bar();
    m.fix_node(0);
    m.add_distributed_load(0, 30.0, -80.0).unwrap();
    m.add_point_load(0, 0.5, 10.0, -20.0, 5.0).unwrap();
    m.add_applied_moment(0, 7.0).unwrap();
    m.add_nodal_force(1, 1, -40.0);

    let s = solve_checked(&m).unwrap();
    assert_all_finite(&s);

    // Free DOFs carry no reaction.
    for (i, v) in s.reactions().iter().enumerate().skip(3) {
        assert_mixed(*v, 0.0, 1e-7, 1e-9, &format!("free-DOF R[{}]", i));
    }

    // Global equilibrium with the applied loads (about node 0 / origin).
    let rx = s.reactions()[0];
    let ry = s.reactions()[1];
    let rz = s.reactions()[2];
    // ΣFx: distributed qx*L + point fx; ΣFy: qy*L + point fy + tip; ΣMz: applied
    // moment + moment of the point load at x=0.5 + tip at x=1 + UDL at 0.5.
    assert_mixed(rx + (30.0 * 1.0 + 10.0), 0.0, 1e-8, 1e-9, "ΣFx");
    assert_mixed(ry + (-80.0 * 1.0 - 20.0 - 40.0), 0.0, 1e-8, 1e-9, "ΣFy");
    assert_mixed(
        rz + 7.0 + (-20.0 * 0.5) + (5.0) + (-80.0 * 1.0 * 0.5) + (-40.0 * 1.0),
        0.0,
        1e-8,
        1e-9,
        "ΣMz",
    );
    println!("[sanity] zero-load ⇒ zero; free-DOF R≈0; ΣFx=ΣFy=ΣMz=0; all finite");
}
