//! Phase 3 — solver selection, manual selection and capability validation.
//!
//! Covers:
//! * manual selection of every applicable backend (dense / skyline / sparse LU /
//!   CG / ICCG), with name, solution and residual checks;
//! * invalid manual selection (no silent fallback);
//! * automatic selection decisions (name + reason), not just solutions;
//! * genuine-singular refactorization invalidating stale factors;
//! * Beam FEM integration (`set_solver` + `solve_configured`) across backends;
//! * observability of the backend that was actually used.

use section_properties::beam_fem::{BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver};
use section_properties::fea::SparseMatrix;
use section_properties::material::Material;
use section_properties::{SelectionReason, SolverError, SolverRegistry, SolverSelection};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Symmetric positive-definite tridiagonal matrix (`diag 4`, `off -1`).
fn spd_tridiagonal(n: usize) -> SparseMatrix {
    let mut a = SparseMatrix::new(n);
    for i in 0..n {
        a.add(i, i, 4.0);
        if i + 1 < n {
            a.add(i, i + 1, 1.0);
            a.add(i + 1, i, 1.0);
        }
    }
    a.compress();
    a
}

/// Non-symmetric matrix (upper triangular with positive diagonal).
fn nonsymmetric(n: usize) -> SparseMatrix {
    let mut a = SparseMatrix::new(n);
    for i in 0..n {
        a.add(i, i, 2.0);
        if i + 1 < n {
            a.add(i, i + 1, 1.0);
        }
    }
    a.compress();
    a
}

/// `‖A x − b‖∞`, computed test-side.
fn residual(a: &SparseMatrix, x: &[f64], b: &[f64]) -> f64 {
    let mut m = a.clone();
    m.compress();
    let (rp, cols, vals) = (m.row_ptr(), m.csr_cols(), m.csr_vals());
    let mut max = 0.0f64;
    for i in 0..a.n {
        let mut s = 0.0;
        for k in rp[i]..rp[i + 1] {
            s += vals[k] * x[cols[k]];
        }
        max = max.max((s - b[i]).abs());
    }
    max
}

fn assert_rel(a: f64, b: f64, tol: f64, label: &str) {
    let scale = a.abs().max(b.abs()).max(1e-30);
    assert!(
        (a - b).abs() / scale <= tol,
        "{}: {} vs {} (rel err {:.3e})",
        label,
        a,
        b,
        (a - b).abs() / scale
    );
}

// ===========================================================================
// A. Manual selection of every applicable backend
// ===========================================================================

#[test]
fn test_manual_selection_each_backend() {
    let registry = SolverRegistry::default();
    let a = spd_tridiagonal(4);
    let b = vec![1.0, 2.0, 3.0, 4.0];

    let selections = [
        SolverSelection::dense(),
        SolverSelection::skyline_ldlt(),
        SolverSelection::sparse_lu(),
        SolverSelection::cg(),
        SolverSelection::iccg(),
    ];

    for sel in &selections {
        let name = sel.requested_name().unwrap().to_string();

        // Capability validation must accept this SPD symmetric system.
        let mut solver = registry
            .create_selected(&a, sel)
            .unwrap_or_else(|e| panic!("{}: create_selected failed: {:?}", name, e));
        assert_eq!(solver.name(), name, "reported solver name");

        // Direct solvers must factor and solve; iterative solvers are allowed to
        // decline (explicit error), but must not return a silently wrong result.
        match solver.factor(&a) {
            Ok(()) => match solver.solve(&b) {
                Ok(x) => {
                    let res = residual(&a, &x, &b);
                    assert!(res < 1e-8, "{}: residual ‖Ax-b‖∞ = {:.3e}", name, res);
                    println!("[manual {:11}] factor+solve OK, residual={:.2e}", name, res);
                }
                Err(e) => {
                    assert!(
                        name == "cg" || name == "iccg",
                        "{}: unexpected solve failure: {:?}",
                        name,
                        e
                    );
                    println!("[manual {:11}] solve declined: {:?}", name, e);
                }
            },
            Err(e) => {
                assert!(
                    name == "cg" || name == "iccg",
                    "{}: unexpected factor failure: {:?}",
                    name,
                    e
                );
                println!("[manual {:11}] factor declined: {:?}", name, e);
            }
        }
    }
}

// ===========================================================================
// B. Invalid manual selection must error and never fall back
// ===========================================================================

#[test]
fn test_manual_selection_rejects_incompatible_matrix() {
    let registry = SolverRegistry::default();
    let a = nonsymmetric(4);

    // Symmetric-only backends must reject a non-symmetric matrix.
    for sel in [
        SolverSelection::skyline_ldlt(),
        SolverSelection::cg(),
        SolverSelection::iccg(),
    ] {
        let name = sel.requested_name().unwrap().to_string();
        let err = match registry.create_selected(&a, &sel) {
            Ok(_) => panic!("{} should reject a non-symmetric matrix", name),
            Err(e) => e,
        };
        assert!(
            matches!(err, SolverError::Unsupported(_)),
            "{}: expected Unsupported, got {:?}",
            name,
            err
        );
        println!("[reject {:11}] {:?}", name, err);
    }

    // General backends accept it (no fallback needed).
    assert!(
        registry
            .create_selected(&a, &SolverSelection::dense())
            .is_ok()
    );
    assert!(
        registry
            .create_selected(&a, &SolverSelection::sparse_lu())
            .is_ok()
    );

    // Unknown names are a structured error, never a silent substitution.
    let err = match registry.create_selected(&a, &SolverSelection::named("nonexistent")) {
        Ok(_) => panic!("unknown solver name must not fall back"),
        Err(e) => e,
    };
    assert!(matches!(err, SolverError::Unsupported(_)), "got {:?}", err);
}

// ===========================================================================
// C. Auto selection decisions
// ===========================================================================

#[test]
fn test_auto_selection_decisions() {
    let registry = SolverRegistry::default();

    // 1. Small system -> dense (any class).
    let info = registry
        .select(&spd_tridiagonal(100), &SolverSelection::Auto)
        .unwrap();
    assert_eq!(info.solver_name, "dense");
    assert_eq!(info.reason, SelectionReason::SmallSystem);

    // 2. Large symmetric, positive diagonal -> skyline.
    let info = registry
        .select(&spd_tridiagonal(1000), &SolverSelection::Auto)
        .unwrap();
    assert_eq!(info.solver_name, "skyline_ldlt");
    assert_eq!(info.reason, SelectionReason::SymmetricPositiveDiagonal);

    // 3. Large symmetric but with a non-positive diagonal -> cannot be SPD;
    //    auto must NOT pick skyline.
    let mut a = SparseMatrix::new(1000);
    for i in 0..1000 {
        a.add(i, i, if i == 0 { -1.0 } else { 4.0 });
        if i + 1 < 1000 {
            a.add(i, i + 1, 1.0);
            a.add(i + 1, i, 1.0);
        }
    }
    a.compress();
    let info = registry.select(&a, &SolverSelection::Auto).unwrap();
    assert_eq!(info.solver_name, "sparse_lu");
    assert_eq!(info.reason, SelectionReason::GeneralSystem);

    // 4. Large non-symmetric -> sparse_lu.
    let info = registry
        .select(&nonsymmetric(1000), &SolverSelection::Auto)
        .unwrap();
    assert_eq!(info.solver_name, "sparse_lu");
    assert_eq!(info.reason, SelectionReason::GeneralSystem);

    // 5. Empty matrix -> error.
    assert!(
        registry
            .select(&SparseMatrix::new(0), &SolverSelection::Auto)
            .is_err()
    );

    // 6. Auto must never pick CG/ICCG (SPD cannot be established).
    for m in [spd_tridiagonal(1000), nonsymmetric(1000)] {
        let name = registry
            .select(&m, &SolverSelection::Auto)
            .unwrap()
            .solver_name;
        assert!(name != "cg" && name != "iccg", "auto picked {}", name);
    }
}

// ===========================================================================
// D. Genuine-singular refactorization invalidates stale factors
// ===========================================================================

#[test]
fn test_singular_refactor_invalidates_previous_factorization() {
    let registry = SolverRegistry::default();

    // Valid SPD 2x2.
    let mut a = SparseMatrix::new(2);
    a.add(0, 0, 4.0);
    a.add(0, 1, 1.0);
    a.add(1, 0, 1.0);
    a.add(1, 1, 3.0);
    a.compress();

    // Genuinely singular: [[1, 0], [0, 0]].
    let mut s = SparseMatrix::new(2);
    s.add(0, 0, 1.0);
    s.compress();

    for name in ["dense", "skyline_ldlt", "sparse_lu"] {
        let mut solver = registry.create(name).unwrap();
        solver.factor(&a).unwrap();
        assert!(solver.solve(&[1.0, 2.0]).is_ok(), "{}: first solve", name);

        assert!(
            solver.factor(&s).is_err(),
            "{}: singular factor should return Err",
            name
        );
        assert!(
            matches!(solver.solve(&[1.0, 2.0]), Err(SolverError::NotFactorized)),
            "{}: stale factors usable after singular factor()",
            name
        );

        solver.factor(&a).unwrap();
        assert!(
            solver.solve(&[1.0, 2.0]).is_ok(),
            "{}: solve after refactor",
            name
        );
    }
}

// ===========================================================================
// E. Beam FEM manual solver selection consistency
// ===========================================================================

fn steel() -> Material {
    Material::new(200e9, 0.3, 7850.0, "Steel")
}

/// Cantilever with a downward tip force; single element of length 1.
fn cantilever_tip_force() -> BeamModel {
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, 1.0, 0.0));
    let sec = BeamSection::new(0.02, 0.1 * 0.2_f64.powi(3) / 12.0);
    model.add_element(BeamElement::new(0, 1, steel(), sec).unwrap());
    model.fix_node(0);
    model.add_nodal_force(1, 1, -1000.0);
    model
}

/// Composite Simpson integration of `f` over a uniform grid of `n` intervals.
fn simpson(f: &[f64], dx: f64) -> f64 {
    let n = f.len() - 1;
    assert!(n.is_multiple_of(2));
    let mut s = f[0] + f[n];
    for (k, &v) in f.iter().enumerate().take(n).skip(1) {
        s += if k % 2 == 0 { 2.0 * v } else { 4.0 * v };
    }
    s * dx / 3.0
}

/// Comparable per-backend record for the beam solver-consistency test.
#[derive(Clone, Copy)]
struct BeamRecord {
    uy: f64,
    rz: f64,
    end: [f64; 6],
    m_mid: f64,
    energy: f64,
}

#[test]
fn test_beam_manual_solver_consistency() {
    let e = 200e9;
    let i = 0.1 * 0.2_f64.powi(3) / 12.0;
    let ei = e * i;
    let p = 1000.0;
    let l = 1.0;

    let names = ["dense", "skyline_ldlt", "sparse_lu"];
    let mut reference: Option<BeamRecord> = None;

    for name in names {
        let mut solver = BeamSolver::from_model(&cantilever_tip_force()).unwrap();
        solver.set_solver(SolverSelection::named(name));
        solver
            .solve_configured()
            .unwrap_or_else(|e| panic!("{}: solve_configured failed: {:?}", name, e));
        assert_eq!(solver.solver_name(), Some(name), "reported backend");

        let r = solver.results();
        let uy = r.displacement(1).unwrap().uy;
        let rz = r.displacement(1).unwrap().rz;
        let fy = r.reaction(0).unwrap().fy;
        let mz = r.reaction(0).unwrap().mz;
        let end = r.element_end_forces(0).unwrap();
        let m_mid = r.section_forces(0, 0.5).unwrap().moment;

        // Strain energy U = ∫ M²/(2EI) dx.
        let n = 100;
        let dx = l / n as f64;
        let m2: Vec<f64> = (0..=n)
            .map(|k| {
                let m = r.section_forces(0, k as f64 / n as f64).unwrap().moment;
                m * m
            })
            .collect();
        let energy = simpson(&m2, dx) / (2.0 * ei);

        // Analytical checks (project sign convention).
        assert_rel(
            uy,
            -p * l.powi(3) / (3.0 * ei),
            1e-9,
            &format!("{} uy", name),
        );
        assert_rel(
            rz,
            -p * l.powi(2) / (2.0 * ei),
            1e-9,
            &format!("{} rz", name),
        );
        assert_rel(fy, p, 1e-9, &format!("{} Ry", name));
        assert_rel(mz, p * l, 1e-9, &format!("{} Rz", name));
        assert_rel(
            energy,
            p * p * l.powi(3) / (6.0 * ei),
            1e-9,
            &format!("{} U", name),
        );

        // Free-DOF residual ≈ 0 (the beam's reaction at the free tip is ~0).
        assert!(
            r.reaction(1).unwrap().fy.abs() < 1e-6,
            "{} free residual",
            name
        );

        let current = BeamRecord {
            uy,
            rz,
            end,
            m_mid,
            energy,
        };
        match &reference {
            None => reference = Some(current),
            Some(rf) => {
                assert_rel(uy, rf.uy, 1e-9, "cross-backend uy");
                assert_rel(rz, rf.rz, 1e-9, "cross-backend rz");
                assert_rel(end[4], rf.end[4], 1e-9, "cross-backend V_j");
                assert_rel(m_mid, rf.m_mid, 1e-9, "cross-backend M(0.5)");
                assert_rel(energy, rf.energy, 1e-9, "cross-backend energy");
            }
        }
        println!(
            "[beam {:11}] uy={:.6e} M(0.5)={:.6e} U={:.6e}",
            name, uy, m_mid, energy
        );
    }
}

/// CG is applicable to the SPD condensed beam system; validate it against dense.
#[test]
fn test_beam_cg_selection() {
    let mut dense = BeamSolver::from_model(&cantilever_tip_force()).unwrap();
    dense.set_solver(SolverSelection::sparse_lu());
    dense.solve_configured().unwrap();

    let mut cg = BeamSolver::from_model(&cantilever_tip_force()).unwrap();
    cg.set_solver(SolverSelection::cg());
    match cg.solve_configured() {
        Ok(()) => {
            assert_eq!(cg.solver_name(), Some("cg"));
            assert_rel(
                cg.displacement(1, 1),
                dense.displacement(1, 1),
                1e-8,
                "cg vs sparse_lu uy",
            );
            assert_rel(
                cg.reactions()[1],
                dense.reactions()[1],
                1e-8,
                "cg vs sparse_lu Ry",
            );
            println!("[beam cg] uy={:.6e}", cg.displacement(1, 1));
        }
        Err(e) => {
            // CG may legitimately decline; the error must be an explicit solver
            // error (never a silently wrong answer).
            assert!(
                matches!(e, section_properties::beam_fem::FemError::SolverError(_)),
                "cg failed with a non-solver error: {:?}",
                e
            );
            println!("[beam cg] declined: {:?}", e);
        }
    }
}

// ===========================================================================
// F. Observability of the selected solver
// ===========================================================================

#[test]
fn test_beam_reports_selected_solver() {
    // solve_configured records the chosen backend.
    let mut solver = BeamSolver::from_model(&cantilever_tip_force()).unwrap();
    assert_eq!(solver.solver_name(), None, "no solve yet");
    solver.set_solver(SolverSelection::sparse_lu());
    solver.solve_configured().unwrap();
    assert_eq!(solver.solver_name(), Some("sparse_lu"));
    assert_eq!(solver.results().solver_name(), Some("sparse_lu"));

    // Auto selection (small system) uses dense and is observable.
    let mut auto = BeamSolver::from_model(&cantilever_tip_force()).unwrap();
    assert!(auto.solver_selection().is_auto());
    auto.solve_configured().unwrap();
    assert_eq!(auto.solver_name(), Some("dense"));
    assert_eq!(auto.results().solver_name(), Some("dense"));

    // The legacy solve(&mut LinearSolver) path also records the name.
    let registry = SolverRegistry::default();
    let mut legacy = BeamSolver::from_model(&cantilever_tip_force()).unwrap();
    let mut ls = registry.create("sparse_lu").unwrap();
    legacy.solve(&mut *ls).unwrap();
    assert_eq!(legacy.solver_name(), Some("sparse_lu"));
}

// ===========================================================================
// G. max_size capability validation
// ===========================================================================

/// Lightweight diagonal SPD matrix (n entries), no dense fill-in.
fn diag(n: usize, v: f64) -> SparseMatrix {
    let mut a = SparseMatrix::new(n);
    for i in 0..n {
        a.add(i, i, v);
    }
    a.compress();
    a
}

/// A explicit selection that violates `max_size` must be rejected with a
/// structured error instead of silently switching to another backend.
#[test]
fn test_max_size_validation() {
    let registry = SolverRegistry::default();

    // Dense: 500 accepted, 501 rejected.
    assert!(
        registry
            .create_selected(&diag(500, 4.0), &SolverSelection::dense())
            .is_ok()
    );
    let err = match registry.create_selected(&diag(501, 4.0), &SolverSelection::dense()) {
        Ok(_) => panic!("dense should reject n=501"),
        Err(e) => e,
    };
    assert!(matches!(err, SolverError::Unsupported(_)), "got {:?}", err);
    let msg = err.to_string();
    assert!(msg.contains("dense"), "{}", msg);
    assert!(msg.contains("500") && msg.contains("501"), "{}", msg);
    println!("[max_size dense] {}", msg);

    // Skyline LDLT: 10000 accepted, 10001 rejected.
    assert!(
        registry
            .create_selected(&diag(10000, 4.0), &SolverSelection::skyline_ldlt())
            .is_ok()
    );
    let err = match registry.create_selected(&diag(10001, 4.0), &SolverSelection::skyline_ldlt()) {
        Ok(_) => panic!("skyline_ldlt should reject n=10001"),
        Err(e) => e,
    };
    assert!(matches!(err, SolverError::Unsupported(_)), "got {:?}", err);
    let msg = err.to_string();
    assert!(msg.contains("10000") && msg.contains("10001"), "{}", msg);
    println!("[max_size skyline] {}", msg);

    // Sparse LU: 3000 accepted, 3001 rejected.
    assert!(
        registry
            .create_selected(&diag(3000, 4.0), &SolverSelection::sparse_lu())
            .is_ok()
    );
    let err = match registry.create_selected(&diag(3001, 4.0), &SolverSelection::sparse_lu()) {
        Ok(_) => panic!("sparse_lu should reject n=3001"),
        Err(e) => e,
    };
    assert!(matches!(err, SolverError::Unsupported(_)), "got {:?}", err);
    let msg = err.to_string();
    assert!(msg.contains("3000") && msg.contains("3001"), "{}", msg);
    println!("[max_size sparse_lu] {}", msg);

    // CG / ICCG have no size limit: a large diagonal system is accepted.
    let big = diag(20000, 4.0);
    assert!(
        registry
            .create_selected(&big, &SolverSelection::cg())
            .is_ok(),
        "cg should accept a large system"
    );
    assert!(
        registry
            .create_selected(&big, &SolverSelection::iccg())
            .is_ok(),
        "iccg should accept a large system"
    );

    // validate_selection enforces the same rule.
    assert!(
        registry
            .validate_selection("dense", &diag(501, 4.0))
            .is_err()
    );
    assert!(
        registry
            .validate_selection("dense", &diag(500, 4.0))
            .is_ok()
    );

    // Auto still selects an appropriate backend (dense is out of range at 501).
    let info = registry
        .select(&diag(501, 4.0), &SolverSelection::Auto)
        .unwrap();
    assert_eq!(info.solver_name, "skyline_ldlt");
}

// ===========================================================================
// H. PARDISO is not a default registry backend
// ===========================================================================

#[test]
fn test_pardiso_not_available_by_default() {
    let registry = SolverRegistry::default();

    // Not registered (the unified wrapper is a stub), so it is never claimed.
    assert!(
        !registry.list().contains(&"pardiso".to_string()),
        "pardiso must not be registered by default"
    );

    let a = spd_tridiagonal(4);
    let err = match registry.create_selected(&a, &SolverSelection::pardiso()) {
        Ok(_) => panic!("pardiso must not be selectable from the default registry"),
        Err(e) => e,
    };
    assert!(
        matches!(err, SolverError::Unsupported(_)),
        "expected Unsupported, got {:?}",
        err
    );

    // Auto must not claim PARDISO for the default registry, even for a very
    // large system.
    let info = registry
        .select(&nonsymmetric(20000), &SolverSelection::Auto)
        .unwrap();
    assert!(
        info.solver_name != "pardiso",
        "auto claimed pardiso: {:?}",
        info
    );
    println!(
        "[pardiso] not registered; auto picked '{}'",
        info.solver_name
    );
}

// ===========================================================================
// I. solver_name must not remain stale after a failed solve
// ===========================================================================

#[test]
fn test_solver_name_invalidated_on_failed_solve() {
    let mut solver = BeamSolver::from_model(&cantilever_tip_force()).unwrap();
    assert_eq!(solver.solver_name(), None, "no solve yet");

    // Successful configured solve records the backend.
    solver.set_solver(SolverSelection::sparse_lu());
    solver.solve_configured().unwrap();
    assert_eq!(solver.solver_name(), Some("sparse_lu"));
    assert_eq!(solver.results().solver_name(), Some("sparse_lu"));

    // Switch to an unavailable backend: the solve must fail and the previously
    // recorded name must be cleared (never claim the failed backend produced
    // the current solution, and never leave a stale "sparse_lu" claim).
    solver.set_solver(SolverSelection::named("nonexistent"));
    assert!(solver.solve_configured().is_err(), "expected solve to fail");
    assert_eq!(
        solver.solver_name(),
        None,
        "stale solver_name after a failed solve"
    );
    assert_eq!(solver.results().solver_name(), None);

    // The legacy solve() path clears the name at the start as well.
    let registry = SolverRegistry::default();
    let mut legacy = BeamSolver::from_model(&cantilever_tip_force()).unwrap();
    let mut ls = registry.create("dense").unwrap();
    legacy.solve(&mut *ls).unwrap();
    assert_eq!(legacy.solver_name(), Some("dense"));
}
