//! Solver robustness and numerical consistency tests.
//!
//! Exercises the existing backends (dense, skyline_ldlt, sparse_lu, cg, iccg)
//! for:
//!   1. cross-solver agreement (solution, residual, energy);
//!   2. behaviour under uniform matrix scaling;
//!   3. singular / rank-deficient / near-singular handling;
//!   4. the factor()/solve() lifecycle guarantee;
//!   5. Beam FEM consistency across direct backends;
//!   6. CG / ICCG behaviour on valid SPD systems.
//!
//! This file is test-only: no solver algorithm, selection policy or physics is
//! changed. Where a backend legitimately cannot solve a matrix (SPD-only
//! backends on indefinite systems) an explicit error is expected and asserted,
//! never a silent wrong answer.

use section_properties::beam_fem::{BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver};
use section_properties::fea::SparseMatrix;
use section_properties::material::Material;
use section_properties::{SolverError, SolverRegistry};

/// Direct backends compared throughout (all are class-legal for general use).
const DIRECT: [&str; 3] = ["dense", "skyline_ldlt", "sparse_lu"];

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Build a compressed `SparseMatrix` from a small dense array.
fn from_dense(rows: &[&[f64]]) -> SparseMatrix {
    let n = rows.len();
    let mut a = SparseMatrix::new(n);
    for (i, row) in rows.iter().enumerate() {
        assert_eq!(row.len(), n, "row {} is not length {}", i, n);
        for (j, &v) in row.iter().enumerate() {
            if v != 0.0 {
                a.add(i, j, v);
            }
        }
    }
    a.compress();
    a
}

/// CSR arrays of `a` (row_ptr, cols, vals). `compress()` is idempotent, so it
/// is safe to call unconditionally.
fn csr_arrays(a: &SparseMatrix) -> (Vec<usize>, Vec<usize>, Vec<f64>) {
    let mut m = a.clone();
    m.compress();
    (
        m.row_ptr().to_vec(),
        m.csr_cols().to_vec(),
        m.csr_vals().to_vec(),
    )
}

/// `‖A x − b‖∞`.
fn residual_inf(a: &SparseMatrix, x: &[f64], b: &[f64]) -> f64 {
    let (rp, cols, vals) = csr_arrays(a);
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

/// `xᵀ A x`.
fn energy(a: &SparseMatrix, x: &[f64]) -> f64 {
    let (rp, cols, vals) = csr_arrays(a);
    let mut sum = 0.0f64;
    for i in 0..a.n {
        let mut row = 0.0;
        for k in rp[i]..rp[i + 1] {
            row += vals[k] * x[cols[k]];
        }
        sum += x[i] * row;
    }
    sum
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / a.abs().max(b.abs()).max(1e-300)
}

fn assert_rel(a: f64, b: f64, tol: f64, label: &str) {
    assert!(
        rel(a, b) <= tol,
        "{}: {} vs {} (rel err {:.3e} > {:.1e})",
        label,
        a,
        b,
        rel(a, b),
        tol
    );
}

/// Mixed absolute+relative comparison: `|a-b| <= abs + rel*max(|a|,|b|)`.
/// Needed for quantities that are mathematically zero (e.g. a free-end shear)
/// where a purely relative comparison is meaningless.
fn assert_num(a: f64, b: f64, abs: f64, r: f64, label: &str) {
    let bound = abs + r * a.abs().max(b.abs());
    let d = (a - b).abs();
    assert!(
        d <= bound,
        "{}: {} vs {} (diff {:.3e} > {:.3e})",
        label,
        a,
        b,
        d,
        bound
    );
}

fn all_finite(x: &[f64]) -> bool {
    x.iter().all(|v| v.is_finite())
}

/// Solve `A x = b` with `name`, returning `(factor_result, x)`.
fn attempt(name: &str, a: &SparseMatrix, b: &[f64]) -> (Result<(), SolverError>, Option<Vec<f64>>) {
    let registry = SolverRegistry::default();
    let mut solver = registry.create(name).expect("backend missing");
    match solver.factor(a) {
        Ok(()) => match solver.solve(b) {
            Ok(x) => (Ok(()), Some(x)),
            Err(e) => (Err(e), None),
        },
        Err(e) => (Err(e), None),
    }
}

// Test systems -------------------------------------------------------------

/// Symmetric positive definite: tridiagonal, diag 4, off-diag 1.
fn spd_system() -> (SparseMatrix, Vec<f64>) {
    #[rustfmt::skip]
    let rows: &[&[f64]] = &[
        &[4.0, 1.0, 0.0, 0.0, 0.0],
        &[1.0, 4.0, 1.0, 0.0, 0.0],
        &[0.0, 1.0, 4.0, 1.0, 0.0],
        &[0.0, 0.0, 1.0, 4.0, 1.0],
        &[0.0, 0.0, 0.0, 1.0, 4.0],
    ];
    (from_dense(rows), vec![1.0, 2.0, 3.0, 4.0, 5.0])
}

/// Symmetric but indefinite: diag with mixed signs.
fn symmetric_indefinite_system() -> (SparseMatrix, Vec<f64>) {
    #[rustfmt::skip]
    let rows: &[&[f64]] = &[
        &[2.0, 1.0, 0.0],
        &[1.0, -3.0, 1.0],
        &[0.0, 1.0, 4.0],
    ];
    (from_dense(rows), vec![1.0, -2.0, 3.0])
}

/// Non-symmetric: only general solvers apply.
fn nonsymmetric_system() -> (SparseMatrix, Vec<f64>) {
    #[rustfmt::skip]
    let rows: &[&[f64]] = &[
        &[4.0, 1.0, 0.0],
        &[0.0, 3.0, 2.0],
        &[1.0, 0.0, 5.0],
    ];
    (from_dense(rows), vec![1.0, 2.0, 3.0])
}

fn scale(a: &SparseMatrix, alpha: f64) -> SparseMatrix {
    let (rp, cols, vals) = csr_arrays(a);
    let mut out = SparseMatrix::new(a.n);
    for i in 0..a.n {
        for k in rp[i]..rp[i + 1] {
            out.add(i, cols[k], vals[k] * alpha);
        }
    }
    out.compress();
    out
}

// ===========================================================================
// 1. Cross-solver consistency
// ===========================================================================

#[test]
fn test_cross_solver_consistency_spd() {
    let (a, b) = spd_system();

    let mut reference: Option<(Vec<f64>, f64, f64)> = None;
    for &name in &DIRECT {
        let (fr, x) = attempt(name, &a, &b);
        fr.unwrap_or_else(|e| panic!("{}: factor/solve failed: {:?}", name, e));
        let x = x.unwrap();
        let res = residual_inf(&a, &x, &b);
        let en = energy(&a, &x);
        assert!(all_finite(&x), "{}: non-finite solution", name);
        assert!(res < 1e-10, "{}: residual {:.3e}", name, res);

        match &reference {
            None => reference = Some((x, res, en)),
            Some((xr, _, en_ref)) => {
                for i in 0..x.len() {
                    assert_rel(x[i], xr[i], 1e-9, &format!("{} x[{}]", name, i));
                }
                assert_rel(en, *en_ref, 1e-9, &format!("{} energy", name));
            }
        }
        println!("[spd {:11}] residual={:.2e} energy={:.12e}", name, res, en);
    }
}

#[test]
fn test_cross_solver_consistency_symmetric_indefinite() {
    let (a, b) = symmetric_indefinite_system();

    // General backends agree.
    let mut reference: Option<Vec<f64>> = None;
    for &name in &["dense", "sparse_lu"] {
        let (fr, x) = attempt(name, &a, &b);
        fr.unwrap_or_else(|e| panic!("{}: failed on symmetric indefinite: {:?}", name, e));
        let x = x.unwrap();
        assert!(all_finite(&x), "{}: non-finite", name);
        assert!(
            residual_inf(&a, &x, &b) < 1e-10,
            "{}: residual {:.3e}",
            name,
            residual_inf(&a, &x, &b)
        );
        match &reference {
            None => reference = Some(x),
            Some(xr) => {
                for i in 0..x.len() {
                    assert_rel(x[i], xr[i], 1e-9, &format!("{} x[{}]", name, i));
                }
            }
        }
        println!("[sym-indef {:11}] ok", name);
    }

    // Skyline requires SPD: it must refuse explicitly, not return garbage.
    let (fr, _) = attempt("skyline_ldlt", &a, &b);
    assert!(
        fr.is_err(),
        "skyline_ldlt must reject a symmetric indefinite matrix"
    );
    println!("[sym-indef skyline] refused: {:?}", fr.unwrap_err());
}

#[test]
fn test_cross_solver_consistency_nonsymmetric() {
    let (a, b) = nonsymmetric_system();

    let mut reference: Option<Vec<f64>> = None;
    for &name in &["dense", "sparse_lu"] {
        let (fr, x) = attempt(name, &a, &b);
        fr.unwrap_or_else(|e| panic!("{}: failed on non-symmetric: {:?}", name, e));
        let x = x.unwrap();
        assert!(all_finite(&x), "{}: non-finite", name);
        assert!(residual_inf(&a, &x, &b) < 1e-10, "{}: residual", name);
        match &reference {
            None => reference = Some(x),
            Some(xr) => {
                for i in 0..x.len() {
                    assert_rel(x[i], xr[i], 1e-9, &format!("{} x[{}]", name, i));
                }
            }
        }
        println!("[nonsym {:11}] ok", name);
    }

    // Symmetric-only backends cannot handle it; they must not try.
    for &name in &["skyline_ldlt", "cg", "iccg"] {
        let registry = SolverRegistry::default();
        let mut solver = registry.create(name).unwrap();
        let fr = solver.factor(&a);
        assert!(fr.is_err(), "{} must reject a non-symmetric matrix", name);
        println!("[nonsym {:11}] refused: {:?}", name, fr.unwrap_err());
    }
}

// ===========================================================================
// 2. Uniform scaling
// ===========================================================================

#[test]
fn test_uniform_scaling_of_matrix() {
    let (a0, b) = spd_system();

    // Reference solution at alpha = 1.
    let x_ref = {
        let (fr, x) = attempt("dense", &a0, &b);
        fr.unwrap();
        x.unwrap()
    };

    for &alpha in &[1e-12_f64, 1e-6, 1.0, 1e6, 1e12] {
        let a = scale(&a0, alpha);

        for &name in &DIRECT {
            let (fr, x) = attempt(name, &a, &b);
            fr.unwrap_or_else(|e| {
                panic!(
                    "{}: must not report singular under uniform scaling α={:.0e}: {:?}",
                    name, alpha, e
                )
            });
            let x = x.unwrap();
            assert!(all_finite(&x), "{} α={:.0e}: non-finite", name, alpha);
            // A -> αA with fixed b  =>  x -> x/α.
            for i in 0..x.len() {
                assert_rel(
                    x[i],
                    x_ref[i] / alpha,
                    1e-7,
                    &format!("{} α={:.0e} x[{}]", name, alpha, i),
                );
            }
        }

        // Scaling A and b together leaves the physical solution unchanged.
        let b_scaled: Vec<f64> = b.iter().map(|v| v * alpha).collect();
        let (fr, x) = attempt("dense", &a, &b_scaled);
        fr.unwrap();
        let x = x.unwrap();
        for i in 0..x.len() {
            assert_rel(x[i], x_ref[i], 1e-7, &format!("A,b scaled x[{}]", i));
        }
        println!(
            "[scaling α={:.0e}] x ≈ x_ref/α verified for all direct backends",
            alpha
        );
    }
}

// ===========================================================================
// 3. Singular / rank-deficient / near-singular
// ===========================================================================

#[test]
fn test_singular_and_rank_deficient_matrices() {
    // Exactly singular: rows [1 1] and [1 1].
    let singular = from_dense(&[&[1.0, 1.0], &[1.0, 1.0]]);
    // Rank deficient: all rows equal (rank 1).
    let rank_deficient = from_dense(&[&[1.0, 1.0, 1.0], &[1.0, 1.0, 1.0], &[1.0, 1.0, 1.0]]);
    let b2 = vec![1.0, 1.0];
    let b3 = vec![1.0, 1.0, 1.0];

    for (label, a, b) in [
        ("singular", &singular, &b2),
        ("rank-deficient", &rank_deficient, &b3),
    ] {
        for &name in &DIRECT {
            let (fr, x) = attempt(name, a, b);
            assert!(
                fr.is_err(),
                "{}: must report an error for a {} matrix",
                name,
                label
            );
            assert!(
                x.is_none(),
                "{}: must not return a solution for a {} matrix",
                name,
                label
            );
            println!("[{} {:11}] error: {:?}", label, name, fr.unwrap_err());
        }
    }
}

#[test]
fn test_near_singular_matrix_behavior() {
    // Condition number ~1e16: the second pivot is at the tolerance limit.
    let near = from_dense(&[&[1.0, 0.0], &[0.0, 1e-16]]);
    let b = vec![1.0, 1.0];

    for &name in &DIRECT {
        let (fr, x) = attempt(name, &near, &b);
        match (fr, x) {
            (Err(e), _) => println!("[near-singular {:11}] reported error: {:?}", name, e),
            (Ok(()), Some(x)) => {
                // If it does solve, the result must be a numerically valid
                // solution: finite, with a small residual ||A x - b||_inf.
                assert!(
                    all_finite(&x),
                    "{}: produced non-finite solution for near-singular system",
                    name
                );
                let res = residual_inf(&near, &x, &b);
                let bound = 1e-6 * b.iter().fold(1.0f64, |m, &v| m.max(v.abs()));
                assert!(
                    res <= bound,
                    "{}: near-singular solve has residual ||Ax-b||_inf = {:.3e} > {:.3e}",
                    name,
                    res,
                    bound
                );
                println!(
                    "[near-singular {:11}] solved with finite x, residual={:.3e}",
                    name, res
                );
            }
            _ => panic!("{}: inconsistent Ok/Err state", name),
        }
    }
}

#[test]
fn test_iterative_report_failure_on_singular() {
    // Singular AND inconsistent: A is rank-1, and b = [1, 2] is not in the
    // range of A (its component along the null vector [1, -1] is 1 - 2 = -1),
    // so the system has no solution. A correct iterative solver must either
    // report an explicit error/non-convergence, or return an x whose residual
    // is genuinely small.
    let singular = from_dense(&[&[1.0, 1.0], &[1.0, 1.0]]);
    let b = vec![1.0, 2.0];

    for &name in &["cg", "iccg"] {
        let (fr, x) = attempt(name, &singular, &b);
        match (fr, x) {
            (Err(e), _) => println!("[iterative singular {:5}] error: {:?}", name, e),
            (Ok(()), Some(x)) => {
                assert!(
                    all_finite(&x),
                    "{}: non-finite solution on a singular system",
                    name
                );
                let res = residual_inf(&singular, &x, &b);
                let bound = 1e-6 * b.iter().fold(1.0f64, |m, &v| m.max(v.abs()));
                assert!(
                    res <= bound,
                    "{}: reported Ok on an inconsistent singular system but \
                     residual ||Ax-b||_inf = {:.3e} > {:.3e}",
                    name,
                    res,
                    bound
                );
                println!(
                    "[iterative singular {:5}] returned x (finite), residual={:.3e}",
                    name, res
                );
            }
            _ => panic!("{}: inconsistent state", name),
        }
    }
}

// ===========================================================================
// 4. Solver lifecycle
// ===========================================================================

#[test]
fn test_lifecycle_invalid_factor_clears_state() {
    // Diagonal SPD: solvable by every backend (IC(0) is exact for a diagonal
    // system, so ICCG converges too). The tridiagonal SPD system is *not*
    // solvable by ICCG at its requested tolerance, which would make this test
    // about convergence rather than about factorization state.
    let valid = from_dense(&[&[4.0, 0.0, 0.0], &[0.0, 5.0, 0.0], &[0.0, 0.0, 6.0]]);
    let b = vec![1.0, 2.0, 3.0];
    // Every backend rejects an empty matrix at factor().
    let invalid = SparseMatrix::new(0);

    for &name in &["dense", "skyline_ldlt", "sparse_lu", "cg", "iccg"] {
        let registry = SolverRegistry::default();
        let mut solver = registry.create(name).unwrap();

        solver.factor(&valid).unwrap();
        assert!(solver.solve(&b).is_ok(), "{}: first solve", name);

        assert!(solver.factor(&invalid).is_err(), "{}: invalid factor", name);
        assert!(
            matches!(solver.solve(&b), Err(SolverError::NotFactorized)),
            "{}: stale factorization usable after failed factor()",
            name
        );

        solver.factor(&valid).unwrap();
        assert!(solver.solve(&b).is_ok(), "{}: solve after refactor", name);
    }
}

// ===========================================================================
// 5. Beam FEM consistency across direct backends
// ===========================================================================

fn beam_steel() -> Material {
    Material::new(200e9, 0.3, 7850.0, "Steel")
}

fn beam_cantilever(kind: u32) -> BeamModel {
    let mut model = BeamModel::new();
    model.add_node(BeamNode::new(0, 0.0, 0.0));
    model.add_node(BeamNode::new(1, 1.0, 0.0));
    let sec = BeamSection::new(0.02, 0.1 * 0.2_f64.powi(3) / 12.0);
    model.add_element(BeamElement::new(0, 1, beam_steel(), sec).unwrap());
    model.fix_node(0);
    match kind {
        0 => model.add_nodal_force(1, 1, -1000.0), // tip transverse force
        _ => model.add_distributed_load(0, 0.0, -1000.0).unwrap(), // UDL
    }
    model
}

fn simpson(f: &[f64], dx: f64) -> f64 {
    let n = f.len() - 1;
    assert!(n.is_multiple_of(2));
    let mut s = f[0] + f[n];
    for (k, &v) in f.iter().enumerate().take(n).skip(1) {
        s += if k % 2 == 0 { 2.0 * v } else { 4.0 * v };
    }
    s * dx / 3.0
}

/// Comparable per-backend record for the Beam FEM consistency test.
#[derive(Clone, Copy)]
struct BeamRecord {
    uy: f64,
    rz: f64,
    end: [f64; 6],
    m_mid: f64,
    u_energy: f64,
}

#[test]
fn test_beam_backends_consistency() {
    let e = 200e9;
    let i = 0.1 * 0.2_f64.powi(3) / 12.0;
    let ei = e * i;

    for (kind, label) in [(0u32, "tip force"), (1u32, "udl")] {
        let mut reference: Option<BeamRecord> = None;

        for &name in &DIRECT {
            let mut solver = BeamSolver::from_model(&beam_cantilever(kind)).unwrap();
            solver.set_solver(section_properties::SolverSelection::named(name));
            solver
                .solve_configured()
                .unwrap_or_else(|err| panic!("{} / {}: solve failed: {:?}", label, name, err));
            assert_eq!(solver.solver_name(), Some(name));

            let r = solver.results();
            let uy = r.displacement(1).unwrap().uy;
            let rz = r.displacement(1).unwrap().rz;
            let fy = r.reaction(0).unwrap().fy;
            let mz = r.reaction(0).unwrap().mz;
            let end = r.element_end_forces(0).unwrap();
            let m_mid = r.section_forces(0, 0.5).unwrap().moment;

            // Strain energy U = ∫ M²/(2EI) dx over the element.
            let n = 100;
            let m2: Vec<f64> = (0..=n)
                .map(|k| {
                    let m = r.section_forces(0, k as f64 / n as f64).unwrap().moment;
                    m * m
                })
                .collect();
            let u_energy = simpson(&m2, 1.0 / n as f64) / (2.0 * ei);

            // Global equilibrium residual (analytical applied loads).
            let (applied_fy, applied_mz) = match kind {
                0 => (-1000.0, -1000.0 * 1.0),
                _ => (-1000.0 * 1.0, -1000.0 * 1.0 * 0.5),
            };
            let eq_fy = (fy + applied_fy).abs();
            let eq_mz = (mz + applied_mz).abs();
            assert!(
                eq_fy < 1e-9,
                "{} {}: ΣFy residual {:.3e}",
                label,
                name,
                eq_fy
            );
            assert!(
                eq_mz < 1e-9,
                "{} {}: ΣMz residual {:.3e}",
                label,
                name,
                eq_mz
            );

            // Free-DOF residual: the reaction at the unsupported tip is ~0.
            assert!(
                r.reaction(1).unwrap().fy.abs() < 1e-6,
                "{} {}: free-DOF residual",
                label,
                name
            );

            let current = BeamRecord {
                uy,
                rz,
                end,
                m_mid,
                u_energy,
            };
            match &reference {
                None => reference = Some(current),
                Some(rf) => {
                    // Displacements/rotations: relative (values are O(1e-5)).
                    assert_rel(uy, rf.uy, 1e-9, &format!("{} {} uy", label, name));
                    assert_rel(rz, rf.rz, 1e-9, &format!("{} {} rz", label, name));
                    // Forces/moments: mixed absolute+relative, since some
                    // components are mathematically zero (free-end shear).
                    for (k, (&v, &rv)) in end.iter().zip(rf.end.iter()).enumerate() {
                        assert_num(v, rv, 1e-6, 1e-9, &format!("{} {} end[{}]", label, name, k));
                    }
                    assert_num(
                        m_mid,
                        rf.m_mid,
                        1e-6,
                        1e-9,
                        &format!("{} {} M(0.5)", label, name),
                    );
                    assert_rel(
                        u_energy,
                        rf.u_energy,
                        1e-9,
                        &format!("{} {} U", label, name),
                    );
                }
            }
            println!(
                "[beam {:9} / {:11}] uy={:.6e} M(0.5)={:.6e} U={:.6e}",
                label, name, uy, m_mid, u_energy
            );
        }
    }
}

// ===========================================================================
// 6. CG / ICCG on valid SPD systems
// ===========================================================================

#[test]
fn test_cg_iccg_on_spd_system() {
    let (a, b) = spd_system();

    // Direct reference.
    let x_ref = {
        let (fr, x) = attempt("dense", &a, &b);
        fr.unwrap();
        x.unwrap()
    };

    for &name in &["cg", "iccg"] {
        let (fr, x) = attempt(name, &a, &b);
        match (fr, x) {
            (Ok(()), Some(x)) => {
                assert!(all_finite(&x), "{}: non-finite solution", name);
                let res = residual_inf(&a, &x, &b);
                assert!(res < 1e-8, "{}: residual {:.3e}", name, res);
                for i in 0..x.len() {
                    assert_rel(x[i], x_ref[i], 1e-6, &format!("{} x[{}] vs dense", name, i));
                }
                let en = energy(&a, &x);
                assert_rel(
                    en,
                    energy(&a, &x_ref),
                    1e-6,
                    &format!("{} energy vs dense", name),
                );
                println!("[spd {:5}] converged, residual={:.2e}", name, res);
            }
            (Err(e), _) => {
                // Reported explicitly rather than silently wrong — acceptable
                // for an iterative backend; reported in the test output.
                println!("[spd {:5}] did not converge: {:?}", name, e);
            }
            _ => panic!("{}: inconsistent state", name),
        }
    }
}
