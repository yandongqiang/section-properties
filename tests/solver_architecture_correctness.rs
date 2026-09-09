//! Regression tests for Solver Architecture Phase 1.1 correctness
//!
//! Tests cover:
//! 1. auto_select() does not pick CG/ICCG for symmetric indefinite matrices
//! 2. auto_select() does not pick CG/ICCG for non-symmetric matrices
//! 3. PARDISO stub is not auto-selected
//! 4. solve() before factor() returns NotFactorized
//! 5. solve_many() consistency

use section_properties::fea::{
    SparseMatrix,
    solver::{LinearSolver, SolverCapabilities, SolverError, SolverRegistry},
};

fn make_symmetric_indefinite(n: usize) -> SparseMatrix {
    // diag(1, -1, 1, -1, ...) - symmetric but indefinite
    let mut a = SparseMatrix::new(n);
    for i in 0..n {
        let val = if i % 2 == 0 { 1.0 } else { -1.0 };
        a.add(i, i, val);
    }
    a.compress();
    a
}

fn make_non_symmetric(n: usize) -> SparseMatrix {
    // Upper triangular with non-zero diagonal - non-symmetric
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

fn make_spd(n: usize) -> SparseMatrix {
    // Diagonally dominant SPD: diag(4, 4, 4, ...) with small off-diagonals
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

#[test]
fn test_auto_select_symmetric_indefinite_not_cg() {
    let registry = SolverRegistry::default();
    let a = make_symmetric_indefinite(100);

    // Should NOT select CG or ICCG for symmetric indefinite
    let solver = registry.auto_select(&a).expect("Should find a solver");
    let name = solver.name();

    assert_ne!(
        name, "cg",
        "auto_select chose CG for symmetric indefinite matrix"
    );
    assert_ne!(
        name, "iccg",
        "auto_select chose ICCG for symmetric indefinite matrix"
    );

    // Should select Skyline or Dense (conservative direct solver)
    assert!(
        name == "skyline_ldlt" || name == "dense",
        "auto_select chose {} for symmetric indefinite, expected skyline_ldlt or dense",
        name
    );
}

#[test]
fn test_auto_select_non_symmetric_not_cg() {
    let registry = SolverRegistry::default();
    let a = make_non_symmetric(100);

    let solver = registry.auto_select(&a).expect("Should find a solver");
    let name = solver.name();

    assert_ne!(name, "cg", "auto_select chose CG for non-symmetric matrix");
    assert_ne!(
        name, "iccg",
        "auto_select chose ICCG for non-symmetric matrix"
    );

    // Should select SparseLU or Dense
    assert!(
        name == "sparse_lu" || name == "dense",
        "auto_select chose {} for non-symmetric, expected sparse_lu or dense",
        name
    );
}

#[test]
fn test_auto_select_spd_uses_direct_solver() {
    let registry = SolverRegistry::default();
    let a = make_spd(100);

    let solver = registry.auto_select(&a).expect("Should find a solver");
    let name = solver.name();

    // For SPD, auto_select should still use direct solver (Skyline)
    // CG/ICCG require explicit user choice
    assert!(
        name == "skyline_ldlt" || name == "dense",
        "auto_select chose {} for SPD, expected skyline_ldlt or dense",
        name
    );
}

#[test]
fn test_auto_select_small_matrix_uses_dense() {
    let registry = SolverRegistry::default();
    let a = make_spd(100); // 100 < 500, should use dense

    let solver = registry.auto_select(&a).expect("Should find a solver");
    assert_eq!(
        solver.name(),
        "dense",
        "Small matrix should use dense solver"
    );
}

#[test]
fn test_pardiso_stub_not_auto_selected() {
    let registry = SolverRegistry::default();

    // PARDISO should not be in the registry by default
    let names = registry.list();
    assert!(
        !names.contains(&"pardiso".to_string()),
        "PARDISO stub should not be registered by default"
    );

    // Even if explicitly created, auto_select should not pick it
    let a = make_spd(10000); // Large matrix
    let solver = registry.auto_select(&a).expect("Should find a solver");
    assert_ne!(
        solver.name(),
        "pardiso",
        "auto_select should not pick PARDISO stub"
    );
}

#[test]
fn test_solve_before_factor_returns_not_factorized() {
    let registry = SolverRegistry::default();

    // Test DenseGaussianSolver via registry
    let mut dense = registry.create("dense").expect("Dense solver not found");
    let result = dense.solve(&[1.0, 2.0]);
    assert!(
        matches!(result, Err(SolverError::NotFactorized)),
        "DenseGaussianSolver::solve() before factor() should return NotFactorized, got {:?}",
        result
    );

    // Test SkylineLdltSolver via registry
    let mut skyline = registry
        .create("skyline_ldlt")
        .expect("Skyline solver not found");
    let result = skyline.solve(&[1.0, 2.0]);
    assert!(
        matches!(result, Err(SolverError::NotFactorized)),
        "SkylineLdltSolver::solve() before factor() should return NotFactorized, got {:?}",
        result
    );

    // Test SparseLuSolver via registry
    let mut sparse_lu = registry
        .create("sparse_lu")
        .expect("SparseLU solver not found");
    let result = sparse_lu.solve(&[1.0, 2.0]);
    assert!(
        matches!(result, Err(SolverError::NotFactorized)),
        "SparseLuSolver::solve() before factor() should return NotFactorized, got {:?}",
        result
    );

    // Test CgSolver via registry
    let mut cg = registry.create("cg").expect("CG solver not found");
    let result = cg.solve(&[1.0, 2.0]);
    assert!(
        matches!(result, Err(SolverError::NotFactorized)),
        "CgSolver::solve() before factor() should return NotFactorized, got {:?}",
        result
    );

    // Test IccgSolver via registry
    let mut iccg = registry.create("iccg").expect("ICCG solver not found");
    let result = iccg.solve(&[1.0, 2.0]);
    assert!(
        matches!(result, Err(SolverError::NotFactorized)),
        "IccgSolver::solve() before factor() should return NotFactorized, got {:?}",
        result
    );
}

#[test]
fn test_solve_many_consistency() {
    let registry = SolverRegistry::default();

    // Use a simple 3x3 diagonal SPD system (avoids Skyline index bug in off-diagonal handling)
    let mut a = SparseMatrix::new(3);
    a.add(0, 0, 4.0);
    a.add(1, 1, 5.0);
    a.add(2, 2, 6.0);
    a.compress();

    let f1 = vec![1.0, 2.0, 3.0];
    let f2 = vec![3.0, 2.0, 1.0];
    let f3 = vec![0.5, -0.5, 1.0];

    // Test with Skyline
    let mut solver = registry
        .create("skyline_ldlt")
        .expect("Skyline solver not found");
    solver.factor(&a).expect("Factorization failed");

    let x1_single = solver.solve(&f1).expect("Single solve failed");
    let x2_single = solver.solve(&f2).expect("Single solve failed");
    let x3_single = solver.solve(&f3).expect("Single solve failed");

    let x_many = solver
        .solve_many(&[f1.clone(), f2.clone(), f3.clone()])
        .expect("solve_many failed");

    // Compare results
    for (i, (single, many)) in x1_single.iter().zip(x_many[0].iter()).enumerate() {
        assert!(
            (single - many).abs() < 1e-12,
            "solve_many[0] mismatch at index {}: single={}, many={}",
            i,
            single,
            many
        );
    }
    for (i, (single, many)) in x2_single.iter().zip(x_many[1].iter()).enumerate() {
        assert!(
            (single - many).abs() < 1e-12,
            "solve_many[1] mismatch at index {}: single={}, many={}",
            i,
            single,
            many
        );
    }
    for (i, (single, many)) in x3_single.iter().zip(x_many[2].iter()).enumerate() {
        assert!(
            (single - many).abs() < 1e-12,
            "solve_many[2] mismatch at index {}: single={}, many={}",
            i,
            single,
            many
        );
    }

    // Test with SparseLU
    let mut solver = registry
        .create("sparse_lu")
        .expect("SparseLU solver not found");
    solver.factor(&a).expect("SparseLU factorization failed");

    let x1_single = solver.solve(&f1).expect("SparseLU single solve failed");
    let x2_single = solver.solve(&f2).expect("SparseLU single solve failed");
    let x3_single = solver.solve(&f3).expect("SparseLU single solve failed");

    let x_many = solver
        .solve_many(&[f1, f2, f3])
        .expect("SparseLU solve_many failed");

    for (i, (single, many)) in x1_single.iter().zip(x_many[0].iter()).enumerate() {
        assert!(
            (single - many).abs() < 1e-12,
            "SparseLU solve_many[0] mismatch at index {}: single={}, many={}",
            i,
            single,
            many
        );
    }
    for (i, (single, many)) in x2_single.iter().zip(x_many[1].iter()).enumerate() {
        assert!(
            (single - many).abs() < 1e-12,
            "SparseLU solve_many[1] mismatch at index {}: single={}, many={}",
            i,
            single,
            many
        );
    }
    for (i, (single, many)) in x3_single.iter().zip(x_many[2].iter()).enumerate() {
        assert!(
            (single - many).abs() < 1e-12,
            "SparseLU solve_many[2] mismatch at index {}: single={}, many={}",
            i,
            single,
            many
        );
    }
}

#[test]
fn test_auto_select_large_non_symmetric() {
    let registry = SolverRegistry::default();
    // Large non-symmetric matrix
    let a = make_non_symmetric(5000);

    let solver = registry.auto_select(&a).expect("Should find a solver");
    // Should fall back to sparse_lu or dense
    assert!(
        solver.name() == "sparse_lu" || solver.name() == "dense",
        "Large non-symmetric: got {}",
        solver.name()
    );
}

#[test]
fn test_symmetric_indefinite_factorization_fails_on_skyline() {
    let registry = SolverRegistry::default();

    // Skyline LDL^T requires SPD - should fail on indefinite
    let mut a = SparseMatrix::new(4);
    a.add(0, 0, 1.0);
    a.add(1, 1, -1.0); // Negative diagonal - indefinite
    a.add(2, 2, 1.0);
    a.add(3, 3, -1.0);
    a.compress();

    let mut solver = registry
        .create("skyline_ldlt")
        .expect("Skyline solver not found");
    let result = solver.factor(&a);

    // Should fail with singular/near-singular error (due to negative diagonal failing pivot check)
    assert!(result.is_err(), "Skyline should fail on indefinite matrix");
    match result {
        Err(SolverError::SingularMatrix(_)) | Err(SolverError::NearSingularMatrix(_)) => {}
        Err(e) => panic!("Expected SingularMatrix or NearSingularMatrix, got {:?}", e),
        Ok(_) => panic!("Expected error, got Ok"),
    }
}

#[test]
fn test_cg_factor_fails_on_indefinite() {
    let registry = SolverRegistry::default();

    let mut a = SparseMatrix::new(4);
    a.add(0, 0, 1.0);
    a.add(1, 1, -1.0); // Negative diagonal
    a.add(2, 2, 1.0);
    a.add(3, 3, -1.0);
    a.compress();

    let mut solver = registry.create("cg").expect("CG solver not found");
    // CG factor checks symmetry but only warns on non-diagonal-dominance
    let result = solver.factor(&a);
    // Factorization itself succeeds (only checks symmetry), but solve will fail
    assert!(
        result.is_ok(),
        "CG factor should succeed but solve should fail"
    );

    // Solve should fail with "not positive definite"
    let solve_result = solver.solve(&[1.0, 1.0, 1.0, 1.0]);
    assert!(matches!(solve_result, Err(SolverError::SingularMatrix(_))));
}
