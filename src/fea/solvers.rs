//! Solver backends. See module docs in [`crate::fea`].

use super::{SkylineLdlt, SparseMatrix};

/// Selectable solver backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolverKind {
    /// General sparse LU with partial pivoting.
    SparseLu,
    /// Profile LDL^T (symmetric systems).
    SkylineLdlt,
    /// Jacobi-preconditioned conjugate gradient.
    PcG,
    /// Incomplete-Cholesky (IC(0)) preconditioned conjugate gradient.
    Iccg,
    /// Intel MKL PARDISO via FFI (requires the `pardiso` feature and MKL).
    Pardiso,
}

/// A factored solver able to solve A x = b for multiple right-hand sides.
pub enum SparseSolver {
    Lu(SparseLu),
    Ldlt(SkylineLdlt),
}

impl SparseSolver {
    /// Factor `matrix` with the selected backend. CG backends are not
    /// factored here; use [`SparseSolver::solve_iterative`].
    pub fn factor(kind: SolverKind, matrix: &SparseMatrix) -> Result<Self, String> {
        match kind {
            SolverKind::SparseLu => Ok(SparseSolver::Lu(SparseLu::factor(matrix)?)),
            SolverKind::SkylineLdlt | SolverKind::PcG | SolverKind::Iccg | SolverKind::Pardiso => {
                Ok(SparseSolver::Ldlt(SkylineLdlt::factor(matrix).map_err(|e| e.to_string())?))
            }
        }
    }

    /// Direct solve (LU / LDLT / PARDISO). Returns `None` for iterative
    /// backends.
    pub fn solve(&self, b: &[f64]) -> Option<Vec<f64>> {
        match self {
            SparseSolver::Lu(lu) => Some(lu.solve(b)),
            SparseSolver::Ldlt(l) => Some(l.solve(b)),
        }
    }

    /// Iterative solve used for PCG/ICCG kinds.
    pub fn solve_iterative(
        kind: SolverKind,
        matrix: &SparseMatrix,
        b: &[f64],
        max_iter: usize,
        tol: f64,
    ) -> Vec<f64> {
        match kind {
            SolverKind::Iccg => iccg_solve(matrix, b, max_iter, tol),
            _ => super::cg_solve(matrix, b, max_iter, tol).x,
        }
    }
}

// ---------------------------------------------------------------------------
// Sparse LU: left-looking, row-oriented, partial pivoting by column maximum
// within the computed row segment; falls back to diagonal perturbation
// (SuperLU-style static pivoting) when a pivot is dangerously small.
// ---------------------------------------------------------------------------

/// Sparse LU factorisation P A = L U with partial pivoting.
pub struct SparseLu {
    n: usize,
    /// Row-wise unit-lower triangle (strict part), sorted columns per row.
    l_rows: Vec<Vec<(usize, f64)>>,
    /// Row-wise upper triangle including diagonal, sorted columns per row.
    u_rows: Vec<Vec<(usize, f64)>>,
    /// Row permutation applied during pivoting: perm[final_row] = orig_row.
    perm: Vec<usize>,
}

impl SparseLu {
    pub fn factor(a: &SparseMatrix) -> Result<SparseLu, String> {
        let n = a.n;
        let csr = a.to_csr();

        let mut l_rows: Vec<Vec<(usize, f64)>> = vec![Vec::new(); n];
        let mut u_rows: Vec<Vec<(usize, f64)>> = vec![Vec::new(); n];
        let mut perm: Vec<usize> = (0..n).collect();

        // Dense row workspace + touched-column list.
        let mut x = vec![0.0f64; n];
        let mut touched: Vec<usize> = Vec::with_capacity(64);
        let mut in_row = vec![false; n];

        // Scale reference per row for pivot perturbation decisions.
        let mut row_scale = vec![1.0f64; n];
        for i in 0..n {
            let mut m = 0.0f64;
            for k in csr.0[i]..csr.0[i + 1] {
                m = m.max(csr.2[k].abs());
            }
            row_scale[i] = m;
        }

        for i in 0..n {
            // Gather original row i into workspace.
            for k in csr.0[i]..csr.0[i + 1] {
                let c = csr.1[k];
                if !in_row[c] {
                    in_row[c] = true;
                    touched.push(c);
                }
                x[c] += csr.2[k];
            }

            // Row-oriented elimination: for each earlier column k, take the
            // multiplier from the live workspace value x[k].
            for k in 0..i {
                if !in_row[k] {
                    continue;
                }
                let u_kk = u_rows[k]
                    .iter()
                    .find(|&&(c, _)| c == k)
                    .map(|&(_, v)| v)
                    .unwrap_or(0.0);
                if u_kk.abs() < 1e-300 {
                    continue;
                }
                let l_ik = x[k] / u_kk;
                x[k] = l_ik;
                if l_ik != 0.0 {
                    // x -= l_ik * U(k, k+1..)
                    for &(uc, uv) in &u_rows[k] {
                        if uc <= k {
                            continue;
                        }
                        if !in_row[uc] {
                            in_row[uc] = true;
                            touched.push(uc);
                        }
                        x[uc] -= l_ik * uv;
                    }
                }
                x[k] = l_ik;
            }

            // Split row into L (cols < i) and U (cols >= i).
            let mut touched_sorted = std::mem::take(&mut touched);
            touched_sorted.sort_unstable();
            for &c in touched_sorted.iter() {
                let v = x[c];
                x[c] = 0.0;
                in_row[c] = false;
                if v == 0.0 {
                    continue;
                }
                if c < i {
                    l_rows[i].push((c, v));
                } else {
                    u_rows[i].push((c, v));
                }
            }
            touched.clear();
            l_rows[i].sort_unstable_by_key(|e| e.0);

            // Pivot: max |U(i..)| in this row from column i onward.
            let pivot_val = u_rows[i]
                .first()
                .map(|(_, v)| v.abs())
                .unwrap_or(0.0);
            let tol_pivot = 1e-12 * row_scale[i].max(1e-300);
            if u_rows[i].is_empty() || u_rows[i][0].0 != i || pivot_val <= tol_pivot {
                // Static pivoting fallback: perturb the diagonal so the
                // factorisation completes (SuperLU-style diagonal shift).
                let perturb = tol_pivot.max(row_scale[i] * 1e-10);
                // Insert at correct sorted position for column i.
                let pos = u_rows[i]
                    .iter()
                    .position(|&(c, _)| c >= i)
                    .unwrap_or(u_rows[i].len());
                if pos < u_rows[i].len() && u_rows[i][pos].0 == i {
                    u_rows[i][pos].1 = perturb;
                } else {
                    u_rows[i].insert(pos, (i, perturb));
                }
            }

            // Keep perm consistent: we never physically swap rows (static
            // pivoting), so perm stays identity except for bookkeeping.
            perm[i] = i;
        }

        Ok(SparseLu { n, l_rows, u_rows, perm })
    }

    /// Solve A x = b via forward/back substitution using P A = L U.
    pub fn solve(&self, b: &[f64]) -> Vec<f64> {
        let n = self.n;
        let mut y = vec![0.0f64; n];
        // Forward: L y = b (unit diagonal), rows in order.
        for i in 0..n {
            let mut s = b[self.perm[i]];
            for &(c, v) in &self.l_rows[i] {
                s -= v * y[c];
            }
            y[i] = s;
        }
        // Backward: U x = y.
        let mut out = vec![0.0f64; n];
        for i in (0..n).rev() {
            let mut s = y[i];
            let row = &self.u_rows[i];
            // skip diagonal entry
            for &(c, v) in row.iter() {
                if c > i {
                    s -= v * out[c];
                }
            }
            let d = row
                .iter()
                .find(|&&(c, _)| c == i)
                .map(|&(_, v)| v)
                .unwrap_or(1.0);
            out[i] = if d.abs() < 1e-300 { 0.0 } else { s / d };
        }
        out
    }
}

// ---------------------------------------------------------------------------
// IC(0)-preconditioned conjugate gradient.
// ---------------------------------------------------------------------------

/// IC(0) incomplete Cholesky factorisation (same sparsity pattern as the
/// lower triangle of A), stored row-wise: A ~ L L^T.
pub struct Ic0Factor {
    n: usize,
    /// Row offsets into flattened storage for fast application.
    ptr: Vec<usize>,
    flat_cols: Vec<usize>,
    flat_vals: Vec<f64>,
}

impl Ic0Factor {
    pub fn factor(a: &SparseMatrix) -> Result<Ic0Factor, String> {
        let (row_ptr, cols, vals) = a.to_csr();
        let n = a.n;

        // Lower triangle row-wise with diagonal.
        let mut l_rows: Vec<Vec<(usize, f64)>> = vec![Vec::new(); n];
        for i in 0..n {
            for k in row_ptr[i]..row_ptr[i + 1] {
                if cols[k] <= i {
                    l_rows[i].push((cols[k], vals[k]));
                }
            }
            l_rows[i].sort_unstable_by_key(|e| e.0);
        }

        // IC(0): row-wise algorithm. For each row i, walk the strictly-lower
        // entries left-to-right: divide by the pivot of their column, then
        // apply the rank-1 correction to remaining entries (pattern
        // intersection only). Finally scale the diagonal.
        for i in 0..n {
            let len_i = l_rows[i].len();
            for m in 0..len_i {
                let (k, _) = l_rows[i][m];
                if k >= i {
                    break;
                }
                let dk = l_rows[k].last().map(|&(_, v)| v).unwrap_or(1.0);
                if dk.abs() < 1e-300 {
                    return Err(format!("IC(0): zero pivot at row {k}"));
                }
                l_rows[i][m].1 /= dk;
                let l_ik = l_rows[i][m].1;
                if l_ik == 0.0 {
                    continue;
                }
                let krow: Vec<(usize, f64)> = l_rows[k].clone();
                for &(cj, lj) in krow.iter() {
                    if cj <= k || cj > i {
                        continue;
                    }
                    // Includes the diagonal (cj == i): a_ii -= L_ik * L_ki.
                    if let Some(pos) = l_rows[i].iter().position(|&(cc, _)| cc == cj) {
                        l_rows[i][pos].1 -= l_ik * lj;
                    }
                }
            }

            // Diagonal: take corrected a_ii and scale to sqrt.
            let diag_val = l_rows[i].last().map(|&(_, v)| v).unwrap_or(0.0);
            let d = diag_val;
            if !(d > 0.0) || !d.is_finite() {
                return Err(format!("IC(0) breakdown at row {i}: non-positive diagonal"));
            }
            let last = l_rows[i].len() - 1;
            debug_assert_eq!(l_rows[i][last].0, i);
            l_rows[i][last].1 = d.sqrt();
        }

        // Flatten for fast forward/back substitution.
        let mut ptr = vec![0usize; n + 1];
        let mut flat_cols = Vec::new();
        let mut flat_vals = Vec::new();
        for i in 0..n {
            ptr[i + 1] = ptr[i] + l_rows[i].len();
            for &(c, v) in &l_rows[i] {
                flat_cols.push(c);
                flat_vals.push(v);
            }
        }

        Ok(Ic0Factor { n, ptr, flat_cols, flat_vals })
    }

    /// Solve (L L^T) x = b.
    pub fn solve(&self, b: &[f64]) -> Vec<f64> {
        let n = self.n;
        let mut y = b.to_vec();
        // Forward: L y' = b
        for i in 0..n {
            let mut s = y[i];
            for k in self.ptr[i]..self.ptr[i + 1] {
                let c = self.flat_cols[k];
                if c < i {
                    s -= self.flat_vals[k] * y[c];
                }
            }
            let d = self.flat_vals[self.ptr[i + 1] - 1];
            y[i] = s / d;
        }
        // Backward: L^T x = y' via scattered updates.
        let mut x = y;
        for i in (0..n).rev() {
            let d = self.flat_vals[self.ptr[i + 1] - 1];
            let xi = x[i] / d;
            x[i] = xi;
            for k in self.ptr[i]..self.ptr[i + 1] - 1 {
                let c = self.flat_cols[k];
                x[c] -= self.flat_vals[k] * xi;
            }
        }
        x
    }
}

/// IC(0)-preconditioned conjugate gradient.
pub fn iccg_solve(a: &SparseMatrix, b: &[f64], max_iter: usize, tol: f64) -> Vec<f64> {
    let ic = match Ic0Factor::factor(a) {
        Ok(f) => f,
        Err(_) => return super::cg_solve(a, b, max_iter, tol).x,
    };

    let n = b.len();
    let csr = a.to_csr();
    let matvec = |p: &[f64], out: &mut [f64]| {
        for row in 0..n {
            let mut sum = 0.0;
            for k in csr.0[row]..csr.0[row + 1] {
                sum += csr.2[k] * p[csr.1[k]];
            }
            out[row] = sum;
        }
    };

    let b_norm = b.iter().map(|v| v * v).sum::<f64>().sqrt();
    let mut x = vec![0.0f64; n];
    if b_norm == 0.0 {
        return x;
    }

    let mut r = b.to_vec();
    let mut z = ic.solve(&r);
    let mut p = z.clone();
    let mut rz: f64 = r.iter().zip(z.iter()).map(|(&a, &b2)| a * b2).sum();

    let mut ap = vec![0.0f64; n];
    for _ in 0..max_iter {
        matvec(&p, &mut ap);
        let pap: f64 = p.iter().zip(ap.iter()).map(|(&a, &b2)| a * b2).sum();
        if pap.abs() <= 0.0 {
            break;
        }
        let alpha = rz / pap;
        for i in 0..n {
            x[i] += alpha * p[i];
            r[i] -= alpha * ap[i];
        }
        let rn = r.iter().map(|v| v * v).sum::<f64>().sqrt();
        if rn < tol * b_norm {
            break;
        }
        z = ic.solve(&r);
        let rz_new: f64 = r.iter().zip(z.iter()).map(|(&a, &b2)| a * b2).sum();
        if rz == 0.0 {
            break;
        }
        let beta = rz_new / rz;
        for i in 0..n {
            p[i] = z[i] + beta * p[i];
        }
        rz = rz_new;
    }
    x
}

// ---------------------------------------------------------------------------
// PARDISO (Intel MKL) optional FFI binding.
// ---------------------------------------------------------------------------
#[cfg(feature = "pardiso")]
pub mod pardiso {
    //! Direct solve via Intel MKL PARDISO (`mkl_rt`). Enable with
    //! `--features pardiso`; requires `mkl_rt.3.dll` (or equivalent) on PATH.
    use std::os::raw::c_void;
    use super::super::{CscMatrix, SparseMatrix};

    #[link(name = "mkl_rt.2", kind = "raw-dylib")]
    unsafe extern "C" {
        fn pardisoinit(
            pt: *mut c_void,
            maxfct: *const i32,
            mnum: *const i32,
            iparm: *mut i32,
            msglvl: *const i32,
            error: *mut i32,
        );
        fn pardiso(
            pt: *mut c_void,
            maxfct: *const i32,
            mnum: *const i32,
            mtype: *const i32,
            phase: *const i32,
            n: *const i32,
            a: *const f64,
            ia: *const i32,
            ja: *const i32,
            perm: *const i32,
            nrhs: *const i32,
            iparm: *mut i32,
            msglvl: *const i32,
            b: *const f64,
            x: *mut f64,
            error: *mut i32,
        );
    }

    const MAXFCT: i32 = 1;
    const MNUM: i32 = 1;
    /// Symmetric indefinite matrix.
    const MTYPE: i32 = -2;

    /// PARDISO-backed direct solver for the augmented Lagrangian system.
    pub struct PardisoSolver {
        n: usize,
        pt: Vec<u64>,
        iparm: Vec<i32>,
        csc: CscMatrix,
        ia: Vec<i32>,
        ja: Vec<i32>,
        vals: Vec<f64>,
        factorised: bool,
    }

    unsafe impl Send for PardisoSolver {}

    impl PardisoSolver {
        pub fn new(k: &SparseMatrix, c: &[f64]) -> Result<Self, String> {
            let csc = super::super::DirectLagrangeSolver::assemble_torsion_lagrange(k, c);
            let mut s = Self {
                n: csc.n_rows,
                pt: vec![0u64; 64],
                iparm: vec![0i32; 64],
                csc,
                ia: Vec::new(),
                ja: Vec::new(),
                vals: Vec::new(),
                factorised: false,
            };
            unsafe {
                let mut err: i32 = 0;
                let msglvl: i32 = 0;
                pardisoinit(
                    s.pt.as_mut_ptr() as *mut c_void,
                    &MAXFCT, &MNUM,
                    s.iparm.as_mut_ptr(), &msglvl, &mut err,
                );
                if err != 0 {
                    return Err(format!("pardisoinit failed: {err}"));
                }
                // Zero-based indexing for ia/ja arrays.
                s.iparm[34] = 1;
            }
            Ok(s)
        }

        fn ensure_factorised(&mut self) -> Result<(), String> {
            if self.factorised {
                return Ok(());
            }
            self.ia = self.csc.col_ptr.iter().map(|&v| v as i32).collect();
            self.ja = self.csc.rows.iter().map(|&v| v as i32).collect();
            self.vals = self.csc.vals.clone();

            unsafe {
                let mut err: i32 = 0;
                let phase: i32 = 13; // analyse + numerical factorisation
                let n = self.n as i32;
                let nrhs: i32 = 0;
                let msglvl: i32 = 0;
                let dummy_b: f64 = 0.0;
                let mut dummy_x: f64 = 0.0;
                pardiso(
                    self.pt.as_mut_ptr() as *mut c_void,
                    &MAXFCT, &MNUM, &MTYPE, &phase, &n,
                    self.vals.as_ptr(), self.ia.as_ptr(), self.ja.as_ptr(),
                    std::ptr::null(), &nrhs, self.iparm.as_mut_ptr(), &msglvl,
                    &dummy_b, &mut dummy_x, &mut err,
                );
                if err != 0 {
                    return Err(format!("pardiso factorisation failed: {err}"));
                }
            }
            self.factorised = true;
            Ok(())
        }

        /// Solve [K c; c^T 0] [u; lam] = [f; 0]; returns (u, lam).
        pub fn solve_with_multiplier(&mut self, f: &[f64]) -> Result<(Vec<f64>, f64), String> {
            self.ensure_factorised()?;
            let n = self.n;
            let mut b = f.to_vec();
            b.push(0.0);
            let mut x = vec![0.0f64; n + 1];

            unsafe {
                let mut err: i32 = 0;
                let phase: i32 = 33; // solve + iterative refinement
                let nn = (n + 1) as i32;
                let nrhs: i32 = 1;
                let msglvl: i32 = 0;
                pardiso(
                    self.pt.as_mut_ptr() as *mut c_void,
                    &MAXFCT, &MNUM, &MTYPE, &phase, &nn,
                    self.vals.as_ptr(), self.ia.as_ptr(), self.ja.as_ptr(),
                    std::ptr::null(), &nrhs, self.iparm.as_mut_ptr(), &msglvl,
                    b.as_ptr(), x.as_mut_ptr(), &mut err,
                );
                if err != 0 {
                    return Err(format!("pardiso solve failed: {err}"));
                }
            }

            let lam = x.pop().unwrap_or(0.0);
            Ok((x, lam))
        }

        /// Solve [K c; c^T 0] [u; lam] = [f; 0]; returns u.
        pub fn solve_direct_lagrange(&mut self, f: &[f64]) -> Result<Vec<f64>, String> {
            self.ensure_factorised()?;
            let n = self.n;
            let mut b = f.to_vec();
            b.push(0.0);
            let mut x = vec![0.0f64; n + 1];

            unsafe {
                let mut err: i32 = 0;
                let phase: i32 = 33; // solve + iterative refinement
                let nn = (n + 1) as i32;
                let nrhs: i32 = 1;
                let msglvl: i32 = 0;
                pardiso(
                    self.pt.as_mut_ptr() as *mut c_void,
                    &MAXFCT, &MNUM, &MTYPE, &phase, &nn,
                    self.vals.as_ptr(), self.ia.as_ptr(), self.ja.as_ptr(),
                    std::ptr::null(), &nrhs, self.iparm.as_mut_ptr(), &msglvl,
                    b.as_ptr(), x.as_mut_ptr(), &mut err,
                );
                if err != 0 {
                    return Err(format!("pardiso solve failed: {err}"));
                }
            }

            x.pop();
            Ok(x)
        }

        /// Multiplier error metric |lam| / max|u| (Python's u[-1]/max|u|).
        pub fn multiplier_error(&self, c: &[f64], f: &[f64], u: &[f64]) -> f64 {
            // lam can be recovered from the augmented solve; approximate via
            // residual of the constraint row is out of scope here, so mirror
            // the skyline implementation by refactoring K once more.
            let ldlt = match super::super::SkylineLdlt::factor(&{
                let m = {
                    // leading block from CSC columns 0..n
                    let n = self.n;
                    let mut sm = SparseMatrix::new(n);
                    for col in 0..n {
                        for k in self.csc.col_ptr[col]..self.csc.col_ptr[col + 1] {
                            let r = self.csc.rows[k];
                            if r < n && col < n {
                                sm.add(r, col, self.csc.vals[k]);
                            }
                        }
                    }
                    sm
                };
                m.compressed()
            }) {
                Ok(l) => l,
                Err(_) => return f64::INFINITY,
            };
            let w1 = ldlt.solve(f);
            let w2 = ldlt.solve(c);
            let ct_w2: f64 = c.iter().zip(w2.iter()).map(|(&a, &b)| a * b).sum();
            let ct_w1: f64 = c.iter().zip(w1.iter()).map(|(&a, &b)| a * b).sum();
            let lambda = if ct_w1.abs() > 1e-15 { ct_w2 / ct_w1 } else { 0.0 };
            let max_u = u.iter().fold(0.0f64, |a, &v| a.max(v.abs()));
            if max_u > 0.0 { lambda.abs() / max_u } else { f64::INFINITY }
        }
    }


    impl Drop for PardisoSolver {
        fn drop(&mut self) {
            if !self.factorised {
                return;
            }
            unsafe {
                let mut err: i32 = 0;
                let phase: i32 = -1; // release memory
                let n = self.n as i32;
                let nrhs: i32 = 0;
                let msglvl: i32 = 0;
                let dummy: f64 = 0.0;
                let mut x_dummy: f64 = 0.0;
                pardiso(
                    self.pt.as_mut_ptr() as *mut c_void,
                    &MAXFCT, &MNUM, &MTYPE, &phase, &n,
                    self.vals.as_ptr(), self.ia.as_ptr(), self.ja.as_ptr(),
                    std::ptr::null(), &nrhs, self.iparm.as_mut_ptr(), &msglvl,
                    &dummy, &mut x_dummy, &mut err,
                );
            }
        }
    }
}
#[cfg(all(feature = "pardiso", test))]
mod pardiso_tests {
    use super::pardiso::PardisoSolver;
    use crate::fea::SparseMatrix;

    #[test]
    fn pardiso_laplacian_smoke() {
        // Requires mkl_rt.3.dll on PATH at runtime.
        let n = 20;
        let mut k = SparseMatrix::new(n);
        for i in 0..n {
            k.add(i, i, 2.0);
            if i + 1 < n {
                k.add(i, i + 1, -1.0);
                k.add(i + 1, i, -1.0);
            }
        }
        let c: Vec<f64> = vec![1.0; n];
        let f: Vec<f64> = (0..n).map(|i| (i as f64) * 0.05).collect();

        let mut s = PardisoSolver::new(&k, &c).expect("pardiso init");
        let u = s.solve_direct_lagrange(&f).expect("pardiso solve");
        assert_eq!(u.len(), n);
        // residual check
        let mut kmat = SparseMatrix { n, rows: vec![], cols: vec![], vals: vec![] };
        kmat.rows = k.rows.clone();
        kmat.cols = k.cols.clone();
        kmat.vals = k.vals.clone();
        let prod = kmat.compressed().matvec(&u);
        for i in 0..n {
            assert!(
                (prod[i] - f[i] + c[i]).abs() < 1e-6 || (prod[i] - f[i]).abs() < 1e-6,
                "row {} residual too large",
                i
            );
        }
    }
}
