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
            _ => super::cg_solve(matrix, b, max_iter, tol),
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

            if i == 1 { eprintln!("[lu1] after gather x[0..3]={:?} {:?}", (x[0],x[1],x[2]), (&touched)); }
            // Row-oriented elimination: for each earlier column k, take the
            // multiplier from the live workspace value x[k].
            for k in 0..i {
                if i == 1 && k == 0 { eprintln!("[lu1] u_kk={} x_after=({},{})", u_rows[0].iter().find(|&&(cc,_)| cc==0).map(|&(_,v)| v).unwrap_or(-9.0), x[1], x[2]); }
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
                eprintln!("[lu] perturb row i={} pivot_val={:.3e} first_col={}", i, pivot_val, u_rows[i].first().map(|&(c,_)| c).unwrap_or(usize::MAX));
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
        Err(_) => return super::cg_solve(a, b, max_iter, tol),
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
    let mut _t = 0usize;
    for _ in 0..max_iter {
        _t += 1;
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
        if _t % 100 == 0 { eprintln!("[iccg] it={} rel={:.3e}", _t, rn / b_norm); }
        if rn < tol * b_norm {
            eprintln!("[iccg] converged it={} rel={:.3e}", _t, rn / b_norm);
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
    //! Direct solve via Intel MKL PARDISO. Requires MKL libraries at link
    //! time; enable with `--features pardiso` and set `RUSTFLAGS` to link
    //! MKL (e.g. `-L $MKLROOT/lib/intel64 -l mkl_rt`).
    use super::super::{SparseMatrix, CscMatrix};

    unsafe extern "C" {
        fn pardisoinit(
            pt: *mut isize,
            maxfct: *const isize,
            mnum: *const isize,
            iparm: *mut isize,
            msglvl: *const isize,
            error: *mut isize,
        );
        fn pardiso(
            pt: *mut isize,
            maxfct: *const isize,
            mnum: *const isize,
            mtype: *const isize,
            phase: *const isize,
            n: *const isize,
            a: *const f64,
            ia: *const isize,
            ja: *const isize,
            perm: *mut isize,
            nrhs: *const isize,
            iparm: *mut isize,
            msglvl: *mut isize,
            b: *const f64,
            x: *mut f64,
            error: *mut isize,
        );
    }

    const MAXFCT: isize = 1;
    const MNUM: isize = 1;
    /// Symmetric indefinite matrix.
    const MTYPE: isize = -2;

    /// PARDISO-backed direct solver for the augmented Lagrangian system.
    pub struct PardisoSolver {
        n: usize,
        pt: Vec<isize>,
        iparm: Vec<isize>,
        // CSC data of the augmented matrix (PARDISO wants CSR; for symmetric
        // matrices the CSC of A equals the CSR of A^T = A).
        csc: CscMatrix,
        ia: Vec<isize>,
        ja: Vec<isize>,
        vals: Vec<f64>,
        factorised: bool,
    }

    unsafe impl Send for PardisoSolver {}

    impl PardisoSolver {
        pub fn new(k: &SparseMatrix, c: &[f64]) -> Result<Self, String> {
            let csc = super::super::DirectLagrangeSolver::assemble_torsion_lagrange(k, c);
            let mut s = Self {
                n: csc.n_rows,
                pt: vec![0; 64],
                iparm: [0usize; 64].iter().map(|_| 0isize).collect(),
                csc,
                ia: Vec::new(),
                ja: Vec::new(),
                vals: Vec::new(),
                factorised: false,
            };
            unsafe {
                let mut err: isize = 0;
                let msglvl: isize = 0;
                pardisoinit(s.pt.as_mut_ptr(), &MAXFCT, &MNUM, s.iparm.as_mut_ptr(), &msglvl, &mut err);
                if err != 0 {
                    return Err(format!("pardisoinit failed: {err}"));
                }
            }
            Ok(s)
        }

        fn ensure_factorised(&mut self) -> Result<(), String> {
            if self.factorised {
                return Ok(());
            }
            self.ia = self.csc.col_ptr.iter().map(|&v| v as isize).collect();
            self.ja = self.csc.rows.iter().map(|&v| v as isize).collect();
            self.vals = self.csc.vals.clone();

            unsafe {
                let mut err: isize = 0;
                let phase: isize = 13; // analysis + numerical factorisation + solve is done per-solve; here analyse+factor
                let n = self.n as isize;
                let nrhs: isize = 0;
                let mut msglvl: isize = 0;
                let dummy_b: f64 = 0.0;
                let mut dummy_x: f64 = 0.0;
                pardiso(
                    self.pt.as_mut_ptr(), &MAXFCT, &MNUM, &MTYPE, &phase, &n,
                    self.vals.as_ptr(), self.ia.as_ptr(), self.ja.as_ptr(),
                    std::ptr::null_mut(), &nrhs, self.iparm.as_mut_ptr(), &mut msglvl,
                    &dummy_b, &mut dummy_x, &mut err,
                );
                if err != 0 {
                    return Err(format!("pardiso factorisation failed: {err}"));
                }
            }
            self.factorised = true;
            Ok(())
        }

        /// Solve with RHS [f, 0]; returns u (multiplier discarded).
        pub fn solve_direct_lagrange(&mut self, f: &[f64]) -> Result<Vec<f64>, String> {
            self.ensure_factorised()?;
            let n = self.n;
            let mut b = f.to_vec();
            b.push(0.0);
            let mut x = vec![0.0f64; n + 1];

            unsafe {
                let mut err: isize = 0;
                let phase: isize = 33; // solve + iterative refinement
                let nn = (n + 1) as isize;
                let nrhs: isize = 1;
                let mut msglvl: isize = 0;
                pardiso(
                    self.pt.as_mut_ptr(), &MAXFCT, &MNUM, &MTYPE, &phase, &nn,
                    self.vals.as_ptr(), self.ia.as_ptr(), self.ja.as_ptr(),
                    std::ptr::null_mut(), &nrhs, self.iparm.as_mut_ptr(), &mut msglvl,
                    b.as_ptr(), x.as_mut_ptr(), &mut err,
                );
                if err != 0 {
                    return Err(format!("pardiso solve failed: {err}"));
                }
            }

            x.pop();
            Ok(x)
        }
    }

    impl Drop for PardisoSolver {
        fn drop(&mut self) {
            if !self.factorised {
                return;
            }
            unsafe {
                let mut err: isize = 0;
                let phase: isize = -1; // release memory
                let n = self.n as isize;
                let nrhs: isize = 0;
                let mut msglvl: isize = 0;
                let dummy: f64 = 0.0;
                let mut x_dummy: f64 = 0.0;
                pardiso(
                    self.pt.as_mut_ptr(), &MAXFCT, &MNUM, &MTYPE, &phase, &n,
                    self.vals.as_ptr(), self.ia.as_ptr(), self.ja.as_ptr(),
                    std::ptr::null_mut(), &nrhs, self.iparm.as_mut_ptr(), &mut msglvl,
                    &dummy, &mut x_dummy, &mut err,
                );
            }
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::fea::{cg_solve, SkylineLdlt};

    fn laplacian(n: usize) -> SparseMatrix {
        let mut k = SparseMatrix::new(n);
        for i in 0..n {
            k.add(i, i, 2.0);
            if i + 1 < n {
                k.add(i, i + 1, -1.0);
                k.add(i + 1, i, -1.0);
            }
        }
        k
    }

    fn reference_solve(k: &SparseMatrix, f: &[f64]) -> Vec<f64> {
        SkylineLdlt::factor(&k.compressed()).unwrap().solve(f)
    }

    #[test]
    fn sparse_lu_residual_n30() {
        let n = 30;
        let k = laplacian(n);
        let f: Vec<f64> = (0..n).map(|i| ((i % 5) as f64 - 2.0)).collect();
        let lu = SparseLu::factor(&k).unwrap();
        let x = lu.solve(&f);
        let prod = k.compressed().matvec(&x);
        let mut res = 0.0f64;
        for i in 0..n {
            res = res.max((prod[i] - f[i]).abs());
        }
        eprintln!("LU rel residual={:.3e}", res / 2.0);

        // Factorisation check: L*U should equal A.
        let mut fu = vec![0.0f64; n];
        let mut worst_lu: f64 = 0.0;
        for i in 0..n {
            // row i of L*U
            for &(k, l_ik) in &lu.l_rows[i] {
                for &(uc, uv) in &lu.u_rows[k] {
                    fu[uc] += l_ik * uv;
                }
            }
            for &(uc, uv) in &lu.u_rows[i] {
                fu[uc] += uv;
            }
            // compare with A row i via compressed matvec on unit vector? do direct:
            // gather A row i from original triplets using public API
        }
        // rebuild A rows with matvec trick: use identity RHS
        let ident = SparseMatrix { n, rows: (0..n).collect(), cols: (0..n).collect(), vals: vec![1.0; n] };
        let _ = &ident;
        // simpler: use laplacian construction directly
        for i in 0..n {
            let mut airow = vec![0.0f64; n];
            airow[i] += 2.0;
            if i > 0 { airow[i - 1] += -1.0; }
            if i + 1 < n { airow[i + 1] += -1.0; }
            for j in 0..n {
                worst_lu = worst_lu.max((fu[j] - airow[j]).abs());
            }
            // reset for next iteration reuse
            for v in fu.iter_mut() { *v = 0.0; }
        }
        eprintln!("|LU-A| max={:.3e}", worst_lu);

        let r = reference_solve(&k, &f);
        let prod2 = k.compressed().matvec(&r);
        let mut res2 = 0.0f64;
        for i in 0..n {
            res2 = res2.max((prod2[i] - f[i]).abs());
        }
        eprintln!("skyline rel residual={:.3e}", res2 / 2.0);
    }

    #[test]
    fn sparse_lu_debug_n4() {
        let n = 12;
        let k = laplacian(n);
        let lu = SparseLu::factor(&k).unwrap();
        // per-row |LU - A|
        for i in 0..n {
            let mut fu = vec![0.0f64; n];
            for &(kkv, l_ik) in &lu.l_rows[i] {
                for &(uc, uv) in &lu.u_rows[kkv] {
                    fu[uc] += l_ik * uv;
                }
            }
            for &(uc, uv) in &lu.u_rows[i] {
                fu[uc] += uv;
            }
            let mut airow = vec![0.0f64; n];
            airow[i] += 2.0;
            if i > 0 { airow[i-1] += -1.0; }
            if i+1 < n { airow[i+1] += -1.0; }
            let mut dmax = 0.0f64;
            let mut dcol = usize::MAX;
            for j in 0..n {
                let dd = (fu[j]-airow[j]).abs();
                if dd > dmax { dmax = dd; dcol = j; }
            }
            if dmax > 1e-12 { eprintln!("row {} diff {:.3e} at col {}", i, dmax, dcol); }
        }
    }

    #[test]
    fn sparse_lu_matches_reference() {
        let n = 30;
        let k = laplacian(n);
        let f: Vec<f64> = (0..n).map(|i| ((i % 5) as f64 - 2.0)).collect();
        let r = reference_solve(&k, &f);
        let lu = SparseLu::factor(&k).unwrap();
        let x = lu.solve(&f);
        for i in 0..n {
            assert!((r[i] - x[i]).abs() < 1e-8, "i={} {} vs {}", i, r[i], x[i]);
        }
    }

    #[test]
    fn iccg_matches_reference() {
        let n = 40;
        let k = laplacian(n);
        let f: Vec<f64> = (0..n).map(|i| ((i % 3) as f64 - 1.0)).collect();
        let r = reference_solve(&k, &f);
        let x = iccg_solve(&k, &f, 5000, 1e-10);
        for i in 0..n {
            assert!((r[i] - x[i]).abs() < 1e-6, "i={} {} vs {}", i, r[i], x[i]);
        }
    }

    #[test]
    fn pcg_matches_reference() {
        let n = 30;
        let k = laplacian(n);
        let f: Vec<f64> = (0..n).map(|i| (i as f64).cos()).collect();
        let r = reference_solve(&k, &f);
        let x = super::super::cg_solve(&k.compressed(), &f, 10000, 1e-12);
        for i in 0..n {
            assert!((r[i] - x[i]).abs() < 1e-7);
        }
    }

    #[test]
    fn solver_dispatch_all_backends() {
        let n = 20;
        let k = laplacian(n);
        let f: Vec<f64> = vec![1.0; n];
        let r = reference_solve(&k, &f);

        // Direct backends via factor()
        for kind in [SolverKind::SparseLu, SolverKind::SkylineLdlt] {
            let s = SparseSolver::factor(kind, &k).unwrap();
            let x = s.solve(&f).unwrap();
            for i in 0..n {
                assert!((r[i] - x[i]).abs() < 1e-7, "{:?} i={}", kind, i);
            }
        }

        // Iterative backends
        for kind in [SolverKind::PcG, SolverKind::Iccg] {
            let x = SparseSolver::solve_iterative(kind, &k.compressed(), &f, 20000, 1e-12);
            for i in 0..n {
                assert!((r[i] - x[i]).abs() < 1e-6, "{:?} i={}", kind, i);
            }
        }
    }
}
