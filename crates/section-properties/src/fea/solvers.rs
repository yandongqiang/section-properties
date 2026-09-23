//! Solver backends. See module docs in [`crate::fea`].

use super::{CgResult, CgStatus, SparseMatrix};

// ---------------------------------------------------------------------------
// Sparse LU: left-looking, column-oriented with true row partial pivoting
// (PA = L U). No static pivoting / diagonal perturbation - returns error
// on near-singular matrices.
// ---------------------------------------------------------------------------
//
// IMPORTANT: This implementation is DENSE-BACKED.
// The input sparse matrix is converted to a dense n×n matrix for Gaussian
// elimination with partial pivoting. Time complexity: O(n³), Space: O(n²).
// Suitable for n up to ~2000-3000 on typical hardware.
// For larger problems, a true sparse LU (e.g. SuiteSparse) should be used.
//
/// Sparse LU factorisation P A = L U with true row partial pivoting.
pub struct SparseLu {
    n: usize,
    /// Row-wise unit-lower triangle (strict part), sorted columns per row.
    l_rows: Vec<Vec<(usize, f64)>>,
    /// Row-wise upper triangle including diagonal, sorted columns per row.
    u_rows: Vec<Vec<(usize, f64)>>,
    /// Row permutation: perm[final_row] = orig_row. P * A = L * U.
    perm: Vec<usize>,
}

impl SparseLu {
    /// Get the permutation vector (perm\[i\] = original row index at position i).
    pub fn perm(&self) -> &[usize] {
        &self.perm
    }

    /// Get L factor rows (strict lower triangle, unit diagonal implicit).
    pub fn l_rows(&self) -> &[Vec<(usize, f64)>] {
        &self.l_rows
    }

    /// Get U factor rows (upper triangle including diagonal).
    pub fn u_rows(&self) -> &[Vec<(usize, f64)>] {
        &self.u_rows
    }

    /// Get matrix size.
    pub fn n(&self) -> usize {
        self.n
    }

    /// Verify PA = LU for debugging purposes.
    /// Returns max absolute difference between PA and LU.
    pub fn verify_pa_eq_lu(&self, a: &SparseMatrix) -> f64 {
        let n = self.n;
        // Build A dense
        let mut a_dense = vec![vec![0.0f64; n]; n];
        for i in 0..n {
            for k in a.row_ptr()[i]..a.row_ptr()[i + 1] {
                let j = a.csr_cols()[k];
                a_dense[i][j] = a.csr_vals()[k];
            }
        }

        // Build PA
        let mut pa = vec![vec![0.0f64; n]; n];
        for i in 0..n {
            let orig_i = self.perm[i];
            for j in 0..n {
                pa[i][j] = a_dense[orig_i][j];
            }
        }

        // Build LU
        let mut lu = vec![vec![0.0f64; n]; n];
        for i in 0..n {
            for j in 0..n {
                let mut sum = 0.0;
                for k in 0..n {
                    let l_ik = if i == k {
                        1.0
                    } else if k < i {
                        self.l_rows[i]
                            .iter()
                            .find(|(c, _)| *c == k)
                            .map(|(_, v)| *v)
                            .unwrap_or(0.0)
                    } else {
                        0.0
                    };
                    let u_kj = if k <= j {
                        self.u_rows[k]
                            .iter()
                            .find(|(c, _)| *c == j)
                            .map(|(_, v)| *v)
                            .unwrap_or(0.0)
                    } else {
                        0.0
                    };
                    sum += l_ik * u_kj;
                }
                lu[i][j] = sum;
            }
        }

        // Max absolute difference
        let mut max_diff = 0.0;
        for i in 0..n {
            for j in 0..n {
                let diff = (pa[i][j] - lu[i][j]).abs();
                if diff > max_diff {
                    max_diff = diff;
                }
            }
        }
        max_diff
    }

    /// Factor A with true row partial pivoting (PA = LU).
    /// Returns error if matrix is singular or nearly singular.
    pub fn factor(a: &SparseMatrix) -> Result<SparseLu, String> {
        let n = a.n;
        let ac = if a.compressed {
            None
        } else {
            let mut m = a.clone();
            m.compress();
            Some(m)
        };
        let a_ref = ac.as_ref().unwrap_or(a);
        let (row_ptr, cols, vals) = a_ref.csr_data();

        // Convert to dense row-major working matrix for pivoting operations
        // This is memory-intensive but necessary for true partial pivoting
        // on a sparse matrix. We use a dense column-major working array.
        let mut a_dense = vec![vec![0.0f64; n]; n];
        for i in 0..n {
            for k in row_ptr[i]..row_ptr[i + 1] {
                let j = cols[k];
                a_dense[i][j] = vals[k];
            }
        }

        // Permutation vector: perm[i] = original row index that ended up at position i
        let mut perm = (0..n).collect::<Vec<usize>>();

        // LU factors stored in dense format during factorization, then compressed
        let mut l = vec![vec![0.0f64; n]; n]; // unit diagonal implicit
        let mut u = vec![vec![0.0f64; n]; n];

        // Compute per-column scales for scale-invariant pivot tolerance
        // Using column-wise infinity norm scaled by machine epsilon
        let mut col_scales = vec![0.0f64; n];
        for j in 0..n {
            let mut max_abs = 0.0f64;
            for i in 0..n {
                max_abs = max_abs.max(a_dense[i][j].abs());
            }
            col_scales[j] = max_abs;
        }
        // Global matrix scale (for reference)
        let _matrix_scale = col_scales.iter().fold(0.0f64, |a, &v| a.max(v));

        // Scale-invariant pivot tolerance:
        // pivot_tol = EPS * n * max(|A[:,k]|_inf) * safety_factor
        // This ensures that small-scale matrices aren't incorrectly flagged as singular
        // and large-scale matrices don't reject legitimate small pivots
        const PIVOT_SAFETY_FACTOR: f64 = 100.0;

        // LU factorization with partial pivoting (Gaussian elimination with row swaps)
        for k in 0..n {
            // Find pivot row: max |A[i][k]| for i >= k
            let mut pivot_row = k;
            let mut max_val = a_dense[k][k].abs();
            for i in (k + 1)..n {
                let v = a_dense[i][k].abs();
                if v > max_val {
                    max_val = v;
                    pivot_row = i;
                }
            }

            // Check for singularity using scale-aware tolerance (per-column)
            // True scale invariance: use actual column scale without artificial floor.
            // If col_scale == 0 or non-finite, the column is all zeros or invalid -> singular.
            let col_scale = col_scales[k];
            if !col_scale.is_finite() || col_scale == 0.0 {
                return Err(format!(
                    "Singular matrix at column {}: column is all zeros or contains NaN/Inf (col_scale = {:.2e})",
                    k, col_scale
                ));
            }
            let pivot_tol = f64::EPSILON * n as f64 * col_scale * PIVOT_SAFETY_FACTOR;

            if max_val <= pivot_tol {
                return Err(format!(
                    "Singular or near-singular matrix at column {}: max pivot = {:.2e}, tolerance = {:.2e} (col_scale = {:.2e})",
                    k, max_val, pivot_tol, col_scale
                ));
            }

            // Swap rows k and pivot_row in A and L, and permutation
            if pivot_row != k {
                a_dense.swap(k, pivot_row);
                // Swap corresponding rows in L (only columns < k are filled)
                // Use split_at_mut to get two mutable references
                let (l_first, l_second) = if k < pivot_row {
                    l.split_at_mut(pivot_row)
                } else {
                    l.split_at_mut(k)
                };
                let (row_k, row_pivot) = if k < pivot_row {
                    (&mut l_first[k], &mut l_second[0])
                } else {
                    (&mut l_second[0], &mut l_first[k])
                };
                for j in 0..k {
                    std::mem::swap(&mut row_k[j], &mut row_pivot[j]);
                }
                perm.swap(k, pivot_row);
            }

            // U[k][k] = A[k][k] (after potential swap)
            u[k][k] = a_dense[k][k];

            // Compute U[k][j] for j > k
            for j in (k + 1)..n {
                u[k][j] = a_dense[k][j];
            }

            // Compute L[i][k] for i > k
            let pivot = u[k][k];
            for i in (k + 1)..n {
                l[i][k] = a_dense[i][k] / pivot;
            }

            // Update trailing submatrix: A[i][j] -= L[i][k] * U[k][j] for i,j > k
            for i in (k + 1)..n {
                let lik = l[i][k];
                if lik != 0.0 {
                    for j in (k + 1)..n {
                        a_dense[i][j] -= lik * u[k][j];
                    }
                    a_dense[i][k] = 0.0; // Below diagonal becomes zero
                }
            }
        }

        // Build sparse L and U factors (only non-zero entries)
        // Use relative drop tolerance: keep entries where |v| > max_row_abs * EPS * n
        // This avoids deleting legitimate small entries in scaled matrices
        let mut l_rows: Vec<Vec<(usize, f64)>> = vec![Vec::new(); n];
        let mut u_rows: Vec<Vec<(usize, f64)>> = vec![Vec::new(); n];

        // Compute per-row scale for relative drop tolerance
        let mut l_row_scale = vec![0.0f64; n];
        let mut u_row_scale = vec![0.0f64; n];
        for i in 0..n {
            for j in 0..i {
                l_row_scale[i] = l_row_scale[i].max(l[i][j].abs());
            }
            for j in i..n {
                u_row_scale[i] = u_row_scale[i].max(u[i][j].abs());
            }
        }

        for i in 0..n {
            // L has unit diagonal (not stored), strictly lower part
            let l_tol = l_row_scale[i] * f64::EPSILON * n as f64;
            for j in 0..i {
                if l[i][j].abs() > l_tol {
                    l_rows[i].push((j, l[i][j]));
                }
            }
            // U has diagonal and upper part
            let u_tol = u_row_scale[i] * f64::EPSILON * n as f64;
            for j in i..n {
                if u[i][j].abs() > u_tol {
                    u_rows[i].push((j, u[i][j]));
                }
            }
            l_rows[i].sort_unstable_by_key(|e| e.0);
            u_rows[i].sort_unstable_by_key(|e| e.0);
        }

        Ok(SparseLu {
            n,
            l_rows,
            u_rows,
            perm,
        })
    }

    /// Solve A x = b via forward/back substitution using P A = L U.
    /// Returns x = A^{-1} b or error if diagonal is near-zero.
    pub fn solve(&self, b: &[f64]) -> Result<Vec<f64>, String> {
        let n = self.n;
        if b.len() != n {
            return Err(format!(
                "Solve failed: RHS length {} does not match matrix size {}",
                b.len(),
                n
            ));
        }
        // Check for NaN/Inf in RHS
        for (i, &val) in b.iter().enumerate() {
            if !val.is_finite() {
                return Err(format!(
                    "Solve failed: RHS contains non-finite value at index {}: {}",
                    i, val
                ));
            }
        }
        // Apply permutation: Pb
        let mut y = vec![0.0f64; n];
        for i in 0..n {
            y[i] = b[self.perm[i]];
        }

        // Forward substitution: L y = Pb (L has unit diagonal)
        for i in 0..n {
            let mut sum = 0.0;
            for &(c, v) in &self.l_rows[i] {
                sum += v * y[c];
            }
            y[i] -= sum;
        }

        // Backward substitution: U x = y
        let mut x = vec![0.0f64; n];
        for i in (0..n).rev() {
            let mut sum = 0.0;
            // Find diagonal and sum U[i][j] * x[j] for j > i
            let mut diag = 1.0;
            for &(c, v) in &self.u_rows[i] {
                if c == i {
                    diag = v;
                } else if c > i {
                    sum += v * x[c];
                }
            }
            if !diag.is_finite() || diag == 0.0 {
                return Err(format!(
                    "Solve failed: near-zero or invalid diagonal at row {}: diag = {:.2e}",
                    i, diag
                ));
            }
            x[i] = (y[i] - sum) / diag;
        }
        Ok(x)
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
        let n = a.n;
        let ac = if a.compressed {
            None
        } else {
            let mut m = a.clone();
            m.compress();
            Some(m)
        };
        let a_ref = ac.as_ref().unwrap_or(a);
        let (row_ptr, cols, vals) = a_ref.csr_data();

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

        Ok(Ic0Factor {
            n,
            ptr,
            flat_cols,
            flat_vals,
        })
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
pub fn iccg_solve(a: &SparseMatrix, b: &[f64], max_iter: usize, tol: f64) -> CgResult {
    let ic = match Ic0Factor::factor(a) {
        Ok(f) => f,
        Err(_) => return super::cg_solve(a, b, max_iter, tol),
    };

    let n = b.len();
    let ac = if a.compressed {
        None
    } else {
        let mut m = a.clone();
        m.compress();
        Some(m)
    };
    let a_ref = ac.as_ref().unwrap_or(a);
    let (rptr, ccols, cvals) = a_ref.csr_data();
    let matvec = |p: &[f64], out: &mut [f64]| {
        for row in 0..n {
            let mut sum = 0.0;
            for k in rptr[row]..rptr[row + 1] {
                sum += cvals[k] * p[ccols[k]];
            }
            out[row] = sum;
        }
    };

    let b_norm = b.iter().map(|v| v * v).sum::<f64>().sqrt();
    let mut x = vec![0.0f64; n];
    if b_norm == 0.0 {
        return CgResult {
            x,
            iterations: 0,
            residual: 0.0,
            status: CgStatus::InvalidInput,
        };
    }

    let mut r = b.to_vec();
    let mut z = ic.solve(&r);
    let mut p = z.clone();
    let mut rz: f64 = r.iter().zip(z.iter()).map(|(&a, &b2)| a * b2).sum();
    let mut iterations = 0;

    let mut ap = vec![0.0f64; n];
    for iter in 0..max_iter {
        matvec(&p, &mut ap);
        let pap: f64 = p.iter().zip(ap.iter()).map(|(&a, &b2)| a * b2).sum();
        if pap.abs() <= 0.0 {
            return CgResult {
                x,
                iterations,
                residual: r.iter().map(|v| v * v).sum::<f64>().sqrt(),
                status: CgStatus::Breakdown,
            };
        }
        let alpha = rz / pap;
        if !alpha.is_finite() {
            return CgResult {
                x,
                iterations,
                residual: r.iter().map(|v| v * v).sum::<f64>().sqrt(),
                status: CgStatus::Breakdown,
            };
        }
        for i in 0..n {
            x[i] += alpha * p[i];
            r[i] -= alpha * ap[i];
        }

        let rn = r.iter().map(|v| v * v).sum::<f64>().sqrt();
        iterations = iter + 1;
        if rn < tol * b_norm {
            return CgResult {
                x,
                iterations,
                residual: rn,
                status: CgStatus::Converged,
            };
        }

        z = ic.solve(&r);
        let rz_new: f64 = r.iter().zip(z.iter()).map(|(&a, &b2)| a * b2).sum();
        if rz == 0.0 || !rz.is_finite() {
            return CgResult {
                x,
                iterations,
                residual: r.iter().map(|v| v * v).sum::<f64>().sqrt(),
                status: CgStatus::Breakdown,
            };
        }
        let beta = rz_new / rz;
        if !beta.is_finite() {
            return CgResult {
                x,
                iterations,
                residual: r.iter().map(|v| v * v).sum::<f64>().sqrt(),
                status: CgStatus::Breakdown,
            };
        }
        for i in 0..n {
            p[i] = z[i] + beta * p[i];
        }
        rz = rz_new;
    }

    CgResult {
        x,
        iterations,
        residual: r.iter().map(|v| v * v).sum::<f64>().sqrt(),
        status: CgStatus::MaxIterations,
    }
}
