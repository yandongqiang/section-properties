//! Structural mechanism diagnostics for a **reduced** stiffness system
//! (Phase 15).
//!
//! This module is a *diagnostic*, not a solver: it never assembles, never
//! factorises for a solution, and never modifies the matrix it inspects. It
//! answers one question about an already-condensed system `K_ff` (the free-free
//! block produced by static condensation, see `docs/beam_fem.md`):
//!
//! > Is `K_ff` rank deficient, and if so, is the deficiency an unconstrained
//! > rigid-body motion or an internal mechanism?
//!
//! # Why the *reduced* system
//!
//! A 2D frame has three legitimate rigid-body modes (Tx, Ty, Rz) before any
//! support is applied, so the *unconstrained* `K` is always singular and a
//! diagnosis of it would be meaningless. Supports are enforced exactly by
//! static condensation (`K_ff u_f = f_f - K_fc u_c`, no penalties), and the
//! only thing the linear solver ever factorises is `K_ff`. A mechanism is
//! therefore a property of `K_ff` alone, which is why this module takes `K_ff`
//! (and nothing else) as its matrix input.
//!
//! `K_ff` is symmetric positive semi-definite for every structure assembled
//! from the beam element stiffness (a principal submatrix of a sum of PSD
//! element matrices), and the probe relies on that property.
//!
//! # What a frame mechanism can be
//!
//! For this element type the only zero-energy deformations of an element are
//! its own rigid-body motions, and the elements share all three DOFs at a
//! rigid joint. Because `K_ff` is PSD, `v != 0` with `vᵀK_ff v = 0` implies the
//! vector extended by zeros on the constrained DOFs lies in `null(K)`, i.e. it
//! *is* a global rigid-body motion. So for a rigid-jointed frame every
//! frame-level mechanism is an **under-restraint** (a rigid-body motion that
//! the supports do not remove), and [`StructuralDiagnostic::Mechanism`] is the
//! mathematically general case rather than the reachable one — it is produced
//! by this module for a reduced system whose null space is *not* a rigid-body
//! motion (exercised directly in `tests/mechanism_diagnostics.rs`), which a
//! frame with moment releases or a different element type could realise.
//!
//! # Criterion (scale-aware — there is no absolute epsilon)
//!
//! Let `s = max|K_ij|` and `A = D K D` with `D_ii = 1/sqrt(K_ii)` be the
//! *equilibrated* (Jacobi-scaled) system. Equilibration is a congruence, so it
//! preserves the rank of `K` exactly while removing the conditioning that comes
//! purely from unit/stiffness/coordinate scaling — which is exactly what a
//! pivot-only test would otherwise mistake for a mechanism. Three scale-relative
//! steps follow:
//!
//! 1. **Structural probe** — pivoted Cholesky of `A`. Pivoting on the largest
//!    remaining diagonal makes the trailing block's round-off `O(n·eps·scale)`
//!    (the standard numerical-rank bound), and the largest diagonal left when
//!    the factorisation stops is the *residual stiffness*. Its comparison with
//!    the project's own singularity floor `PIVOT_TOL_BASE` decides whether the
//!    deficiency is a structural rank deficiency or only extreme conditioning:
//!
//!    | residual after equilibration | reading |
//!    |---|---|
//!    | `0` (full rank) | no structural deficiency |
//!    | `<= PIVOT_TOL_BASE * scale` | **deep**: a genuine rank deficiency |
//!    | `> PIVOT_TOL_BASE * scale` | **shallow**: numerically singular but not separable from ill-conditioning → `IllConditioned`, never `Mechanism` |
//!
//! 2. **Rigid-body test** (only for a deep deficiency) — for the caller's
//!    rigid-motion candidates `R` (three global rigid motions restricted to the
//!    free DOFs) the number of independent directions with zero strain energy is
//!    `rank(R) - rank(Rᵀ K R)`, again from the same scale-invariant probe. If
//!    that accounts for the whole nullity the diagnosis is
//!    [`StructuralDiagnostic::RigidBodyMode`] (under-restraint), otherwise
//!    [`StructuralDiagnostic::Mechanism`] (internal).
//! 3. **Numerical probe** (only when the structural probe finds full rank) —
//!    the same pivoted factorisation of the **raw** `K`. A deficiency there is
//!    a conditioning limit of the raw system, reported as
//!    [`StructuralDiagnostic::IllConditioned`] — never as a mechanism.
//!
//! Every threshold is a multiple of the analysed matrix's own scale (`n·eps`
//! for the rank/indefiniteness bound, `PIVOT_TOL_BASE` for the structural
//! singularity floor), so scaling the structure uniformly — coordinates by `a`,
//! `A` by `a²`, `I` by `a⁴` — leaves the verdict unchanged, and so does any
//! change of units. Scales on the raw system that are *not* unity-consistent
//! are removed by the equilibration before any rank claim is made.
//!
//! # Limits (honest, not papered over)
//!
//! * The probe is a **bounded dense** one: `O(n³)` time and `O(n²)` memory for
//!   `n` free DOFs, and it refuses systems larger than [`MAX_DENSE_PROBE_DOF`]
//!   (the size at which this project already accepts a dense factorisation)
//!   with [`DiagnosticLimit::SystemTooLarge`] rather than allocating. Large
//!   mechanisms are therefore *not* classified here — the solver error is all
//!   that remains in that case.
//! * The probe classifies the *matrix*. It neither proves the model well-posed
//!   nor checks load equilibrium: a stable system with an inconsistent load
//!   vector is `Stable` here and still fails later.
//! * `Mechanism`/`RigidBodyMode` versus `IllConditioned` is decided by
//!   tolerance-based probes, not exact arithmetic. A deficiency shallower than
//!   `PIVOT_TOL_BASE` (a scaled condition number beyond ~1e15) is *not* claimed
//!   as a mechanism: it is reported as `IllConditioned`. Where a matrix is
//!   simultaneously rank deficient and ill-conditioned the two cannot be
//!   separated in double precision, and this module does not pretend otherwise.
//! * A non-symmetric or non-positive-semi-definite matrix is reported as
//!   [`DiagnosticLimit::NotSymmetric`] / [`DiagnosticLimit::NotPositiveSemidefinite`];
//!   no mechanism claim is made at all, because the probe is only rank-revealing
//!   for symmetric PSD matrices.

use crate::fea::{PIVOT_TOL_BASE, SparseMatrix};

/// Largest reduced system the bounded dense probe will allocate a copy of.
///
/// This is not a free choice: 500 is exactly the size at which the project
/// already accepts an `O(n^3)` dense factorisation — see
/// [`SolverCapabilities::dense`](crate::fea::solver::SolverCapabilities::dense)'s
/// `max_size`, which
/// [`SolverRegistry::auto_select_info`](crate::fea::solver::SolverRegistry::auto_select_info)
/// uses as its "small system -> dense" bound. A reduced system larger than this
/// is reported as [`DiagnosticLimit::SystemTooLarge`] instead of forcing an
/// `O(n^2)` dense allocation.
pub const MAX_DENSE_PROBE_DOF: usize = 500;

/// Why a reduced system could not be classified.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagnosticLimit {
    /// The reduced system is larger than [`MAX_DENSE_PROBE_DOF`].
    SystemTooLarge,
    /// The reduced matrix is not symmetric (within the project's relative
    /// symmetry tolerance) or contains non-finite entries.
    NotSymmetric,
    /// The reduced matrix is not positive semi-definite, so the probe is not
    /// rank-revealing and no mechanism claim is made.
    NotPositiveSemidefinite,
}

/// Outcome of a structural diagnosis of a reduced (boundary-conditioned)
/// stiffness system.
///
/// `n_free` is the number of free (unconstrained) DOFs in the analysed system
/// and `rank` is its numerically determined rank.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StructuralDiagnostic {
    /// The reduced system has full structural rank and the raw system is not
    /// numerically singular: no mechanism, solvable by a direct solver.
    Stable,
    /// The reduced system is rank deficient and **every** null direction is a
    /// global rigid-body motion (Tx, Ty and/or Rz) that the supports do not
    /// remove — the structure is under-restrained, not internally deficient.
    RigidBodyMode {
        /// Number of free DOFs in the analysed system.
        n_free: usize,
        /// Numerically determined rank of the reduced system.
        rank: usize,
        /// Number of independent rigid-body motions in the null space.
        rigid_modes: usize,
    },
    /// The reduced system is rank deficient and at least one null direction is
    /// *not* a rigid-body motion: an internal structural mechanism (rank
    /// deficiency beyond under-restraint).
    Mechanism {
        /// Number of free DOFs in the analysed system.
        n_free: usize,
        /// Numerically determined rank of the reduced system.
        rank: usize,
    },
    /// No structural deficiency, but the **raw** reduced system is numerically
    /// singular / too ill-conditioned for a reliable direct solve. This is a
    /// conditioning limit of the numbers, **not** a mechanism.
    IllConditioned {
        /// Number of free DOFs in the analysed system.
        n_free: usize,
        /// Numerically determined rank of the equilibrated system.
        rank: usize,
    },
    /// Not classified; see [`DiagnosticLimit`]. Deliberately *not* a mechanism
    /// claim.
    Indeterminate {
        /// Number of free DOFs in the analysed system.
        n_free: usize,
        /// Why the system could not be classified.
        reason: DiagnosticLimit,
    },
}

impl StructuralDiagnostic {
    /// Whether the reduced system is structurally sound and numerically
    /// solvable by a direct solver.
    pub fn is_stable(&self) -> bool {
        matches!(self, Self::Stable)
    }

    /// Whether the reduced system is structurally singular — under-restrained
    /// or internally deficient. `IllConditioned` and `Indeterminate` are **not**
    /// mechanisms.
    pub fn is_mechanism(&self) -> bool {
        matches!(self, Self::RigidBodyMode { .. } | Self::Mechanism { .. })
    }
}

impl std::fmt::Display for StructuralDiagnostic {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Stable => {
                write!(f, "stable (full structural rank, not numerically singular)")
            }
            Self::RigidBodyMode {
                n_free,
                rank,
                rigid_modes,
            } => write!(
                f,
                "rigid-body mechanism: {rigid_modes} unrestrained rigid-body mode(s) \
                 ({n_free} free DOFs, system rank {rank}); restrain the remaining \
                 translations/rotation"
            ),
            Self::Mechanism { n_free, rank } => write!(
                f,
                "structural mechanism: rank deficient by {} of {n_free} free DOFs \
                 (rank {rank}); the structure has an internal zero-energy mode",
                n_free - rank
            ),
            Self::IllConditioned { n_free, rank } => write!(
                f,
                "numerically ill-conditioned: structurally full rank ({rank} of \
                 {n_free} free DOFs) but the raw reduced system is singular to the \
                 probe's scale-relative tolerance"
            ),
            Self::Indeterminate { n_free, reason } => match reason {
                DiagnosticLimit::SystemTooLarge => write!(
                    f,
                    "indeterminate: {n_free} free DOFs exceed the bounded dense probe \
                     ({MAX_DENSE_PROBE_DOF})"
                ),
                DiagnosticLimit::NotSymmetric => {
                    write!(f, "indeterminate: reduced matrix is not symmetric")
                }
                DiagnosticLimit::NotPositiveSemidefinite => write!(
                    f,
                    "indeterminate: reduced matrix is not positive semi-definite"
                ),
            },
        }
    }
}

/// Diagnose a reduced (free-free) stiffness system.
///
/// `k_ff` is the symmetric positive semi-definite system the linear solver
/// would factorise: the free-free block of the assembled stiffness matrix after
/// static condensation. `rigid_candidates` are the global rigid-body motions
/// (Tx, Ty, Rz) *restricted to the free DOFs*, in the same DOF order as `k_ff`
/// and with zero entries where a DOF is constrained; they are used only to tell
/// under-restraint apart from an internal mechanism and may be omitted
/// (`&[]`), in which case a deep deficiency is reported as
/// [`StructuralDiagnostic::Mechanism`].
///
/// Cost: `O(n³)` time / `O(n²)` memory for `n = k_ff.n`, and only for
/// `n <= `[`MAX_DENSE_PROBE_DOF`]. Never panics.
pub fn diagnose_reduced(
    k_ff: &SparseMatrix,
    rigid_candidates: &[Vec<f64>],
) -> StructuralDiagnostic {
    let n = k_ff.n;
    if n == 0 {
        // Nothing to solve: every DOF is constrained. Not a mechanism.
        return StructuralDiagnostic::Stable;
    }
    if n > MAX_DENSE_PROBE_DOF {
        return StructuralDiagnostic::Indeterminate {
            n_free: n,
            reason: DiagnosticLimit::SystemTooLarge,
        };
    }

    let mut probe = k_ff.clone();
    probe.compress();
    if !probe.is_symmetric(1e-12) {
        return StructuralDiagnostic::Indeterminate {
            n_free: n,
            reason: DiagnosticLimit::NotSymmetric,
        };
    }
    let a = match to_dense(&probe) {
        Some(a) => a,
        None => {
            return StructuralDiagnostic::Indeterminate {
                n_free: n,
                reason: DiagnosticLimit::NotSymmetric,
            };
        }
    };

    let scale = max_abs(&a);
    if scale.is_nan() {
        return StructuralDiagnostic::Indeterminate {
            n_free: n,
            reason: DiagnosticLimit::NotSymmetric,
        };
    }
    if scale <= 0.0 {
        // Exactly zero stiffness for every free DOF: every DOF is unrestrained.
        return StructuralDiagnostic::Mechanism { n_free: n, rank: 0 };
    }

    // (1) Structural probe on the equilibrated system: is any DOF-free direction
    // left with no stiffness once unit/stiffness scaling is removed?
    let (equilibrated, equilibrating) = equilibrate(&a);
    let eq_scale = max_abs(&equilibrated);
    let structural = match pivoted_cholesky(&equilibrated, eq_scale) {
        Some(o) => o,
        None => {
            return StructuralDiagnostic::Indeterminate {
                n_free: n,
                reason: DiagnosticLimit::NotPositiveSemidefinite,
            };
        }
    };
    let nullity = n - structural.rank;
    if nullity > 0 {
        return if structural.residual > PIVOT_TOL_BASE * eq_scale {
            // Shallow: numerically singular, but not separable from extreme
            // conditioning. Refuse to call it a mechanism.
            StructuralDiagnostic::IllConditioned {
                n_free: n,
                rank: structural.rank,
            }
        } else {
            let rigid = rigid_nullity(&equilibrated, &equilibrating, rigid_candidates);
            if rigid >= nullity {
                StructuralDiagnostic::RigidBodyMode {
                    n_free: n,
                    rank: structural.rank,
                    rigid_modes: rigid,
                }
            } else {
                StructuralDiagnostic::Mechanism {
                    n_free: n,
                    rank: structural.rank,
                }
            }
        };
    }

    // (2) Full structural rank: only a conditioning limit of the *raw* system
    // can remain.
    match pivoted_cholesky(&a, scale) {
        Some(raw) if raw.rank == n => StructuralDiagnostic::Stable,
        Some(_) => StructuralDiagnostic::IllConditioned {
            n_free: n,
            rank: structural.rank,
        },
        None => StructuralDiagnostic::Indeterminate {
            n_free: n,
            reason: DiagnosticLimit::NotPositiveSemidefinite,
        },
    }
}

/// Result of the rank probe.
struct CholeskyOutcome {
    /// Number of pivots above the numerical-rank threshold.
    rank: usize,
    /// Largest diagonal left in the un-eliminated trailing block (a measure of
    /// residual stiffness; `0` when the matrix has full rank).
    residual: f64,
}

/// Dense symmetric copy of a **compressed** matrix, or `None` if any entry is
/// non-finite.
fn to_dense(m: &SparseMatrix) -> Option<Vec<Vec<f64>>> {
    let n = m.n;
    let mut a = vec![vec![0.0f64; n]; n];
    for (i, row) in a.iter_mut().enumerate() {
        for k in m.row_ptr()[i]..m.row_ptr()[i + 1] {
            row[m.csr_cols()[k]] = m.csr_vals()[k];
        }
    }
    if a.iter().flatten().all(|v| v.is_finite()) {
        Some(a)
    } else {
        None
    }
}

/// Largest `|entry|`, or `NaN` if any entry is not finite.
fn max_abs(m: &[Vec<f64>]) -> f64 {
    let mut s = 0.0f64;
    for row in m {
        for &v in row {
            if !v.is_finite() {
                return f64::NAN;
            }
            s = s.max(v.abs());
        }
    }
    s
}

/// Jacobi equilibration `A = D M D`, returning `(A, d)` with `D_ii = d_i =
/// 1/sqrt(M_ii)`.
///
/// `A` has a unit diagonal (so its own scale is 1 and rank claims on it are
/// inherently relative), and `d` maps a vector `y` of the equilibrated space
/// back to the physical one as `c = D y`, i.e. `y = D⁻¹ c`.
///
/// A DOF with a non-positive diagonal has no stiffness of its own; its row and
/// column are zeroed (for a PSD matrix such a DOF is decoupled anyway, and the
/// zero row makes the probe count it as deficient), and its `d_i` stays `0`.
fn equilibrate(m: &[Vec<f64>]) -> (Vec<Vec<f64>>, Vec<f64>) {
    let n = m.len();
    let mut d = vec![0.0f64; n];
    for (i, di) in d.iter_mut().enumerate() {
        let dii = m[i][i];
        if dii > 0.0 {
            *di = 1.0 / dii.sqrt();
        }
    }
    let mut out = vec![vec![0.0f64; n]; n];
    for i in 0..n {
        for j in 0..n {
            out[i][j] = m[i][j] * d[i] * d[j];
        }
    }
    (out, d)
}

/// **Pivoted** Cholesky rank probe for a symmetric positive semi-definite
/// matrix.
///
/// Pivoting on the largest remaining diagonal is what makes this reliable: the
/// trailing block is never divided by a value at the noise level, so its
/// round-off stays at the standard `O(n·eps·scale)` backward-error bound, and a
/// diagonal at or below `n·eps·scale` is numerically zero. The factorisation
/// stops as soon as the largest remaining diagonal crosses that threshold and
/// returns the remaining stiffness as `residual`; for a PSD matrix the number of
/// pivots taken is its numerical rank, whatever the ordering.
///
/// Returns `None` if the matrix is not positive semi-definite: with pivoting the
/// largest remaining diagonal cannot be more than `n·eps·scale` below zero
/// unless that is a genuine property of the matrix.
fn pivoted_cholesky(m: &[Vec<f64>], scale: f64) -> Option<CholeskyOutcome> {
    let n = m.len();
    if !scale.is_finite() {
        return None;
    }
    if scale <= 0.0 {
        // No stiffness at all in this system: nothing is determined by it.
        return Some(CholeskyOutcome {
            rank: 0,
            residual: 0.0,
        });
    }
    let tol = n as f64 * f64::EPSILON * scale;

    let mut a = m.to_vec();
    for k in 0..n {
        let mut p = k;
        for i in (k + 1)..n {
            if a[i][i] > a[p][p] {
                p = i;
            }
        }
        let pivot = a[p][p];
        if pivot < -tol {
            return None; // not positive semi-definite
        }
        if pivot <= tol {
            // The whole remaining block is numerically zero.
            return Some(CholeskyOutcome {
                rank: k,
                residual: pivot,
            });
        }
        if p != k {
            a.swap(k, p); // rows
            for row in a.iter_mut() {
                row.swap(k, p); // columns
            }
        }
        // L_ik = A_ik / d_k, then the Schur complement A_ij -= L_ik * L_jk * d_k.
        for row in a.iter_mut().skip(k + 1) {
            row[k] /= pivot;
        }
        let l: Vec<f64> = a[k + 1..].iter().map(|row| row[k]).collect();
        for (offset, row) in a[k + 1..].iter_mut().enumerate() {
            let lik = l[offset];
            for (j, value) in row.iter_mut().enumerate().skip(k + 1) {
                *value -= lik * l[j - k - 1] * pivot;
            }
        }
    }
    Some(CholeskyOutcome {
        rank: n,
        residual: 0.0,
    })
}

/// Number of independent rigid-body motions contained in `null(K)`.
///
/// `equilibrated` is `D K D` and `d` its scaling vector (`d_i = 1/sqrt(K_ii)`),
/// as returned by [`equilibrate`]. The candidates are mapped into the
/// equilibrated space (`y = D⁻¹ c`) and orthonormalised there, so the
/// restriction of the equilibrated stiffness to the candidate space is
/// expressed in a basis whose diagonal entries are the Rayleigh quotients of
/// that space — directly comparable with the equilibrated unit scale. The
/// answer is `dim(span R) - rank(A restricted to span R)`, i.e. the dimension
/// of the candidate space spanned by zero-energy directions.
///
/// Working in the equilibrated metric is what makes this meaningful: the rigid
/// space may be spanned by *combinations* of the supplied candidates (a rigid
/// rotation about a point other than the origin, say), and the strain energy of
/// a whole subspace shrinks to round-off only when the subspace really is a
/// mechanism.
fn rigid_nullity(equilibrated: &[Vec<f64>], d: &[f64], candidates: &[Vec<f64>]) -> usize {
    let n = equilibrated.len();
    if d.contains(&0.0) {
        // A DOF with no stiffness of its own: the metric is not invertible, so
        // no rigid-body claim is made.
        return 0;
    }
    let dependent_tol = n as f64 * f64::EPSILON;

    // Modified Gram-Schmidt in the equilibrated metric.
    let mut basis: Vec<Vec<f64>> = Vec::new();
    for c in candidates {
        if c.len() != n {
            continue;
        }
        let mut y: Vec<f64> = (0..n).map(|i| c[i] / d[i]).collect();
        let norm0 = norm(&y);
        if norm0 <= 0.0 {
            continue; // zero vector: no direction
        }
        for b in &basis {
            let projection = dot(&y, b);
            for i in 0..n {
                y[i] -= projection * b[i];
            }
        }
        let norm = norm(&y);
        if norm <= dependent_tol * norm0 {
            continue; // linearly dependent on the directions already taken
        }
        for v in y.iter_mut() {
            *v /= norm;
        }
        basis.push(y);
    }

    let m = basis.len();
    if m == 0 {
        return 0;
    }

    let mut energy = vec![vec![0.0f64; m]; m];
    for p in 0..m {
        for q in 0..m {
            let mut s = 0.0f64;
            for i in 0..n {
                let mut row = 0.0f64;
                for j in 0..n {
                    row += equilibrated[i][j] * basis[q][j];
                }
                s += basis[p][i] * row;
            }
            energy[p][q] = s;
        }
    }

    // The comparison scale is the equilibrated one (unit diagonal), *not* the
    // magnitude of `energy`, which is itself the (dimensionless) evidence.
    let rank = pivoted_cholesky(&energy, 1.0).map(|o| o.rank).unwrap_or(m);
    m - rank
}

/// Euclidean dot product.
fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b.iter()).map(|(x, y)| x * y).sum()
}

/// Euclidean norm.
fn norm(a: &[f64]) -> f64 {
    dot(a, a).sqrt()
}
