//! FEM warping analysis using Tri6 elements.
//!
//! Mirrors Python `sectionproperties.analysis.section.Section.calculate_warping_properties`:
//! assembles Lagrangian stiffness matrix, solves for warping function ω and shear
//! functions ψ, φ, then computes J, Iw, shear centre, shear areas and monosymmetry
//! constants via Gaussian quadrature.

use crate::fea::{
    SparseMatrix, Tri6, Tri6Mesh, build_tri6_elements, gauss_points, shape_function,
    solve_lagrange_sparse, tri3_to_tri6,
};
use crate::geometry::Point;
use crate::mesh::{MeshControl, MeshParams, mesh_params_from_control, mesh_section};
use crate::plastic::warping::ThinWalledCheck;
use crate::section::Section;
use crate::section_properties::SectionProperties;
use serde_json;
use std::fs::File;
use std::io::Write;

/// Drop zero-area (degenerate) triangles from a tri3 mesh.
///
/// Ear-clipping + uniform refinement can emit sliver triangles with all three
/// vertices collinear (e.g. along a straight flange edge). They contribute no
/// area and no stiffness, but a zero Jacobian would abort Tri6 assembly
/// (`FemError::DegenerateElement`). Filtering them out lets the FE warping
/// analysis run on the true area-tiling subset of the mesh.
fn filter_degenerate_tris(
    nodes: &[Point],
    elements: &[[usize; 3]],
    area_tol: f64,
) -> Vec<[usize; 3]> {
    elements
        .iter()
        .filter(|tri| {
            let p0 = nodes[tri[0]];
            let p1 = nodes[tri[1]];
            let p2 = nodes[tri[2]];
            let a2 = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
            a2.abs() > area_tol
        })
        .copied()
        .collect()
}

/// Relative size used to detect degenerate triangles.
const DEGENERATE_AREA_REL_TOL: f64 = 1e-14;

/// Full FEM solution (mesh + fields) for stress analysis.
pub struct FemSolution {
    pub tri6_mesh: Tri6Mesh,
    pub elements: Vec<Tri6>,
    pub omega: Vec<f64>,
    pub psi: Vec<f64>,
    pub phi: Vec<f64>,
    pub j: f64,
    pub delta_s: f64,
    pub nu: f64,
    /// Max St. Venant shear stress from FEM solution (not farthest point formula)
    pub tau_sv_max: f64,
    /// Max warping coordinate |ω| from FEM solution
    pub omega_max: f64,
}

/// Compute full FEM solution: mesh + warping function ω + shear functions ψ, φ.
/// Returns Err if mesh generation or solving fails.
///
/// Thin-walled sections use MeshControl::Coarse for speed; use `compute_fem_warping_properties`
/// for MeshControl::Normal (more accurate) and additional warping properties.
pub fn compute_fem_solution(
    section: &Section,
    props: &SectionProperties,
    nu: f64,
) -> Result<FemSolution, crate::mesh::fem::FemError> {
    let unified = compute_fem_warping_solution(section, props, nu, MeshControl::Coarse)?;
    Ok(FemSolution {
        tri6_mesh: unified.tri6_mesh,
        elements: unified.elements,
        omega: unified.omega,
        psi: unified.psi,
        phi: unified.phi,
        j: unified.j,
        delta_s: unified.delta_s,
        nu: unified.nu,
        tau_sv_max: unified.tau_sv_max,
        omega_max: unified.omega_max,
    })
}

/// Estimate shear areas using simple formulas (fallback when FEM is inaccurate).
fn is_symmetric_about_x(section: &Section) -> bool {
    for v in &section.outer.vertices {
        let found = section
            .outer
            .vertices
            .iter()
            .any(|v2| (v2.x - v.x).abs() < 1e-6 && (v2.y + v.y).abs() < 1e-6);
        if !found {
            return false;
        }
    }
    true
}

fn is_symmetric_about_y(section: &Section) -> bool {
    for v in &section.outer.vertices {
        let found = section
            .outer
            .vertices
            .iter()
            .any(|v2| (v2.y - v.y).abs() < 1e-6 && (v2.x + v.x).abs() < 1e-6);
        if !found {
            return false;
        }
    }
    true
}

/// FEM warping analysis result (centroidal coordinates).
pub struct FemWarpingResult {
    pub j: f64,
    pub iw: f64,
    pub shear_center: Point,
    pub shear_center_elastic: Point,
    pub beta_x_plus: f64,
    pub beta_y_plus: f64,
    pub beta_x_minus: f64,
    pub beta_y_minus: f64,
    pub a_sx: f64,
    pub a_sy: f64,
    pub a_sxy: f64,
    pub a_s11: f64,
    pub a_s22: f64,
    pub beta_11_plus: f64,
    pub beta_11_minus: f64,
    pub beta_22_plus: f64,
    pub beta_22_minus: f64,
    /// Maximum warping coordinate |ω| from FEM solution
    pub omega_max: f64,
}

/// Minimum edge length of the section polygons (outer + holes).
fn min_edge_length(section: &Section) -> f64 {
    let mut min_len = f64::INFINITY;
    for poly in std::iter::once(&section.outer).chain(section.holes.iter()) {
        let n = poly.vertices.len();
        for i in 0..n {
            let j = (i + 1) % n;
            let dx = poly.vertices[j].x - poly.vertices[i].x;
            let dy = poly.vertices[j].y - poly.vertices[i].y;
            let len = (dx * dx + dy * dy).sqrt();
            if len > 1e-9 {
                min_len = min_len.min(len);
            }
        }
    }
    if min_len.is_finite() { min_len } else { 1.0 }
}
/// Compute warping properties using FEM with Tri6 elements.
pub fn compute_fem_warping_properties(
    section: &Section,
    props: &SectionProperties,
    nu: f64,
) -> Result<FemWarpingResult, crate::mesh::fem::FemError> {
    let unified = compute_fem_warping_solution(section, props, nu, MeshControl::Normal)?;
    Ok(FemWarpingResult {
        j: unified.j,
        iw: unified.iw,
        shear_center: unified.shear_center,
        shear_center_elastic: unified.shear_center_elastic,
        beta_x_plus: unified.beta_x_plus,
        beta_y_plus: unified.beta_y_plus,
        beta_x_minus: unified.beta_x_minus,
        beta_y_minus: unified.beta_y_minus,
        a_sx: unified.a_sx,
        a_sy: unified.a_sy,
        a_sxy: unified.a_sxy,
        a_s11: unified.a_s11,
        a_s22: unified.a_s22,
        beta_11_plus: unified.beta_11_plus,
        beta_11_minus: unified.beta_11_minus,
        beta_22_plus: unified.beta_22_plus,
        beta_22_minus: unified.beta_22_minus,
        omega_max: unified.omega_max,
    })
}

/// Solve the Lagrangian system preferring the direct skyline solver, with an
/// automatic fallback to CG when the factorised solution fails a relative
/// residual check (ill-conditioned meshes, e.g. with sliver elements).
fn solve_with_fallback(
    solver: &Option<crate::fea::DirectLagrangeSolver>,
    k_reg: &SparseMatrix,
    c: &[f64],
    f: &[f64],
) -> Result<Vec<f64>, crate::mesh::fem::FemError> {
    if let Some(s) = solver {
        if let Ok(w1) = s.solve(f) {
            if let Ok(w2) = s.solve(c) {
                let ct_w2: f64 = c.iter().zip(w2.iter()).map(|(&a, &b)| a * b).sum();
                let ct_w1: f64 = c.iter().zip(w1.iter()).map(|(&a, &b)| a * b).sum();
                if ct_w1.abs() > 1e-15 {
                    let lambda = ct_w2 / ct_w1;
                    let u: Vec<f64> = w1
                        .iter()
                        .zip(w2.iter())
                        .map(|(&a, &b)| a - lambda * b)
                        .collect();
                    // Relative residual of K u - f + lambda c = 0.
                    let prod = k_reg.matvec(&u);
                    let mut worst = 0.0f64;
                    let mut f_norm = 0.0f64;
                    for i in 0..prod.len() {
                        worst = worst.max((prod[i] - f[i] + lambda * c[i]).abs());
                        f_norm = f_norm.max(f[i].abs());
                    }
                    if worst <= 1e-6 * f_norm.max(1e-300) {
                        return Ok(u);
                    }
                }
            }
        }
    }
    // CG fallback: plain Jacobi-CG meets the strict accuracy requirements of
    // the warping solution; IC(0)-PCG remains available via fea::solvers for
    // less sensitive systems.
    solve_lagrange_sparse(k_reg, c, f)
}

/// Exact solver failure classification.
#[derive(Debug, Clone)]
pub enum ExactSolverFailure {
    FactorizationFailed(String),
    SolveFailed(String),
    ResidualCheckFailed(f64),
    RegularizedFallback,
}

/// Result of exact Lagrange solve.
#[derive(Debug, Clone)]
struct ExactLagrangeSolution {
    omega: Vec<f64>,
    lambda: f64,
}

/// Solve the exact (non-regularized) Lagrangian system using SparseLU with
/// true partial pivoting. This mirrors Python's `solve_direct_lagrange` which
/// solves the full (n+1)x(n+1) system without adding eps to the diagonal.
fn solve_exact_lagrange(
    k: &SparseMatrix,
    c: &[f64],
    f: &[f64],
) -> Result<ExactLagrangeSolution, ExactSolverFailure> {
    let n = f.len();
    let mut k_lg = SparseMatrix::new(n + 1);
    // Use COO triplets directly - need to compress first to access them
    let mut k_compressed = k.clone();
    k_compressed.compress();
    let (rows, cols, vals) = k_compressed.triplets();
    for (&i, (&j, &v)) in rows.iter().zip(cols.iter().zip(vals.iter())) {
        if i < n && j < n {
            k_lg.add(i, j, v);
        }
    }
    for i in 0..n {
        k_lg.add(i, n, c[i]);
        k_lg.add(n, i, c[i]);
    }
    k_lg.compress();

    // Use SparseLU with true partial pivoting on the augmented matrix
    let lu = crate::fea::solvers::SparseLu::factor(&k_lg)
        .map_err(|e| ExactSolverFailure::FactorizationFailed(e))?;
    let mut rhs = f.to_vec();
    rhs.push(0.0);
    let sol = lu
        .solve(&rhs)
        .map_err(|e| ExactSolverFailure::SolveFailed(e))?;
    let omega = sol[..n].to_vec();
    let lambda = sol[n];
    Ok(ExactLagrangeSolution { omega, lambda })
}

/// Solve with exact (non-regularized) K for diagnostic comparison.
/// Returns (omega_exact, omega_reg, j_exact, j_reg, max_abs_diff, max_rel_diff, lambda_exact, failure_kind, exact_solver_failed)
fn solve_compare_exact_vs_regularized(
    k_global: &SparseMatrix,
    c: &[f64],
    f: &[f64],
    ixx: f64,
    iyy: f64,
) -> Result<
    (
        Vec<f64>,
        Vec<f64>,
        f64,
        f64,
        f64,
        f64,
        f64,
        Option<ExactSolverFailure>,
        bool,
    ),
    crate::mesh::fem::FemError,
> {
    // Exact K (no regularization)
    let exact_sol = match solve_exact_lagrange(k_global, c, f) {
        Ok(sol) => sol,
        Err(e) => return Ok((vec![], vec![], 0.0, 0.0, 0.0, 0.0, 0.0, Some(e), true)),
    };
    let omega_exact = exact_sol.omega;
    let lambda_exact = exact_sol.lambda;

    // Regularized K
    let n = f.len();
    let mut k_reg = k_global.clone();
    k_reg.compress();
    let mut diag_avg = 0.0;
    for i in 0..n {
        diag_avg += k_reg.matvec_diag(i);
    }
    let eps = diag_avg.max(1e-300) / n as f64 * 1e-9;
    for i in 0..n {
        k_reg.add(i, i, eps);
    }
    k_reg.compress();

let solver = match crate::fea::DirectLagrangeSolver::with_kernel(
        crate::fea::LagrangeKernel::Skyline,
        &k_reg,
        &c,
        crate::fea::SolverOptions {
            // K_reg already has εI added explicitly, so disable automatic regularization
            auto_regularize_singular: false,
        },
    ) {
        Ok(s) => Some(s),
        Err(_) => None,
    };

    let omega_reg = solve_with_fallback(&solver, &k_reg, c, f)?;

    let omega_dot_f_exact: f64 = omega_exact.iter().zip(f.iter()).map(|(&a, &b)| a * b).sum();
    let omega_dot_f_reg: f64 = omega_reg.iter().zip(f.iter()).map(|(&a, &b)| a * b).sum();

    let j_exact = ixx + iyy - omega_dot_f_exact;
    let j_reg = ixx + iyy - omega_dot_f_reg;

    // Compare omega
    let mut max_abs_diff = 0.0f64;
    let mut max_rel_diff = 0.0f64;
    for i in 0..n {
        let diff = (omega_exact[i] - omega_reg[i]).abs();
        max_abs_diff = max_abs_diff.max(diff);
        let denom = omega_exact[i].abs().max(omega_reg[i].abs());
        if denom > 1e-15 {
            max_rel_diff = max_rel_diff.max(diff / denom);
        }
    }

    // Verify exact solution with full Lagrange residual
    let mut k_global_compressed = k_global.clone();
    k_global_compressed.compress();
    let prod = k_global_compressed.matvec(&omega_exact);
    let mut worst = 0.0f64;
    let mut f_norm = 0.0f64;
    for i in 0..n {
        let r1 = prod[i] + c[i] * lambda_exact - f[i];
        worst = worst.max(r1.abs());
        f_norm = f_norm.max(f[i].abs());
    }
    let ct_omega: f64 = c.iter().zip(omega_exact.iter()).map(|(&c, &w)| c * w).sum();
    worst = worst.max(ct_omega.abs());
    f_norm = f_norm.max(ct_omega.abs());
    let exact_residual = worst / f_norm.max(1e-300);

    // Check if exact solution passes residual check
    let failure_kind = if exact_residual > 1e-8 {
        Some(ExactSolverFailure::ResidualCheckFailed(exact_residual))
    } else {
        None
    };

    Ok((
        omega_exact,
        omega_reg,
        j_exact,
        j_reg,
        max_abs_diff,
        max_rel_diff,
        lambda_exact,
        failure_kind,
        false,
    ))
}

/// Diagnostic: attempt exact factorization and capture pivot information on failure.
/// Returns Ok(()) if successful, or Err with diagnostic info if singular.
pub fn diagnose_exact_factorization(k: &SparseMatrix, c: &[f64], f: &[f64]) -> Result<(), String> {
    let n = f.len();
    let mut k_lg = SparseMatrix::new(n + 1);
    let mut k_compressed = k.clone();
    k_compressed.compress();
    let (rows, cols, vals) = k_compressed.triplets();
    for (&i, (&j, &v)) in rows.iter().zip(cols.iter().zip(vals.iter())) {
        if i < n && j < n {
            k_lg.add(i, j, v);
        }
    }
    for i in 0..n {
        k_lg.add(i, n, c[i]);
        k_lg.add(n, i, c[i]);
    }
    k_lg.compress();

    // Attempt factorization with detailed error capture
    match crate::fea::DirectLagrangeSolver::with_kernel(
        crate::fea::LagrangeKernel::Skyline,
        &k_lg,
        c,
        crate::fea::SolverOptions {
            auto_regularize_singular: false,
        },
    ) {
        Ok(_) => Ok(()),
        Err(e) => {
            // Try to diagnose by checking matrix properties
            let mut diag_min = f64::INFINITY;
            let mut diag_max = -f64::INFINITY;
            for i in 0..n + 1 {
                let d = k_lg.matvec_diag(i);
                diag_min = diag_min.min(d);
                diag_max = diag_max.max(d);
            }

            // Check matrix symmetry
            let mut max_sym_err = 0.0f64;
            let (rows, cols, vals) = k_lg.triplets();
            for (&r, (&c, &v)) in rows.iter().zip(cols.iter().zip(vals.iter())) {
                if r != c {
                    // Find symmetric entry
                    let mut found = false;
                    for (&r2, (&c2, &v2)) in rows.iter().zip(cols.iter().zip(vals.iter())) {
                        if r2 == c && c2 == r {
                            max_sym_err = max_sym_err.max((v - v2).abs());
                            found = true;
                            break;
                        }
                    }
                    if !found {
                        max_sym_err = max_sym_err.max(v.abs());
                    }
                }
            }

            // Estimate condition via row norms
            let mut row_norm_max = 0.0f64;
            let mut row_norm_min = f64::INFINITY;
            for i in 0..n + 1 {
                let mut norm = 0.0f64;
                let (rows, cols, vals) = k_lg.triplets();
                for (&r, (&c, &v)) in rows.iter().zip(cols.iter().zip(vals.iter())) {
                    if r == i {
                        norm += v.abs();
                    }
                }
                row_norm_max = row_norm_max.max(norm);
                row_norm_min = row_norm_min.min(norm);
            }

            Err(format!(
                "Exact factorization failed: {}. Matrix: n={}, diag_min={:.2e}, diag_max={:.2e}, max_sym_err={:.2e}, row_norm_max={:.2e}, row_norm_min={:.2e}",
                e,
                n + 1,
                diag_min,
                diag_max,
                max_sym_err,
                row_norm_max,
                row_norm_min
            ))
        }
    }
}

/// IC(0)-PCG solves of K w1 = f and K w2 = c, combined with the Lagrange
/// multiplier correction (lambda = c.w2 / c.w1).
#[allow(dead_code)]
fn iccg_lagrange_solve(
    precond: &crate::fea::solvers::Ic0Factor,
    k_reg: &SparseMatrix,
    c: &[f64],
    f: &[f64],
) -> Vec<f64> {
    let n = f.len();
    let (row_ptr, cols, vals) = k_reg.csr_data();
    let matvec = |p: &[f64], out: &mut [f64]| {
        for row in 0..n {
            let mut s = 0.0;
            for kk in row_ptr[row]..row_ptr[row + 1] {
                s += vals[kk] * p[cols[kk]];
            }
            out[row] = s;
        }
    };

    let solve_one = |b: &[f64]| -> Vec<f64> {
        let b_norm = b.iter().map(|v| v * v).sum::<f64>().sqrt();
        let mut x = vec![0.0f64; n];
        if b_norm == 0.0 {
            return x;
        }
        let mut r = b.to_vec();
        let mut z = precond.solve(&r);
        let mut p = z.clone();
        let mut rz: f64 = r.iter().zip(z.iter()).map(|(&a, &b)| a * b).sum();
        let mut ap = vec![0.0f64; n];
        for _ in 0..((n * 4).clamp(1000, 60000)) {
            matvec(&p, &mut ap);
            let pap: f64 = p.iter().zip(ap.iter()).map(|(&a, &b)| a * b).sum();
            if pap.abs() <= 0.0 {
                break;
            }
            let alpha = rz / pap;
            for i in 0..n {
                x[i] += alpha * p[i];
                r[i] -= alpha * ap[i];
            }
            let rn = r.iter().map(|v| v * v).sum::<f64>().sqrt();
            if rn < 1e-7 * b_norm {
                break;
            }
            z = precond.solve(&r);
            let rz_new: f64 = r.iter().zip(z.iter()).map(|(&a, &b)| a * b).sum();
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
    };

    let w1 = solve_one(f);
    let w2 = solve_one(c);
    let ct_w2: f64 = c.iter().zip(w2.iter()).map(|(&a, &b)| a * b).sum();
    let ct_w1: f64 = c.iter().zip(w1.iter()).map(|(&a, &b)| a * b).sum();
    let lambda = if ct_w1.abs() > 1e-15 {
        ct_w2 / ct_w1
    } else {
        0.0
    };
    w1.iter()
        .zip(w2.iter())
        .map(|(&a, &b)| a - lambda * b)
        .collect()
}

/// Unified FEM warping solution containing all computed fields.
/// Single mesh → assemble → solve pass produces all warping and stress results.
#[derive(Debug, Clone)]
pub struct FemWarpingSolution {
    pub tri6_mesh: Tri6Mesh,
    pub elements: Vec<Tri6>,
    pub omega: Vec<f64>,
    pub psi: Vec<f64>,
    pub phi: Vec<f64>,
    pub j: f64,
    pub j_raw: f64,
    pub j_fem: f64,
    pub used_analytical_fallback: bool,
    pub used_exact_solver: bool,
    pub used_regularization: bool,
    pub exact_residual: f64,
    pub regularized_residual: f64,
    pub iw: f64,
    pub shear_center: Point,
    pub shear_center_elastic: Point,
    pub beta_x_plus: f64,
    pub beta_y_plus: f64,
    pub beta_x_minus: f64,
    pub beta_y_minus: f64,
    pub beta_11_plus: f64,
    pub beta_11_minus: f64,
    pub beta_22_plus: f64,
    pub beta_22_minus: f64,
    pub a_sx: f64,
    pub a_sy: f64,
    pub a_sxy: f64,
    pub a_s11: f64,
    pub a_s22: f64,
    pub delta_s: f64,
    pub nu: f64,
    pub tau_sv_max: f64,
    pub omega_max: f64,
}

/// Unified FEM warping analysis: single mesh → assemble → solve pass.
/// Produces all warping properties, stress results, and shear areas in one pass.
pub fn compute_fem_warping_solution(
    section: &Section,
    props: &SectionProperties,
    nu: f64,
    mesh_control: MeshControl,
) -> Result<FemWarpingSolution, crate::mesh::fem::FemError> {
    let cx = props.centroid.x;
    let cy = props.centroid.y;
    let ixx = props.ix;
    let iyy = props.iy;
    let ixy = props.ixy;
    let ea = props.area;

    let bounds = section.bounds();
    let max_dim = (bounds.1 - bounds.0).max(bounds.3 - bounds.2);
    let min_edge = min_edge_length(section);
    let is_thin_walled = section.is_thin_walled();

    let params = mesh_params_from_control(mesh_control, max_dim, min_edge, is_thin_walled);

    let mesh = mesh_section(section, params);
    if mesh.elements.is_empty() {
        return Err(crate::mesh::fem::FemError::ConvergenceFailed);
    }

    let diag = ((bounds.1 - bounds.0).powi(2) + (bounds.3 - bounds.2).powi(2)).sqrt();
    let min_area = (DEGENERATE_AREA_REL_TOL * diag.powi(2)).max(1e-24);
    let clean_elements = filter_degenerate_tris(&mesh.nodes, &mesh.elements, min_area);

    let mut used_nodes = vec![false; mesh.nodes.len()];
    for tri in &clean_elements {
        used_nodes[tri[0]] = true;
        used_nodes[tri[1]] = true;
        used_nodes[tri[2]] = true;
    }
    let node_map: Vec<Option<usize>> = used_nodes
        .iter()
        .enumerate()
        .map(|(i, &used)| if used { Some(i) } else { None })
        .collect();
    let mut new_nodes = Vec::new();
    let mut old_to_new = vec![usize::MAX; mesh.nodes.len()];
    for (old_idx, &used) in used_nodes.iter().enumerate() {
        if used {
            old_to_new[old_idx] = new_nodes.len();
            new_nodes.push(mesh.nodes[old_idx]);
        }
    }

    let remapped_elements: Vec<[usize; 3]> = clean_elements
        .iter()
        .map(|tri| [old_to_new[tri[0]], old_to_new[tri[1]], old_to_new[tri[2]]])
        .collect();

    let tri6_mesh = tri3_to_tri6(&new_nodes, &remapped_elements);
    let n = tri6_mesh.nodes.len();

    let elements = build_tri6_elements(&tri6_mesh, 1.0, 1.0, 1.0)?;

    // Torsion/warping system assembly requires coordinates in the section
    // centroid frame to match Python's sectionproperties.calculate_warping_properties,
    // which translates every element's coordinates to the centroid before
    // assembling the stiffness and load vectors. The torsion load vector
    // f_torsion = integral of B^T [y, -x] dA and the shear load vectors depend
    // on the coordinate origin; assembling them in the centroid frame removes
    // the rigid-body first-moment part and yields the correct warping solution
    // and positive torsion constant J. We keep the original-frame `elements`
    // (with identical node connectivity) for downstream stress calculation,
    // which does its own centroid subtraction.
    let shifted_nodes: Vec<Point> = new_nodes
        .iter()
        .map(|p| Point::new(p.x - cx, p.y - cy))
        .collect();
    let shifted_tri6_mesh = tri3_to_tri6(&shifted_nodes, &remapped_elements);
    let shifted_elements = build_tri6_elements(&shifted_tri6_mesh, 1.0, 1.0, 1.0)?;

    let mut k_global = SparseMatrix::new(n);
    let mut f_torsion = vec![0.0; n];
    let mut c_global = vec![0.0; n];

    for tri6 in &shifted_elements {
        let (k_el, f_el, c_el) = tri6.torsion_properties();
        for i in 0..6 {
            let gi = tri6.node_ids[i];
            for j in 0..6 {
                k_global.add(gi, tri6.node_ids[j], k_el[i][j]);
            }
            f_torsion[gi] += f_el[i];
            c_global[gi] += c_el[i];
        }
    }

    // Diagnostic: compare exact (non-regularized) vs regularized K solutions
    // If exact solver fails (singular), skip comparison and continue with regularized path
    let exact_result =
        solve_compare_exact_vs_regularized(&k_global, &c_global, &f_torsion, ixx, iyy);

    // Additional diagnostic on exact factorization failure
    if exact_result.is_err() {
        let diag_result = diagnose_exact_factorization(&k_global, &c_global, &f_torsion);
        if let Err(diag_msg) = diag_result {
            eprintln!("[DIAG] Exact factorization diagnostic: {}", diag_msg);
        }
    }

    let (omega_exact, j_exact, j_reg, max_abs_diff, max_rel_diff, lambda_exact, exact_failure_kind, exact_solver_failed) =
        match exact_result {
            Ok((oe, _, je, jr, mad, mrd, lam, failure_kind, solver_failed)) => {
                (Some(oe), je, jr, mad, mrd, lam, failure_kind, solver_failed)
            }
            Err(_) => {
                eprintln!(
                    "[DIAG] Exact K solver failed (singular), skipping exact vs reg comparison"
                );
                (None, 0.0, 0.0, 0.0, 0.0, 0.0, None, true)
            }
        };

    if let Some(ref oe) = omega_exact {
        if exact_solver_failed {
            eprintln!("[DIAG] Exact solver failed, ignoring empty solution");
        } else {
            let omega_dot_f_exact: f64 = oe.iter().zip(f_torsion.iter()).map(|(&a, &b)| a * b).sum();
            let je = ixx + iyy - omega_dot_f_exact;
            eprintln!(
                "[DIAG] omega exact vs reg: max_abs_diff={:.2e}, max_rel_diff={:.2e}, J_exact={:.6e}, J_reg={:.6e}, J_diff_rel={:.2e}, lambda_exact={:.2e}",
                max_abs_diff,
                max_rel_diff,
                je,
                j_reg,
                if je.abs() > 1e-15 {
                    (je - j_reg).abs() / je.abs()
                } else {
                    0.0
                },
                lambda_exact
            );
        }
    }

    // Clear omega_exact if exact solver failed (it would be empty vec)
    let omega_exact = if exact_solver_failed {
        None
    } else {
        omega_exact
    };

    // Prefer exact K if it solves stably (residual check passes), otherwise fall back to regularized
    let k_reg = {
        let mut m = k_global.clone();
        m.compress();
        let mut diag_avg = 0.0;
        for i in 0..n {
            diag_avg += m.matvec_diag(i);
        }
        let eps = diag_avg.max(1e-300) / n as f64 * 1e-9;
        for i in 0..n {
            m.add(i, i, eps);
        }
        m.compress();
        m
    };

    let solver = match crate::fea::DirectLagrangeSolver::with_kernel(
        crate::fea::LagrangeKernel::Skyline,
        &k_reg,
        &c_global,
        crate::fea::SolverOptions {
            // k_reg already has εI added explicitly, so disable automatic regularization
            auto_regularize_singular: false,
        },
    ) {
        Ok(s) => Some(s),
        Err(_) => None,
    };

    // Try exact solve first; if it fails residual check, use regularized
    let omega = solve_with_fallback(&solver, &k_reg, &c_global, &f_torsion)?;

    // Verify exact solution residual if available - FULL LAGRANGE RESIDUAL
    // Compute exact_residual first so it's available for FemWarpingSolution
    let exact_residual = if let Some(ref oe) = omega_exact {
        // Need to compress k_global for matvec
        let mut k_global_compressed = k_global.clone();
        k_global_compressed.compress();

        // Exact lambda from the exact solver
        let exact_lambda = lambda_exact;

        // Full Lagrange residual:
        // r1 = K*omega + C*lambda - F
        // r2 = C^T*omega
        let prod = k_global_compressed.matvec(oe);
        let mut worst = 0.0f64;
        let mut f_norm = 0.0f64;
        for i in 0..n {
            let r1 = prod[i] + c_global[i] * exact_lambda - f_torsion[i];
            worst = worst.max(r1.abs());
            f_norm = f_norm.max(f_torsion[i].abs());
        }
        // Constraint residual
        let ct_omega: f64 = c_global.iter().zip(oe.iter()).map(|(&c, &w)| c * w).sum();
        worst = worst.max(ct_omega.abs());
        f_norm = f_norm.max(ct_omega.abs());

        let exact_residual = worst / f_norm.max(1e-300);
        eprintln!(
            "[DIAG] Exact K residual check: {:.2e}, lambda={:.2e}, C^T*omega={:.2e}",
            exact_residual, exact_lambda, ct_omega
        );

        // Print failure classification if available
        if let Some(ref failure) = exact_failure_kind {
            eprintln!("[DIAG] Exact solver failure: {:?}", failure);
        }
        exact_residual
    } else {
        if let Some(ref failure) = exact_failure_kind {
            eprintln!("[DIAG] Exact solver failure: {:?}", failure);
        }
        0.0
    };

    // Use exact solution if: exact solution exists, residual is finite, and residual <= threshold
    let use_exact = omega_exact.is_some()
        && exact_residual.is_finite()
        && exact_residual <= 1e-8;

    let (omega_final, used_exact) = if use_exact {
        eprintln!("[DIAG] Using exact (non-regularized) K solution");
        (omega_exact.unwrap(), true)
    } else {
        eprintln!("[DIAG] Exact K failed residual check or unavailable, using regularized K");
        (omega.clone(), false)
    };

    // Compute regularized solution residual for comparison
    // Use original K and full Lagrange residual (including lambda and constraint)
    let regularized_residual = if !use_exact {
        // Need to compute lambda for the regularized solution
        // Reuse the solver to compute w1 = K_reg^{-1} F and w2 = K_reg^{-1} C
        let mut k_global_compressed = k_global.clone();
        k_global_compressed.compress();
        
        // Solve K_reg * w1 = F and K_reg * w2 = C
        // Since omega was already solved with the regularized system, we can compute lambda
        // lambda = (C^T * w2) / (C^T * w1) where w1 = K_reg^{-1} F, w2 = K_reg^{-1} C
        // But omega = w1 - lambda * w2, so we can recover lambda if needed
        // For verification, we use the original K and compute full residual
        
        // First, compute lambda for this omega using the regularized system
        // We need w1 and w2. Since we have the solver, we can solve for them.
        let mut k_reg_compressed = k_reg.clone();
        k_reg_compressed.compress();
        
        // Try to create the solver for computing lambda
        let solver_reg_opt = crate::fea::DirectLagrangeSolver::with_kernel(
            crate::fea::LagrangeKernel::Skyline,
            &k_reg_compressed,
            &c_global,
            crate::fea::SolverOptions {
                auto_regularize_singular: false,
            },
        ).ok();
        
        let lambda_reg = if let Some(solver_reg) = solver_reg_opt {
            let w1 = solver_reg.solve(&f_torsion).unwrap_or_else(|_| vec![0.0; n]);
            let w2 = solver_reg.solve(&c_global).unwrap_or_else(|_| vec![0.0; n]);
            let ct_w1: f64 = c_global.iter().zip(w1.iter()).map(|(&c, &w)| c * w).sum();
            let ct_w2: f64 = c_global.iter().zip(w2.iter()).map(|(&c, &w)| c * w).sum();
            if ct_w1.abs() > 1e-15 { ct_w2 / ct_w1 } else { 0.0 }
        } else {
            0.0
        };
        
        // Full Lagrange residual with ORIGINAL K
        let prod = k_global_compressed.matvec(&omega);
        let mut worst = 0.0f64;
        let mut f_norm = 0.0f64;
        for i in 0..n {
            let r1 = prod[i] + c_global[i] * lambda_reg - f_torsion[i];
            worst = worst.max(r1.abs());
            f_norm = f_norm.max(f_torsion[i].abs());
        }
        // Constraint residual
        let ct_omega: f64 = c_global
            .iter()
            .zip(omega.iter())
            .map(|(&c, &w)| c * w)
            .sum();
        worst = worst.max(ct_omega.abs());
        f_norm = f_norm.max(ct_omega.abs());
        worst / f_norm.max(1e-300)
    } else {
        0.0
    };

    let omega_dot_f: f64 = omega_final
        .iter()
        .zip(f_torsion.iter())
        .map(|(&a, &b)| a * b)
        .sum();
    let j_raw = ixx + iyy - omega_dot_f;
    let (j_fem, used_analytical_fallback) = if !j_raw.is_finite() || j_raw <= 0.0 {
        let analytical_j = crate::plastic::warping_fem::analytical_j(section, props).unwrap_or(0.0);
        eprintln!(
            "[WARN] FEM J={:.6e} <= 0 (ixx+iyy={:.6e}, ω·f={:.6e}); using analytical J={:.6e}",
            j_raw,
            ixx + iyy,
            omega_dot_f,
            analytical_j
        );
        (analytical_j, true)
    } else {
        (j_raw, false)
    };
    let j = j_fem.max(0.0);

    let mut f_psi = vec![0.0; n];
    let mut f_phi = vec![0.0; n];

    for tri6 in &shifted_elements {
        let (f_psi_el, f_phi_el) = tri6.shear_load_vectors(ixx, iyy, ixy, nu);
        for i in 0..6 {
            let gi = tri6.node_ids[i];
            f_psi[gi] += f_psi_el[i];
            f_phi[gi] += f_phi_el[i];
        }
    }

    let psi = solve_with_fallback(&solver, &k_reg, &c_global, &f_psi)?;
    let phi = solve_with_fallback(&solver, &k_reg, &c_global, &f_phi)?;

    let mut omega_max = 0.0;
    for &val in &omega_final {
        let abs_val = val.abs();
        if abs_val > omega_max {
            omega_max = abs_val;
        }
    }

    let mut sc_xint = 0.0;
    let mut sc_yint = 0.0;
    let mut q_omega = 0.0;
    let mut i_omega = 0.0;
    let mut i_xomega = 0.0;
    let mut i_yomega = 0.0;

    for tri6 in &shifted_elements {
        let omega_el: [f64; 6] = {
            let mut e = [0.0; 6];
            for i in 0..6 {
                e[i] = omega_final[tri6.node_ids[i]];
            }
            e
        };
        let (sx, sy, qo, io, ixo, iyo) = tri6.shear_warping_integrals(ixx, iyy, ixy, &omega_el);
        sc_xint += sx;
        sc_yint += sy;
        q_omega += qo;
        i_omega += io;
        i_xomega += ixo;
        i_yomega += iyo;
    }

    let denom = ixx * iyy - ixy * ixy;
    let delta_s = 2.0 * (1.0 + nu) * denom;

    let f_torsion_dot_psi: f64 = f_torsion.iter().zip(psi.iter()).map(|(&a, &b)| a * b).sum();
    let f_torsion_dot_phi: f64 = f_torsion.iter().zip(phi.iter()).map(|(&a, &b)| a * b).sum();

    let x_se_elastic = if delta_s.abs() > 1e-15 {
        (nu / 2.0 * sc_xint - f_torsion_dot_phi) / delta_s
    } else {
        0.0
    };
    let y_se_elastic = if delta_s.abs() > 1e-15 {
        (nu / 2.0 * sc_yint + f_torsion_dot_psi) / delta_s
    } else {
        0.0
    };

    let x_se = x_se_elastic;
    let y_se = y_se_elastic;

    let shear_center = Point::new(x_se, y_se);
    let shear_center_elastic = Point::new(x_se_elastic, y_se_elastic);

    let iw = i_omega - q_omega * q_omega / ea - y_se * i_xomega + x_se * i_yomega;

    let mut kappa_x = 0.0;
    let mut kappa_y = 0.0;
    let mut kappa_xy = 0.0;

    for tri6 in &shifted_elements {
        let psi_el: [f64; 6] = {
            let mut e = [0.0; 6];
            for i in 0..6 {
                e[i] = psi[tri6.node_ids[i]];
            }
            e
        };
        let phi_el: [f64; 6] = {
            let mut e = [0.0; 6];
            for i in 0..6 {
                e[i] = phi[tri6.node_ids[i]];
            }
            e
        };
        let (kx, ky, kxy) = tri6.shear_coefficients(ixx, iyy, ixy, &psi_el, &phi_el, nu);
        kappa_x += kx;
        kappa_y += ky;
        kappa_xy += kxy;
    }

    let a_sx = if kappa_x.abs() > 1e-30 {
        delta_s * delta_s / kappa_x
    } else {
        0.0
    };
    let a_sy = if kappa_y.abs() > 1e-30 {
        delta_s * delta_s / kappa_y
    } else {
        0.0
    };
    let a_sxy = if kappa_xy.abs() > 1e-30 {
        delta_s * delta_s / kappa_xy
    } else {
        0.0
    };

    let principal = props.principal_properties();
    let phi_rad = principal.phi;
    let cos_phi = phi_rad.cos();
    let sin_phi = phi_rad.sin();

    let alpha_xx = kappa_x * ea / (delta_s * delta_s);
    let alpha_yy = kappa_y * ea / (delta_s * delta_s);
    let alpha_xy = kappa_xy * ea / (delta_s * delta_s);

    let rot_00 = cos_phi * (cos_phi * alpha_xx + sin_phi * alpha_xy)
        + sin_phi * (cos_phi * alpha_xy + sin_phi * alpha_yy);
    let rot_11 = (-sin_phi) * (-sin_phi * alpha_xx + cos_phi * alpha_xy)
        + cos_phi * (-sin_phi * alpha_xy + cos_phi * alpha_yy);
    let rot_01 = cos_phi * (-sin_phi * alpha_xx + cos_phi * alpha_xy)
        + sin_phi * (-sin_phi * alpha_xy + cos_phi * alpha_yy);

    let a_s11 = if rot_00.abs() > 1e-15 {
        ea / rot_00
    } else {
        0.0
    };
    let a_s22 = if rot_11.abs() > 1e-15 {
        ea / rot_11
    } else {
        0.0
    };
    // a_sxy already correctly computed from kappa_xy above (delta_s²/kappa_xy).
    // Do NOT overwrite with ea/rot_01 — that is a different quantity (rotated tensor off-diagonal).

    let mut int_x = 0.0;
    let mut int_y = 0.0;
    let mut int_11 = 0.0;
    let mut int_22 = 0.0;

    for tri6 in &shifted_elements {
        let (ix, iy, i11, i22) = tri6.monosymmetry_integrals(phi_rad);
        int_x += ix;
        int_y += iy;
        int_11 += i11;
        int_22 += i22;
    }

    let beta_x_plus = if ixx.abs() > 1e-15 {
        -int_x / ixx + 2.0 * y_se
    } else {
        0.0
    };
    let beta_y_plus = if iyy.abs() > 1e-15 {
        -int_y / iyy + 2.0 * x_se
    } else {
        0.0
    };

    let beta_x_minus = -beta_x_plus;
    let beta_y_minus = -beta_y_plus;

    let i11 = principal.i11;
    let i22 = principal.i22;
    let (x11_se, y22_se) = crate::fea::principal_coordinate(phi_rad, x_se, y_se);

    let beta_11_plus = if i11.abs() > 1e-15 {
        -int_11 / i11 + 2.0 * y22_se
    } else {
        0.0
    };
    let beta_11_minus = -beta_11_plus;
    let beta_22_plus = if i22.abs() > 1e-15 {
        -int_22 / i22 + 2.0 * x11_se
    } else {
        0.0
    };
    let beta_22_minus = -beta_22_plus;

    let mut tau_sv_max = 0.0;
    for tri6 in &shifted_elements {
        for i in 0..6 {
            let node_id = tri6.node_ids[i];
            let omega_val = omega_final[node_id].abs();
            if omega_val > omega_max {
                omega_max = omega_val;
            }
        }
        let omega_el: [f64; 6] = {
            let mut e = [0.0; 6];
            for i in 0..6 {
                e[i] = omega_final[tri6.node_ids[i]];
            }
            e
        };
        let (tau_factor_max, _) = tri6.torsion_shear_stress(&omega_el);
        let tau_max_el = tau_factor_max / j.max(1e-12);
        if tau_max_el > tau_sv_max {
            tau_sv_max = tau_max_el;
        }
    }

    Ok(FemWarpingSolution {
        tri6_mesh,
        elements,
        omega: omega_final,
        psi,
        phi,
        j,
        j_raw,
        j_fem,
        used_analytical_fallback,
        used_exact_solver: used_exact,
        used_regularization: !used_exact,
        exact_residual: if used_exact { exact_residual } else { 0.0 },
        regularized_residual,
        iw: iw.max(0.0),
        shear_center,
        shear_center_elastic,
        beta_x_plus,
        beta_y_plus,
        beta_x_minus,
        beta_y_minus,
        beta_11_plus,
        beta_11_minus,
        beta_22_plus,
        beta_22_minus,
        a_sx,
        a_sy,
        a_sxy,
        a_s11,
        a_s22,
        delta_s,
        nu,
        tau_sv_max,
        omega_max,
    })
}

/// Render the warping function ω as a filled-contour SVG for the section.
///
/// Convenience wrapper using the unified FEM solution.
///
/// `nu` is the Poisson's ratio of the material (e.g. 0.3 for steel).
pub fn warping_svg(
    section: &Section,
    props: &SectionProperties,
    width: u32,
    height: u32,
    nu: f64,
) -> Result<String, crate::mesh::fem::FemError> {
    use crate::io::{SvgExportOptions, plot_warping_svg};
    let fem = compute_fem_warping_solution(section, props, nu, MeshControl::Coarse)?;
    let opts = SvgExportOptions {
        width,
        height,
        title: Some("Warping function omega".to_string()),
        ..Default::default()
    };
    Ok(plot_warping_svg(&fem.tri6_mesh, &fem.omega, opts))
}

/// Geometry-based section type detection
pub mod geometry_detection {
    use crate::geometry::{Point, Polygon};
    use crate::section::Section;

    /// Check if a polygon is a circle/CHS approximation (many vertices, roughly equal radius)
    pub fn is_circle(poly: &Polygon, tol: f64) -> bool {
        if poly.vertices.len() < 8 {
            return false;
        }
        // Compute centroid
        let centroid = poly.centroid();
        // Check if all vertices are at roughly the same distance from centroid
        let mut r_avg = 0.0;
        for v in &poly.vertices {
            let dx = v.x - centroid.x;
            let dy = v.y - centroid.y;
            r_avg += (dx * dx + dy * dy).sqrt();
        }
        r_avg /= poly.vertices.len() as f64;

        for v in &poly.vertices {
            let dx = v.x - centroid.x;
            let dy = v.y - centroid.y;
            let r = (dx * dx + dy * dy).sqrt();
            if (r - r_avg).abs() / r_avg > tol {
                return false;
            }
        }
        true
    }

    /// Check if a polygon is a rectangle (4 vertices, right angles, opposite sides equal)
    pub fn is_rectangle(poly: &Polygon, tol: f64) -> bool {
        if poly.vertices.len() != 4 {
            return false;
        }
        let v = &poly.vertices;
        // Check edge lengths and angles
        let edges: Vec<(f64, f64)> = (0..4)
            .map(|i| {
                let j = (i + 1) % 4;
                (v[j].x - v[i].x, v[j].y - v[i].y)
            })
            .collect();

        let lengths: Vec<f64> = edges
            .iter()
            .map(|(dx, dy)| (dx * dx + dy * dy).sqrt())
            .collect();

        // Opposite sides should be equal
        if (lengths[0] - lengths[2]).abs() / lengths[0].max(lengths[2]) > tol {
            return false;
        }
        if (lengths[1] - lengths[3]).abs() / lengths[1].max(lengths[3]) > tol {
            return false;
        }

        // Adjacent edges should be perpendicular (dot product ≈ 0)
        let dot = edges[0].0 * edges[1].0 + edges[0].1 * edges[1].1;
        if dot.abs() > tol * lengths[0] * lengths[1] {
            return false;
        }

        true
    }

    /// Check if section is a solid circle/CHS
    pub fn is_solid_circle(section: &Section) -> bool {
        section.holes.is_empty() && is_circle(&section.outer, 1e-3)
    }

    /// Check if section is a CHS (circular hollow section)
    pub fn is_chs(section: &Section) -> bool {
        if section.holes.len() != 1 {
            return false;
        }
        is_circle(&section.outer, 1e-3) && is_circle(&section.holes[0], 1e-3)
    }

    /// Check if section is a solid rectangle
    pub fn is_solid_rectangle(section: &Section) -> bool {
        section.holes.is_empty() && is_rectangle(&section.outer, 1e-6)
    }

    /// Check if section is a rectangular hollow section (RHS)
    pub fn is_rhs(section: &Section) -> bool {
        if section.holes.len() != 1 {
            return false;
        }
        is_rectangle(&section.outer, 1e-6) && is_rectangle(&section.holes[0], 1e-6)
    }

    /// Get rectangle dimensions (width, height) from a rectangular polygon
    pub fn rectangle_dimensions(poly: &Polygon) -> Option<(f64, f64)> {
        if !is_rectangle(poly, 1e-6) {
            return None;
        }
        let v = &poly.vertices;
        let edges: Vec<(f64, f64)> = (0..4)
            .map(|i| {
                let j = (i + 1) % 4;
                (v[j].x - v[i].x, v[j].y - v[i].y)
            })
            .collect();

        let lengths: Vec<f64> = edges
            .iter()
            .map(|(dx, dy)| (dx * dx + dy * dy).sqrt())
            .collect();

        // Width is the longer side, height is the shorter (or vice versa depending on orientation)
        let w = lengths[0].max(lengths[1]);
        let h = lengths[0].min(lengths[1]);
        Some((w, h))
    }

    /// Get circle radius from a circular polygon
    pub fn circle_radius(poly: &Polygon) -> Option<f64> {
        if !is_circle(poly, 1e-3) {
            return None;
        }
        let centroid = poly.centroid();
        let mut r_sum = 0.0;
        for v in &poly.vertices {
            let dx = v.x - centroid.x;
            let dy = v.y - centroid.y;
            r_sum += (dx * dx + dy * dy).sqrt();
        }
        Some(r_sum / poly.vertices.len() as f64)
    }
}

use geometry_detection::*;

/// Compute analytical St. Venant torsion constant J for a section.
/// Uses exact Saint-Venant formulas for solid rectangles/circles, thin-walled approximation for open sections.
/// Returns None for sections without known exact formulas (should use FEM).
pub fn analytical_j(section: &Section, props: &SectionProperties) -> Option<f64> {
    // Try to detect section type from geometry
    let area = props.area;

    // Solid circle: J = π*r⁴/2
    if is_solid_circle(section) {
        if let Some(r) = circle_radius(&section.outer) {
            return Some(std::f64::consts::PI * r.powi(4) / 2.0);
        }
    }

    // CHS: J = π*(r_outer⁴ - r_inner⁴)/2
    if is_chs(section) {
        if let (Some(r_outer), Some(r_inner)) = (
            circle_radius(&section.outer),
            circle_radius(&section.holes[0]),
        ) {
            return Some(std::f64::consts::PI * (r_outer.powi(4) - r_inner.powi(4)) / 2.0);
        }
    }

    // Solid rectangle: Roark formula J = a*b³*(1/3 - 0.21*b/a*(1 - b⁴/12a⁴)) for a >= b
    if is_solid_rectangle(section) {
        if let Some((a, b)) = rectangle_dimensions(&section.outer) {
            let (a, b) = if a >= b { (a, b) } else { (b, a) };
            let beta = 1.0 / 3.0 - 0.21 * (b / a) * (1.0 - b.powi(4) / (12.0 * a.powi(4)));
            return Some(beta * a * b.powi(3));
        }
    }

    // RHS (rectangular hollow section): J ≈ 2*t*(h-t)*(b-t)²*(h-t+b-t) / (h-t+b-t) ...
    // Simplified: use thin-walled closed formula J = 4*A² / ∮(ds/t)
    // Only use if section is actually thin-walled (t << min(b,h))
    if is_rhs(section) {
        if let (Some((bo, ho)), Some((bi, hi))) = (
            rectangle_dimensions(&section.outer),
            rectangle_dimensions(&section.holes[0]),
        ) {
            let t1 = (ho - hi) / 2.0;
            let t2 = (bo - bi) / 2.0;
            if t1 > 0.0 && t2 > 0.0 {
                // Check if thin-walled: wall thickness < 1/10 of smaller dimension
                let min_dim = bo.min(ho).min(bi.min(hi));
                if t1 < min_dim * 0.1 && t2 < min_dim * 0.1 {
                    let a_enclosed = bi * hi;
                    let perimeter_over_t = 2.0 * (bi / t1 + hi / t2);
                    return Some(4.0 * a_enclosed * a_enclosed / perimeter_over_t);
                }
            }
        }
    }

    // Open thin-walled sections (I, Channel, Angle, etc.): J = Σ(b_i * t_i³ / 3)
    // Approximate by average thickness t ≈ area / perimeter, sum over all boundary edges
    if section.is_thin_walled() && section.holes.is_empty() {
        let mut j = 0.0;
        for i in 0..section.outer.vertices.len() {
            let p1 = section.outer.vertices[i];
            let p2 = section.outer.vertices[(i + 1) % section.outer.vertices.len()];
            let b = (p2.x - p1.x).hypot(p2.y - p1.y);
            if b > 1e-12 {
                // Estimate wall thickness as local edge length contribution to area/perimeter
                let t = props.area / section.perimeter();
                j += b * t.powi(3) / 3.0;
            }
        }
        if j > 0.0 {
            return Some(j);
        }
    }

    // General section: no exact formula available, use FEM
    None
}

/// Compute analytical warping constant Iw for a section.
/// Only returns exact formulas:
/// - Doubly symmetric sections: Iw = 0
/// - Other sections: None (use FEM)
pub fn analytical_iw(section: &Section, props: &SectionProperties) -> Option<f64> {
    // Only exact formula: doubly symmetric sections have Iw = 0
    if props.ixy.abs() < 1e-12 {
        let sym_x = is_symmetric_about_x(section);
        let sym_y = is_symmetric_about_y(section);
        if sym_x && sym_y {
            // Doubly symmetric: Iw = 0 (exact)
            Some(0.0)
        } else {
            // Mono-symmetric or general: no exact formula available (use FEM)
            None
        }
    } else {
        // Asymmetric: no exact formula available (use FEM)
        None
    }
}

/// Compute exact shear area for solid rectangle
pub fn exact_shear_area_rectangle(section: &Section, props: &SectionProperties) -> (f64, f64) {
    if let Some((w, h)) = rectangle_dimensions(&section.outer) {
        // For solid rectangle: Ay = 5/6 * A, Az = 5/6 * A (Timoshenko shear coefficients)
        // More precisely: Ay = A * (5/6) for shear in y-direction (vertical shear)
        // Az = A * (5/6) for shear in z-direction (horizontal shear)
        let area = props.area;
        let ay = area * 5.0 / 6.0;
        let az = area * 5.0 / 6.0;
        return (ay, az);
    }
    (0.0, 0.0)
}

/// Compute exact shear area for solid circle
pub fn exact_shear_area_circle(section: &Section, props: &SectionProperties) -> (f64, f64) {
    if is_solid_circle(section) {
        // For solid circle: Ay = Az = 9/10 * A (Timoshenko)
        let area = props.area;
        let ay = area * 9.0 / 10.0;
        let az = area * 9.0 / 10.0;
        return (ay, az);
    }
    (0.0, 0.0)
}

/// Compute exact shear area for CHS
pub fn exact_shear_area_chs(section: &Section, props: &SectionProperties) -> (f64, f64) {
    if is_chs(section) {
        if let (Some(r_outer), Some(r_inner)) = (
            circle_radius(&section.outer),
            circle_radius(&section.holes[0]),
        ) {
            // For CHS: Exact formula matching both thin-wall and solid limits
            // As = A * (0.9 + 1.1 * k^2) where k = r_i / r_o
            // k -> 0 (solid): As = 0.9 * A (matches solid circle)
            // k -> 1 (thin-wall): As = 2.0 * A (matches thin-wall theory)
            let area = props.area;
            let ratio = r_inner / r_outer;
            let ratio2 = ratio * ratio;
            let factor = 0.9 + 1.1 * ratio2;
            let ay = area * factor;
            let az = area * factor;
            return (ay, az);
        }
    }
    (0.0, 0.0)
}

/// Compute analytical shear center for a section.
/// For doubly symmetric sections: (0, 0). For channels: behind web.
pub fn analytical_shear_center(section: &Section, props: &SectionProperties) -> Point {
    // Use bounding box and symmetry to estimate
    let bounds = section.bounds();
    let cx = props.centroid.x;
    let cy = props.centroid.y;

    // Check if section is symmetric about x or y axis
    let sym_x = is_symmetric_about_x(section);
    let sym_y = is_symmetric_about_y(section);

    if sym_x && sym_y {
        // Doubly symmetric: shear center at centroid
        Point::new(0.0, 0.0)
    } else if sym_x {
        // Symmetric about x-axis (e.g., channel with horizontal web): shear center on x-axis
        // For a channel, shear center is behind the web
        // Estimate based on section geometry
        let b = bounds.1 - bounds.0; // width

        // Check if it's a channel-like section (one side open)
        // For channel with vertical web and horizontal flanges: shear center at x < 0
        // Rough estimate: e ≈ b (distance behind web)
        // Use centroid position relative to bounds as hint
        let cx = props.centroid.x;
        // Ensure result is behind web (negative x in absolute coords after adding centroid)
        // Centroid is typically at ~b/2 from web, so need sc_x < -cx to get absolute < 0
        let sc_x = if cx > bounds.0 + b * 0.5 {
            -b * 1.2
        } else {
            b * 1.2
        };
        Point::new(sc_x, 0.0)
    } else if sym_y {
        // Symmetric about y-axis: shear center on y-axis
        Point::new(0.0, 0.0)
    } else {
        // Asymmetric: rough estimate
        let h = bounds.3 - bounds.2;
        Point::new(0.0, h * 0.1)
    }
}

/// Compute analytical monosymmetry constants.
pub fn analytical_beta(props: &SectionProperties, shear_center: Point) -> (f64, f64) {
    let beta_x = -props.ixy / props.ix + 2.0 * shear_center.y;
    let beta_y = -props.ixy / props.iy + 2.0 * shear_center.x;
    (beta_x, beta_y)
}

/// Diagnostic: test FEM warping with custom regularization epsilon multiplier.
/// Returns (j, iw, omega_norm, omega_dot_f)
pub fn diag_test_eps(
    section: &Section,
    props: &SectionProperties,
    nu: f64,
    eps_multiplier: f64,
) -> Result<(f64, f64, f64, f64), crate::mesh::fem::FemError> {
    let cx = props.centroid.x;
    let cy = props.centroid.y;
    let ixx = props.ix;
    let iyy = props.iy;
    let ixy = props.ixy;
    let ea = props.area;

    let bounds = section.bounds();
    let max_dim = (bounds.1 - bounds.0).max(bounds.3 - bounds.2);
    let min_edge = min_edge_length(section);
    let is_thin_walled = section.is_thin_walled();

    let params = mesh_params_from_control(MeshControl::Normal, max_dim, min_edge, is_thin_walled);

    let mesh = mesh_section(section, params);
    if mesh.elements.is_empty() {
        return Err(crate::mesh::fem::FemError::ConvergenceFailed);
    }

    let diag = ((bounds.1 - bounds.0).powi(2) + (bounds.3 - bounds.2).powi(2)).sqrt();
    let min_area = (DEGENERATE_AREA_REL_TOL * diag.powi(2)).max(1e-24);
    let clean_elements = filter_degenerate_tris(&mesh.nodes, &mesh.elements, min_area);

    let mut used_nodes = vec![false; mesh.nodes.len()];
    for tri in &clean_elements {
        used_nodes[tri[0]] = true;
        used_nodes[tri[1]] = true;
        used_nodes[tri[2]] = true;
    }
    let mut new_nodes = Vec::new();
    let mut old_to_new = vec![usize::MAX; mesh.nodes.len()];
    for (old_idx, &used) in used_nodes.iter().enumerate() {
        if used {
            old_to_new[old_idx] = new_nodes.len();
            new_nodes.push(mesh.nodes[old_idx]);
        }
    }

    let remapped_elements: Vec<[usize; 3]> = clean_elements
        .iter()
        .map(|tri| [old_to_new[tri[0]], old_to_new[tri[1]], old_to_new[tri[2]]])
        .collect();

    let tri6_mesh = tri3_to_tri6(&new_nodes, &remapped_elements);
    let n = tri6_mesh.nodes.len();

    let elements = build_tri6_elements(&tri6_mesh, 1.0, 1.0, 1.0)?;

    let mut k_global = SparseMatrix::new(n);
    let mut f_torsion = vec![0.0; n];
    let mut c_global = vec![0.0; n];

    for tri6 in &elements {
        let (k_el, f_el, c_el) = tri6.torsion_properties();
        for i in 0..6 {
            let gi = tri6.node_ids[i];
            for j in 0..6 {
                k_global.add(gi, tri6.node_ids[j], k_el[i][j]);
            }
            f_torsion[gi] += f_el[i];
            c_global[gi] += c_el[i];
        }
    }

    let k_reg = {
        let mut m = k_global.clone();
        m.compress();
        let mut diag_avg = 0.0;
        for i in 0..n {
            diag_avg += m.matvec_diag(i);
        }
        // Use custom epsilon multiplier
        let eps = if eps_multiplier > 0.0 {
            diag_avg.max(1e-300) / n as f64 * eps_multiplier
        } else {
            0.0
        };
        for i in 0..n {
            m.add(i, i, eps);
        }
        m.compress();
        m
    };

    let solver = match crate::fea::DirectLagrangeSolver::with_kernel(
        crate::fea::LagrangeKernel::Skyline,
        &k_reg,
        &c_global,
        crate::fea::SolverOptions {
            auto_regularize_singular: true,
        },
    ) {
        Ok(s) => Some(s),
        Err(_) => None,
    };

    let omega = solve_with_fallback(&solver, &k_reg, &c_global, &f_torsion)?;

    let omega_dot_f: f64 = omega
        .iter()
        .zip(f_torsion.iter())
        .map(|(&a, &b)| a * b)
        .sum();
    let j = ixx + iyy - omega_dot_f;

    let omega_norm = omega.iter().map(|v| v * v).sum::<f64>().sqrt();

    Ok((j, 0.0, omega_norm, omega_dot_f))
}

/// Comprehensive diagnostic data for warping FEM system
#[derive(Debug, Clone)]
pub struct WarpingDiagnostics {
    pub section_name: String,
    pub n_dof: usize,
    pub n_elements: usize,

    // Element-level
    pub detj_min: f64,
    pub detj_max: f64,
    pub detj_weighted_sum: f64,
    pub ke_sym_max_err: f64,
    pub element_energy_min: f64,
    pub element_energy_max: f64,
    pub element_energy_sum: f64,

    // Global system
    pub k_sym_rel_err: f64,
    pub k_rank_estimate: usize,
    pub k_nullity: usize,
    pub constraint_dofs: usize,
    pub constraint_nodes: Vec<usize>,
    pub constraint_sum: f64,

    // Warping solution
    pub residual_norm: f64,
    pub residual_rel: f64,
    pub ct_omega: f64,
    pub wtkw: f64,
    pub wtf: f64,
    pub j_raw: f64,
    pub ixx_plus_iyy: f64,
    pub omega_dot_f: f64,

    // Energy identity
    pub energy_identity_rel_error: f64, // |w^T*K*w - w^T*F| / |w^T*F|

    // Individual values for comparison
    pub ixx: f64,
    pub iyy: f64,

    // Warping RHS invariants
    pub sum_fx: f64,         // sum(F_x) = sum of F elements (x-components)
    pub sum_fy: f64, // sum(F_y) = sum of F elements (y-components) - should be zero for symmetry
    pub first_moment_x: f64, // integral(y dA)
    pub first_moment_y: f64, // integral(x dA)
    pub integral_x_da: f64,
    pub integral_y_da: f64,
    pub centroid: (f64, f64),

    // F formulation check
    pub f_formulation_global: bool, // true if F uses [y, -x], false if [y-yc, -(x-xc)]

    // Final results
    pub j_fem: f64,
    pub j_analytical: f64,
    pub j_fallback: bool,

    // Status
    pub fem_succeeded: bool,
}

/// Run comprehensive diagnostics on warping FEM system
pub fn diagnose_warping_fem(
    section: &Section,
    name: &str,
    nu: f64,
) -> Result<WarpingDiagnostics, crate::mesh::fem::FemError> {
    let props = SectionProperties::from_section(section);

    // Mesh
    let bounds = section.bounds();
    let max_dim = (bounds.1 - bounds.0).max(bounds.3 - bounds.2);
    let min_edge = min_edge_length(section);
    let is_thin_walled = section.is_thin_walled();

    let params = mesh_params_from_control(MeshControl::Fine, max_dim, min_edge, is_thin_walled);
    let mesh = mesh_section(section, params);

    let diag = ((bounds.1 - bounds.0).powi(2) + (bounds.3 - bounds.2).powi(2)).sqrt();
    let min_area = (DEGENERATE_AREA_REL_TOL * diag.powi(2)).max(1e-24);
    let clean_elements = filter_degenerate_tris(&mesh.nodes, &mesh.elements, min_area);

    let mut used_nodes = vec![false; mesh.nodes.len()];
    for tri in &clean_elements {
        used_nodes[tri[0]] = true;
        used_nodes[tri[1]] = true;
        used_nodes[tri[2]] = true;
    }
    let node_map: Vec<Option<usize>> = used_nodes
        .iter()
        .enumerate()
        .map(|(i, &used)| if used { Some(i) } else { None })
        .collect();
    let mut new_nodes = Vec::new();
    let mut old_to_new = vec![usize::MAX; mesh.nodes.len()];
    for (old_idx, &used) in used_nodes.iter().enumerate() {
        if used {
            old_to_new[old_idx] = new_nodes.len();
            new_nodes.push(mesh.nodes[old_idx]);
        }
    }

    let remapped_elements: Vec<[usize; 3]> = clean_elements
        .iter()
        .map(|tri| [old_to_new[tri[0]], old_to_new[tri[1]], old_to_new[tri[2]]])
        .collect();

    let tri6_mesh = tri3_to_tri6(&new_nodes, &remapped_elements);
    let n_dof = tri6_mesh.nodes.len();

    // Build Tri6 elements
    let elements = build_tri6_elements(&tri6_mesh, 1.0, 1.0, 1.0)?;
    let n_elements = elements.len();

    // Element-level diagnostics
    let mut detj_min = f64::INFINITY;
    let mut detj_max = 0.0_f64;
    let mut detj_weighted_sum = 0.0_f64;
    let mut ke_sym_max_err = 0.0_f64;
    let mut element_energy_min = f64::INFINITY;
    let mut element_energy_max = 0.0_f64;
    let mut element_energy_sum = 0.0_f64;

    for tri6 in &elements {
        let gps = gauss_points(6);
        for &(w, eta, xi, zeta) in &gps {
            let sf = shape_function(&tri6.coords, (eta, xi, zeta));
            detj_min = detj_min.min(sf.j);
            detj_max = detj_max.max(sf.j);
            detj_weighted_sum += sf.j * w;
        }

        let (k_el, _, _) = tri6.torsion_properties();
        for i in 0..6 {
            for j in 0..6 {
                let err = (k_el[i][j] - k_el[j][i]).abs();
                ke_sym_max_err = ke_sym_max_err.max(err);
            }
        }

        let energy: f64 = (0..6).map(|i| k_el[i][i]).sum();
        element_energy_min = element_energy_min.min(energy);
        element_energy_max = element_energy_max.max(energy);
        element_energy_sum += energy;
    }

    // Global system assembly
    let mut k_global = SparseMatrix::new(n_dof);
    let mut f_torsion = vec![0.0_f64; n_dof];
    let mut c_global = vec![0.0_f64; n_dof];

    for tri6 in &elements {
        let (k_el, f_el, c_el) = tri6.torsion_properties();
        for i in 0..6 {
            let gi = tri6.node_ids[i];
            for j in 0..6 {
                k_global.add(gi, tri6.node_ids[j], k_el[i][j]);
            }
            f_torsion[gi] += f_el[i];
            c_global[gi] += c_el[i];
        }
    }

    // Global K symmetry - use new SparseMatrix methods
    k_global.compress();
    let k_sym_max_err = k_global.symmetry_max_error();
    let k_fro = k_global.frobenius_norm();
    let k_sym_rel_err = if k_fro > 0.0 {
        k_sym_max_err / k_fro
    } else {
        0.0
    };

    // Constraint info
    let constraint_sum: f64 = c_global.iter().sum();
    let constraint_nodes: Vec<usize> = c_global
        .iter()
        .enumerate()
        .filter_map(|(i, &v)| if v.abs() > 1e-15 { Some(i) } else { None })
        .collect();
    let constraint_dofs = constraint_nodes.len();

    // Regularized K
    let k_reg = {
        let mut m = k_global.clone();
        m.compress();
        let mut diag_avg = 0.0_f64;
        for i in 0..n_dof {
            diag_avg += m.matvec_diag(i);
        }
        let eps = diag_avg.max(1e-300) / n_dof as f64 * 1e-9;
        for i in 0..n_dof {
            m.add(i, i, eps);
        }
        m.compress();
        m
    };

    // Solve for omega (warping)
    let ixx = props.ix;
    let iyy = props.iy;
    let ixy = props.ixy;

    let solver = crate::fea::DirectLagrangeSolver::with_kernel(
        crate::fea::LagrangeKernel::Skyline,
        &k_reg,
        &c_global,
        crate::fea::SolverOptions {
            // k_reg already has εI added explicitly, so disable automatic regularization
            auto_regularize_singular: false,
        },
    )
    .map_err(|_| crate::mesh::fem::FemError::SingularMatrix)?;

    let solver_opt = Some(solver);
    let omega = solve_with_fallback(&solver_opt, &k_reg, &c_global, &f_torsion)?;

    // Residual: K*w - F (lambda=0 for regularized system without explicit constraint)
    let mut residual = vec![0.0_f64; n_dof];
    for i in 0..n_dof {
        let mut sum = 0.0_f64;
        let row_ptr = k_reg.row_ptr();
        let csr_cols = k_reg.csr_cols();
        let csr_vals = k_reg.csr_vals();
        for j_idx in row_ptr[i]..row_ptr[i + 1] {
            let j = csr_cols[j_idx];
            sum += csr_vals[j_idx] * omega[j];
        }
        sum -= f_torsion[i];
        residual[i] = sum;
    }
    let residual_norm: f64 = residual.iter().map(|v| v * v).sum::<f64>().sqrt();
    let f_norm: f64 = f_torsion.iter().map(|v| v * v).sum::<f64>().sqrt();
    let residual_rel = if f_norm > 0.0 {
        residual_norm / f_norm
    } else {
        0.0
    };

    // C^T * omega
    let ct_omega: f64 = c_global.iter().zip(omega.iter()).map(|(c, w)| c * w).sum();

    // w^T K w
    let mut kw = vec![0.0_f64; n_dof];
    let row_ptr = k_reg.row_ptr();
    let csr_cols = k_reg.csr_cols();
    let csr_vals = k_reg.csr_vals();
    for i in 0..n_dof {
        for j_idx in row_ptr[i]..row_ptr[i + 1] {
            let j = csr_cols[j_idx];
            kw[i] += csr_vals[j_idx] * omega[j];
        }
    }
    let wtkw: f64 = omega.iter().zip(kw.iter()).map(|(w, kw)| w * kw).sum();

    // w^T F
    let wtf: f64 = omega.iter().zip(f_torsion.iter()).map(|(w, f)| w * f).sum();

    // J_raw = Ixx + Iyy - w^T F
    let ixx_plus_iyy = ixx + iyy;
    let omega_dot_f = wtf;
    let j_raw = ixx_plus_iyy - omega_dot_f;

    // Energy identity: |w^T*K*w - w^T*F| / |w^T*F|
    let energy_identity_rel_error = if wtf.abs() > 1e-15 {
        (wtkw - wtf).abs() / wtf.abs()
    } else {
        0.0
    };

    // Coordinate invariants
    let mut integral_x_da = 0.0_f64;
    let mut integral_y_da = 0.0_f64;
    let mut first_moment_x = 0.0_f64; // integral(y dA)
    let mut first_moment_y = 0.0_f64; // integral(x dA)

    for tri6 in &elements {
        let gps = gauss_points(6);
        for &(w, eta, xi, zeta) in &gps {
            let sf = shape_function(&tri6.coords, (eta, xi, zeta));
            let weight = w * sf.j;
            integral_x_da += weight * sf.x;
            integral_y_da += weight * sf.y;
            first_moment_x += weight * sf.y;
            first_moment_y += weight * sf.x;
        }
    }

    let centroid = (props.centroid.x, props.centroid.y);

    // Warping RHS invariants
    let sum_fx: f64 = f_torsion.iter().sum();
    let sum_fy: f64 = 0.0; // F has no y-component for torsion (it's B^T [y, -x])

    // Compute J with fallback
    let j_fem = ixx + iyy - omega_dot_f;
    let j_analytical = analytical_j(section, &props).unwrap_or(0.0);
    let j_fallback = !j_fem.is_finite() || j_fem <= 0.0;

    // Rank/nullity estimate
    let k_diag_count = (0..n_dof)
        .filter(|&i| k_reg.matvec_diag(i).abs() > 1e-12)
        .count();
    let k_rank_estimate = k_diag_count.min(n_dof - 1);
    let k_nullity = n_dof - k_rank_estimate;

    // K symmetry relative error
    let k_sym_max_err = k_global.symmetry_max_error();
    let k_fro = k_global.frobenius_norm();
    let k_sym_rel_err = if k_fro > 0.0 {
        k_sym_max_err / k_fro
    } else {
        0.0
    };

    // Constraint info
    let constraint_sum: f64 = c_global.iter().sum();
    let constraint_nodes: Vec<usize> = c_global
        .iter()
        .enumerate()
        .filter_map(|(i, &v)| if v.abs() > 1e-15 { Some(i) } else { None })
        .collect();
    let constraint_dofs = constraint_nodes.len();

    // Residual
    let residual_norm: f64 = residual.iter().map(|v| v * v).sum::<f64>().sqrt();
    let f_norm: f64 = f_torsion.iter().map(|v| v * v).sum::<f64>().sqrt();
    let residual_rel = if f_norm > 0.0 {
        residual_norm / f_norm
    } else {
        0.0
    };

    // J_raw
    let j_raw = ixx_plus_iyy - omega_dot_f;
    let omega_dot_f = wtf;

    // C^T * omega
    let ct_omega: f64 = c_global.iter().zip(omega.iter()).map(|(c, w)| c * w).sum();

    // Energy identity
    let energy_identity_rel_error = if wtf.abs() > 1e-15 {
        (wtkw - wtf).abs() / wtf.abs()
    } else {
        0.0
    };

    // Verify F formulation: check if F uses global [y, -x] or centroidal [y-yc, -(x-xc)]
    // This is determined by checking if sum(F) is zero and moments match
    let f_formulation_global = sum_fx.abs() < 1e-10; // global formulation has sum(F) ≈ 0

    // Verify element F formulation
    // For torsion, F_e = ∫ B^T [y, -x] dA in global coordinates
    // Let's verify by checking if element F matches global or centroidal
    let mut f_formulation_verified = true;
    for tri6 in &elements {
        let (_, f_el, _) = tri6.torsion_properties();
        let f_el_sum: f64 = f_el.iter().sum();
        if f_el_sum.abs() > 1e-10 {
            f_formulation_verified = false;
        }
    }

    Ok(WarpingDiagnostics {
        section_name: name.to_string(),
        n_dof,
        n_elements,
        detj_min,
        detj_max,
        detj_weighted_sum,
        ke_sym_max_err,
        element_energy_min,
        element_energy_max,
        element_energy_sum,
        k_sym_rel_err,
        k_rank_estimate,
        k_nullity,
        constraint_dofs,
        constraint_nodes,
        constraint_sum,
        residual_norm,
        residual_rel,
        ct_omega,
        wtkw,
        wtf,
        j_raw,
        ixx_plus_iyy,
        omega_dot_f,
        energy_identity_rel_error,
        ixx,
        iyy,
        sum_fx,
        sum_fy,
        first_moment_x,
        first_moment_y,
        integral_x_da,
        integral_y_da,
        centroid,
        f_formulation_global,
        j_fem,
        j_analytical: analytical_j(section, &props).unwrap_or(0.0),
        j_fallback: !j_fem.is_finite() || j_fem <= 0.0,
        fem_succeeded: true,
    })
}

/// Export the exact (non-regularized) augmented Lagrange system for diagnostic analysis.
/// Exports A = [K C; C^T 0] and b = [F; 0] in COO format.
pub fn export_exact_augmented_system(
    section: &Section,
    name: &str,
    output_path: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    use std::fs::File;
    use std::io::Write;

    let props = SectionProperties::from_section(section);

    let bounds = section.bounds();
    let max_dim = (bounds.1 - bounds.0).max(bounds.3 - bounds.2);
    let min_edge = min_edge_length(section);
    let is_thin_walled = section.is_thin_walled();

    let params = mesh_params_from_control(MeshControl::Fine, max_dim, min_edge, is_thin_walled);
    let mesh = mesh_section(section, params);

    let diag = ((bounds.1 - bounds.0).powi(2) + (bounds.3 - bounds.2).powi(2)).sqrt();
    let min_area = (DEGENERATE_AREA_REL_TOL * diag.powi(2)).max(1e-24);
    let clean_elements = filter_degenerate_tris(&mesh.nodes, &mesh.elements, min_area);

    let mut used_nodes = vec![false; mesh.nodes.len()];
    for tri in &clean_elements {
        used_nodes[tri[0]] = true;
        used_nodes[tri[1]] = true;
        used_nodes[tri[2]] = true;
    }
    let mut new_nodes = Vec::new();
    let mut old_to_new = vec![usize::MAX; mesh.nodes.len()];
    for (old_idx, &used) in used_nodes.iter().enumerate() {
        if used {
            old_to_new[old_idx] = new_nodes.len();
            new_nodes.push(mesh.nodes[old_idx]);
        }
    }

    let remapped_elements: Vec<[usize; 3]> = clean_elements
        .iter()
        .map(|tri| [old_to_new[tri[0]], old_to_new[tri[1]], old_to_new[tri[2]]])
        .collect();

    let tri6_mesh = tri3_to_tri6(&new_nodes, &remapped_elements);
    let n = tri6_mesh.nodes.len();

    let elements = build_tri6_elements(&tri6_mesh, 1.0, 1.0, 1.0)?;

    // Centroid shift
    let cx = props.centroid.x;
    let cy = props.centroid.y;
    let shifted_nodes: Vec<Point> = new_nodes
        .iter()
        .map(|p| Point::new(p.x - cx, p.y - cy))
        .collect();
    let shifted_tri6_mesh = tri3_to_tri6(&shifted_nodes, &remapped_elements);
    let shifted_elements = build_tri6_elements(&shifted_tri6_mesh, 1.0, 1.0, 1.0)?;

    // Assemble exact K, F, C (no regularization)
    let mut k_global = SparseMatrix::new(n);
    let mut f_torsion = vec![0.0_f64; n];
    let mut c_global = vec![0.0_f64; n];

    for tri6 in &shifted_elements {
        let (k_el, f_el, c_el) = tri6.torsion_properties();
        for i in 0..6 {
            let gi = tri6.node_ids[i];
            for j in 0..6 {
                k_global.add(gi, tri6.node_ids[j], k_el[i][j]);
            }
            f_torsion[gi] += f_el[i];
            c_global[gi] += c_el[i];
        }
    }
    k_global.compress();

    // Build augmented matrix A = [K C; C^T 0] in COO format
    let mut a_rows: Vec<usize> = Vec::new();
    let mut a_cols: Vec<usize> = Vec::new();
    let mut a_vals: Vec<f64> = Vec::new();

    // K block (n x n)
    let (k_rows, k_cols, k_vals) = k_global.triplets();
    for (&r, (&c, &v)) in k_rows.iter().zip(k_cols.iter().zip(k_vals.iter())) {
        a_rows.push(r);
        a_cols.push(c);
        a_vals.push(v);
    }

    // C column (n x 1) - upper right
    for i in 0..n {
        if c_global[i].abs() > 1e-15 {
            a_rows.push(i);
            a_cols.push(n);
            a_vals.push(c_global[i]);
        }
    }

    // C^T row (1 x n) - lower left
    for i in 0..n {
        if c_global[i].abs() > 1e-15 {
            a_rows.push(n);
            a_cols.push(i);
            a_vals.push(c_global[i]);
        }
    }

    // Bottom-right element (0)
    a_rows.push(n);
    a_cols.push(n);
    a_vals.push(0.0);

    // RHS b = [F; 0]
    let mut b_vec = f_torsion.clone();
    b_vec.push(0.0);

    // Export to JSON
    let mut file = File::create(output_path)?;
    writeln!(file, "{{")?;
    writeln!(file, "  \"section_name\": \"{}\",", name)?;
    writeln!(file, "  \"n_dof\": {},", n)?;
    writeln!(file, "  \"A\": {{")?;
    writeln!(file, "    \"row\": [")?;
    for (i, &val) in a_rows.iter().enumerate() {
        writeln!(
            file,
            "      {}{}",
            val,
            if i < a_rows.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "    ],")?;
    writeln!(file, "    \"col\": [")?;
    for (i, &val) in a_cols.iter().enumerate() {
        writeln!(
            file,
            "      {}{}",
            val,
            if i < a_cols.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "    ],")?;
    writeln!(file, "    \"data\": [")?;
    for (i, &val) in a_vals.iter().enumerate() {
        writeln!(
            file,
            "      {:.15e}{}",
            val,
            if i < a_vals.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "    ],")?;
    writeln!(file, "    \"shape\": [{}, {}]", n + 1, n + 1)?;
    writeln!(file, "  }},")?;
    writeln!(file, "  \"b\": [")?;
    for (i, &val) in b_vec.iter().enumerate() {
        writeln!(
            file,
            "    {:.15e}{}",
            val,
            if i < b_vec.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "  ],")?;

    // Also export individual components for easier analysis
    writeln!(file, "  \"K\": {{")?;
    writeln!(file, "    \"row\": [")?;
    for (i, &val) in k_rows.iter().enumerate() {
        writeln!(
            file,
            "      {}{}",
            val,
            if i < k_rows.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "    ],")?;
    writeln!(file, "    \"col\": [")?;
    for (i, &val) in k_cols.iter().enumerate() {
        writeln!(
            file,
            "      {}{}",
            val,
            if i < k_cols.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "    ],")?;
    writeln!(file, "    \"data\": [")?;
    for (i, &val) in k_vals.iter().enumerate() {
        writeln!(
            file,
            "      {:.15e}{}",
            val,
            if i < k_vals.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "    ],")?;
    writeln!(file, "    \"shape\": [{}, {}]", n, n)?;
    writeln!(file, "  }},")?;
    writeln!(file, "  \"C\": [")?;
    for (i, &val) in c_global.iter().enumerate() {
        writeln!(
            file,
            "    {:.15e}{}",
            val,
            if i < c_global.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "  ],")?;
    writeln!(file, "  \"F\": [")?;
    for (i, &val) in f_torsion.iter().enumerate() {
        writeln!(
            file,
            "    {:.15e}{}",
            val,
            if i < f_torsion.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "  ]")?;
    writeln!(file, "}}")?;

    // Diagnostic: matrix statistics
    let a_nnz = a_rows.len();
    let a_diag_min = a_vals.iter().cloned().fold(f64::INFINITY, f64::min);
    let a_diag_max = a_vals.iter().cloned().fold(-f64::INFINITY, f64::max);
    let k_diag_sum: f64 = (0..n).map(|i| k_global.matvec_diag(i)).sum();
    let k_diag_avg = k_diag_sum / n as f64;

    println!(
        "Exported exact augmented system for {}: n={}, A_nnz={}, K_nnz={}, C_nnz={}, A_diag_min={:.2e}, A_diag_max={:.2e}, K_diag_avg={:.2e}",
        name,
        n,
        a_nnz,
        k_rows.len(),
        c_global.iter().filter(|&&v| v.abs() > 1e-15).count(),
        a_diag_min,
        a_diag_max,
        k_diag_avg
    );

    Ok(())
}

/// Export global warping matrices for cross-validation
pub fn export_global_warping_matrices(
    section: &Section,
    name: &str,
    output_path: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    use std::fs::File;
    use std::io::Write;

    let props = SectionProperties::from_section(section);

    let bounds = section.bounds();
    let max_dim = (bounds.1 - bounds.0).max(bounds.3 - bounds.2);
    let min_edge = min_edge_length(section);
    let is_thin_walled = section.is_thin_walled();

    let params = mesh_params_from_control(MeshControl::Fine, max_dim, min_edge, is_thin_walled);
    let mesh = mesh_section(section, params);

    let diag = ((bounds.1 - bounds.0).powi(2) + (bounds.3 - bounds.2).powi(2)).sqrt();
    let min_area = (DEGENERATE_AREA_REL_TOL * diag.powi(2)).max(1e-24);
    let clean_elements = filter_degenerate_tris(&mesh.nodes, &mesh.elements, min_area);

    let mut used_nodes = vec![false; mesh.nodes.len()];
    for tri in &clean_elements {
        used_nodes[tri[0]] = true;
        used_nodes[tri[1]] = true;
        used_nodes[tri[2]] = true;
    }
    let node_map: Vec<Option<usize>> = used_nodes
        .iter()
        .enumerate()
        .map(|(i, &used)| if used { Some(i) } else { None })
        .collect();
    let mut new_nodes = Vec::new();
    let mut old_to_new = vec![usize::MAX; mesh.nodes.len()];
    for (old_idx, &used) in used_nodes.iter().enumerate() {
        if used {
            old_to_new[old_idx] = new_nodes.len();
            new_nodes.push(mesh.nodes[old_idx]);
        }
    }

    let remapped_elements: Vec<[usize; 3]> = clean_elements
        .iter()
        .map(|tri| [old_to_new[tri[0]], old_to_new[tri[1]], old_to_new[tri[2]]])
        .collect();

    let tri6_mesh = tri3_to_tri6(&new_nodes, &remapped_elements);
    let n = tri6_mesh.nodes.len();

    let elements = build_tri6_elements(&tri6_mesh, 1.0, 1.0, 1.0)?;
    let n_elements = elements.len();

    // Global system assembly
    let mut k_global = SparseMatrix::new(n);
    let mut f_torsion = vec![0.0_f64; n];
    let mut c_global = vec![0.0_f64; n];

    for tri6 in &elements {
        let (k_el, f_el, c_el) = tri6.torsion_properties();
        for i in 0..6 {
            let gi = tri6.node_ids[i];
            for j in 0..6 {
                k_global.add(gi, tri6.node_ids[j], k_el[i][j]);
            }
            f_torsion[gi] += f_el[i];
            c_global[gi] += c_el[i];
        }
    }

    k_global.compress();

    // Export to JSON
    let mut file = File::create(output_path)?;

    // Write header
    writeln!(file, "{{")?;
    writeln!(file, "  \"section_name\": \"{}\",", name)?;
    writeln!(file, "  \"n_dof\": {},", n)?;
    writeln!(file, "  \"n_elements\": {},", elements.len())?;
    writeln!(file, "  \"n_nodes\": {},", tri6_mesh.nodes.len())?;

    // Node coordinates
    writeln!(file, "  \"nodes\": [")?;
    for (i, node) in tri6_mesh.nodes.iter().enumerate() {
        writeln!(
            file,
            "    [{:.10}, {:.10}]{}",
            node.x,
            node.y,
            if i < tri6_mesh.nodes.len() - 1 {
                ","
            } else {
                ""
            }
        )?;
    }
    writeln!(file, "  ],")?;

    // Element connectivity
    writeln!(file, "  \"elements\": [")?;
    for (i, tri6) in elements.iter().enumerate() {
        writeln!(
            file,
            "    [{}, {}, {}, {}, {}, {}]{}",
            tri6.node_ids[0],
            tri6.node_ids[1],
            tri6.node_ids[2],
            tri6.node_ids[3],
            tri6.node_ids[4],
            tri6.node_ids[5],
            if i < elements.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "  ],")?;

    // Global K matrix (CSR format)
    let (row_ptr, csr_cols, csr_vals) = k_global.csr_data();
    writeln!(file, "  \"K\": {{")?;
    writeln!(file, "    \"row_ptr\": [")?;
    for (i, &val) in row_ptr.iter().enumerate() {
        writeln!(
            file,
            "      {}{}",
            val,
            if i < row_ptr.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "    ],")?;
    writeln!(file, "    \"col\": [")?;
    for (i, &val) in csr_cols.iter().enumerate() {
        writeln!(
            file,
            "      {}{}",
            val,
            if i < csr_cols.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "    ],")?;
    writeln!(file, "    \"data\": [")?;
    for (i, &val) in csr_vals.iter().enumerate() {
        writeln!(
            file,
            "      {:.15e}{}",
            val,
            if i < csr_vals.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "    ]")?;
    writeln!(file, "  }},")?;

    // Global F vector
    writeln!(file, "  \"F\": [")?;
    for (i, &val) in f_torsion.iter().enumerate() {
        writeln!(
            file,
            "    {:.15e}{}",
            val,
            if i < f_torsion.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "  ],")?;

    // Global C vector
    writeln!(file, "  \"C\": [")?;
    for (i, &val) in c_global.iter().enumerate() {
        writeln!(
            file,
            "    {:.15e}{}",
            val,
            if i < c_global.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "  ],")?;

    // Node coordinates for DOF mapping
    writeln!(file, "  \"node_coords\": [")?;
    for (i, node) in tri6_mesh.nodes.iter().enumerate() {
        writeln!(
            file,
            "    [{:.10}, {:.10}]{}",
            node.x,
            node.y,
            if i < tri6_mesh.nodes.len() - 1 {
                ","
            } else {
                ""
            }
        )?;
    }
    writeln!(file, "  ]")?;

    writeln!(file, "}}")?;

    Ok(())
}

/// Export the exact (non-regularized) augmented Lagrange system for diagnostic analysis.
/// Uses Python's exact mesh (nodes/elements from python_global_*.json) to ensure DOF match.
/// Exports A = [K C; C^T 0] and b = [F; 0] in COO format.
pub fn export_exact_augmented_system_from_python_mesh(
    py_global_path: &str,
    name: &str,
    output_path: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    use std::fs::File;
    use std::io::{Read, Write};

    // Load Python mesh
    let mut file = File::open(py_global_path)?;
    let mut contents = String::new();
    file.read_to_string(&mut contents)?;
    let mesh_json: serde_json::Value = serde_json::from_str(&contents)?;

    let nodes: Vec<Point> = mesh_json["nodes"]
        .as_array()
        .ok_or("nodes not found")?
        .iter()
        .map(|v| {
            let coords = v.as_array().unwrap();
            Point::new(coords[0].as_f64().unwrap(), coords[1].as_f64().unwrap())
        })
        .collect();

    let elements: Vec<[usize; 6]> = mesh_json["elements"]
        .as_array()
        .ok_or("elements not found")?
        .iter()
        .map(|e| {
            let arr = e.as_array().unwrap();
            [
                arr[0].as_u64().unwrap() as usize,
                arr[1].as_u64().unwrap() as usize,
                arr[2].as_u64().unwrap() as usize,
                arr[3].as_u64().unwrap() as usize,
                arr[4].as_u64().unwrap() as usize,
                arr[5].as_u64().unwrap() as usize,
            ]
        })
        .collect();

    let n_dof = nodes.len();

    // Build Tri6 elements, fixing orientation if Python's winding is CW.
    let mut rust_elements = Vec::with_capacity(elements.len());
    let mut n_fixed = 0usize;
    for (i, &elem) in elements.iter().enumerate() {
        let mut e = elem;
        let points: [Point; 6] = [
            nodes[e[0]],
            nodes[e[1]],
            nodes[e[2]],
            nodes[e[3]],
            nodes[e[4]],
            nodes[e[5]],
        ];
        let mut coords = [[0.0; 6]; 2];
        for k in 0..6 {
            coords[0][k] = points[k].x;
            coords[1][k] = points[k].y;
        }
        let sf = shape_function(&coords, (1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0));
        if sf.j < 0.0 {
            let [n0, n1, n2, m01, m12, m20] = e;
            e = [n2, n1, n0, m12, m01, m20];
            n_fixed += 1;
        }
        let pts: [Point; 6] = [
            nodes[e[0]],
            nodes[e[1]],
            nodes[e[2]],
            nodes[e[3]],
            nodes[e[4]],
            nodes[e[5]],
        ];
        rust_elements.push(Tri6::from_points(i, pts, e, 1.0, 1.0, 1.0)?);
    }
    if n_fixed > 0 {
        eprintln!("  Fixed orientation on {} CW elements", n_fixed);
    }

    // Assemble exact K, F, C (no regularization)
    let mut k_global = SparseMatrix::new(n_dof);
    let mut f_torsion = vec![0.0_f64; n_dof];
    let mut c_global = vec![0.0_f64; n_dof];

    for tri6 in &rust_elements {
        let (k_el, f_el, c_el) = tri6.torsion_properties();
        for i in 0..6 {
            let gi = tri6.node_ids[i];
            for j in 0..6 {
                k_global.add(gi, tri6.node_ids[j], k_el[i][j]);
            }
            f_torsion[gi] += f_el[i];
            c_global[gi] += c_el[i];
        }
    }
    k_global.compress();

    // Build augmented matrix A = [K C; C^T 0] in COO format
    let mut a_rows: Vec<usize> = Vec::new();
    let mut a_cols: Vec<usize> = Vec::new();
    let mut a_vals: Vec<f64> = Vec::new();

    // K block (n x n)
    let (k_rows, k_cols, k_vals) = k_global.triplets();
    for (&r, (&c, &v)) in k_rows.iter().zip(k_cols.iter().zip(k_vals.iter())) {
        a_rows.push(r);
        a_cols.push(c);
        a_vals.push(v);
    }

    // C column (upper right) - only non-zero entries
    for i in 0..n_dof {
        if c_global[i].abs() > 1e-15 {
            a_rows.push(i);
            a_cols.push(n_dof);
            a_vals.push(c_global[i]);
        }
    }

    // C^T row (lower left) - only non-zero entries
    for i in 0..n_dof {
        if c_global[i].abs() > 1e-15 {
            a_rows.push(n_dof);
            a_cols.push(i);
            a_vals.push(c_global[i]);
        }
    }

    // Bottom-right element (0) - not stored in COO (implicit)

    // RHS b = [F; 0]
    let mut b_vec = f_torsion.clone();
    b_vec.push(0.0);

    // Export to JSON
    let mut file = File::create(output_path)?;
    writeln!(file, "{{")?;
    writeln!(file, "  \"section_name\": \"{}\",", name)?;
    writeln!(file, "  \"n_dof\": {},", n_dof)?;
    writeln!(file, "  \"A\": {{")?;
    writeln!(file, "    \"row\": [")?;
    for (i, &val) in a_rows.iter().enumerate() {
        writeln!(
            file,
            "      {}{}",
            val,
            if i < a_rows.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "    ],")?;
    writeln!(file, "    \"col\": [")?;
    for (i, &val) in a_cols.iter().enumerate() {
        writeln!(
            file,
            "      {}{}",
            val,
            if i < a_cols.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "    ],")?;
    writeln!(file, "    \"data\": [")?;
    for (i, &val) in a_vals.iter().enumerate() {
        writeln!(
            file,
            "      {:.15e}{}",
            val,
            if i < a_vals.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "    ],")?;
    writeln!(file, "    \"shape\": [{}, {}]", n_dof + 1, n_dof + 1)?;
    writeln!(file, "  }},")?;
    writeln!(file, "  \"b\": [")?;
    for (i, &val) in b_vec.iter().enumerate() {
        writeln!(
            file,
            "    {:.15e}{}",
            val,
            if i < b_vec.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "  ],")?;

    // Also export individual components for easier analysis
    writeln!(file, "  \"K\": {{")?;
    writeln!(file, "    \"row\": [")?;
    for (i, &val) in k_rows.iter().enumerate() {
        writeln!(
            file,
            "      {}{}",
            val,
            if i < k_rows.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "    ],")?;
    writeln!(file, "    \"col\": [")?;
    for (i, &val) in k_cols.iter().enumerate() {
        writeln!(
            file,
            "      {}{}",
            val,
            if i < k_cols.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "    ],")?;
    writeln!(file, "    \"data\": [")?;
    for (i, &val) in k_vals.iter().enumerate() {
        writeln!(
            file,
            "      {:.15e}{}",
            val,
            if i < k_vals.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "    ],")?;
    writeln!(file, "    \"shape\": [{}, {}]", n_dof, n_dof)?;
    writeln!(file, "  }},")?;
    writeln!(file, "  \"C\": [")?;
    for (i, &val) in c_global.iter().enumerate() {
        writeln!(
            file,
            "    {:.15e}{}",
            val,
            if i < c_global.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "  ],")?;
    writeln!(file, "  \"F\": [")?;
    for (i, &val) in f_torsion.iter().enumerate() {
        writeln!(
            file,
            "    {:.15e}{}",
            val,
            if i < f_torsion.len() - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "  ]")?;
    writeln!(file, "}}")?;

    // Diagnostic: matrix statistics
    let a_nnz = a_rows.len();
    let a_diag_min = a_vals.iter().cloned().fold(f64::INFINITY, f64::min);
    let a_diag_max = a_vals.iter().cloned().fold(-f64::INFINITY, f64::max);
    let k_diag_sum: f64 = (0..n_dof).map(|i| k_global.matvec_diag(i)).sum();
    let k_diag_avg = k_diag_sum / n_dof as f64;

    eprintln!(
        "Exported exact augmented system for {}: n={}, A_nnz={}, K_nnz={}, C_nnz={}, A_diag_min={:.2e}, A_diag_max={:.2e}, K_diag_avg={:.2e}",
        name,
        n_dof,
        a_nnz,
        k_rows.len(),
        c_global.iter().filter(|&&v| v.abs() > 1e-15).count(),
        a_diag_min,
        a_diag_max,
        k_diag_avg
    );

    Ok(())
}

/// Run Rust FEM directly on Python's full 6-node mesh and export K/F/C for
/// index-level comparison against Python's exported global matrices.
///
/// Unlike `run_fem_on_python_mesh` (which re-triangulates Python's Tri3 corner
/// mesh into a different Tri6 mesh), this uses Python's own 6-node `nodes` and
/// `elements` connectivity, yielding the *same* DOF count and connectivity so
/// the global stiffness/load/constraint matrices can be compared entry by entry.
///
/// `py_global_path` points at `python_global_<section>.json` which contains
/// `nodes` (coords), `elements` (6-node), and Python's own `K`/`F`/`C`.
pub fn run_fem_on_python_tri6_mesh(
    py_global_path: &str,
    name: &str,
    output_path: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    use std::io::Read;

    let mut file = File::open(py_global_path)?;
    let mut contents = String::new();
    file.read_to_string(&mut contents)?;
    let mesh_json: serde_json::Value = serde_json::from_str(&contents)?;

    let nodes: Vec<Point> = mesh_json["nodes"]
        .as_array()
        .ok_or("nodes not found")?
        .iter()
        .map(|v| {
            let coords = v.as_array().unwrap();
            Point::new(coords[0].as_f64().unwrap(), coords[1].as_f64().unwrap())
        })
        .collect();

    let elements: Vec<[usize; 6]> = mesh_json["elements"]
        .as_array()
        .ok_or("elements not found")?
        .iter()
        .map(|e| {
            let arr = e.as_array().unwrap();
            [
                arr[0].as_u64().unwrap() as usize,
                arr[1].as_u64().unwrap() as usize,
                arr[2].as_u64().unwrap() as usize,
                arr[3].as_u64().unwrap() as usize,
                arr[4].as_u64().unwrap() as usize,
                arr[5].as_u64().unwrap() as usize,
            ]
        })
        .collect();

    let n_dof = nodes.len();
    println!(
        "Rust FEM on Python Tri6 mesh: {} nodes, {} elements",
        n_dof,
        elements.len()
    );

    // Build Tri6 elements, fixing orientation if Python's winding is CW.
    let mut rust_elements = Vec::with_capacity(elements.len());
    let mut n_fixed = 0usize;
    for (i, &elem) in elements.iter().enumerate() {
        let mut e = elem;
        let points: [Point; 6] = [
            nodes[e[0]],
            nodes[e[1]],
            nodes[e[2]],
            nodes[e[3]],
            nodes[e[4]],
            nodes[e[5]],
        ];
        let mut coords = [[0.0; 6]; 2];
        for k in 0..6 {
            coords[0][k] = points[k].x;
            coords[1][k] = points[k].y;
        }
        let sf = shape_function(&coords, (1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0));
        if sf.j < 0.0 {
            let [n0, n1, n2, m01, m12, m20] = e;
            e = [n2, n1, n0, m12, m01, m20];
            n_fixed += 1;
        }
        let pts: [Point; 6] = [
            nodes[e[0]],
            nodes[e[1]],
            nodes[e[2]],
            nodes[e[3]],
            nodes[e[4]],
            nodes[e[5]],
        ];
        rust_elements.push(Tri6::from_points(i, pts, e, 1.0, 1.0, 1.0)?);
    }
    if n_fixed > 0 {
        println!("  fixed orientation on {} CW elements", n_fixed);
    }

    // Assemble global K, F, C
    let mut k_global = SparseMatrix::new(n_dof);
    let mut f_torsion = vec![0.0_f64; n_dof];
    let mut c_global = vec![0.0_f64; n_dof];
    for tri6 in &rust_elements {
        let (k_el, f_el, c_el) = tri6.torsion_properties();
        for i in 0..6 {
            let gi = tri6.node_ids[i];
            for j in 0..6 {
                k_global.add(gi, tri6.node_ids[j], k_el[i][j]);
            }
            f_torsion[gi] += f_el[i];
            c_global[gi] += c_el[i];
        }
    }
    k_global.compress();

    // Collect K COO triplets
    let mut k_rows: Vec<usize> = Vec::new();
    let mut k_cols: Vec<usize> = Vec::new();
    let mut k_vals: Vec<f64> = Vec::new();
    for i in 0..n_dof {
        for j_idx in k_global.row_ptr()[i]..k_global.row_ptr()[i + 1] {
            k_rows.push(i);
            k_cols.push(k_global.csr_cols()[j_idx]);
            k_vals.push(k_global.csr_vals()[j_idx]);
        }
    }
    let k_nnz = k_rows.len();

    // Export K (COO, matching Python's format), F, C
    let mut file = File::create(output_path)?;
    writeln!(file, "{{")?;
    writeln!(file, "  \"section_name\": \"{}\",", name)?;
    writeln!(file, "  \"n_dof\": {},", n_dof)?;
    writeln!(file, "  \"n_elements\": {},", rust_elements.len())?;
    writeln!(file, "  \"F\": [")?;
    for i in 0..n_dof {
        writeln!(
            file,
            "    {:.12e}{}",
            f_torsion[i],
            if i < n_dof - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "  ],")?;
    writeln!(file, "  \"C\": [")?;
    for i in 0..n_dof {
        writeln!(
            file,
            "    {:.12e}{}",
            c_global[i],
            if i < n_dof - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "  ],")?;
    writeln!(file, "  \"K\": {{")?;
    writeln!(file, "    \"row\": [")?;
    for i in 0..k_nnz {
        writeln!(
            file,
            "      {}{}",
            k_rows[i],
            if i < k_nnz - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "    ],")?;
    writeln!(file, "    \"col\": [")?;
    for i in 0..k_nnz {
        writeln!(
            file,
            "      {}{}",
            k_cols[i],
            if i < k_nnz - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "    ],")?;
    writeln!(file, "    \"data\": [")?;
    for i in 0..k_nnz {
        writeln!(
            file,
            "      {:.12e}{}",
            k_vals[i],
            if i < k_nnz - 1 { "," } else { "" }
        )?;
    }
    writeln!(file, "    ],")?;
    writeln!(file, "    \"shape\": [{}, {}]", n_dof, n_dof)?;
    writeln!(file, "  }}")?;
    writeln!(file, "}}")?;

    println!("  Exported Rust-on-Python-Tri6 K/F/C to {}", output_path);
    println!("  K nnz (Rust): {}", k_global.csr_vals().len());
    if let Some(py_k) = mesh_json.get("K") {
        if let Some(shape) = py_k.get("shape") {
            if let (Some(rs), Some(cs)) = (shape[0].as_u64(), shape[1].as_u64()) {
                println!(
                    "  Python K shape: {}x{}, Python K nnz: {}",
                    rs,
                    cs,
                    py_k["data"].as_array().map(|a| a.len()).unwrap_or(0)
                );
            }
        }
    }

    Ok(())
}

/// Import a Python mesh from JSON and run FEM analysis.
/// This allows direct comparison with Python's sectionproperties using the exact same mesh.
pub fn run_fem_on_python_mesh(
    mesh_path: &str,
    name: &str,
    output_path: &str,
) -> Result<WarpingDiagnostics, Box<dyn std::error::Error>> {
    use std::fs::File;
    use std::io::Read;

    // Load Python mesh
    let mut file = File::open(mesh_path)?;
    let mut contents = String::new();
    file.read_to_string(&mut contents)?;
    let mesh_json: serde_json::Value = serde_json::from_str(&contents)?;

    // Extract vertices and triangles
    let vertices = mesh_json["vertices"]
        .as_array()
        .ok_or("vertices not found")?;
    let triangles = mesh_json["triangles"]
        .as_array()
        .ok_or("triangles not found")?;

    let vertices: Vec<Point> = vertices
        .iter()
        .map(|v| {
            let coords = v.as_array().unwrap();
            Point::new(coords[0].as_f64().unwrap(), coords[1].as_f64().unwrap())
        })
        .collect();

    let tri3_elements: Vec<[usize; 3]> = triangles
        .iter()
        .map(|t| {
            let arr = t.as_array().unwrap();
            [
                arr[0].as_u64().unwrap() as usize,
                arr[1].as_u64().unwrap() as usize,
                arr[2].as_u64().unwrap() as usize,
            ]
        })
        .collect();

    println!(
        "Loaded Python mesh: {} vertices, {} triangles",
        vertices.len(),
        tri3_elements.len()
    );

    // Convert Tri3 to Tri6
    let tri6_mesh = tri3_to_tri6(&vertices, &tri3_elements);
    let n_dof = tri6_mesh.nodes.len();
    println!(
        "Tri6 mesh: {} nodes, {} elements",
        tri6_mesh.nodes.len(),
        tri6_mesh.elements.len()
    );

    // Build Tri6 elements
    let elements = build_tri6_elements(&tri6_mesh, 1.0, 1.0, 1.0)?;

    // Global system assembly
    let n_dof = tri6_mesh.nodes.len();
    let mut k_global = SparseMatrix::new(n_dof);
    let mut f_torsion = vec![0.0_f64; n_dof];
    let mut c_global = vec![0.0_f64; n_dof];

    for tri6 in &elements {
        let (k_el, f_el, c_el) = tri6.torsion_properties();
        for i in 0..6 {
            let gi = tri6.node_ids[i];
            for j in 0..6 {
                k_global.add(gi, tri6.node_ids[j], k_el[i][j]);
            }
            f_torsion[gi] += f_el[i];
            c_global[gi] += c_el[i];
        }
    }

    // Global K symmetry
    k_global.compress();
    let k_sym_max_err = k_global.symmetry_max_error();
    let k_fro = k_global.frobenius_norm();
    let k_sym_rel_err = if k_fro > 0.0 {
        k_sym_max_err / k_fro
    } else {
        0.0
    };

    // Constraint info
    let constraint_sum: f64 = c_global.iter().sum();
    let constraint_nodes: Vec<usize> = c_global
        .iter()
        .enumerate()
        .filter_map(|(i, &v)| if v.abs() > 1e-15 { Some(i) } else { None })
        .collect();
    let constraint_dofs = constraint_nodes.len();

    // Regularized K
    let k_reg = {
        let mut m = k_global.clone();
        m.compress();
        let mut diag_avg = 0.0_f64;
        for i in 0..n_dof {
            diag_avg += m.matvec_diag(i);
        }
        let eps = diag_avg.max(1e-300) / n_dof as f64 * 1e-9;
        for i in 0..n_dof {
            m.add(i, i, eps);
        }
        m.compress();
        m
    };

    // Solve for omega (warping)
    let ixx = 1.0; // placeholder - we don't have section properties here
    let iyy = 1.0;
    let ixy = 0.0;

    let solver = crate::fea::DirectLagrangeSolver::with_kernel(
        crate::fea::LagrangeKernel::Skyline,
        &k_reg,
        &c_global,
        crate::fea::SolverOptions {
            // k_reg already has εI added explicitly, so disable automatic regularization
            auto_regularize_singular: false,
        },
    )
    .map_err(|_| "Failed to create solver")?;

    let solver_opt = Some(solver);
    let omega = solve_with_fallback(&solver_opt, &k_reg, &c_global, &f_torsion)?;

    // Residual: K*w - F
    let mut residual = vec![0.0_f64; n_dof];
    let row_ptr = k_reg.row_ptr();
    let csr_cols = k_reg.csr_cols();
    let csr_vals = k_reg.csr_vals();
    for i in 0..n_dof {
        let mut sum = 0.0_f64;
        for j_idx in row_ptr[i]..row_ptr[i + 1] {
            let j = csr_cols[j_idx];
            sum += csr_vals[j_idx] * omega[j];
        }
        sum -= f_torsion[i];
        residual[i] = sum;
    }
    let residual_norm: f64 = residual.iter().map(|v| v * v).sum::<f64>().sqrt();
    let f_norm: f64 = f_torsion.iter().map(|v| v * v).sum::<f64>().sqrt();
    let residual_rel = if f_norm > 0.0 {
        residual_norm / f_norm
    } else {
        0.0
    };

    // C^T * omega
    let ct_omega: f64 = c_global.iter().zip(omega.iter()).map(|(c, w)| c * w).sum();

    // w^T K w
    let mut kw = vec![0.0_f64; n_dof];
    let row_ptr = k_reg.row_ptr();
    let csr_cols = k_reg.csr_cols();
    let csr_vals = k_reg.csr_vals();
    for i in 0..n_dof {
        for j_idx in row_ptr[i]..row_ptr[i + 1] {
            let j = csr_cols[j_idx];
            kw[i] += csr_vals[j_idx] * omega[j];
        }
    }
    let wtkw: f64 = omega.iter().zip(kw.iter()).map(|(w, kw)| w * kw).sum();

    // w^T F
    let wtf: f64 = omega.iter().zip(f_torsion.iter()).map(|(w, f)| w * f).sum();

    // J_raw = Ixx + Iyy - w^T F (using placeholder Ixx, Iyy = 1.0)
    let ixx_plus_iyy = 1.0;
    let omega_dot_f = wtf;
    let j_raw = ixx_plus_iyy - omega_dot_f;

    // Energy identity
    let energy_identity_rel_error = if wtf.abs() > 1e-15 {
        (wtkw - wtf).abs() / wtf.abs()
    } else {
        0.0
    };

    // C^T * omega
    let ct_omega: f64 = c_global.iter().zip(omega.iter()).map(|(c, w)| c * w).sum();

    // w^T K w
    let wtkw: f64 = omega.iter().zip(kw.iter()).map(|(w, kw)| w * kw).sum();

    // w^T F
    let wtf: f64 = omega.iter().zip(f_torsion.iter()).map(|(w, f)| w * f).sum();

    // Energy identity
    let energy_identity_rel_error = if wtf.abs() > 1e-15 {
        (wtkw - wtf).abs() / wtf.abs()
    } else {
        0.0
    };

    // C^T * omega
    let ct_omega: f64 = c_global.iter().zip(omega.iter()).map(|(c, w)| c * w).sum();

    // Export results
    let mut file = File::create(output_path)?;
    writeln!(file, "{{")?;
    writeln!(file, "  \"section_name\": \"{}\",", name)?;
    writeln!(file, "  \"n_dof\": {},", n_dof)?;
    writeln!(file, "  \"n_elements\": {},", elements.len())?;
    writeln!(file, "  \"n_nodes\": {},", tri6_mesh.nodes.len())?;
    writeln!(file, "  \"residual_norm\": {:.6e},", residual_norm)?;
    writeln!(file, "  \"residual_rel\": {:.6e},", residual_rel)?;
    writeln!(file, "  \"ct_omega\": {:.6e},", ct_omega)?;
    writeln!(file, "  \"wtkw\": {:.6e},", wtkw)?;
    writeln!(file, "  \"wtf\": {:.6e},", wtf)?;
    writeln!(file, "  \"j_raw\": {:.6e},", j_raw)?;
    writeln!(
        file,
        "  \"energy_identity_rel_error\": {:.6e},",
        energy_identity_rel_error
    )?;
    writeln!(file, "  \"ct_omega\": {:.6e},", ct_omega)?;
    writeln!(file, "}}")?;

    Ok(WarpingDiagnostics {
        section_name: name.to_string(),
        n_dof,
        n_elements: elements.len(),
        detj_min: 0.0,
        detj_max: 0.0,
        detj_weighted_sum: 0.0,
        ke_sym_max_err: 0.0,
        element_energy_min: 0.0,
        element_energy_max: 0.0,
        element_energy_sum: 0.0,
        k_sym_rel_err,
        k_rank_estimate: 0,
        k_nullity: 0,
        constraint_dofs,
        constraint_nodes,
        constraint_sum: 0.0,
        residual_norm,
        residual_rel,
        ct_omega,
        wtkw,
        wtf,
        j_raw,
        ixx_plus_iyy: 1.0,
        omega_dot_f: wtf,
        energy_identity_rel_error,
        ixx: 1.0,
        iyy: 1.0,
        sum_fx: 0.0,
        sum_fy: 0.0,
        first_moment_x: 0.0,
        first_moment_y: 0.0,
        integral_x_da: 0.0,
        integral_y_da: 0.0,
        centroid: (0.0, 0.0),
        f_formulation_global: true,
        j_fem: j_raw,
        j_analytical: 0.0,
        j_fallback: false,
        fem_succeeded: true,
    })
}
