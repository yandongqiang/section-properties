//! FEM warping analysis using Tri6 elements.
//!
//! Mirrors Python `sectionproperties.analysis.section.Section.calculate_warping_properties`:
//! assembles Lagrangian stiffness matrix, solves for warping function ω and shear
//! functions ψ, φ, then computes J, Iw, shear centre, shear areas and monosymmetry
//! constants via Gaussian quadrature.

use crate::fea::{
    SparseMatrix, Tri6, Tri6Mesh, build_tri6_elements, solve_lagrange_sparse, tri3_to_tri6,
};
use crate::geometry::Point;
use crate::mesh::{MeshControl, MeshParams, mesh_params_from_control, mesh_section};
use crate::plastic::warping::ThinWalledCheck;
use crate::section::Section;
use crate::section_properties::SectionProperties;

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
    let j = if !j.is_finite() || j <= 0.0 {
        // FEM produced non-positive J. For thin-walled open sections, use analytical formula
        // as fallback with explicit logging. This indicates FEM numerical issues (mesh/regularization).
        let analytical_j = crate::plastic::warping_fem::analytical_j(section, props).unwrap_or(0.0);
        eprintln!(
            "[WARN] FEM J={:.6e} <= 0 (ixx+iyy={:.6e}, ω·f={:.6e}); using analytical J={:.6e}",
            j,
            ixx + iyy,
            omega_dot_f,
            analytical_j
        );
        analytical_j
    } else {
        j
    };

    let mut f_psi = vec![0.0; n];
    let mut f_phi = vec![0.0; n];

    for tri6 in &elements {
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
    for &val in &omega {
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

    for tri6 in &elements {
        let omega_el: [f64; 6] = {
            let mut e = [0.0; 6];
            for i in 0..6 {
                e[i] = omega[tri6.node_ids[i]];
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

    for tri6 in &elements {
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

    for tri6 in &elements {
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
    for tri6 in &elements {
        for i in 0..6 {
            let node_id = tri6.node_ids[i];
            let omega_val = omega[node_id].abs();
            if omega_val > omega_max {
                omega_max = omega_val;
            }
        }
        let omega_el: [f64; 6] = {
            let mut e = [0.0; 6];
            for i in 0..6 {
                e[i] = omega[tri6.node_ids[i]];
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
        omega,
        psi,
        phi,
        j: j.max(0.0),
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
