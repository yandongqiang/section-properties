//! FEM warping analysis using Tri6 elements.
//!
//! Mirrors Python `sectionproperties.analysis.section.Section.calculate_warping_properties`:
//! assembles Lagrangian stiffness matrix, solves for warping function ω and shear
//! functions ψ, φ, then computes J, Iw, shear centre, shear areas and monosymmetry
//! constants via Gaussian quadrature.

use crate::fea::{SparseMatrix, Tri6, Tri6Mesh, solve_lagrange_sparse, tri3_to_tri6, build_tri6_elements};
use crate::geometry::Point;
use crate::mesh::{MeshParams, mesh_section};
use crate::section::Section;
use crate::section_properties::SectionProperties;

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
}

/// Compute full FEM solution: mesh + warping function ω + shear functions ψ, φ.
/// Returns None if mesh generation or solving fails.
pub fn compute_fem_solution(section: &Section, props: &SectionProperties) -> Option<FemSolution> {
    let cx = props.centroid.x;
    let cy = props.centroid.y;
    let ixx = props.ix;
    let iyy = props.iy;
    let ixy = props.ixy;
    let _ea = props.area;
    let nu = 0.0;

    let bounds = section.bounds();
    let max_dim = (bounds.1 - bounds.0).max(bounds.3 - bounds.2);
    let _min_edge = min_edge_length(section);
    let _ = props;
    let target_size = (max_dim / 8.0).max(1e-4);

    // Bridged triangulation keeps hole boundaries as exact mesh edges.
    let base_mesh = mesh_section(
        section,
        MeshParams {
            target_size,
            max_size: target_size * 2.0,
            min_size: target_size * 0.3,
            quality_threshold: 0.3,
            use_delaunay: true,
            max_iterations: 10,
        },
    );
    if base_mesh.elements.is_empty() {
        return None;
    }

    // mesh_section already refines uniformly to target_size.
    let tri6_mesh = tri3_to_tri6(&base_mesh.nodes, &base_mesh.elements);
    let n = tri6_mesh.nodes.len();

    // Build elements with proper orientation (handled by build_tri6_elements)
    let elements = build_tri6_elements(&tri6_mesh, 1.0, 1.0, 1.0);

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

    // Python-style direct solve: assemble the augmented (N+1)x(N+1)
    // Lagrangian matrix in CSC format and factor the leading K block once.
    let k_reg = {
        let mut m = k_global.compressed();
        let mut diag_avg = 0.0;
        for i in 0..n {
            diag_avg += m.matvec_diag(i);
        }
        let eps = diag_avg.max(1e-300) / n as f64 * 1e-9;
        for i in 0..n {
            m.add(i, i, eps);
        }
        m.compressed()
    };
    let _k_lg = crate::fea::DirectLagrangeSolver::assemble_torsion_lagrange(&k_global, &c_global);
    let solver = match crate::fea::DirectLagrangeSolver::new(&k_reg, &c_global) {
        Ok(s) => Some(s),
        Err(_) => None,
    };

    let omega = solve_with_fallback(&solver, &k_reg, &c_global, &f_torsion);

    let omega_dot_f: f64 = omega
        .iter()
        .zip(f_torsion.iter())
        .map(|(&a, &b)| a * b)
        .sum();
    let j = ixx + iyy - omega_dot_f;

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

    let psi = solve_with_fallback(&solver, &k_reg, &c_global, &f_psi);
    let phi = solve_with_fallback(&solver, &k_reg, &c_global, &f_phi);

    let delta_s = 2.0 * (1.0 + nu) * (ixx * iyy - ixy * ixy);

    Some(FemSolution {
        tri6_mesh,
        elements,
        omega,
        psi,
        phi,
        j: j.max(0.0),
        delta_s,
        nu,
    })
}

/// Estimate shear areas using simple formulas (fallback when FEM is inaccurate).
pub fn estimate_shear_areas_fallback(section: &Section, props: &SectionProperties) -> (f64, f64) {
    let bounds = section.bounds();
    let h = bounds.3 - bounds.2;
    let b = bounds.1 - bounds.0;
    let area = props.area;

    let sym_x = is_symmetric_about_x(section);
    let sym_y = is_symmetric_about_y(section);

    if sym_x || (sym_x && sym_y) {
        let hw = h * 0.9;
        let tw = area / (2.0 * b + hw);
        let tf = tw;
        let ay = 2.0 * b * tf;
        let az = hw * tw;
        (ay.max(area * 0.1), az.max(area * 0.1))
    } else {
        (area * 0.85, area * 0.85)
    }
}

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
    pub a_sx: f64,
    pub a_sy: f64,
    pub a_sxy: f64,
    pub a_s11: f64,
    pub a_s22: f64,
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
) -> FemWarpingResult {
    let cx = props.centroid.x;
    let cy = props.centroid.y;
    let ixx = props.ix;
    let iyy = props.iy;
    let ixy = props.ixy;
    let ea = props.area;
    let nu = 0.0;

    let bounds = section.bounds();
    let max_dim = (bounds.1 - bounds.0).max(bounds.3 - bounds.2);
    let _min_edge = min_edge_length(section);
    let _ = props;
    let target_size = (max_dim / 8.0).max(1e-4);
    
    

    let mesh = mesh_section(
        section,
        MeshParams {
            target_size,
            max_size: target_size * 2.0,
            min_size: target_size * 0.3,
            quality_threshold: 0.3,
            use_delaunay: true,
            max_iterations: 10,
        },
    );


    let _t0 = std::time::Instant::now();

    if mesh.elements.is_empty() {
        return FemWarpingResult {
            j: ixx + iyy,
            iw: 0.0,
            shear_center: Point::new(0.0, 0.0),
            shear_center_elastic: Point::new(0.0, 0.0),
            beta_x_plus: 0.0,
            beta_y_plus: 0.0,
            a_sx: ea * 0.85,
            a_sy: ea * 0.85,
            a_sxy: 0.0,
            a_s11: ea * 0.85,
            a_s22: ea * 0.85,
        };
    }

    let tri6_mesh = tri3_to_tri6(&mesh.nodes, &mesh.elements);
    let n = tri6_mesh.nodes.len();

    // Build elements with proper orientation (handled by build_tri6_elements)
    let elements = build_tri6_elements(&tri6_mesh, 1.0, 1.0, 1.0);

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

    // Direct solver: factor once, solve omega/psi/phi.
    let k_reg = {
        let mut m = k_global.compressed();
        let mut diag_avg = 0.0;
        for i in 0..n {
            diag_avg += m.matvec_diag(i);
        }
        let eps = diag_avg.max(1e-300) / n as f64 * 1e-9;
        for i in 0..n {
            m.add(i, i, eps);
        }
        m.compressed()
    };
    let solver = match crate::fea::DirectLagrangeSolver::new(&k_reg, &c_global) {
        Ok(s) => Some(s),
        Err(_) => None,
    };

    let omega = solve_with_fallback(&solver, &k_reg, &c_global, &f_torsion);

    let omega_dot_f: f64 = omega
        .iter()
        .zip(f_torsion.iter())
        .map(|(&a, &b)| a * b)
        .sum();
    let j = ixx + iyy - omega_dot_f;

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

    let psi = solve_with_fallback(&solver, &k_reg, &c_global, &f_psi);
    let phi = solve_with_fallback(&solver, &k_reg, &c_global, &f_phi);

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

    // Elastic shear centre exactly as Python sectionproperties
    // (analysis/section.py): x pairs with phi, y pairs with psi. The
    // geometric integral terms require a realistic Poisson's ratio.
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

    let a_sx = if kappa_x.abs() > 1e-15 {
        delta_s * delta_s / kappa_x
    } else {
        0.0
    };
    let a_sy = if kappa_y.abs() > 1e-15 {
        delta_s * delta_s / kappa_y
    } else {
        0.0
    };
    let a_sxy = if kappa_xy.abs() > 1e-15 {
        delta_s * delta_s / kappa_xy
    } else {
        0.0
    };

    // Principal axis shear areas via tensor rotation (Python method)
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

    let mut int_x = 0.0;
    let mut int_y = 0.0;

    for tri6 in &elements {
        let (ix, iy, _, _) = tri6.monosymmetry_integrals(0.0);
        int_x += ix;
        int_y += iy;
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

    FemWarpingResult {
        j: j.max(0.0),
        iw: iw.max(0.0),
        shear_center,
        shear_center_elastic,
        beta_x_plus,
        beta_y_plus,
        a_sx,
        a_sy,
        a_sxy,
        a_s11,
        a_s22,
    }
}





/// Solve the Lagrangian system preferring the direct skyline solver, with an
/// automatic fallback to CG when the factorised solution fails a relative
/// residual check (ill-conditioned meshes, e.g. with sliver elements).
fn solve_with_fallback(
    solver: &Option<crate::fea::DirectLagrangeSolver>,
    k_reg: &SparseMatrix,
    c: &[f64],
    f: &[f64],
) -> Vec<f64> {
    if let Some(s) = solver {
        let w1 = s.solve(f);
        let w2 = s.solve(c);
        let ct_w2: f64 = c.iter().zip(w2.iter()).map(|(&a, &b)| a * b).sum();
        let ct_w1: f64 = c.iter().zip(w1.iter()).map(|(&a, &b)| a * b).sum();
        if ct_w1.abs() > 1e-15 {
            let lambda = ct_w2 / ct_w1;
            let u: Vec<f64> =
                w1.iter().zip(w2.iter()).map(|(&a, &b)| a - lambda * b).collect();
            // Relative residual of K u - f + lambda c = 0.
            let prod = k_reg.matvec(&u);
            let mut worst = 0.0f64;
            let mut f_norm = 0.0f64;
            for i in 0..prod.len() {
                worst = worst.max((prod[i] - f[i] + lambda * c[i]).abs());
                f_norm = f_norm.max(f[i].abs());
            }
            if worst <= 1e-6 * f_norm.max(1e-300) {
                return u;
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
    let (row_ptr, cols, vals) = k_reg.compressed().to_csr();
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
    let lambda = if ct_w1.abs() > 1e-15 { ct_w2 / ct_w1 } else { 0.0 };
    w1.iter().zip(w2.iter()).map(|(&a, &b)| a - lambda * b).collect()
}

/// Render the warping function ω as a filled-contour SVG for the section.
///
/// Convenience wrapper combining [`compute_fem_solution`] with
/// `sectionproperties`-style warping contour output.
pub fn warping_svg(
    section: &Section,
    props: &SectionProperties,
    width: u32,
    height: u32,
) -> Option<String> {
    use crate::io::{SvgExportOptions, plot_warping_svg};
    let fem = compute_fem_solution(section, props)?;
    let opts = SvgExportOptions {
        width,
        height,
        title: Some("Warping function omega".to_string()),
        ..Default::default()
    };
    Some(plot_warping_svg(&fem.tri6_mesh, &fem.omega, opts))
}