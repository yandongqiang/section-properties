//! FEM warping analysis using Tri6 elements.
//!
//! Mirrors Python `sectionproperties.analysis.section.Section.calculate_warping_properties`:
//! assembles Lagrangian stiffness matrix, solves for warping function ω and shear
//! functions ψ, φ, then computes J, Iw, shear centre, shear areas and monosymmetry
//! constants via Gaussian quadrature.

use crate::fea::{SparseMatrix, Tri6, Tri6Mesh, solve_lagrange_sparse, tri3_to_tri6};
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
    let min_edge = min_edge_length(section);
    let _ = props;
    let target_size = (max_dim / 30.0).min(min_edge * 1.5).max(1e-4);

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

    let mut refined_nodes = base_mesh.nodes.clone();
    let mut refined_elements = base_mesh.elements.clone();
    refine_mesh(
        &mut refined_nodes,
        &mut refined_elements,
        target_size,
        10,
        5000,
    );

    if refined_elements.is_empty() {
        return None;
    }


    let tri6_mesh = tri3_to_tri6(&refined_nodes, &refined_elements);
    let n = tri6_mesh.nodes.len();

    let mut elements: Vec<Tri6> = Vec::with_capacity(tri6_mesh.elements.len());
    for (i, &elem) in tri6_mesh.elements.iter().enumerate() {
        let mut points = [Point::new(0.0, 0.0); 6];
        for k in 0..6 {
            let p = tri6_mesh.nodes[elem[k]];
            points[k] = Point::new(p.x - cx, p.y - cy);
        }
        elements.push(Tri6::from_points(i, points, elem, 1.0, 1.0, 1.0));
    }

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

    let omega = solve_lagrange_sparse(&k_global, &c_global, &f_torsion);

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

    let psi = solve_lagrange_sparse(&k_global, &c_global, &f_psi);
    let phi = solve_lagrange_sparse(&k_global, &c_global, &f_phi);

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

/// Refine a Tri3 mesh by edge midpoint subdivision until all edges < target_size.
fn refine_mesh(
    nodes: &mut Vec<Point>,
    elements: &mut Vec<[usize; 3]>,
    target_size: f64,
    max_iterations: usize,
    max_nodes: usize,
) {
    use std::collections::HashMap;

    // Uniform (red) refinement: subdivide every element each pass so the
    // mesh stays conforming. Selective refinement would create hanging
    // nodes, which breaks stiffness assembly and prevents convergence.
    for _ in 0..max_iterations {
        // Does any element still exceed the target size?
        let needs = elements.iter().any(|elem| {
            let p0 = nodes[elem[0]];
            let p1 = nodes[elem[1]];
            let p2 = nodes[elem[2]];
            let d01 = ((p1.x - p0.x).powi(2) + (p1.y - p0.y).powi(2)).sqrt();
            let d12 = ((p2.x - p1.x).powi(2) + (p2.y - p1.y).powi(2)).sqrt();
            let d20 = ((p0.x - p2.x).powi(2) + (p0.y - p2.y).powi(2)).sqrt();
            d01.max(d12).max(d20) > target_size * 1.5
        });
        if !needs || nodes.len() >= max_nodes / 4 {
            break;
        }

        let mut edge_map: HashMap<(usize, usize), usize> = HashMap::new();
        let mut new_elements = Vec::with_capacity(elements.len() * 4);

        for &elem in elements.iter() {
            let m01 =
                get_or_create_midpoint_refine(nodes, &mut edge_map, elem[0], elem[1]);
            let m12 =
                get_or_create_midpoint_refine(nodes, &mut edge_map, elem[1], elem[2]);
            let m20 =
                get_or_create_midpoint_refine(nodes, &mut edge_map, elem[2], elem[0]);

            new_elements.push([elem[0], m01, m20]);
            new_elements.push([m01, elem[1], m12]);
            new_elements.push([m20, m12, elem[2]]);
            new_elements.push([m01, m12, m20]);
        }

        *elements = new_elements;
    }
}

fn get_or_create_midpoint_refine(
    nodes: &mut Vec<Point>,
    edge_map: &mut std::collections::HashMap<(usize, usize), usize>,
    a: usize,
    b: usize,
) -> usize {
    let key = if a < b { (a, b) } else { (b, a) };
    if let Some(&mid) = edge_map.get(&key) {
        return mid;
    }
    let pa = nodes[a];
    let pb = nodes[b];
    let mid = Point::new(0.5 * (pa.x + pb.x), 0.5 * (pa.y + pb.y));
    let mid_idx = nodes.len();
    nodes.push(mid);
    edge_map.insert(key, mid_idx);
    mid_idx
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
    let min_edge = min_edge_length(section);
    let _ = props;
    let target_size = (max_dim / 30.0).min(min_edge * 1.5).max(1e-4);
    let _t00 = std::time::Instant::now();

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

    let mut refined_nodes = mesh.nodes.clone();
    let mut refined_elements = mesh.elements.clone();
    refine_mesh(
        &mut refined_nodes,
        &mut refined_elements,
        target_size,
        10,
        5000,
    );

    let tri6_mesh = tri3_to_tri6(&refined_nodes, &refined_elements);
    let n = tri6_mesh.nodes.len();

    let mut elements: Vec<Tri6> = Vec::with_capacity(tri6_mesh.elements.len());
    for (i, &elem) in tri6_mesh.elements.iter().enumerate() {
        let mut points = [Point::new(0.0, 0.0); 6];
        for k in 0..6 {
            let p = tri6_mesh.nodes[elem[k]];
            points[k] = Point::new(p.x - cx, p.y - cy);
        }
        elements.push(Tri6::from_points(i, points, elem, 1.0, 1.0, 1.0));
    }

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

    let omega = solve_lagrange_sparse(&k_global, &c_global, &f_torsion);

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

    let psi = solve_lagrange_sparse(&k_global, &c_global, &f_psi);
    let phi = solve_lagrange_sparse(&k_global, &c_global, &f_phi);

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

    // Elastic shear center (Python elasticity approach)
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

    // Shear center: use sc_xint/sc_yint formula (robust for symmetric sections)
    let x_se = if denom.abs() > 1e-15 {
        (ixy * sc_xint - ixx * sc_yint) / denom / ea
    } else {
        0.0
    };
    let y_se = if denom.abs() > 1e-15 {
        (iyy * sc_xint - ixy * sc_yint) / denom / ea
    } else {
        0.0
    };

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




