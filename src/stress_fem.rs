//! FEM stress analysis using Tri6 elements.
//!
//! Mirrors Python `sectionproperties.post.stress_post.StressPost`:
//! computes stress distributions at mesh nodes using Tri6 element stress
//! computation with Gauss-point extrapolation.

use crate::plastic::warping_fem::{FemSolution, compute_fem_solution};
use crate::section::Section;
use crate::section_properties::SectionProperties;
use crate::stress::{SectionLoads, StressAtPoint};

/// Compute FEM stresses at all mesh nodes.
///
/// Returns `None` if FEM mesh generation or solving fails.
pub fn calculate_stress_fem(
    section: &Section,
    props: &SectionProperties,
    loads: SectionLoads,
) -> Option<Vec<StressAtPoint>> {
    let fem = compute_fem_solution(section, props)?;
    compute_stress_from_fem(&fem, props, loads)
}

/// Compute stresses from a pre-computed FEM solution.
fn compute_stress_from_fem(
    fem: &FemSolution,
    props: &SectionProperties,
    loads: SectionLoads,
) -> Option<Vec<StressAtPoint>> {
    let _cx = props.centroid.x;
    let _cy = props.centroid.y;
    let ea = props.area;
    let ixx = props.ix;
    let iyy = props.iy;
    let ixy = props.ixy;

    let principal = props.principal_properties();
    let phi = principal.phi;
    let i11 = principal.i11;
    let i22 = principal.i22;

    let j = fem.j.max(1e-15);
    let nu = fem.nu;
    let delta_s = fem.delta_s;

    let n_nodes = fem.tri6_mesh.nodes.len();

    // Accumulate stress components at each node (for averaging)
    let mut acc = vec![NodeAccum::new(); n_nodes];

    for tri6 in &fem.elements {
        let omega_el = extract_nodal(&fem.omega, tri6.node_ids);
        let psi_el = extract_nodal(&fem.psi, tri6.node_ids);
        let phi_el = extract_nodal(&fem.phi, tri6.node_ids);

        // Elements use section-centroid coordinates; pass section centroid for stress calc
        let cx = props.centroid.x;
        let cy = props.centroid.y;
        let stresses = tri6.element_stress(
            loads.n, loads.mxx, loads.myy, loads.m11, loads.m22, loads.mzz, loads.vx, loads.vy, ea,
            cx, cy, ixx, iyy, ixy, i11, i22, phi, j, nu, &omega_el, &psi_el, &phi_el, delta_s,
        );

        for k in 0..6 {
            let gid = tri6.node_ids[k];
            acc[gid].add(&stresses, k);
        }
    }

    // Average and build StressAtPoint for each node
    let mut result: Vec<StressAtPoint> = Vec::with_capacity(n_nodes);
    for (i, node) in fem.tri6_mesh.nodes.iter().enumerate() {
        let a = &acc[i];
        if a.count == 0 {
            continue;
        }
        let c = a.count as f64;

        let sig_zz_n = a.sig_zz_n / c;
        let sig_zz_mxx = a.sig_zz_mxx / c;
        let sig_zz_myy = a.sig_zz_myy / c;
        let sig_zz_m11 = a.sig_zz_m11 / c;
        let sig_zz_m22 = a.sig_zz_m22 / c;
        let sig_zx_mzz = a.sig_zx_mzz / c;
        let sig_zy_mzz = a.sig_zy_mzz / c;
        let sig_zx_vx = a.sig_zx_vx / c;
        let sig_zy_vx = a.sig_zy_vx / c;
        let sig_zx_vy = a.sig_zx_vy / c;
        let sig_zy_vy = a.sig_zy_vy / c;

        let sap = build_stress_at_point(
            node.x, node.y, sig_zz_n, sig_zz_mxx, sig_zz_myy, sig_zz_m11, sig_zz_m22, sig_zx_vx,
            sig_zy_vx, sig_zx_vy, sig_zy_vy, sig_zx_mzz, sig_zy_mzz,
        );
        result.push(sap);
    }

    if result.is_empty() {
        None
    } else {
        Some(result)
    }
}

/// Accumulator for nodal stresses (shared nodes get averaged).
#[derive(Clone)]
struct NodeAccum {
    count: usize,
    sig_zz_n: f64,
    sig_zz_mxx: f64,
    sig_zz_myy: f64,
    sig_zz_m11: f64,
    sig_zz_m22: f64,
    sig_zx_mzz: f64,
    sig_zy_mzz: f64,
    sig_zx_vx: f64,
    sig_zy_vx: f64,
    sig_zx_vy: f64,
    sig_zy_vy: f64,
}

impl NodeAccum {
    fn new() -> Self {
        Self {
            count: 0,
            sig_zz_n: 0.0,
            sig_zz_mxx: 0.0,
            sig_zz_myy: 0.0,
            sig_zz_m11: 0.0,
            sig_zz_m22: 0.0,
            sig_zx_mzz: 0.0,
            sig_zy_mzz: 0.0,
            sig_zx_vx: 0.0,
            sig_zy_vx: 0.0,
            sig_zx_vy: 0.0,
            sig_zy_vy: 0.0,
        }
    }

    fn add(&mut self, s: &[[f64; 6]; 11], k: usize) {
        self.count += 1;
        self.sig_zz_n += s[0][k];
        self.sig_zz_mxx += s[1][k];
        self.sig_zz_myy += s[2][k];
        self.sig_zz_m11 += s[3][k];
        self.sig_zz_m22 += s[4][k];
        self.sig_zx_mzz += s[5][k];
        self.sig_zy_mzz += s[6][k];
        self.sig_zx_vx += s[7][k];
        self.sig_zy_vx += s[8][k];
        self.sig_zx_vy += s[9][k];
        self.sig_zy_vy += s[10][k];
    }
}

fn extract_nodal(field: &[f64], node_ids: [usize; 6]) -> [f64; 6] {
    let mut e = [0.0; 6];
    for i in 0..6 {
        e[i] = field[node_ids[i]];
    }
    e
}

fn build_stress_at_point(
    x: f64,
    y: f64,
    sig_zz_n: f64,
    sig_zz_mxx: f64,
    sig_zz_myy: f64,
    sig_zz_m11: f64,
    sig_zz_m22: f64,
    sig_zx_vx: f64,
    sig_zy_vx: f64,
    sig_zx_vy: f64,
    sig_zy_vy: f64,
    sig_zx_mzz: f64,
    sig_zy_mzz: f64,
) -> StressAtPoint {
    let sig_zz_m = sig_zz_mxx + sig_zz_myy + sig_zz_m11 + sig_zz_m22;
    let sig_zxy_mzz = (sig_zx_mzz * sig_zx_mzz + sig_zy_mzz * sig_zy_mzz).sqrt();
    let sig_zxy_vx = (sig_zx_vx * sig_zx_vx + sig_zy_vx * sig_zy_vx).sqrt();
    let sig_zxy_vy = (sig_zx_vy * sig_zx_vy + sig_zy_vy * sig_zy_vy).sqrt();
    let sig_zx_v = sig_zx_vx + sig_zx_vy;
    let sig_zy_v = sig_zy_vx + sig_zy_vy;
    let sig_zxy_v = (sig_zx_v * sig_zx_v + sig_zy_v * sig_zy_v).sqrt();

    let sigma_z = sig_zz_n + sig_zz_m;
    let tau_xz = sig_zx_mzz + sig_zx_v;
    let tau_yz = sig_zy_mzz + sig_zy_v;
    let tau_zxy = (tau_xz * tau_xz + tau_yz * tau_yz).sqrt();

    let von_mises = (sigma_z * sigma_z + 3.0 * tau_zxy * tau_zxy).sqrt();

    let half = sigma_z / 2.0;
    let disc = (half * half + tau_zxy * tau_zxy).sqrt();
    let sigma_1 = half + disc;
    let sigma_2 = half - disc;

    StressAtPoint {
        x,
        y,
        sig_zz_n,
        sig_zz_mxx,
        sig_zz_myy,
        sig_zz_m11,
        sig_zz_m22,
        sig_zx_vx,
        sig_zy_vx,
        sig_zx_vy,
        sig_zy_vy,
        sig_zx_mzz,
        sig_zy_mzz,
        sig_zz_m,
        sig_zxy_mzz,
        sig_zxy_vx,
        sig_zxy_vy,
        sig_zx_v,
        sig_zy_v,
        sig_zxy_v,
        sigma_z,
        tau_xz,
        tau_yz,
        tau_zxy,
        von_mises,
        sigma_1,
        sigma_2,
    }
}
