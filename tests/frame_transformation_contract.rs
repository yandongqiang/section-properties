//! Phase 12 - 2D frame architecture audit: transformation contract and
//! multi-member (frame) behaviour of the existing Beam FEM core.
//!
//! This is audit evidence, not a new feature. It establishes that the existing
//! element transformation and global solver already satisfy the requirements a
//! 2D frame layer needs:
//!
//! 1. `T` is orthogonal (`T^T T = I`), so local/global mappings need no stored
//!    inverse and the rotational DOF is carried unchanged;
//! 2. `k_global(theta) = R k_global(0) R^T` - the transformation is applied
//!    exactly once and is consistent for any member orientation;
//! 3. branched multi-member models (two-member apex frame, portal frame) solve
//!    today: global equilibrium holds and all direct backends agree.

use section_properties::SolverSelection;
use section_properties::beam_fem::{BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver};
use section_properties::material::Material;

const E: f64 = 200e9;
const A: f64 = 5e-3;
const I: f64 = 2e-5;
const L: f64 = 2.0;

fn mat() -> Material {
    Material::new(E, 0.3, 7850.0, "Steel")
}
fn sec() -> BeamSection {
    BeamSection::new(A, I)
}

fn matmul6(a: &[[f64; 6]; 6], b: &[[f64; 6]; 6]) -> [[f64; 6]; 6] {
    let mut out = [[0.0f64; 6]; 6];
    for i in 0..6 {
        for j in 0..6 {
            for k in 0..6 {
                out[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    out
}

fn transpose6(a: &[[f64; 6]; 6]) -> [[f64; 6]; 6] {
    let mut t = [[0.0f64; 6]; 6];
    for i in 0..6 {
        for j in 0..6 {
            t[i][j] = a[j][i];
        }
    }
    t
}

fn scale6(a: &[[f64; 6]; 6]) -> f64 {
    a.iter().flatten().fold(0.0f64, |m, v| m.max(v.abs()))
}

/// Single element with `node_i` at the origin and `node_j` at angle `deg`.
fn angled_element(deg: f64, len: f64) -> BeamElement {
    let th = deg.to_radians();
    let mut m = BeamModel::new();
    m.add_node(BeamNode::new(0, 0.0, 0.0));
    m.add_node(BeamNode::new(1, len * th.cos(), len * th.sin()));
    let el = BeamElement::new(0, 1, mat(), sec()).unwrap();
    m.add_element(el);
    m.elements[0].clone()
}

fn node_pts(
    el_angle_deg: f64,
    len: f64,
) -> (
    section_properties::geometry::Point,
    section_properties::geometry::Point,
) {
    let th = el_angle_deg.to_radians();
    (
        section_properties::geometry::Point::new(0.0, 0.0),
        section_properties::geometry::Point::new(len * th.cos(), len * th.sin()),
    )
}

// ---------------------------------------------------------------------------
// 1. Transformation contract (angles 0/45/90/-45/135)
// ---------------------------------------------------------------------------

#[test]
fn transformation_is_orthogonal_at_all_angles() {
    for deg in [0.0_f64, 90.0, 45.0, -45.0, 135.0] {
        let (ni, nj) = node_pts(deg, L);
        let el = angled_element(deg, L);
        let t = el.transformation_matrix(ni, nj);

        // T^T T = I: T is a rotation, so T^-1 = T^T (no stored inverse needed,
        // and the mapping global -> local is an isometry).
        let ttt = matmul6(&transpose6(&t), &t);
        for (i, row) in ttt.iter().enumerate() {
            for (j, &v) in row.iter().enumerate() {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (v - expected).abs() < 1e-12,
                    "{deg}deg: (T^T T)[{i}][{j}] = {v} != {expected}"
                );
            }
        }

        // The rotational DOF is carried unchanged: theta is not transformed.
        assert!((t[2][2] - 1.0).abs() < 1e-15, "{deg}deg: T[2][2]");
        assert!((t[5][5] - 1.0).abs() < 1e-15, "{deg}deg: T[5][5]");
        assert!(
            t[2].iter()
                .enumerate()
                .filter(|(k, _)| *k != 2)
                .all(|(_, v)| v.abs() < 1e-15)
                && t[5]
                    .iter()
                    .enumerate()
                    .filter(|(k, _)| *k != 5)
                    .all(|(_, v)| v.abs() < 1e-15),
            "{deg}deg: rotation DOF coupled to translations"
        );
    }
    println!("  T orthogonal (T^T T = I) and rotation DOF decoupled at 0/45/90/-45/135 deg");
}

#[test]
fn element_stiffness_rotates_consistently() {
    // k_global(theta) must equal R k_global(0) R^T with R = T^T (the global
    // rotation). This pins "the transformation is applied exactly once".
    let (ni0, nj0) = node_pts(0.0, L);
    let el0 = angled_element(0.0, L);
    let k0 = el0.global_stiffness(ni0, nj0);

    for deg in [45.0_f64, 90.0, -45.0, 135.0] {
        let (ni, nj) = node_pts(deg, L);
        let el = angled_element(deg, L);
        let kg = el.global_stiffness(ni, nj);
        let r = transpose6(&el.transformation_matrix(ni, nj));
        let expected = matmul6(&matmul6(&r, &k0), &transpose6(&r));

        let sc = scale6(&kg).max(1.0);
        for (i, row) in kg.iter().enumerate() {
            for (j, &v) in row.iter().enumerate() {
                assert!(
                    (v - expected[i][j]).abs() <= 1e-6 * sc,
                    "{deg}deg: k_global[{i}][{j}] = {v} vs rotated {}",
                    expected[i][j]
                );
            }
        }
        // And the element stiffness stays symmetric after rotation.
        for (i, row) in kg.iter().enumerate() {
            for (j, &v) in row.iter().enumerate() {
                assert!(
                    (v - kg[j][i]).abs() <= 1e-6 * sc,
                    "{deg}deg: k_global not symmetric at [{i}][{j}]"
                );
            }
        }
    }
    println!("  k_global(theta) == R k_global(0) R^T and symmetric at 45/90/-45/135 deg");
}

// ---------------------------------------------------------------------------
// 2. Multi-member frame behaviour of the existing core
// ---------------------------------------------------------------------------

fn solve_all(model: &BeamModel) -> Vec<(String, Vec<f64>, Vec<f64>)> {
    DIRECT
        .iter()
        .map(|name| {
            let mut s = BeamSolver::from_model(model).unwrap();
            s.set_solver(SolverSelection::named(*name));
            s.solve_configured().unwrap();
            (
                (*name).to_string(),
                s.displacements().to_vec(),
                s.reactions(),
            )
        })
        .collect()
}

const DIRECT: [&str; 3] = ["dense", "skyline_ldlt", "sparse_lu"];

fn assert_global_equilibrium(model: &BeamModel, reactions: &[f64], label: &str) {
    // Apply support reactions as equivalent global nodal loads and verify
    // sum(F) = 0 and sum(M about the origin) = 0.
    let mut fx = 0.0;
    let mut fy = 0.0;
    let mut mz = 0.0; // about the origin (0, 0)

    // Support reactions, per node.
    for (idx, node) in model.nodes.iter().enumerate() {
        let p = node.point();
        let rx = reactions.get(3 * idx).copied().unwrap_or(0.0);
        let ry = reactions.get(3 * idx + 1).copied().unwrap_or(0.0);
        let rz = reactions.get(3 * idx + 2).copied().unwrap_or(0.0);
        fx += rx;
        fy += ry;
        mz += p.x * ry - p.y * rx + rz;
    }

    // Applied nodal loads, per node.
    for (node, dof, v) in &model.nodal_forces {
        let p = model.nodes[*node].point();
        match dof {
            0 => {
                fx += v;
                mz += -p.y * v;
            }
            1 => {
                fy += v;
                mz += p.x * v;
            }
            _ => mz += v,
        }
    }
    for am in &model.applied_moments {
        mz += am.value;
    }

    let fscale = (fx.abs() + fy.abs()).max(1.0);
    let mscale = mz.abs().max(fscale * 10.0);
    assert!(fx.abs() <= 1e-6 * fscale, "{label}: sum Fx = {fx}");
    assert!(fy.abs() <= 1e-6 * fscale, "{label}: sum Fy = {fy}");
    assert!(
        mz.abs() <= 1e-6 * mscale,
        "{label}: sum M about origin = {mz}"
    );
}

#[test]
fn two_member_angled_frame() {
    // Apex frame: fixed bases at (0,0) and (2,0), apex at (1,1), downward load
    // at the apex. Axial + bending couple through the transformation.
    let p = 25e3;
    let mut m = BeamModel::new();
    m.add_node(BeamNode::new(0, 0.0, 0.0));
    m.add_node(BeamNode::new(1, 1.0, 1.0));
    m.add_node(BeamNode::new(2, 2.0, 0.0));
    m.add_element(BeamElement::new(0, 1, mat(), sec()).unwrap());
    m.add_element(BeamElement::new(1, 2, mat(), sec()).unwrap());
    m.try_fix_node(0).unwrap();
    m.try_fix_node(2).unwrap();
    m.try_add_nodal_force(1, 1, -p).unwrap();

    let sols = solve_all(&m);
    for (name, u, r) in &sols {
        assert_global_equilibrium(&m, r, &format!("apex frame ({name})"));
        assert!(u.iter().all(|v| v.is_finite()));
    }
    // Symmetry: zero horizontal apex movement, equal vertical reactions, and an
    // equal-and-opposite horizontal thrust at the two bases - the expected
    // behaviour of an inclined two-member frame, NOT zero horizontal reaction.
    // DOF layout: node 0 (base) = 0..2, node 1 (apex) = 3..5, node 2 (base) = 6..8.
    let (_, u0, r0) = &sols[0];
    assert!(u0[3].abs() < 1e-9, "apex ux = {} (symmetry)", u0[3]);
    assert!(u0[4] < 0.0, "apex must deflect downward");
    assert!(
        (r0[0] + r0[6]).abs() < 1e-6,
        "base horizontal reactions must balance: {} + {}",
        r0[0],
        r0[6]
    );
    assert!(
        r0[0].abs() > 1.0,
        "an inclined frame must develop a horizontal thrust, got {}",
        r0[0]
    );
    assert!((r0[1] - p / 2.0).abs() < 1e-6, "Ry at base 0 = {}", r0[1]);
    assert!((r0[7] - p / 2.0).abs() < 1e-6, "Ry at base 1 = {}", r0[7]);
    // The apex is free: no reaction there.
    for (k, &v) in r0.iter().enumerate().take(6).skip(3) {
        assert!(v.abs() < 1e-6, "apex reaction r0[{k}] = {v}");
    }

    // Backend independence.
    for (_, u, _) in &sols[1..] {
        for i in 0..u.len() {
            assert!(
                (u[i] - u0[i]).abs() <= 1e-12 * u0[i].abs().max(1.0),
                "backend disagreement at u[{i}]: {} vs {}",
                u[i],
                u0[i]
            );
        }
    }
    println!(
        "  apex frame: ux={:.3e} uy={:.6e} Ry(base0)={:.6e} (equilibrium + 3 backends agree)",
        u0[3], u0[4], r0[1]
    );
}

#[test]
fn portal_frame() {
    // Portal frame: two fixed columns (h = 2) and a beam (w = 3), lateral load
    // at the top of the left column.
    let (h, w, f) = (2.0_f64, 3.0, 30e3);
    let mut m = BeamModel::new();
    m.add_node(BeamNode::new(0, 0.0, 0.0));
    m.add_node(BeamNode::new(1, 0.0, h));
    m.add_node(BeamNode::new(2, w, h));
    m.add_node(BeamNode::new(3, w, 0.0));
    m.add_element(BeamElement::new(0, 1, mat(), sec()).unwrap());
    m.add_element(BeamElement::new(1, 2, mat(), sec()).unwrap());
    m.add_element(BeamElement::new(2, 3, mat(), sec()).unwrap());
    m.try_fix_node(0).unwrap();
    m.try_fix_node(3).unwrap();
    m.try_add_nodal_force(1, 0, f).unwrap();

    let sols = solve_all(&m);
    for (name, u, r) in &sols {
        assert_global_equilibrium(&m, r, &format!("portal ({name})"));
        assert!(u.iter().all(|v| v.is_finite()));
        // Total horizontal reaction must balance the applied lateral load.
        assert!(
            (r[0] + r[9] + f).abs() <= 1e-6 * f,
            "{name}: sum Fx = {}",
            r[0] + r[9] + f
        );
    }
    let (_, u0, r0) = &sols[0];
    assert!(
        u0[3] > 0.0,
        "left column top must sway in the load direction, got {}",
        u0[3]
    );
    for (_, u, _) in &sols[1..] {
        for i in 0..u.len() {
            assert!(
                (u[i] - u0[i]).abs() <= 1e-12 * u0[i].abs().max(1.0),
                "backend disagreement at u[{i}]"
            );
        }
    }
    println!(
        "  portal frame: sway={:.6e} sum Rx={:.6e} (equilibrium + 3 backends agree)",
        u0[3],
        r0[0] + r0[9]
    );
}

#[test]
fn prescribed_support_settlement_in_a_frame() {
    // Prescribed displacement at one support of the portal frame: the reactions
    // must include the settlement-induced forces and the prescribed value must
    // be recovered exactly (K_ff u_f = f_f - K_fc u_c contract).
    let (h, w) = (2.0_f64, 3.0);
    let delta = -0.005;
    let mut m = BeamModel::new();
    m.add_node(BeamNode::new(0, 0.0, 0.0));
    m.add_node(BeamNode::new(1, 0.0, h));
    m.add_node(BeamNode::new(2, w, h));
    m.add_node(BeamNode::new(3, w, 0.0));
    m.add_element(BeamElement::new(0, 1, mat(), sec()).unwrap());
    m.add_element(BeamElement::new(1, 2, mat(), sec()).unwrap());
    m.add_element(BeamElement::new(2, 3, mat(), sec()).unwrap());
    m.try_fix_node(0).unwrap();
    m.try_fix_node_with_values(3, 0.0, delta, 0.0).unwrap();

    let sols = solve_all(&m);
    let (_, u0, r0) = &sols[0];
    // The prescribed DOF is enforced exactly.
    assert_eq!(u0[10], delta, "prescribed settlement must be exact");
    // No external load: the reactions must be self-equilibrated.
    assert!(
        r0[0].abs() + r0[9].abs() > 0.0,
        "settlement must induce reactions"
    );
    assert_global_equilibrium(&m, r0, "settlement frame");
    for (_, u, r) in &sols[1..] {
        assert_eq!(u[10], delta);
        for i in 0..u.len() {
            assert!((u[i] - u0[i]).abs() <= 1e-9 * u0[i].abs().max(1.0));
        }
        for i in 0..r.len() {
            assert!((r[i] - r0[i]).abs() <= 1e-3);
        }
    }
    println!(
        "  settlement: uy={delta} enforced exactly, induced Rx(base0)={:.6e} Rx(base1)={:.6e}",
        r0[0], r0[9]
    );
}
