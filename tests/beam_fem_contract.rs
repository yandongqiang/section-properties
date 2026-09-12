//! Beam FEM contract regression tests.
//!
//! This file pins only the statements in `docs/beam_fem.md` that are not
//! already asserted elsewhere, so that a future change to the public contract
//! fails a test instead of silently altering semantics.
//!
//! Deliberately **not** duplicated here (covered by the referenced suites):
//!
//! | Contract | Covered by |
//! | --- | --- |
//! | point-load boundary (`xi = 0` / `xi = 1`) | `tests/beam_fem_api_semantics.rs` |
//! | applied-moment end-force convention (`M_j = -M`) | `tests/beam_fem_api_semantics.rs` |
//! | reaction equilibrium / free-DOF reactions | `tests/beam_fem_api_semantics.rs` |
//! | `from_model` snapshot semantics | `tests/beam_fem_api_semantics.rs` |
//! | element end-force formula `f_end = f_equiv - K_e u_e` | `tests/beam_fem_api_semantics.rs` |
//! | local/global transformation | `tests/beam_fem_api_semantics.rs`, `tests/beam_fem_reference.rs` |

use section_properties::beam_fem::{BeamElement, BeamModel, BeamNode, BeamSection, BeamSolver};
use section_properties::material::Material;

/// Unit cantilever (E = A = I = 1, L = 1), fixed at node 0.
fn unit_cantilever() -> BeamModel {
    let mut m = BeamModel::new();
    m.add_node(BeamNode::new(0, 0.0, 0.0));
    m.add_node(BeamNode::new(1, 1.0, 0.0));
    m.add_element(
        BeamElement::new(
            0,
            1,
            Material::new(1.0, 0.3, 1.0, "unit"),
            BeamSection::new(1.0, 1.0),
        )
        .unwrap(),
    );
    m.fix_node(0);
    m
}

#[test]
fn test_contract_dof_and_displacement_layout() {
    let mut m = unit_cantilever();
    // Axial-only load: only the ux slots may be non-zero.
    m.add_nodal_force(1, 0, 100.0);

    // Documented DOF mapping: dof(node, d) = 3*node + d.
    assert_eq!(m.dof_index(0, 0), 0, "ux0");
    assert_eq!(m.dof_index(0, 1), 1, "uy0");
    assert_eq!(m.dof_index(0, 2), 2, "rz0");
    assert_eq!(m.dof_index(1, 0), 3, "ux1");
    assert_eq!(m.dof_index(1, 1), 4, "uy1");
    assert_eq!(m.dof_index(1, 2), 5, "rz1");

    let mut s = BeamSolver::from_model(&m).unwrap();
    s.solve_configured().unwrap();
    let u = s.displacements();

    // Layout: [ux0, uy0, rz0, ux1, uy1, rz1].
    assert_eq!(u.len(), 6, "3 DOFs per node");
    assert_eq!(u[0], 0.0, "ux0 constrained");
    assert_eq!(u[1], 0.0, "uy0 constrained");
    assert_eq!(u[2], 0.0, "rz0 constrained");
    assert!(
        (u[3] - 100.0).abs() < 1e-9,
        "ux1 = F L / EA = 100, got {}",
        u[3]
    );
    assert!(
        u[4].abs() < 1e-9,
        "uy1 must be zero for an axial-only load, got {}",
        u[4]
    );
    assert!(
        u[5].abs() < 1e-9,
        "rz1 must be zero for an axial-only load, got {}",
        u[5]
    );
}

#[test]
fn test_contract_sagging_positive_moment_sign() {
    // Uniform downward load on a cantilever: the internal moment is hogging
    // (negative in the sagging-positive convention) and vanishes at the free
    // end:
    //   M(x) = -q (L - x)^2 / 2
    let q = 100.0;
    let l = 1.0;
    let mut m = unit_cantilever();
    m.add_distributed_load(0, 0.0, -q).unwrap();

    let mut s = BeamSolver::from_model(&m).unwrap();
    s.solve_configured().unwrap();

    let m0 = s.element_section_forces(0, 0.0).unwrap().moment;
    let m_mid = s.element_section_forces(0, 0.5).unwrap().moment;
    let m_end = s.element_section_forces(0, 1.0).unwrap().moment;

    assert!(
        (m0 - (-q * l * l / 2.0)).abs() < 1e-9,
        "M(0) = -qL^2/2 = {}, got {} (sagging-positive convention)",
        -q * l * l / 2.0,
        m0
    );
    assert!(m0 < 0.0, "hogging moment must be negative, got {}", m0);
    assert!(
        (m_mid - (-q * l * l / 8.0)).abs() < 1e-9,
        "M(L/2) = {}, got {}",
        -q * l * l / 8.0,
        m_mid
    );
    assert!(
        m_end.abs() < 1e-12,
        "free-end moment must vanish, got {}",
        m_end
    );
}
