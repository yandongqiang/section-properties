//! Warping FEM Full System Diagnostics
//!
//! Detailed diagnostic validation of the complete warping FEM system.
//! Compares I-section (working) vs Angle/Channel (negative J).
//!
//! Run with: cargo test --test warping_fem_diagnostics -- --nocapture

use section_properties::section::Section;
use section_properties::section_library::steel::{ChannelSection, ISection, AngleSection};
use section_properties::section_library::ParametricSection;
use section_properties::plastic::warping_fem::{diagnose_warping_fem, WarpingDiagnostics};

fn print_diagnostics(d: &WarpingDiagnostics) {
    println!("\n{}", "=".repeat(80));
    println!(" WARPING FEM DIAGNOSTICS: {}", d.section_name);
    println!("{}", "=".repeat(80));
    
    println!("\n--- Element-Level ---");
    println!("  N elements:           {}", d.n_elements);
    println!("  detJ range:           [{:.6e}, {:.6e}]", d.detj_min, d.detj_max);
    println!("  detJ weighted sum:    {:.6e} (should ≈ 6*A/6)", d.detj_weighted_sum);
    println!("  K_e max sym err:      {:.2e}", d.ke_sym_max_err);
    println!("  Element energy:       min={:.6e}, max={:.6e}, sum={:.6e}", 
             d.element_energy_min, d.element_energy_max, d.element_energy_sum);
    
    println!("\n--- Global System ---");
    println!("  N DOF:                {}", d.n_dof);
    println!("  K sym rel err:        {:.2e}", d.k_sym_rel_err);
    println!("  K est rank:           {} / {} (nullity={})", d.k_rank_estimate, d.n_dof, d.k_nullity);
    println!("  Constraint DOFs:      {}", d.constraint_dofs);
    println!("  Constraint nodes:     {} (first 20: {:?})", d.constraint_nodes.len(), &d.constraint_nodes[..20.min(d.constraint_nodes.len())]);
    println!("  Sum(C):               {:.6e} (should = area)", d.constraint_sum);
    
    println!("\n--- Warping Solution ---");
    println!("  ||Kω - F||:           {:.6e}", d.residual_norm);
    println!("  ||res||/||F||:        {:.2e} {}", d.residual_rel, if d.residual_rel < 1e-6 { "✓" } else { "✗" });
    println!("  Cᵀω:                  {:.6e} {}", d.ct_omega, if d.ct_omega.abs() < 1e-9 { "✓" } else { "✗" });
    println!("  ωᵀKω:                 {:.6e}", d.wtkw);
    println!("  ωᵀF:                  {:.6e}", d.wtf);
    println!("  Ixx:                  {:.6e}", d.ixx);
    println!("  Iyy:                  {:.6e}", d.iyy);
    println!("  Ixx+Iyy:              {:.6e}", d.ixx_plus_iyy);
    println!("  ωᵀF:                  {:.6e}", d.omega_dot_f);
    println!("  J_raw = Ixx+Iyy-ωᵀF:  {:.6e} {}", d.j_raw, if d.j_raw > 0.0 { "✓ POSITIVE" } else { "✗ NEGATIVE" });
    
    println!("\n--- Energy Identity ---");
    println!("  ωᵀKω - ωᵀF:           {:.6e}", d.wtkw - d.wtf);
    println!("  rel_error:            {:.2e} {}", d.energy_identity_rel_error, if d.energy_identity_rel_error < 1e-10 { "✓" } else { "✗" });
    
    println!("\n--- Warping RHS Invariants ---");
    println!("  sum(F_x):             {:.6e} (should = 0)", d.sum_fx);
    println!("  sum(F_y):             {:.6e} (should = 0)", d.sum_fy);
    println!("  ∫x dA:                {:.6e} (centroid.x={:.6e})", d.integral_x_da, d.centroid.0);
    println!("  ∫y dA:                {:.6e} (centroid.y={:.6e})", d.integral_y_da, d.centroid.1);
    println!("  first_moment_x (∫y dA): {:.6e}", d.first_moment_x);
    println!("  first_moment_y (∫x dA): {:.6e}", d.first_moment_y);
    println!("  centroid:             ({:.6e}, {:.6e})", d.centroid.0, d.centroid.1);
    println!("  F formulation:        {}", if d.f_formulation_global { "GLOBAL [y, -x]" } else { "CENTROIDAL [y-yc, -(x-xc)]" });
    
    println!("\n--- Final Results ---");
    println!("  J_raw (FEM):          {:.6e}", d.j_fem);
    println!("  J_analytical:         {:.6e}", d.j_analytical);
    println!("  Fallback used:        {}", if d.j_fallback { "YES - NEGATIVE J" } else { "NO" });
    println!("  FEM succeeded:        {}", d.fem_succeeded);
    
    println!("\n{}", "=".repeat(80));
}

#[test]
fn warping_fem_diagnostics_i_section() {
    let i_section = ISection::new(300.0, 150.0, 8.0, 12.0, 15.0);
    let section = i_section.build();
    let d = diagnose_warping_fem(&section, "I-section 300x150", 0.3).expect("diagnostics failed");
    print_diagnostics(&d);
}

#[test]
fn warping_fem_diagnostics_angle() {
    let angle = AngleSection::new(100.0, 100.0, 8.0, 10.0, 12.0);
    let section = angle.build();
    let d = diagnose_warping_fem(&section, "Angle 100x100x8", 0.3).expect("diagnostics failed");
    print_diagnostics(&d);
}

#[test]
fn warping_fem_diagnostics_channel() {
    let channel = ChannelSection::new(200.0, 75.0, 8.0, 10.0, 12.0, 0.0);
    let section = channel.build();
    let d = diagnose_warping_fem(&section, "Channel 200x75x8x10", 0.3).expect("diagnostics failed");
    print_diagnostics(&d);
}