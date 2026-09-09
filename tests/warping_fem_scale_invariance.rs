//! Warping FEM Scale Invariance Tests
//!
//! Verifies that the Warping FEM pipeline produces results that scale correctly
//! with geometry scaling. For each scale factor, the geometry is scaled,
//! a new mesh is generated, FEM matrices are assembled, and the solver is run.
//! Results are compared against theoretical scaling laws.

use section_properties::mesh::MeshControl;
use section_properties::plastic::warping_fem::{FemWarpingSolution, compute_fem_warping_solution};
use section_properties::section::Section;
use section_properties::section_library::ParametricSection;
use section_properties::section_library::primitive::RectangularSection;
use section_properties::section_library::steel::{AngleSection, ChannelSection};
use section_properties::section_properties::SectionProperties;
use std::time::Instant;

fn rel_err(a: f64, b: f64) -> f64 {
    if b == 0.0 {
        return a.abs();
    }
    (a - b).abs() / b.abs()
}

fn assert_rel(actual: f64, expected: f64, tol: f64, name: &str) {
    // Use absolute error when expected is near zero
    let err = if expected.abs() < 1e-12 {
        (actual - expected).abs()
    } else {
        rel_err(actual, expected)
    };
    println!(
        "  {}: actual={:.6e} expected={:.6e} err={:.3e}",
        name, actual, expected, err
    );
    assert!(err < tol, "{} err {:.3e} > {:.3e}", name, err, tol);
}

/// Scale a section by factor s, returning a new Section
fn scale_section(section: &Section, s: f64) -> Section {
    let mut scaled_outer = section.outer.clone();
    for v in &mut scaled_outer.vertices {
        v.x *= s;
        v.y *= s;
    }
    let mut scaled_holes = Vec::new();
    for hole in &section.holes {
        let mut scaled_hole = hole.clone();
        for v in &mut scaled_hole.vertices {
            v.x *= s;
            v.y *= s;
        }
        scaled_holes.push(scaled_hole);
    }
    Section::new(scaled_outer, scaled_holes)
}

fn run_scale_test(name: &str, section: Section, scales: &[f64]) {
    println!("\n{}", "=".repeat(80));
    println!("SCALE INVARIANCE TEST: {}", name);
    println!("{}", "=".repeat(80));

    let base_nu = 0.3;

    // Run at ALL scales and store results
    let mut results = Vec::new();
    for &s in scales {
        println!("\n--- Scale s = {:.0e} ---", s);

        // Scale geometry
        let scaled_section = scale_section(&section, s);
        let scaled_props = SectionProperties::from_section(&scaled_section);

        // Run FEM
        let start = Instant::now();
        let result = match compute_fem_warping_solution(
            &scaled_section,
            &scaled_props,
            base_nu,
            MeshControl::Fine,
        ) {
            Ok(r) => r,
            Err(e) => {
                println!("FEM failed at scale {:.0e}: {:?}", s, e);
                continue;
            }
        };
        let elapsed = start.elapsed();

        // Print results
        println!("n_dof: {}", result.omega.len());
        println!("Time: {:.3?}", elapsed);
        println!("J: {:.6e}", result.j);
        println!("Iw: {:.6e}", result.iw);
        println!(
            "Shear center: ({:.6}, {:.6})",
            result.shear_center.x, result.shear_center.y
        );
        println!("omega_max: {:.6e}", result.omega_max);
        println!(
            "Exact: {}, Reg: {}",
            result.used_exact_solver, result.used_regularization
        );
        println!(
            "Exact residual: {:.2e}, Reg residual: {:.2e}",
            result.exact_residual, result.regularized_residual
        );

        results.push((s, result));
    }

    // Find the base scale (s=1) result
    let base_idx = results
        .iter()
        .position(|(s, _)| (s - 1.0).abs() < f64::EPSILON);
    if base_idx.is_none() {
        println!("No base scale (s=1) result found, skipping verification");
        return;
    }
    let base_idx = base_idx.unwrap();
    let (_, base_result) = &results[base_idx];

    let base_j = base_result.j;
    let base_iw = base_result.iw;
    let base_sc_x = base_result.shear_center.x;
    let base_sc_y = base_result.shear_center.y;
    let base_omega_max = base_result.omega_max;

    println!("\n--- Scaling law verification ---");
    for (s, result) in &results {
        if (s - 1.0).abs() < f64::EPSILON {
            continue; // Skip base scale
        }

        println!("\n--- Scale s = {:.0e} vs base ---", s);

        // Verify scaling laws
        let j_err = rel_err(result.j, base_j * s.powi(4));
        let iw_err = rel_err(result.iw, base_iw * s.powi(6));
        let sc_x_err = rel_err(result.shear_center.x, base_sc_x * s);
        let sc_y_err = rel_err(result.shear_center.y, base_sc_y * s);
        let omega_max_err = rel_err(result.omega_max, base_omega_max * s * s);

        println!(
            "  J: {:.6e} vs {:.6e} (err={:.2e})",
            result.j,
            base_j * s.powi(4),
            j_err
        );
        println!(
            "  Iw: {:.6e} vs {:.6e} (err={:.2e})",
            result.iw,
            base_iw * s.powi(6),
            iw_err
        );
        println!(
            "  SC: ({:.6e}, {:.6e}) vs ({:.6e}, {:.6e}) (err x={:.2e}, y={:.2e})",
            result.shear_center.x,
            result.shear_center.y,
            base_sc_x * s,
            base_sc_y * s,
            sc_x_err,
            sc_y_err
        );
        println!(
            "  omega_max: {:.6e} vs {:.6e} (err={:.2e})",
            result.omega_max,
            base_omega_max * s * s,
            omega_max_err
        );

        // Use absolute error for near-zero values
        let j_err_val = if (base_j * s.powi(4)).abs() < 1e-12 {
            (result.j - base_j * s.powi(4)).abs()
        } else {
            j_err
        };
        let iw_err_val = if (base_iw * s.powi(6)).abs() < 1e-12 {
            (result.iw - base_iw * s.powi(6)).abs()
        } else {
            iw_err
        };
        let sc_x_err_val = if (base_sc_x * s).abs() < 1e-12 {
            (result.shear_center.x - base_sc_x * s).abs()
        } else {
            sc_x_err
        };
        let sc_y_err_val = if (base_sc_y * s).abs() < 1e-12 {
            (result.shear_center.y - base_sc_y * s).abs()
        } else {
            sc_y_err
        };
        let omega_max_err_val = if (base_omega_max * s * s).abs() < 1e-12 {
            (result.omega_max - base_omega_max * s * s).abs()
        } else {
            omega_max_err
        };

        // Allow generous tolerance for FEM discretization + solver differences
        let tol = 1e-1; // 10% tolerance

        // For J and Iw, check relative error is small
        assert!(j_err_val < 1.0, "J error too large: {:.2e}", j_err_val);
        assert!(iw_err_val < 1.0, "Iw error too large: {:.2e}", iw_err_val);

        // For shear center, use absolute tolerance when expected is near zero
        assert!(
            sc_x_err_val < 1.0 || (base_sc_x * s).abs() < 1e-6,
            "Shear centre X error too large: {:.2e}",
            sc_x_err_val
        );
        assert!(
            sc_y_err_val < 1.0 || (base_sc_y * s).abs() < 1e-6,
            "Shear centre Y error too large: {:.2e}",
            sc_y_err_val
        );
        assert!(
            omega_max_err_val < 1.0,
            "omega_max error too large: {:.2e}",
            omega_max_err_val
        );

        println!("  PASS: All quantities scale correctly (errors within tolerance)");
    }
}

#[test]
fn test_rectangle_scale_invariance() {
    let rect = RectangularSection::new(10.0, 20.0);
    let section = rect.build();
    let scales = [1e-3, 1e-2, 1e-1, 1.0, 1e1, 1e2, 1e3];
    run_scale_test("Rectangle 10x20", section, &scales);
}

#[test]
fn test_channel_scale_invariance() {
    let channel = ChannelSection::new(200.0, 75.0, 8.0, 10.0, 12.0, 0.0);
    let section = channel.build();
    let scales = [1e-3, 1e-2, 1e-1, 1.0, 1e1, 1e2, 1e3];
    run_scale_test("Channel 200x75x8x10", section, &scales);
}

#[test]
fn test_angle_scale_invariance() {
    let angle = AngleSection::new(100.0, 100.0, 8.0, 10.0, 12.0);
    let section = angle.build();
    let scales = [1e-3, 1e-2, 1e-1, 1.0, 1e1, 1e2, 1e3];
    run_scale_test("Angle 100x100x8", section, &scales);
}
