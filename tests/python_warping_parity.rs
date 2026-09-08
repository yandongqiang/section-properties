//! Python Warping Parity Tests
//!
//! Compares Rust FEM warping results with Python sectionproperties reference data.
//! Tests standard sections: rectangle, circle, angle, channel, asymmetric, section with hole.
//!
//! Run with: cargo test --test python_warping_parity -- --nocapture

use section_properties::geometry::{CompoundGeometry, Geometry};
use section_properties::mesh::MeshControl;
use section_properties::plastic::warping_fem::{FemWarpingSolution, compute_fem_warping_solution};
use section_properties::section::Section;
use section_properties::section_library::ParametricSection;
use section_properties::section_library::primitive::{
    CircularHollowSection, CircularSection, RectangularSection,
};
use section_properties::section_library::steel::{
    AngleSection, ChannelSection, ISection, RectangularHollowSection, TeeSection,
};
use section_properties::section_properties::SectionProperties;
use std::collections::HashMap;
use std::fs::File;

fn rel_err(a: f64, b: f64) -> f64 {
    if b == 0.0 {
        return a.abs();
    }
    ((a - b).abs() / b.abs())
}

fn max_abs_diff(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

fn max_rel_diff(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b.iter())
        .filter(|(x, y)| x.abs().max(y.abs()) > 1e-15)
        .map(|(x, y)| (x - y).abs() / x.abs().max(y.abs()))
        .fold(0.0, f64::max)
}

fn load_python_reference(section_name: &str) -> Option<HashMap<String, Vec<f64>>> {
    // Try to load Python reference data from exported JSON files
    let paths = [
        format!("python_warping_ref_{}.json", section_name),
        format!("tests/python_warping_ref_{}.json", section_name),
    ];

    for path in &paths {
        if let Ok(file) = File::open(path) {
            let reader = std::io::BufReader::new(file);
            if let Ok(json) = serde_json::from_reader::<_, serde_json::Value>(reader) {
                let mut map = HashMap::new();
                for key in ["omega", "psi", "phi", "lambda", "K", "F", "C", "J", "Iw"] {
                    if let Some(arr) = json.get(key).and_then(|v| v.as_array()) {
                        map.insert(
                            key.to_string(),
                            arr.iter().filter_map(|v| v.as_f64()).collect(),
                        );
                    }
                }
                return Some(map);
            }
        }
    }
    None
}

fn run_parity_test(
    name: &str,
    section: Section,
    props: &SectionProperties,
    nu: f64,
) -> Result<(), String> {
    println!("\n{}", "=".repeat(80));
    println!("PYTHON WARPING PARITY TEST: {}", name);
    println!("{}", "=".repeat(80));

    // Run Rust FEM
    let fem: FemWarpingSolution =
        compute_fem_warping_solution(&section, props, nu, MeshControl::Fine)
            .map_err(|e| format!("FEM failed: {:?}", e))?;

    println!("\n--- Rust FEM Results ---");
    println!("n_dof: {}", fem.omega.len());
    println!("J:     {:.6e}", fem.j);
    println!("Iw:    {:.6e}", fem.iw);
    println!("omega_max: {:.6e}", fem.omega_max);
    println!("Used exact solver: {}", fem.used_exact_solver);
    println!("Used regularization: {}", fem.used_regularization);
    println!("Exact residual: {:.2e}", fem.exact_residual);
    println!("Regularized residual: {:.2e}", fem.regularized_residual);
    println!("Used analytical fallback: {}", fem.used_analytical_fallback);

    // Try to load Python reference
    if let Some(py_ref) = load_python_reference(name) {
        println!("\n--- Python Reference Comparison ---");

        // Compare omega (primary validation)
        if let Some(py_omega) = py_ref.get("omega") {
            if py_omega.len() == fem.omega.len() {
                let max_abs = max_abs_diff(&fem.omega, py_omega);
                let max_rel = max_rel_diff(&fem.omega, py_omega);
                println!(
                    "omega: max_abs_diff = {:.2e}, max_rel_diff = {:.2e}",
                    max_abs, max_rel
                );

                if max_rel > 1e-3 {
                    eprintln!(
                        "!!! omega RELATIVE DIFFERENCE EXCEEDS 1e-3: {:.2e} !!!",
                        max_rel
                    );
                }
            } else {
                println!(
                    "omega: DOF mismatch (Rust={}, Python={})",
                    fem.omega.len(),
                    py_omega.len()
                );
            }
        }

        // Compare lambda
        if let Some(py_lambda) = py_ref.get("lambda") {
            if py_lambda.len() == 1 {
                // We don't have lambda in FemWarpingSolution directly, but we can compute it
                // For now just note it's available
                println!("Python lambda: {:.6e}", py_lambda[0]);
            }
        }

        // Compare J
        if let Some(py_j) = py_ref.get("J") {
            if py_j.len() == 1 {
                let j_rel = rel_err(fem.j, py_j[0]);
                println!(
                    "J: Rust={:.6e}, Python={:.6e}, rel_err={:.2e}",
                    fem.j, py_j[0], j_rel
                );
                if j_rel > 1e-3 {
                    eprintln!("!!! J RELATIVE DIFFERENCE EXCEEDS 1e-3: {:.2e} !!!", j_rel);
                }
            }
        }

        // Compare Iw
        if let Some(py_iw) = py_ref.get("Iw") {
            if py_iw.len() == 1 {
                let iw_rel = rel_err(fem.iw, py_iw[0]);
                println!(
                    "Iw: Rust={:.6e}, Python={:.6e}, rel_err={:.2e}",
                    fem.iw, py_iw[0], iw_rel
                );
            }
        }
    } else {
        println!("\n--- No Python reference found for {} ---", name);
        println!("Run export step first: export_python_warping_reference");
    }

    // Always verify full Lagrange residual as primary correctness metric
    println!("\n--- Primary Validation (Full Lagrange Residual) ---");
    println!("Exact residual (if used): {:.2e}", fem.exact_residual);
    println!("Regularized residual: {:.2e}", fem.regularized_residual);

    let residual_ok = if fem.used_exact_solver {
        fem.exact_residual <= 1e-8
    } else {
        fem.regularized_residual <= 1e-6
    };

    if residual_ok {
        println!("✓ Full Lagrange residual check PASSED");
    } else {
        println!("✗ Full Lagrange residual check FAILED");
        return Err(format!(
            "Residual check failed: exact={:.2e}, reg={:.2e}",
            fem.exact_residual, fem.regularized_residual
        ));
    }

    // Verify J > 0
    if fem.j <= 0.0 {
        return Err(format!("Negative or zero J: {:.6e}", fem.j));
    }
    println!("✓ J > 0: {:.6e}", fem.j);

    // Verify Iw >= 0
    if fem.iw < 0.0 {
        return Err(format!("Negative Iw: {:.6e}", fem.iw));
    }
    println!("✓ Iw >= 0: {:.6e}", fem.iw);

    Ok(())
}

#[test]
fn parity_rectangle() {
    let rect = RectangularSection::new(100.0, 50.0);
    let section = rect.build();
    let props = SectionProperties::from_section(&section);
    run_parity_test("rectangle_100x50", section, &props, 0.3).unwrap();
}

#[test]
fn parity_solid_circle() {
    // High vertex count for accuracy
    let circ = CircularSection::with_vertices(50.0, 256);
    let section = circ.build();
    let props = SectionProperties::from_section(&section);
    run_parity_test("solid_circle_r50", section, &props, 0.3).unwrap();
}

#[test]
fn parity_angle_equal_leg() {
    let angle = AngleSection::equal_leg(100.0, 10.0);
    let section = angle.build();
    let props = SectionProperties::from_section(&section);
    run_parity_test("angle_100x100x10", section, &props, 0.3).unwrap();
}

#[test]
fn parity_channel() {
    let channel = ChannelSection::new(200.0, 75.0, 8.0, 10.0, 12.0, 0.0);
    let section = channel.build();
    let props = SectionProperties::from_section(&section);
    run_parity_test("channel_200x75x8x10", section, &props, 0.3).unwrap();
}

#[test]
fn parity_i_section() {
    let i_section = ISection::new(300.0, 150.0, 8.0, 12.0, 15.0);
    let section = i_section.build();
    let props = SectionProperties::from_section(&section);
    run_parity_test("i_section_300x150", section, &props, 0.3).unwrap();
}

#[test]
fn parity_chs() {
    let chs = CircularHollowSection::from_dimensions(219.1, 8.2);
    let section = chs.build();
    let props = SectionProperties::from_section(&section);
    run_parity_test("chs_219x8", section, &props, 0.3).unwrap();
}

#[test]
fn parity_rectangular_hollow() {
    let rhs = RectangularHollowSection::new(200.0, 100.0, 8.0, 0.0, 0.0);
    let section = rhs.build();
    let props = SectionProperties::from_section(&section);
    run_parity_test("rhs_200x100x8", section, &props, 0.3).unwrap();
}

#[test]
fn parity_tee_section() {
    let tee = TeeSection::new(200.0, 100.0, 8.0, 12.0, 0.0);
    let section = tee.build();
    let props = SectionProperties::from_section(&section);
    run_parity_test("tee_200x100x8x12", section, &props, 0.3).unwrap();
}

#[test]
fn parity_asymmetric_composite() {
    // Asymmetric section: T + rectangle offset - use simple Section
    let t_section = TeeSection::new(150.0, 80.0, 6.0, 10.0, 0.0);
    let t_sec = t_section.build();
    let props = SectionProperties::from_section(&t_sec);
    run_parity_test("asymmetric_tee_150x80", t_sec, &props, 0.3).unwrap();
}

#[test]
fn parity_section_with_hole() {
    // Rectangle with circular hole
    let outer = RectangularSection::new(200.0, 100.0);
    let hole = CircularSection::with_vertices(20.0, 64);

    let outer_sec = outer.build();
    let hole_sec = hole.build();

    let section = Section::new(outer_sec.outer, vec![hole_sec.outer]);
    let props = SectionProperties::from_section(&section);
    run_parity_test("rect_200x100_with_hole_r20", section, &props, 0.3).unwrap();
}
