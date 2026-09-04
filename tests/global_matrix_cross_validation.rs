//! Cross-validation of global K, F, C matrices: Rust vs Python
//!
//! Runs Rust's FEM directly on Python's full 6-node mesh (read from
//! `python_global_<section>.json`), then compares the assembled global
//! stiffness K, load F and constraint C entries against Python's exported
//! matrices — same DOF count, same connectivity, so an index-level comparison
//! is meaningful.
//!
//! Run with: cargo test --test global_matrix_cross_validation -- --nocapture

use std::io::Read;

fn load_json(path: &str) -> serde_json::Value {
    let mut file = std::fs::File::open(path).expect("Failed to open file");
    let mut contents = String::new();
    file.read_to_string(&mut contents).expect("Failed to read file");
    serde_json::from_str(&contents).expect("Failed to parse JSON")
}

fn compare_coo_vecs(
    rust_data: &serde_json::Value,
    py_data: &serde_json::Value,
    label: &str,
) {
    let rust_row: Vec<usize> = serde_json::from_value(rust_data["K"]["row"].clone()).unwrap();
    let rust_col: Vec<usize> = serde_json::from_value(rust_data["K"]["col"].clone()).unwrap();
    let rust_vals: Vec<f64> = serde_json::from_value(rust_data["K"]["data"].clone()).unwrap();
    let py_row: Vec<usize> = serde_json::from_value(py_data["K"]["row"].clone()).unwrap();
    let py_col: Vec<usize> = serde_json::from_value(py_data["K"]["col"].clone()).unwrap();
    let py_vals: Vec<f64> = serde_json::from_value(py_data["K"]["data"].clone()).unwrap();

    // Build value maps keyed by (row, col) so sparse ordering (CSR vs CSC)
    // does not affect the comparison.
    use std::collections::HashMap;
    let rust_map: HashMap<(usize, usize), f64> =
        rust_row.iter().zip(rust_col.iter()).zip(rust_vals.iter())
            .map(|((&r, &c), &v)| ((r, c), v))
            .collect();
    let py_map: HashMap<(usize, usize), f64> =
        py_row.iter().zip(py_col.iter()).zip(py_vals.iter())
            .map(|((&r, &c), &v)| ((r, c), v))
            .collect();

    // Union of keys; treat missing side as zero.
    let mut keys: Vec<(usize, usize)> = rust_map.keys().copied().collect();
    for k in py_map.keys() {
        if !rust_map.contains_key(k) {
            keys.push(*k);
        }
    }
    keys.sort_unstable();
    keys.dedup();

    let mut max_abs = 0.0_f64;
    let mut max_rel = 0.0_f64;
    let mut n_over_tol = 0usize;
    let mut n_both_diff_sign = 0usize;
    for k in &keys {
        let rv = rust_map.get(k).copied().unwrap_or(0.0);
        let pv = py_map.get(k).copied().unwrap_or(0.0);
        let diff = (rv - pv).abs();
        max_abs = max_abs.max(diff);
        let denom = rv.abs().max(pv.abs());
        if denom > 1e-15 {
            max_rel = max_rel.max(diff / denom);
        }
        if diff > 1e-8 {
            n_over_tol += 1;
        }
        if rv.signum() != pv.signum() && rv.abs() > 1e-8 && pv.abs() > 1e-8 {
            n_both_diff_sign += 1;
        }
    }
    println!(
        "{}: K keys={}, rust_nnz={}, py_nnz={}, max_abs_diff={:.2e}, \
         max_rel_diff={:.2e}, entries>1e-8: {}, opposite_sign: {}",
        label,
        keys.len(),
        rust_map.len(),
        py_map.len(),
        max_abs,
        max_rel,
        n_over_tol,
        n_both_diff_sign
    );
}

fn compare_vecs(rust_vec: &[f64], py_vec: &[f64], label: &str) {
    assert_eq!(
        rust_vec.len(),
        py_vec.len(),
        "{}: length mismatch: rust={}, py={}",
        label,
        rust_vec.len(),
        py_vec.len()
    );
    let mut max_abs = 0.0_f64;
    let mut max_rel = 0.0_f64;
    let mut n_over_tol = 0usize;
    for i in 0..rust_vec.len() {
        let diff = (rust_vec[i] - py_vec[i]).abs();
        max_abs = max_abs.max(diff);
        let denom = rust_vec[i].abs().max(py_vec[i].abs());
        if denom > 1e-15 {
            max_rel = max_rel.max(diff / denom);
        }
        if diff > 1e-8 {
            n_over_tol += 1;
        }
    }
    println!(
        "{}: len={}, max_abs_diff={:.2e}, max_rel_diff={:.2e}, entries>1e-8: {}",
        label,
        rust_vec.len(),
        max_abs,
        max_rel,
        n_over_tol
    );
}

fn test_section(section_name: &str) {
    let tmp_out = format!(
        "{}/tmp_rust_on_py_tri6_{}.json",
        std::env::temp_dir().to_string_lossy(),
        section_name
    );

    section_properties::plastic::warping_fem::run_fem_on_python_tri6_mesh(
        &format!("python_global_{}.json", section_name),
        section_name,
        &tmp_out,
    )
    .expect("run_fem_on_python_tri6_mesh failed");

    println!("\n=== {} ===", section_name);
    let rust_data = load_json(&tmp_out);
    let py_data = load_json(&format!("python_global_{}.json", section_name));

    let rust_n = rust_data["n_dof"].as_u64().unwrap() as usize;
    let py_n = py_data["n_dof"].as_u64().unwrap() as usize;
    println!(
        "  n_dof: rust={}, py={}{}",
        rust_n,
        py_n,
        if rust_n == py_n { "" } else { "  <-- MISMATCH!" }
    );

    let rust_F: Vec<f64> = serde_json::from_value(rust_data["F"].clone()).unwrap();
    let py_F: Vec<f64> = serde_json::from_value(py_data["F"].clone()).unwrap();
    let rust_C: Vec<f64> = serde_json::from_value(rust_data["C"].clone()).unwrap();
    let py_C: Vec<f64> = serde_json::from_value(py_data["C"].clone()).unwrap();

    compare_coo_vecs(&rust_data, &py_data, &format!("{} K", section_name));
    compare_vecs(&rust_F, &py_F, &format!("{} F", section_name));
    compare_vecs(&rust_C, &py_C, &format!("{} C", section_name));

    std::fs::copy(&tmp_out, "keep_rust_k_Channel.json").ok();
    let _ = std::fs::remove_file(&tmp_out);
}

#[test]
fn global_matrix_cross_validation() {
    println!("=== GLOBAL MATRIX CROSS-VALIDATION (Rust on Python 6-node mesh) ===");

    test_section("Channel_200x75");
    test_section("I_section_300x150");
    test_section("Angle_100x100");
    test_section("Thin_Channel_300x100x3");

    println!("\n=== CROSS-VALIDATION COMPLETE ===");
}
