use section_properties::fea::solvers::SparseLu;
use section_properties::fea::SparseMatrix;
use std::fs::File;
use std::io::Read;
use serde_json;

fn load_json(path: &str) -> serde_json::Value {
    let mut file = File::open(path).expect("Failed to open file");
    let mut contents = String::new();
    file.read_to_string(&mut contents).expect("Failed to read file");
    serde_json::from_str(&contents).expect("Failed to parse JSON")
}

fn main() {
    // Load the exact augmented system for Angle_100x100 (the one that fails)
    let data = load_json("rust_augmented_Angle_100x100.json");
    
    let n_dof = data["n_dof"].as_u64().unwrap() as usize;
    let A_data = &data["A"];
    let b: Vec<f64> = serde_json::from_value(data["b"].clone()).unwrap();
    
    // Build sparse matrix
    let mut A = SparseMatrix::new(n_dof + 1);
    let rows: Vec<usize> = serde_json::from_value(A_data["row"].clone()).unwrap();
    let cols: Vec<usize> = serde_json::from_value(A_data["col"].clone()).unwrap();
    let data: Vec<f64> = serde_json::from_value(A_data["data"].clone()).unwrap();
    for ((r, c), v) in rows.into_iter().zip(cols).zip(data) {
        A.add(r, c, v);
    }
    A.compress();
    
    println!("Matrix size: {}x{}, nnz: {}", n_dof + 1, n_dof + 1, A_data["data"].as_array().unwrap().len());
    
    // Try SparseLU
    let lu = SparseLu::factor(&A).unwrap();
    
    // Verify PA = LU
    let max_diff = lu.verify_pa_eq_lu(&A);
    println!("max|PA - LU| = {:.2e}", max_diff);
    
    let x = lu.solve(&b);
    
    println!("Solution x (first 10): {:?}", &x[..10.min(x.len())]);
    println!("x len: {}", x.len());
    println!("Last element (lambda): {}", x.last().unwrap());
    
    // Check if PA = LU holds
    let max_diff = lu.verify_pa_eq_lu(&A);
    println!("max|PA - LU| = {:.2e}", max_diff);
    
    // Compute residual
    let n = A.n;
    let mut res = vec![0.0; n];
    let (rows, cols, vals) = A.triplets();
    for (&i, (&j, &v)) in rows.iter().zip(cols.iter().zip(vals.iter())) {
        res[i] += v * x[j];
    }
    
    let max_res = res.iter().zip(b.iter()).map(|(&r, &b)| (r - b).abs()).fold(0.0, f64::max);
    let b_norm = b.iter().map(|&v| v.abs()).fold(0.0, f64::max);
    println!("Max residual: {:.2e}, rel: {:.2e}", max_res, max_res / b_norm.max(1e-300));
    
    // Check constraint residual C^T * omega
    let c_data: Vec<f64> = serde_json::from_value(load_json("rust_augmented_Angle_100x100.json")["C"].clone()).unwrap();
    let omega = &x[..x.len()-1];
    let ct_omega: f64 = c_data.iter().zip(x.iter().take(n_dof)).map(|(&c, &w)| c * w).sum();
    println!("C^T * omega: {:.2e}", ct_omega);
    println!("Lambda: {:.2e}", x.last().unwrap());
}