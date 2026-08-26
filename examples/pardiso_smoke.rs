#![cfg(feature = "pardiso")]
use section_properties::fea::{SparseMatrix, DirectLagrangeSolver};
fn main() {
    eprintln!("start");
    let n = 20;
    let mut k = SparseMatrix::new(n);
    for i in 0..n {
        k.add(i, i, 2.0);
        if i + 1 < n {
            k.add(i, i + 1, -1.0);
            k.add(i + 1, i, -1.0);
        }
    }
    let c = vec![1.0f64; n];
    let f: Vec<f64> = (0..n).map(|i| i as f64 * 0.05).collect();
    let mut s = match section_properties::fea::solvers::pardiso::PardisoSolver::new(&k, &c) {
        Ok(s) => { eprintln!("init ok"); s }
        Err(e) => { eprintln!("init err: {e}"); return; }
    };
    match s.solve_direct_lagrange(&f) {
        Ok(u) => eprintln!("solved, u0={}", u[0]),
        Err(e) => eprintln!("solve err: {e}"),
    }
}
