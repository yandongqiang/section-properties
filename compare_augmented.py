#!/usr/bin/env python3
"""Compare Rust vs Python augmented matrices"""
import json
import numpy as np
from scipy.sparse import coo_matrix, csc_matrix

def load_json(path):
    with open(path, 'r') as f:
        return json.load(f)

def compare_matrices(rust_data, py_data, label):
    """Compare two sparse matrices given as COO format dicts"""
    rust_A = rust_data["A"]
    py_A = py_data["A"]
    
    # Build sparse matrices
    rust_mat = coo_matrix((rust_A["data"], (rust_A["row"], rust_A["col"])), shape=rust_A["shape"])
    py_mat = coo_matrix((py_A["data"], (py_A["row"], py_A["col"])), shape=py_A["shape"])
    
    rust_csc = rust_mat.tocsc()
    py_csc = py_mat.tocsc()
    
    # Convert to dense for comparison (these are small enough)
    rust_dense = rust_csc.toarray()
    py_dense = py_csc.toarray()
    
    diff = rust_dense - py_dense
    max_abs = np.abs(diff).max()
    max_rel = np.abs(diff / np.maximum(np.abs(rust_dense), np.abs(py_dense))).max()
    
    print(f"\n{label}:")
    print(f"  Shape: {rust_dense.shape}")
    print(f"  Max abs diff: {max_abs:.2e}")
    print(f"  Max rel diff: {max_rel:.2e}")
    print(f"  Rust nnz: {rust_mat.nnz}, Python nnz: {py_mat.nnz}")
    
    # Check specific blocks
    n = rust_data["n_dof"]
    
    # K block
    rust_K = rust_dense[:n, :n]
    py_K = py_dense[:n, :n]
    K_diff = rust_K - py_K
    print(f"  K block: max_abs={np.abs(K_diff).max():.2e}, max_rel={np.abs(K_diff / np.maximum(np.abs(rust_K), np.abs(py_K))).max():.2e}")
    
    # C column (upper right)
    rust_C_ur = rust_dense[:n, n:n+1].flatten()
    py_C_ur = py_dense[:n, n:n+1].flatten()
    C_ur_diff = rust_C_ur - py_C_ur
    print(f"  C upper-right: max_abs={np.abs(C_ur_diff).max():.2e}, max_rel={np.abs(C_ur_diff / np.maximum(np.abs(rust_C_ur), np.abs(py_C_ur))).max():.2e}")
    
    # C^T row (lower left)
    rust_C_ll = rust_dense[n:n+1, :n].flatten()
    py_C_ll = py_dense[n:n+1, :n].flatten()
    C_ll_diff = rust_C_ll - py_C_ll
    print(f"  C^T lower-left: max_abs={np.abs(C_ll_diff).max():.2e}, max_rel={np.abs(C_ll_diff / np.maximum(np.abs(rust_C_ll), np.abs(py_C_ll))).max():.2e}")
    
    # Bottom-right (0)
    rust_br = rust_dense[n, n]
    py_br = py_dense[n, n]
    print(f"  Bottom-right (0): rust={rust_br:.2e}, py={py_br:.2e}, diff={abs(rust_br - py_br):.2e}")
    
    # Symmetry check
    sym_diff = rust_dense - rust_dense.T
    print(f"  Rust symmetry max abs: {np.abs(sym_diff).max():.2e}")
    sym_diff_py = py_dense - py_dense.T
    print(f"  Python symmetry max abs: {np.abs(sym_diff_py).max():.2e}")
    
    # RHS comparison
    rust_b = np.array(rust_data["b"])
    py_b = np.array(py_data["b"])
    b_diff = rust_b - py_b
    print(f"  b: max_abs={np.abs(b_diff).max():.2e}, max_rel={np.abs(b_diff / np.maximum(np.abs(rust_b), np.abs(py_b))).max():.2e}")
    
    return max_abs, max_rel

def main():
    sections = [
        "Channel_200x75",
        "I_section_300x150", 
        "Angle_100x100",
        "Thin_Channel_300x100x3"
    ]
    
    for sec in sections:
        rust = load_json(f"rust_augmented_{sec}.json")
        py = load_json(f"python_global_{sec}.json")
        compare_matrices(rust, py, sec)

if __name__ == "__main__":
    main()