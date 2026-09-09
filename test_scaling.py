#!/usr/bin/env python3
"""Test row/column scaling for the augmented system"""
import json
import numpy as np
from scipy.sparse import coo_matrix, csc_matrix
from scipy.sparse.linalg import spsolve
from scipy.sparse import diags

def load_json(path):
    with open(path, 'r') as f:
        return json.load(f)

def solve_scaled(data, label):
    """Solve with row/column scaling"""
    A_data = data["A"]
    b = np.array(data["b"])
    
    # Build sparse matrix
    A = coo_matrix((A_data["data"], (A_data["row"], A_data["col"])), shape=A_data["shape"]).tocsc()
    
    print(f"\n{label} - Testing scaling...")
    n = A.shape[0]
    
    # Method 1: Row scaling (equilibrate rows)
    row_sums = np.abs(A.toarray()).sum(axis=1)
    row_sums[row_sums == 0] = 1.0
    D_r = diags(1.0 / np.sqrt(row_sums))
    
    # Method 2: Column scaling
    col_sums = np.abs(A.toarray()).sum(axis=0)
    col_sums[col_sums == 0] = 1.0
    D_c = diags(1.0 / np.sqrt(col_sums))
    
    # Method 3: Both
    A_scaled = D_r @ A @ D_c
    b_scaled = D_r @ b
    
    print(f"  Original: cond_est={np.linalg.cond(A.toarray()):.2e}, diag_min={np.diag(A.toarray()).min():.2e}")
    print(f"  Scaled: cond_est={np.linalg.cond(A_scaled.toarray()):.2e}, diag_min={np.diag(A_scaled.toarray()).min():.2e}")
    
    # Solve scaled
    try:
        x_scaled = spsolve(A_scaled, b_scaled)
        x = D_c @ x_scaled
        print(f"  Scaled solve: SUCCESS")
        
        res = np.abs(A @ x - b).max()
        res_rel = res / np.abs(b).max()
        print(f"  Residual: {res:.2e}, rel: {res_rel:.2e}")
        return x
    except Exception as e:
        print(f"  Scaled solve: FAILED - {e}")
        return None

def main():
    sections = ["Angle_100x100"]  # This is the one that fails
    for sec in sections:
        print(f"\n{'='*60}")
        print(f"Section: {sec}")
        print(f"{'='*60}")
        
        py_data = load_json(f"python_global_{sec}.json")
        rust_data = load_json(f"rust_augmented_{sec}.json")
        
        print("\n--- Python matrices ---")
        solve_scaled(py_data, f"Python {sec}")
        
        print("\n--- Rust matrices ---")
        solve_scaled(rust_data, f"Rust {sec}")

if __name__ == "__main__":
    main()