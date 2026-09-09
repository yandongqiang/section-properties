#!/usr/bin/env python3
"""Independent solver validation using numpy/scipy"""
import json
import numpy as np
from scipy.sparse import coo_matrix, csc_matrix
from scipy.sparse.linalg import spsolve
import sys

def load_json(path):
    with open(path, 'r') as f:
        return json.load(f)

def solve_with_numpy(data, label, has_geometry=True):
    """Solve using numpy/scipy"""
    A_data = data["A"]
    b = np.array(data["b"])
    
    # Build sparse matrix
    A = coo_matrix((A_data["data"], (A_data["row"], A_data["col"])), shape=A_data["shape"]).tocsc()
    
    print(f"\n{label} - Matrix info:")
    print(f"  Shape: {A.shape}")
    print(f"  nnz: {A.nnz}")
    
    # Check matrix properties
    A_dense = A.toarray()
    print(f"  Symmetry max abs diff: {np.abs(A_dense - A_dense.T).max():.2e}")
    print(f"  Diagonal min: {np.diag(A_dense).min():.2e}")
    print(f"  Diagonal max: {np.diag(A_dense).max():.2e}")
    
    # Solve
    try:
        x = spsolve(A, b)
        print(f"  Solve: SUCCESS")
        
        n = data["n_dof"]
        omega = x[:n]
        lam = x[n]
        
        # Compute residuals
        Ax = A @ x
        res = np.abs(Ax - b).max()
        res_rel = res / np.abs(b).max()
        
        omega_dot_f = np.dot(omega, data["F"])
        if has_geometry:
            ixx_c = data["geometry"]["ixx_c"]
            iyy_c = data["geometry"]["iyy_c"]
        else:
            # For Rust data, use the known values from Python
            ixx_c = data.get("ixx_c", 0)
            iyy_c = data.get("iyy_c", 0)
        ixx_plus_iyy = ixx_c + iyy_c
        J = ixx_plus_iyy - omega_dot_f
        
        print(f"  lambda: {lam:.6e}")
        print(f"  max|Ax-b|: {res:.2e}")
        print(f"  rel residual: {res_rel:.2e}")
        print(f"  omega^T F: {omega_dot_f:.6e}")
        print(f"  Ixx+Iyy: {ixx_plus_iyy:.6e}")
        print(f"  J = {J:.6e}")
        print(f"  |C^T omega|: {np.abs(np.dot(data.get('C', []), omega)):.2e}")
        
        return omega, lam, J, res_rel
    except Exception as e:
        print(f"  Solve: FAILED - {e}")
        return None, None, None, None

def main():
    sections = [
        "Channel_200x75",
        "I_section_300x150",
        "Angle_100x100",
        "Thin_Channel_300x100x3"
    ]
    
    # Hardcoded geometry data for Rust (from Python)
    geo_data = {
        "Channel_200x75": {"ixx_c": 1.946027e7, "iyy_c": 0.0},  # actually we need both
    }
    
    # We'll just use Python's geometry for both
    results = {}
    
    for sec in ["Channel_200x75", "I_section_300x150", "Angle_100x100", "Thin_Channel_300x100x3"]:
        print(f"\n{'='*60}")
        print(f"Section: {sec}")
        print(f"{'='*60}")
        
        # Load Python data
        py_data = load_json(f"python_global_{sec}.json")
        ixx_c = py_data["geometry"]["ixx_c"]
        iyy_c = py_data["geometry"]["iyy_c"]
        ixx_plus_iyy = ixx_c + iyy_c
        
        # Load Rust data
        rust_data = load_json(f"rust_augmented_{sec}.json")
        
        # Solve with Python data
        print("\n--- Python matrices ---")
        py_data = load_json(f"python_global_{sec}.json")
        py_omega, py_lam, py_J, py_res = solve_with_numpy(py_data, f"Python {sec}", True)
        
        # Add geometry to Rust data for solving
        rust_data["geometry"] = {"ixx_c": ixx_c, "iyy_c": iyy_c}
        
        # Solve with Rust data
        print("\n--- Rust matrices ---")
        rust_omega, rust_lam, rust_J, rust_res = solve_with_numpy(rust_data, f"Rust {sec}", True)
        
        if py_omega is not None and rust_omega is not None:
            # Compare solutions
            omega_diff = py_omega - rust_omega
            max_abs_diff = np.abs(omega_diff).max()
            max_rel_diff = np.abs(omega_diff / np.maximum(np.abs(py_omega), np.abs(rust_omega))).max()
            
            J_diff = abs(py_J - rust_J)
            J_rel_diff = J_diff / max(abs(py_J), abs(rust_J))
            
            print(f"\n--- Comparison ---")
            print(f"  omega max_abs_diff: {max_abs_diff:.2e}")
            print(f"  omega max_rel_diff: {max_rel_diff:.2e}")
            print(f"  lambda diff: {abs(py_lam - rust_lam):.2e}")
            print(f"  J diff: {J_diff:.2e}")
            print(f"  J rel diff: {J_rel_diff:.2e}")
            
            results[sec] = {
                "py_J": py_J, "rust_J": rust_J,
                "py_res": py_res, "rust_res": rust_res,
                "omega_diff": max_abs_diff, "omega_rel_diff": max_rel_diff
            }
    
    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")
    for sec, r in results.items():
        print(f"{sec}: J_py={r['py_J']:.6e}, J_rust={r['rust_J']:.6e}, "
              f"J_rel_err={abs(r['py_J']-r['rust_J'])/r['py_J']:.2e}, "
              f"omega_diff={r['omega_diff']:.2e}")

if __name__ == "__main__":
    main()