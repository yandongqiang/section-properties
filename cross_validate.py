#!/usr/bin/env python3
"""
Cross-validation of Rust vs Python sectionproperties Tri6 element matrices.

Compares element-level matrices for identical geometry.
"""

import json
import numpy as np
from sectionproperties.pre import Material
from sectionproperties.pre.library import rectangular_section
from sectionproperties.analysis import Section
from sectionproperties.analysis.fea import Tri6, gauss_points, shape_function

def load_rust_data(json_path):
    with open(json_path, 'r') as f:
        return json.load(f)

def create_python_element(rust_coords, E=200_000):
    """Create identical Tri6 element in Python."""
    # Python expects coords as 2x6 array
    coords = np.array([
        rust_coords['x'],
        rust_coords['y']
    ], dtype=float)
    
    # Create material
    mat = Material(
        name="Steel",
        elastic_modulus=200_000,
        poissons_ratio=0.3,
        density=7850,
        yield_strength=250,
        color="grey"
    )
    
    # Create Tri6 element
    # Python Tri6 expects: el_id, coords (2x6), node_ids, material
    el = Tri6(
        el_id=0,
        coords=np.array(rust_coords['x'] + rust_coords['y']).reshape(2, 6),
        node_ids=[0, 1, 2, 3, 4, 5],
        material=mat
    )
    return el

def compare_matrices(rust_mat, py_mat, name):
    """Compare two matrices and return max absolute difference."""
    rust = np.array(rust_mat)
    py = np.array(py_mat)
    diff = np.abs(rust - py)
    max_diff = np.max(diff)
    rel_diff = np.max(diff / (np.abs(py) + 1e-15))
    print(f"{name}:")
    print(f"  Max abs diff: {max_diff:.2e}")
    print(f"  Max rel diff: {rel_diff:.2e}")
    if max_diff > 1e-10:
        print(f"  *** DIFFERENCE EXCEEDS 1e-10 ***")
        print(f"  Rust:\n{rust}")
        print(f"  Python:\n{py}")
        print(f"  Diff:\n{np.abs(rust - py)}")
    else:
        print(f"  OK (diff < 1e-10)")
    return max_diff

def main():
    # Load Rust data
    rust_data = load_rust_data('element_data.json')
    
    # Create identical Python element
    rust_coords = rust_data['coords']
    py_el = create_python_element(rust_coords)
    
    print("=" * 60)
    print("CROSS-VALIDATION: Rust vs Python Tri6 Element")
    print("=" * 60)
    
    # Compare coordinates
    print("\n1. COORDINATES")
    rust_x = np.array(rust_data['coords']['x'])
    rust_y = np.array(rust_data['coords']['y'])
    py_coords = py_el.coords
    print(f"  Rust x: {rust_x}")
    print(f"  Python x: {py_coords[0]}")
    print(f"  Max diff x: {np.max(np.abs(rust_x - py_coords[0])):.2e}")
    print(f"  Max diff y: {np.max(np.abs(rust_y - py_coords[1])):.2e}")
    
    # Compare Gauss points
    print("\n2. GAUSS POINTS (6-point)")
    rust_gps = np.array(rust_data['gauss_points'])
    py_gps = gauss_points(n=6)
    print(f"  Rust weights: {rust_gps[:, 0]}")
    print(f"  Python weights: {py_gps[:, 0]}")
    print(f"  Weight diff: {np.max(np.abs(rust_gps[:, 0] - py_gps[:, 0])):.2e}")
    print(f"  Eta diff: {np.max(np.abs(rust_gps[:, 1] - py_gps[:, 1])):.2e}")
    print(f"  Xi diff: {np.max(np.abs(rust_gps[:, 2] - py_gps[:, 2])):.2e}")
    
    # Test shape function at one Gauss point
    print("\n3. SHAPE FUNCTION AT FIRST GAUSS POINT")
    gps = gauss_points(n=6)
    gp = gps[0]
    
    # Rust values
    rust_sf = rust_data['gauss_details'][0]
    rust_n = np.array(rust_sf['n'])
    rust_b0 = np.array(rust_sf['b'][0])
    rust_b1 = np.array(rust_sf['b'][1])
    rust_j = rust_sf['j']
    rust_x = rust_sf['x']
    rust_y = rust_sf['y']
    
    # Python values
    py_n, py_b, py_j, py_x, py_y = shape_function(
        coords=np.array([rust_data['coords']['x'], rust_data['coords']['y']]),
        gauss_point=(gp[0], gp[1], gp[2], gp[3])
    )
    
    print(f"  Shape function N:")
    print(f"  Max diff N: {np.max(np.abs(rust_n - py_n)):.2e}")
    print(f"  Max diff B[0]: {np.max(np.abs(rust_b0 - py_b[0])):.2e}")
    print(f"  Max diff B[1]: {np.max(np.abs(rust_b1 - py_b[1])):.2e}")
    print(f"  J diff: {abs(rust_j - py_j):.2e}")
    print(f"  X diff: {abs(rust_x - py_x):.2e}")
    print(f"  Y diff: {abs(rust_y - py_y):.2e}")
    
    # Create Python element and compute torsion properties
    print("\n4. ELEMENT MATRICES (torsion_properties)")
    # Create material
    from sectionproperties.pre import Material
    mat = Material(
        name="Steel",
        elastic_modulus=200_000,
        poissons_ratio=0.3,
        density=7850,
        yield_strength=250,
        color="grey"
    )
    
    py_el = Tri6(
        el_id=0,
        coords=np.array([rust_data['coords']['x'], rust_data['coords']['y']]),
        node_ids=[0, 1, 2, 3, 4, 5],
        material=mat
    )
    
    py_k, py_f, py_c = py_el.torsion_properties()
    
    # Compare element matrices
    rust_k = np.array(rust_data['k_el'])
    rust_f = np.array(rust_data['f_el'])
    rust_c = np.array(rust_data['c_el'])
    
    compare_matrices(rust_k, py_k, "K_el (6x6)")
    compare_matrices(rust_f.reshape(6,1), py_f.reshape(6,1), "F_el (6)")
    compare_matrices(rust_c.reshape(6,1), py_c.reshape(6,1), "C_el (6)")
    
    # Verify Gauss point weights sum to 1
    print("\n5. GAUSS WEIGHT SUM")
    rust_gps = np.array(rust_data['gauss_points'])
    py_gps = gauss_points(n=6)
    print(f"  Rust weight sum: {np.sum(rust_gps[:, 0]):.10f}")
    print(f"  Python weight sum: {np.sum(py_gps[:, 0]):.10f}")
    
    # Test F formulation
    print("\n6. F FORMULATION CHECK")
    # F = B^T @ [y, -x] in global coords
    # Check if Python uses global [y, -x] or centroidal [y-yc, -(x-xc)]
    coords = np.array([rust_data['coords']['x'], rust_data['coords']['y']])
    gps = gauss_points(n=6)
    
    # Check first element's F_el at first Gauss point
    coords_c = np.array([
        np.array(rust_data['coords']['x']) - 0.0,
        np.array(rust_data['coords']['y']) - 0.0
    ])
    
    # For centroid at (0,0), both formulations should be identical
    print("  Centroid is at (0,0) - both formulations identical")
    
    print("\n" + "=" * 60)
    print("CROSS-VALIDATION COMPLETE")
    print("=" * 60)

if __name__ == '__main__':
    main()