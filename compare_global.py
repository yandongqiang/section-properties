#!/usr/bin/env python3
"""
Compare Rust vs Python global warping matrices (K, F, C) assembly.

Uses Python's sectionproperties internal assembly functions to reproduce
the exact same global matrices, then compares with Rust's exported data.
"""

import json
import numpy as np
from scipy.sparse import csc_matrix
from sectionproperties.pre import Material
from sectionproperties.pre.library import channel_section, i_section, angle_section
from sectionproperties.analysis import Section

def load_rust_data(json_path):
    with open(json_path, 'r') as f:
        return json.load(f)

def assemble_python_global(section_name):
    """Assemble global K, F, C using Python's exact assembly logic."""
    # Create the same geometry
    if section_name == "Channel_200x75":
        channel = channel_section(d=200.0, b=75.0, t_w=8.0, t_f=10.0, r=12.0, n_r=10)
        channel.create_mesh(mesh_sizes=[10])
        geom = channel
    elif section_name == "I_section_300x150":
        from sectionproperties.pre.library import i_section
        i_sec = i_section(d=300.0, b=150.0, t_f=8.0, t_w=12.0, r=15.0, n_r=10)
        i_sec.create_mesh(mesh_sizes=[15])
        geom = i_sec
    elif section_name == "Angle_100x100":
        angle = angle_section(d=100.0, b=100.0, t=8.0, r_r=10.0, r_t=5.0, n_r=8)
        angle.create_mesh(mesh_sizes=[5])
        geom = angle
    elif section_name == "Thin_Channel_300x100x3":
        channel = channel_section(d=300.0, b=100.0, t_w=3.0, t_f=6.0, r=8.0, n_r=8)
        channel.create_mesh(mesh_sizes=[5])
        geom = channel
    else:
        raise ValueError(f"Unknown section: {section_name}")

    steel = Material(
        name="Steel",
        elastic_modulus=200_000,
        poissons_ratio=0.3,
        density=7850,
        yield_strength=250,
        color="grey"
    )
    
    # Create section and run warping analysis
    sec = Section(geometry=geom)
    sec.calculate_geometric_properties()
    sec.calculate_warping_properties()
    
    # Now access the internal warping analysis
    # The warping analysis uses Tri6 elements and assembles global matrices
    # We can access the internal assembly through the warping analysis
    # The warping analysis is stored in sec.section_props
    # But we need the global K, F, C matrices
    # Let's use the internal assemble_torsion method
    warping_section = Section(geometry=geom)
    warping_section.calculate_geometric_properties()
    warping_section.calculate_warping_properties()
    
    # The warping analysis creates elements and assembles global matrices
    # We can access the internal assemble_torsion method
    k_lg, f_torsion = warping_section.assemble_torsion()
    
    # k_lg is the Lagrangian matrix [K C; C^T 0] in CSC format
    # f_torsion is the load vector [F; 0]
    n_size = warping_section.num_nodes
    
    # Extract K, F, C from the augmented matrix
    K = k_lg[:n_size, :n_size]
    C = k_lg[:n_size, n_size:n_size+1].toarray().flatten()
    F = f_torsion[:n_size]
    
    return {
        'K': K.tocsc(),
        'F': F,
        'C': C,
        'n_dof': n_size
    }

def load_sparse_matrix(k_data):
    """Load sparse matrix from Rust JSON format (CSR)."""
    row_ptr = np.array(k_data['row_ptr'])
    col = np.array(k_data['col'])
    data = np.array(k_data['data'])
    n = len(row_ptr) - 1
    return csc_matrix((data, col, row_ptr), shape=(n, n))

def load_rust_data(json_path):
    with open(json_path, 'r') as f:
        return json.load(f)

def compare_sparse_matrices(rust_mat, py_mat, name):
    """Compare two sparse matrices."""
    n = rust_mat.shape[0]
    if n > 2000:
        diff_data = rust_mat - py_mat
        max_abs = np.max(np.abs(diff_data.data)) if diff_data.nnz > 0 else 0.0
        norm_rust = np.linalg.norm(rust_mat.data)
        norm_py = np.linalg.norm(py_mat.data)
        rel = max_abs / (norm_rust + 1e-15)
    else:
        rust_dense = rust_mat.toarray()
        py_dense = py_mat.toarray()
        diff = np.abs(rust_dense - py_dense)
        max_abs = np.max(diff)
        norm_rust = np.linalg.norm(rust_dense)
        norm_py = np.linalg.norm(py_dense)
        rel = max_abs / (norm_rust + 1e-15)
    
    print(f"  {name}: max_abs_diff={max_abs:.2e}, rel_diff={rel:.2e}")
    return max_abs, rel

def compare_vectors(rust_vec, py_vec, name):
    rust = np.array(rust_vec)
    py = np.array(py_vec)
    diff = np.abs(rust - py)
    max_abs = np.max(diff)
    rel = max_abs / (np.linalg.norm(rust) + 1e-15)
    print(f"  {name}: max_abs_diff={max_abs:.2e}, rel_diff={rel:.2e}")
    return max_abs, rel

def main():
    sections = [
        "Channel_200x75",
        "I_section_300x150",
        "Angle_100x100",
        "Thin_Channel_300x100x3"
    ]
    
    for name in sections:
        print(f"\n{'='*60}")
        print(f"COMPARING: {name}")
        print(f"{'='*60}")
        
        # Load Rust data
        rust_path = f"global_matrix_exports/{name}.json"
        with open(f"global_matrix_exports/{name}.json", 'r') as f:
            rust_data = json.load(f)
        
        print(f"\nRust data loaded:")
        print(f"  n_dof: {rust_data['n_dof']}")
        print(f"  n_elements: {rust_data['n_elements']}")
        print(f"  n_nodes: {rust_data['n_nodes']}")
        
        # Load Rust sparse matrices
        rust_K = load_sparse_matrix(rust_data['K'])
        rust_F = np.array(rust_data['F'])
        rust_C = np.array(rust_data['C'])
        
        print(f"\n  Rust K shape: {rust_K.shape} nnz: {rust_K.nnz}")
        print(f"  Rust F shape: {rust_F.shape}")
        print(f"  Rust C shape: {rust_C.shape}")
        print(f"  Rust F sum: {np.sum(rust_F):.2e}")
        print(f"  Rust C sum: {np.sum(rust_C):.2e}")
        
        # Assemble Python matrices
        print("\n  Assembling Python matrices...")
        py_data = assemble_python_global(name)
        py_K = py_data['K']
        py_F = py_data['F']
        py_C = py_data['C']
        
        print(f"  Python K shape: {py_K.shape} nnz: {py_K.nnz}")
        print(f"  Python F shape: {py_F.shape}")
        print(f"  Python C shape: {py_C.shape}")
        print(f"  Python F sum: {np.sum(py_F):.2e}")
        print(f"  Python C sum: {np.sum(py_C):.2e}")
        
        # Compare
        print("\n  Comparison:")
        compare_sparse_matrices(rust_K, py_K, "K")
        compare_vectors(rust_F, py_F, "F")
        compare_vectors(rust_C, py_C, "C")
        
        # Also check DOF mapping by comparing node coordinates
        print("\n  Node coordinates (first 5):")
        for i in range(min(5, len(rust_data['nodes']))):
            print(f"  Rust node {i}: ({rust_data['nodes'][i][0]:.6f}, {rust_data['nodes'][i][1]:.6f})")
        
        # Element connectivity (first 3)
        print("\n  Element connectivity (first 3):")
        for i in range(min(3, len(rust_data['elements']))):
            print(f"  Rust element {i}: {rust_data['elements'][i]}")

if __name__ == '__main__':
    import json
    from scipy.sparse import csc_matrix
    main()