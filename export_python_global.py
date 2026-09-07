#!/usr/bin/env python3
"""Authoritative export of Python's global warping K, F, C.

Mirrors sectionproperties.calculate_warping_properties() exactly:

- a fresh Section is created on the (already meshed) geometry
- every element's coordinates are translated to the centroid frame
  (``el.coords[0, :] -= cx``, ``el.coords[1, :] -= cy``)
- only then ``assemble_torsion()`` is called, so the exported K, F, C are
  assembled in the centroid frame just like the reference implementation.

Node coordinates are reconstructed from the (shifted) element coordinates and
the true constraint vector C (last column of k_lg) is exported, together with
centroidal second moments of area and Python's authoritative solution J.
"""
import json

import numpy as np
from sectionproperties.analysis import Section
from sectionproperties.analysis.solver import solve_direct_lagrange
from sectionproperties.pre.library import (
    angle_section,
    channel_section,
    i_section,
)


def build_section(name):
    if name == "Channel_200x75":
        geom = channel_section(d=200.0, b=75.0, t_w=8.0, t_f=10.0, r=12.0, n_r=10)
        geom.create_mesh(mesh_sizes=[10])
    elif name == "I_section_300x150":
        geom = i_section(d=300.0, b=150.0, t_f=8.0, t_w=12.0, r=15.0, n_r=10)
        geom.create_mesh(mesh_sizes=[15])
    elif name == "Angle_100x100":
        geom = angle_section(d=100.0, b=100.0, t=8.0, r_r=10.0, r_t=5.0, n_r=8)
        geom.create_mesh(mesh_sizes=[5])
    elif name == "Thin_Channel_300x100x3":
        geom = channel_section(d=300.0, b=100.0, t_w=3.0, t_f=6.0, r=8.0, n_r=8)
        geom.create_mesh(mesh_sizes=[5])
    else:
        raise ValueError(name)
    return geom


def export(name):
    geom = build_section(name)
    sec = Section(geometry=geom)
    sec.calculate_geometric_properties()

    # Mirror calculate_warping_properties: solve on a fresh section whose
    # element coordinates are translated to the centroid frame.
    warping_section = Section(geometry=geom)
    cx, cy = sec.get_c()
    for el in warping_section.elements:
        el.coords[0, :] -= cx
        el.coords[1, :] -= cy

    n_size = warping_section.num_nodes
    k_lg, f_torsion = warping_section.assemble_torsion()
    k_lg_csc = k_lg.tocsc()

    # Exact Lagrange solution of the (n+1)x(n+1) system (Python semantics,
    # no eps regularization of the leading block).
    omega_py = solve_direct_lagrange(k_lg_csc, f_torsion)

    ixx_c, iyy_c, ixy_c = sec.get_ic()
    area = sec.get_area()
    ixx_g, iyy_g, ixy_g = sec.get_ig()

    j_python = ixx_c + iyy_c - float(np.dot(omega_py, f_torsion))
    omega_dot_f = float(np.dot(omega_py, f_torsion))

    # Extract K block and true C column (last column of k_lg)
    K = k_lg_csc[:n_size, :n_size].tocoo()
    C_col = k_lg_csc[:n_size, n_size].toarray().flatten()

    # Reconstruct node coords from (shifted) element coords
    coords = {}
    for el in warping_section.elements:
        for i, nid in enumerate(el.node_ids):
            coords[int(nid)] = [float(el.coords[0][i]), float(el.coords[1][i])]
    nodes = [coords[i] for i in range(n_size)]

    elements = [[int(x) for x in el.node_ids] for el in warping_section.elements]

    # Build augmented matrix A = [K C; C^T 0] and RHS b = [F; 0]
    # K is n x n, C is n x 1, C^T is 1 x n, bottom-right is 0
    # Only store non-zero entries in COO format
    A_row = list(K.row)
    A_col = list(K.col)
    A_data = list(K.data)
    
    # C column (upper right) - only non-zero entries
    for i, val in enumerate(C_col):
        if abs(val) > 1e-15:
            A_row.append(i)
            A_col.append(n_size)
            A_data.append(float(val))
    
    # C^T row (lower left) - only non-zero entries
    for i, val in enumerate(C_col):
        if abs(val) > 1e-15:
            A_row.append(n_size)
            A_col.append(i)
            A_data.append(float(val))
    
    # Bottom-right element (0) - not stored in COO
    # (0,0) at (n,n) is implicit
    
    # RHS b = [F; 0]
    b = list(f_torsion) + [0.0]

    data = {
        "n_dof": int(n_size),
        "K": {
            "row": [int(x) for x in K.row],
            "col": [int(x) for x in K.col],
            "data": [float(x) for x in np.asarray(K.data)],
            "shape": [K.shape[0], K.shape[1]],
        },
        "F": [float(x) for x in f_torsion],
        "C": [float(x) for x in C_col],
        "nodes": nodes,
        "elements": elements,
        "geometry": {
            "centroid": [float(cx), float(cy)],
            "area": float(area),
            "ixx_c": float(ixx_c),
            "iyy_c": float(iyy_c),
            "ixy_c": float(ixy_c),
            "ixx_g": float(ixx_g),
            "iyy_g": float(iyy_g),
            "ixy_g": float(ixy_g),
        },
        "j_python": float(j_python),
        "omega_dot_f_python": omega_dot_f,
        # Augmented system
        "A": {
            "row": [int(x) for x in A_row],
            "col": [int(x) for x in A_col],
            "data": [float(x) for x in A_data],
            "shape": [n_size + 1, n_size + 1],
        },
        "b": [float(x) for x in b],
    }
    out = f"python_global_{name}.json"
    with open(out, "w") as f:
        json.dump(data, f, indent=2)
    c_nnz = int(np.count_nonzero(np.abs(C_col) > 1e-12))
    print(
        f"Wrote {out}: n_dof={n_size}, K nnz={len(K.data)}, C nnz={c_nnz}, "
        f"J_py={j_python:.6e}, Ixx_c+Iyy_c={ixx_c + iyy_c:.6e}, "
        f"omega.f={omega_dot_f:.6e}, centroid=({cx:.3f},{cy:.3f})"
    )


if __name__ == "__main__":
    for s in [
        "Channel_200x75",
        "I_section_300x150",
        "Angle_100x100",
        "Thin_Channel_300x100x3",
    ]:
        export(s)