#!/usr/bin/env python3
"""Authoritative export of Python's global warping K, F, C.

Reproduces assemble_torsion() exactly and dumps the real constraint vector C
(the previous python_global_*.json had C zeroed out, which was an artifact).
Node coordinates are reconstructed from element coords.
"""
import json
import numpy as np
from sectionproperties.pre.library import channel_section, i_section, angle_section
from sectionproperties.analysis import Section


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

    n_size = sec.num_nodes
    k_lg, f_torsion = sec.assemble_torsion()
    k_lg_coo = k_lg.tocoo()

    # Extract K block and true C column
    K = k_lg.tocsc()[:n_size, :n_size].tocoo()
    C_col = k_lg.tocsc()[:n_size, n_size].toarray().flatten()

    # Reconstruct node coords from element coords (2x6 per element)
    coords = {}
    for el in sec.elements:
        for i, nid in enumerate(el.node_ids):
            coords[int(nid)] = [float(el.coords[0][i]), float(el.coords[1][i])]
    nodes = [coords[i] for i in range(n_size)]

    elements = [[int(x) for x in el.node_ids] for el in sec.elements]

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
    }
    out = f"python_global_{name}.json"
    with open(out, "w") as f:
        json.dump(data, f, indent=2)
    c_nnz = int(np.count_nonzero(np.abs(C_col) > 1e-12))
    print(f"Wrote {out}: n_dof={n_size}, K nnz={len(K.data)}, C nnz={c_nnz}")


if __name__ == "__main__":
    for s in ["Channel_200x75", "I_section_300x150", "Angle_100x100",
              "Thin_Channel_300x100x3"]:
        export(s)
